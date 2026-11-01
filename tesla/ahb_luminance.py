#!/usr/bin/env python3
"""
ahb_luminance.py - road-scene brightness for the auto high beam / auto low beam controllers.

No physical lux sensor exists on this car (verified: no rain/light-sensor signal in the Tesla DBC, CarState or captured
CAN). So brightness here is a RELATIVE LUMINANCE ESTIMATE, not calibrated lux. Two sources, in order of preference:

  1. Camera auto-exposure (always available, ~free in the control loop): the road camera's `frame` message gives
     globalGain and integLines. Auto-exposure holds the sensor's collected light roughly constant, so the exposure it
     needs, E = gain * integLines, is inversely proportional to scene brightness. We turn that into a brightness index
     in photographic "stops above the darkest night" (log2). This is a WHOLE-FRAME measure.

  2. A road-region pixel luminance (optional, from an external camera-reading daemon, never the control loop): mean and
     median grey level over a forward road ROI that excludes sky, hood and dashboard. region_luminance() computes it;
     feed its result to LuminanceEstimator.update_pixels(). Not wired on the EON by default (reading the image buffer is
     heavier and version-specific), but the interface and the math are here and unit-tested.

The estimator smooths over time, rejects exposure-transient spikes, handles missing/!valid frames, and classifies the
scene into DARK / DIM / BRIGHT with hysteresis so a single streetlight or an underpass does not flip the decision.
Pure Python; numpy is only needed for the optional image helper (so the exposure path runs on the EON without numpy).

All thresholds are module-level constants, tunable, with conservative defaults. The real night-vs-oncoming numbers must
be set from a logged night drive (see the calibration note at the bottom) - the defaults are deliberately cautious.
"""
import math

# --- exposure model -------------------------------------------------------------------------------------------------
# The darkest the camera gets on this EON (both at maximum): gain 510, integLines ~5408 (measured, parked at night).
# E_DARK is the reference exposure for "no light at all"; brightness index = log2(E_DARK / E), so 0 at darkest night,
# +1 per doubling of scene brightness. Daylight is ~10-15 stops, a lit street a few stops, oncoming headlights a
# fraction of a stop to a few stops depending on range.
GAIN_MAX = 510.
INTEG_MAX = 5408.
E_DARK = GAIN_MAX * INTEG_MAX
BI_MIN, BI_MAX = 0.0, 16.0          # clamp the index to a sane range

# --- classification (brightness index, "stops above darkest night") -------------------------------------------------
# Conservative: only call it DARK (high-beam eligible) when it is really dark. Hysteresis gap so it does not chatter.
DARK_ENTER = 1.2        # below this (and falling): DARK          (<~2.3x the darkest -> genuinely unlit road)
DARK_EXIT = 2.0         # above this (and rising): leave DARK
BRIGHT_ENTER = 5.0      # above this: BRIGHT (daylight / well-lit urban) -> never high beam, and low beam not needed
BRIGHT_EXIT = 4.0       # below this: leave BRIGHT
# between DARK and BRIGHT is DIM (dusk, lit suburb): low beam yes, high beam no.

# --- smoothing and spike rejection ----------------------------------------------------------------------------------
TAU_S = 2.0             # exponential smoothing time constant (s): follows real brightness changes, ignores flicker
SPIKE_STOPS = 3.0       # a jump larger than this from the smoothed value is treated as a possible exposure transient
SPIKE_WEIGHT = 0.15     # how much of a (still-transient) spike sample is let in (0 = ignore entirely)
SPIKE_PERSIST_S = 0.4   # ... but if the big deviation lasts at least this long it is a real change (tunnel, dusk) and
                        # is accepted at full weight; only a momentary one-or-two-frame flash/shadow stays rejected
STALE_S = 1.0           # no fresh frame for this long -> estimate is stale -> brightness unknown (conservative)

# --- levels ---------------------------------------------------------------------------------------------------------
UNKNOWN, DARK, DIM, BRIGHT = 'unknown', 'dark', 'dim', 'bright'


def relative_luminance_index(gain, integ_lines):
  """brightness index (stops above the darkest night) from camera exposure, or None if the inputs are unusable.
  Higher = brighter. 0 = as dark as the camera ever reports. This is a RELATIVE ESTIMATE, not lux."""
  try:
    g = float(gain); il = float(integ_lines)
  except (TypeError, ValueError):
    return None
  e = g * il
  if not (e > 0.) or g <= 0. or il <= 0.:
    return None
  bi = math.log(E_DARK / e, 2.)
  return max(BI_MIN, min(BI_MAX, bi))


def region_luminance(np, img, roi=None, is_rgb=True):
  """mean and median grey level (0..255) over a forward road ROI of an image. For the optional external camera daemon
  and for tests; NOT called in the control loop. roi = (x0, y0, x1, y1) fractions of width/height; default is a band
  that skips the top (sky) and bottom (hood/dash) and the far sides.
  Returns (mean, median) or (None, None) if the image is unusable."""
  try:
    a = np.asarray(img)
    if a.ndim == 3 and is_rgb:
      a = 0.299 * a[..., 0] + 0.587 * a[..., 1] + 0.114 * a[..., 2]   # Rec.601 luma
    elif a.ndim == 3:
      a = a[..., 0]                                                    # already luma (e.g. YUV Y plane)
    h, w = a.shape[:2]
    x0, y0, x1, y1 = roi or (0.2, 0.45, 0.8, 0.85)
    sub = a[int(y0 * h):int(y1 * h), int(x0 * w):int(x1 * w)]
    if sub.size == 0:
      return None, None
    return float(sub.mean()), float(np.median(sub))
  except Exception:
    return None, None


def pixel_mean_to_index(mean_grey):
  """turn a 0..255 road-ROI mean grey level into the same brightness-index scale as the exposure path, so either
  source feeds the same classifier. Rough log map: mid-grey (128) ~ well-lit, near-black ~ darkest night."""
  if mean_grey is None or mean_grey < 0:
    return None
  g = max(0.5, min(255., float(mean_grey)))
  return max(BI_MIN, min(BI_MAX, math.log(g / 1.0, 2.)))   # 1->0 stops, 2->1, 128->7, 255->~8


class LuminanceEstimator(object):
  """keeps a smoothed brightness index and a hysteretic DARK/DIM/BRIGHT classification. Feed it exposure samples
  (update_exposure) or pixel luminance (update_pixels); both land on the same scale. now is seconds (monotonic)."""

  def __init__(self):
    self.bi = None              # smoothed brightness index
    self.raw = None             # last raw index
    self.level = UNKNOWN
    self.last_t = None
    self.valid = False
    self._dev_since = None       # when the raw reading first deviated far from the smoothed value

  def _ingest(self, bi, now):
    if bi is None:
      return
    self.raw = bi
    if self.bi is None or self.last_t is None:
      self.bi = bi
    else:
      dt = max(0., now - self.last_t)
      w = 1. - math.exp(-dt / TAU_S) if TAU_S > 0 else 1.
      if abs(bi - self.bi) > SPIKE_STOPS:
        if self._dev_since is None:
          self._dev_since = now
        if now - self._dev_since < SPIKE_PERSIST_S:   # brief outlier (flash/shadow/gain step): let little of it in
          w *= SPIKE_WEIGHT
        # else: the deviation has lasted -> it is a real change (tunnel, dusk) -> full weight
      else:
        self._dev_since = None
      self.bi += w * (bi - self.bi)
    self.last_t = now
    self.valid = True
    self._classify()

  def update_exposure(self, gain, integ_lines, now):
    self._ingest(relative_luminance_index(gain, integ_lines), now)
    return self.level

  def update_pixels(self, mean_grey, now):
    self._ingest(pixel_mean_to_index(mean_grey), now)
    return self.level

  def tick(self, now):
    """call every cycle: expires the estimate if no fresh frame arrived (camera stall / disconnect)."""
    if self.last_t is not None and now - self.last_t > STALE_S:
      self.valid = False
      self.level = UNKNOWN
    return self.level

  def _classify(self):
    b = self.bi
    if b is None:
      self.level = UNKNOWN; return
    lv = self.level
    if lv == DARK:
      if b > DARK_EXIT: lv = BRIGHT if b > BRIGHT_ENTER else DIM
    elif lv == BRIGHT:
      if b < BRIGHT_EXIT: lv = DARK if b < DARK_ENTER else DIM
    else:                                   # DIM or UNKNOWN: resolve by band (hysteresis lives at the edges above)
      lv = DARK if b < DARK_ENTER else BRIGHT if b > BRIGHT_ENTER else DIM
    self.level = lv

  def is_dark(self):
    """dark enough to consider high beam (and headlights needed). Unknown is NOT dark (conservative)."""
    return self.valid and self.level == DARK

  def needs_headlights(self):
    """dim or dark -> low beam wanted. Unknown -> False (do not command lights on uncertain data)."""
    return self.valid and self.level in (DARK, DIM)

  def diagnostics(self):
    return {'lum_index': None if self.bi is None else round(self.bi, 2),
            'lum_raw': None if self.raw is None else round(self.raw, 2),
            'lum_level': self.level, 'lum_valid': self.valid, 'lum_is_estimate': True}


# --- calibration note -----------------------------------------------------------------------------------------------
# DARK_ENTER/EXIT and BRIGHT_* are the numbers that decide the feature's behaviour and they are set cautiously here.
# To calibrate: log relative_luminance_index(gain, integLines) on a night drive that includes unlit road, a lit town,
# an oncoming car and (if possible) a tunnel. DARK_ENTER should sit just above the unlit-road index; BRIGHT_ENTER below
# the lit-town index. The existing tools/drivetrace.py already records gain-derived brightness and can be extended.
