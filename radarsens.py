#!/usr/bin/env python3
"""
radarsens.py - more of the car's own sensors for radarweb (display, warnings and recording only; nothing is sent).
Plain Python, no openpilot imports. Tests: tests/test_radarsens.py

  sun_state      UI_solarData (MCU): night (sun below the horizon) and sun glare (low sun straight ahead: the
                 camera is blinded, lane lines may drop out)
  fleet_state    UI_driverAssistRoadSign (MCU, multiplexed): Tesla's crowd-sourced speeds for the road you are on
  WheelMonitor   ESP_B: the four 8-bit wheel pulse counters, counted on every frame (they wrap several times a second).
                 Slip = front and rear axle turning at different speeds (wheelspin / locking). Tyres = each wheel's
                 share of the pulses on straight steady driving, against a baseline you set after checking pressures:
                 a softer tyre has a smaller rolling radius and turns slightly faster.
  CurveCheck     UI_roadCurvature (the MCU's map) against the curvature actually driven (GPS heading change / speed),
                 to see whether the map curvature can be trusted for curve warnings.
"""
import json
import math
import os

# ---------------- sun ----------------
NIGHT_ELEV = -3.0          # deg: sun this far below the horizon = dark enough for headlights (civil twilight starts at -6)
GLARE_ELEV = (0.0, 15.0)   # deg: low sun ...
GLARE_AZ = 25.0            # deg: ... within this angle of straight ahead


def sun_state(elev, az_car, is_up=None):
  if elev is None: return None
  return {'elev': round(elev, 1), 'az': None if az_car is None else round(az_car, 1),
          'night': elev < NIGHT_ELEV,
          'glare': az_car is not None and GLARE_ELEV[0] <= elev <= GLARE_ELEV[1] and abs(az_car) <= GLARE_AZ}


# ---------------- fleet speed ----------------
def fleet_state(sig):
  """sig: decoded UI_driverAssistRoadSign signals (both multiplex pages merged). m/s, None when 0 / missing."""
  g = lambda k: (sig.get(k) or None) if sig else None
  out = {'median': g('UI_medianFleetSpeedMPS'), 'low': g('UI_bottomQrtlFleetSpeedMPS'), 'high': g('UI_topQrtlFleetSpeedMPS'),
         'spline': g('UI_meanFleetSplineSpeedMPS'), 'map_limit': g('UI_baseMapSpeedLimitMPS'),
         'accel': sig.get('UI_meanFleetSplineAccelMPS2') if sig else None, 'conf': sig.get('UI_splineLocConfidence') if sig else None}
  return out if any(out[k] for k in ('median', 'high', 'spline')) else None


FLEET_FAST = 1.10          # faster than the fleet's fast quarter by 10% ...
FLEET_FAST_MIN = 8.0       # ... and the fleet speed is above 8 m/s (not a car park)


def faster_than_fleet(v, fleet):
  if not fleet or not fleet.get('high') or fleet['high'] < FLEET_FAST_MIN or v is None: return False
  return v > fleet['high'] * FLEET_FAST


# ---------------- wheels ----------------
SLIP_RATIO = 0.08          # front vs rear axle speed differing by 8% from normal = slip
SLIP_MIN_V = 3.0           # m/s
SLIP_HOLD_S = 0.3
TYRE_V = (13.0, 35.0)      # m/s: steady driving speeds used for the tyre comparison
TYRE_STEER = 3.0           # deg steering wheel: straight
TYRE_ACCEL = 0.3           # m/s2: steady
TYRE_MIN_KM = 30.0         # km of usable driving before a tyre verdict
TYRE_FLAG = 0.25           # %: a wheel turning this much faster than at the baseline = check its pressure
WHEELS = ('fl', 'fr', 'rl', 'rr')
SAVE_FILE = '/data/wheels.json'


class WheelMonitor:
  def __init__(self, save_file=SAVE_FILE):
    self.save_file = save_file
    self.last = None                 # last raw counters
    self.acc = [0, 0, 0, 0]          # pulses since the last window
    self.win_t = None
    self.slip_since = None; self.slip_events = []; self.slip_now = False
    self.fr_norm = None              # front/rear ratio when driving normally (learned)
    self.tyre_sum = [0.0] * 4; self.tyre_n = 0; self.tyre_m = 0.0
    self.baseline = None; self.base_m = 0.0
    self.pulses_per_m = None
    self.current = None
    self._load()

  def _load(self):
    try:
      with open(self.save_file) as f: d = json.load(f)
      self.baseline = d.get('baseline'); self.tyre_sum = d.get('sum', self.tyre_sum); self.tyre_n = d.get('n', 0)
      self.tyre_m = d.get('m', 0.0); self.fr_norm = d.get('fr_norm'); self.slip_events = d.get('slips', [])[-100:]
      self.pulses_per_m = d.get('ppm')
    except Exception:
      pass

  def save(self):
    try:
      dn = os.path.dirname(self.save_file)
      if dn: os.makedirs(dn, exist_ok=True)
      tmp = self.save_file + '.tmp'
      with open(tmp, 'w') as f:
        json.dump({'baseline': self.baseline, 'sum': self.tyre_sum, 'n': self.tyre_n, 'm': self.tyre_m,
                   'fr_norm': self.fr_norm, 'slips': self.slip_events[-100:], 'ppm': self.pulses_per_m}, f)
      os.replace(tmp, self.save_file)
    except Exception as e:
      print('radarsens: could not save wheels:', e)

  @staticmethod
  def decode(b):
    """ESP_B: FrL, FrR, ReL, ReR 8-bit counters in bytes 0..3 (Motorola start bits 7/15/23/31, 8 bits each)"""
    return (b[0], b[1], b[2], b[3]) if len(b) >= 4 else None

  def feed(self, b):
    """every ESP_B frame: add the pulses since the previous frame (8-bit wrap handled)"""
    c = self.decode(b)
    if c is None: return
    if self.last is not None:
      for i in range(4): self.acc[i] += (c[i] - self.last[i]) & 0xFF
    self.last = c

  def window(self, now, v, steer_deg, accel, win_s=0.2):
    """call often (10 Hz). Every win_s: slip check; every window also feeds the tyre statistics. Returns a slip event
    dict when one starts."""
    if self.win_t is None: self.win_t = now; self.acc = [0, 0, 0, 0]; return None
    if now - self.win_t < win_s: return None
    d = self.acc; dt = now - self.win_t; self.acc = [0, 0, 0, 0]; self.win_t = now
    tot = sum(d)
    if tot <= 0 or v is None: self.current = None; return None
    if v > 3 and dt > 0: self.pulses_per_m = round(0.98 * (self.pulses_per_m or tot / 4 / (v * dt)) + 0.02 * (tot / 4 / (v * dt)), 3)
    mean = tot / 4.0
    self.current = {k: round(d[i] / mean, 4) for i, k in enumerate(WHEELS)}
    front, rear = d[0] + d[1], d[2] + d[3]
    ev = None
    if rear > 0 and v > SLIP_MIN_V:
      fr = front / float(rear)
      steady = abs(steer_deg or 0) < 30 and abs(accel or 0) < 1.0
      if self.fr_norm is None: self.fr_norm = fr if steady else None
      elif steady: self.fr_norm = 0.995 * self.fr_norm + 0.005 * fr
      if self.fr_norm and abs(fr / self.fr_norm - 1.0) > SLIP_RATIO:
        if self.slip_since is None: self.slip_since = now
        if not self.slip_now and now - self.slip_since >= SLIP_HOLD_S:
          self.slip_now = True
          ev = {'t': round(now, 1), 'kind': 'rear faster (wheelspin or front locking)' if fr < self.fr_norm else
                'front faster (front wheelspin or rear locking)', 'diff_pct': round((fr / self.fr_norm - 1) * 100, 1),
                'kmh': round(v * 3.6)}
          self.slip_events.append(ev); self.slip_events = self.slip_events[-100:]
      else:
        self.slip_since = None; self.slip_now = False
    # tyres: straight, steady, at speed
    if TYRE_V[0] <= v <= TYRE_V[1] and abs(steer_deg or 0) < TYRE_STEER and abs(accel or 0) < TYRE_ACCEL and not self.slip_now:
      for i in range(4): self.tyre_sum[i] += d[i] / mean
      self.tyre_n += 1; self.tyre_m += v * dt
    return ev

  def tyre_ratios(self):
    if self.tyre_n == 0: return None
    return [s / self.tyre_n for s in self.tyre_sum]

  def set_baseline(self):
    """after checking all four pressures: what the tyres look like now becomes 'normal'"""
    r = self.tyre_ratios()
    self.baseline = r
    self.tyre_sum = [0.0] * 4; self.tyre_n = 0; self.base_m = self.tyre_m; self.tyre_m = 0.0
    self.save()
    return r is not None

  def summary(self):
    r = self.tyre_ratios()
    out = {'now': self.current, 'km': round(self.tyre_m / 1000.0, 1), 'baseline_set': self.baseline is not None,
           'slips': len(self.slip_events), 'last_slip': self.slip_events[-1] if self.slip_events else None,
           'pulses_per_m': self.pulses_per_m}
    if r and self.baseline:
      q = [r[i] / self.baseline[i] for i in range(4)]            # each wheel against itself at the baseline ...
      dev = {k: round((q[i] / sorted(q[j] for j in range(4) if j != i)[1] - 1.0) * 100.0, 3)   # ... against the median of
             for i, k in enumerate(WHEELS)}                                                         # the other three (one odd tyre can't skew it)
      out['dev_pct'] = dev
      ready = self.tyre_m / 1000.0 >= TYRE_MIN_KM
      out['ready'] = ready
      out['check'] = [k for k, x in dev.items() if x >= TYRE_FLAG] if ready else []
    elif r:
      out['share'] = {k: round(r[i], 4) for i, k in enumerate(WHEELS)}
    return out


# ---------------- map curvature check ----------------
CURVE_MIN_V = 10.0         # m/s
CURVE_KEEP = 600


class CurveCheck:
  """pairs of (map curvature at the car, driven curvature from GPS heading change); correlation and scale"""
  def __init__(self):
    self.prev = None; self.pairs = []

  def update(self, now, v, hdg_deg, c2):
    if hdg_deg is None or c2 is None or v is None:
      self.prev = None; return
    if self.prev is not None and v >= CURVE_MIN_V:
      t0, h0 = self.prev
      dt = now - t0
      if 0.5 <= dt <= 3.0:
        dh = (hdg_deg - h0 + 540.0) % 360.0 - 180.0
        driven = math.radians(dh) / dt / v           # 1/m, + = turning right (heading increases clockwise)
        self.pairs.append((2.0 * c2, driven)); self.pairs = self.pairs[-CURVE_KEEP:]
    self.prev = (now, hdg_deg)

  def summary(self):
    n = len(self.pairs)
    if n < 30: return {'n': n}
    xs = [p[0] for p in self.pairs]; ys = [p[1] for p in self.pairs]
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs); syy = sum((y - my) ** 2 for y in ys); sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0 or syy <= 0: return {'n': n}
    r = sxy / math.sqrt(sxx * syy)
    return {'n': n, 'r': round(r, 2), 'scale': round(sxy / sxx, 2),
            'verdict': 'matches the road' if abs(r) >= 0.7 else 'partly matches' if abs(r) >= 0.4 else 'does not match the road'}
