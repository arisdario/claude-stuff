#!/usr/bin/env python3
"""AHB SHADOW - what a camera-based automatic high beam WOULD do. Nothing is switched: the driving logger records the
decision with pictures, so a night drive shows how often it would be right (false dips, missed cars) BEFORE it is ever
connected to the light lever (tesla/ahb_stalk.py decides whether that is possible at all).

Meant to work like Tesla's own: only while YOU hold the light lever out (high beams selected) - it then dips for
traffic and restores when the road is clear. In shadow mode it decides all the time, so the data is complete.

Inputs, 5 times a second:
  camerad's light finder (/dev/c920_lights.json, cameras/ahb_vision.cc; on with touch /data/ahb_vision_on):
    n            light SOURCES (headlights) at/near the horizon - a car's lights are always there; street lamps are above
    lamps_above  light sources clearly above the horizon: street lamps -> a lit road (no high beams there)
    band_luma    brightness of the band around the horizon (0-255)
  the car: speed, and openpilot's lead (radar + camera track cars ahead to 160-200 m; tail lights are left to them)
Decision:
  DIP at once when any of: no fresh camera data, below OFF_KPH, headlights seen, a car ahead within LEAD_M, a lit road
  (more than LAMPS_MAX street lamps), a bright band (BRIGHT_LUMA: dusk / lit road).
  HIGH only after everything has been clear for HOLD_S (no flicker). Thresholds are a first guess - the log is for tuning.
"""
import json
import os

LIGHTS_FILE = '/dev/c920_lights.json'
STALE_S = 1.0          # older camera data = no camera = dip
ON_KPH = 40.           # high beams allowed from here ...
OFF_KPH = 30.          # ... and dipped below this (the gap stops flicker)
LEAD_M = 150.          # a car ahead closer than this: dip (its driver sees our beams in the mirrors)
LAMPS_MAX = 2          # more street lamps above the horizon than this: a lit road
BRIGHT_LUMA = 90.      # the band around the horizon brighter than this: not dark enough (first guess)
HOLD_S = 3.0           # everything clear this long before high again


def read_lights(now_wall, path=LIGHTS_FILE):
  """(data, fresh): data = camerad's last result or None if the file does not exist (the feature is off)"""
  try:
    with open(path) as h:
      d = json.load(h)
  except (OSError, ValueError):
    return None, False
  t = d.get('t')
  return d, (isinstance(t, (int, float)) and abs(now_wall - t) < STALE_S)


class AhbShadow(object):
  def __init__(self):
    self.state, self.why, self.clear_since, self.slow = 'dip', 'start', None, True

  def _block(self, v_ego, lead_d, lights, fresh):
    """why high beams must NOT be on right now (first reason), or None"""
    kph = (v_ego or 0.) * 3.6
    self.slow = kph < (OFF_KPH if not self.slow else ON_KPH)        # hysteresis on speed
    if not lights or not fresh:
      return 'no_camera'
    if self.slow:
      return 'slow'
    if (lights.get('n') or 0) > 0:
      return 'headlights'
    if lead_d is not None and lead_d < LEAD_M:
      return 'car_ahead'
    if (lights.get('lamps_above') or 0) > LAMPS_MAX:
      return 'lit_road'
    if (lights.get('band_luma') or 0.) > BRIGHT_LUMA:
      return 'bright'
    return None

  def step(self, now, v_ego, lead_d, lights, fresh):
    """one decision; returns (state, why, changed). lead_d = distance to the car ahead (m) or None"""
    block = self._block(v_ego, lead_d, lights, fresh)
    prev = self.state
    if block:
      self.state, self.why, self.clear_since = 'dip', block, None
    else:
      if self.clear_since is None:
        self.clear_since = now
      if self.state == 'dip' and now - self.clear_since >= HOLD_S:
        self.state = 'high'
      self.why = 'clear'
    return self.state, self.why, self.state != prev

  def line(self, now_wall, v_ego, lead_d, lights):
    """the AHB log line (src 'ahb')"""
    L = lights or {}
    return json.dumps({'src': 'ahb', 't': round(now_wall, 3), 'state': self.state, 'why': self.why,
                       'v': round(v_ego or 0., 2), 'lead_d': None if lead_d is None else round(lead_d, 1),
                       'n': L.get('n'), 'lamps': L.get('lamps_above'), 'luma': L.get('band_luma'),
                       'horizon': L.get('horizon'), 'lights': L.get('lights')}, separators=(',', ':'))


def feature_on():
  return os.path.exists('/data/ahb_vision_on')
