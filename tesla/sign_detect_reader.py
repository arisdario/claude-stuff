#!/usr/bin/env python3
"""sign_detect_reader.py - pure functions reading camerad's optional sign/light detector output
(/data/sign_detect.json, written by sign_detect_io.cc - see camerad/cameras/SIGN_DETECT.md). Same
shape as steer_limits.py: no openpilot imports, file I/O isolated to one small function, everything
else pure and unit-testable without a car or cereal.

Read-only: this module never writes anything, never touches CAN directly (carcontroller.py does
that, using teslacan.create_fake_DAS_sign_msg - the same packer the rest of this repo already uses).

Only stop-sign detections are turned into the IC display message: the detector gives stop signs a
real geometric distance estimate (known sign width + camera focal length), but NOT for traffic
lights (there's no comparably reliable standard lens size to estimate distance from) - and the IC's
own stopLightWarning logic compares a distance against a threshold, so sending a light without a real
distance would be presenting false precision on the cluster. Traffic light color/presence is still
read and shown read-only in lightdash (sign_detect.py/.cc there), just not sent to the car's display.
"""
import json
import time

PATH = '/data/sign_detect.json'
MAX_AGE_S = 1.0          # the detector writes at ~2.5 Hz; anything older than this is stale, not current
ROAD_SIGN_TYPE_STOP = 0x00   # matches carcontroller.py's own roadSignType convention (checkWhichSign)

EMPTY = {'stop_sign': False, 'stop_sign_dist_m': None, 'stop_sign_confidence': None,
         'light_present': False, 'light_color': None, 'light_confidence': None}


def read_detection(path=PATH, max_age_s=MAX_AGE_S, now=None):
  """the latest detection, or EMPTY if the file is missing, malformed, or older than max_age_s.
  Never raises."""
  now = time.time() if now is None else now
  try:
    with open(path) as f:
      d = json.load(f)
  except Exception:
    return dict(EMPTY)
  t = d.get('t')
  if t is None or (now - t) > max_age_s or (now - t) < 0:
    return dict(EMPTY)
  return {
    'stop_sign': bool(d.get('stop_sign')), 'stop_sign_dist_m': d.get('stop_sign_dist_m'),
    'stop_sign_confidence': d.get('stop_sign_confidence'),
    'light_present': bool(d.get('light_present')), 'light_color': d.get('light_color'),
    'light_confidence': d.get('light_confidence'),
  }


def pick_ic_sign(detection, min_confidence=0.5):
  """-> (roadSignType, dist_m, color, control_active) for teslacan.create_fake_DAS_sign_msg, or None
  if nothing confident enough to show. Stop signs only - see module docstring for why."""
  if not detection.get('stop_sign'):
    return None
  dist = detection.get('stop_sign_dist_m')
  conf = detection.get('stop_sign_confidence')
  if dist is None or dist <= 0 or conf is None or conf < min_confidence:
    return None
  return (ROAD_SIGN_TYPE_STOP, float(dist), 0, 0)   # color 0 (n/a for a stop sign), not already resuming
