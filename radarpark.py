#!/usr/bin/env python3
"""
radarpark.py - checks the radar's distance against the parking sensors, for radarweb. Measures only, changes nothing.

Each time the car stands still for STOP_MIN_S or more with something in front within the parking sensors' reach, the
stop gives one measurement: the radar's distance to the object straight ahead minus the parking-sensor gap (the
closer of the two front corner sensors). The median over many stops is the radar's distance offset (mounting depth
behind the bumper, plus any bias). Stops where the parking sensors see something close but the radar sees nothing
straight ahead are counted separately: that is the radar's real minimum range on this car.

The corner sensors look slightly diagonally at a car straight ahead, so expect roughly +-10 cm.
Saved in /data/radar_offset.json (survives restarts). Tests: tests/test_radarpark.py
"""
import json
import os

STOP_V = 0.2             # m/s: standing still
MOVE_V = 0.5             # m/s: the stop has ended
STOP_MIN_S = 2.0         # s standing before samples count
STOP_MAX_S = 30.0        # s of samples per stop at most (a long wait does not outweigh many short ones)
PARK_MIN_CM, PARK_MAX_CM = 40, 200     # parking-sensor gap used (sensors read 255 = nothing)
AHEAD_Y = 1.0            # m either side of straight ahead for the radar object
RADAR_MAX_D = 5.0        # m: radar objects farther than this are not the thing the sensors see
MIN_SAMPLES = 10         # radar+sensor samples needed in one stop to accept it
MAX_SPREAD = 0.15        # m: inside one stop the difference must be steady (same object, no flicker)
KEEP = 200               # stops kept
SAVE_FILE = '/data/radar_offset.json'


def _median(xs):
  s = sorted(xs); n = len(s)
  if not n: return None
  return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _quartiles(xs):
  s = sorted(xs); n = len(s)
  if n < 4: return None, None
  return s[n // 4], s[(3 * n) // 4]


def front_gap_cm(park):
  """closer of the two front corner sensors in cm, or None (nothing in range / sensors inactive)"""
  f = (park or {}).get('f') or {}
  vals = [v for v in (f.get('l'), f.get('r')) if v is not None and 0 < v < 255]
  return min(vals) if vals else None


def radar_ahead_m(objs):
  """distance of the nearest radar object straight ahead within RADAR_MAX_D, or None"""
  ds = [o['d'] for o in objs or [] if o.get('d') is not None and 0 < o['d'] < RADAR_MAX_D and abs(o.get('y') or 0) < AHEAD_Y]
  return min(ds) if ds else None


class OffsetMeter:
  def __init__(self, save_file=SAVE_FILE):
    self.save_file = save_file
    self.stops = []          # accepted: [time, park_m, radar_m, diff_m]
    self.missed = []         # parking sensors saw something close, radar nothing straight ahead: [time, park_m]
    self.t_stop = None; self.diffs = []; self.pairs = []; self.gap_only = []
    self._load()

  def _load(self):
    try:
      with open(self.save_file) as f: d = json.load(f)
      self.stops = d.get('stops', [])[-KEEP:]; self.missed = d.get('missed', [])[-KEEP:]
    except Exception:
      pass

  def _save(self):
    try:
      d = os.path.dirname(self.save_file)
      if d: os.makedirs(d, exist_ok=True)
      tmp = self.save_file + '.tmp'
      with open(tmp, 'w') as f: json.dump({'stops': self.stops, 'missed': self.missed}, f)
      os.replace(tmp, self.save_file)
    except Exception as e:
      print('radarpark: could not save:', e)

  def update(self, now, v, park, objs):
    """feed radarweb's 10 Hz state; returns True when a stop was just evaluated"""
    if v is not None and v < STOP_V:
      if self.t_stop is None: self.t_stop, self.diffs, self.pairs, self.gap_only = now, [], [], []
      if now - self.t_stop >= STOP_MIN_S and now - self.t_stop <= STOP_MIN_S + STOP_MAX_S:
        gap = front_gap_cm(park)
        if gap is not None and PARK_MIN_CM <= gap <= PARK_MAX_CM:
          r = radar_ahead_m(objs)
          if r is not None:
            self.diffs.append(r - gap / 100.0); self.pairs.append((gap / 100.0, r))
          else:
            self.gap_only.append(gap / 100.0)
      return False
    if self.t_stop is not None and (v is None or v > MOVE_V):
      done = self._finish(now); self.t_stop = None
      return done
    return False

  def _finish(self, now):
    if len(self.diffs) >= MIN_SAMPLES:
      lo, hi = min(self.diffs), max(self.diffs)
      if hi - lo <= MAX_SPREAD * 2:
        park = _median([p for p, r in self.pairs]); rad = _median([r for p, r in self.pairs])
        self.stops.append([round(now, 1), round(park, 3), round(rad, 3), round(_median(self.diffs), 3)])
        self.stops = self.stops[-KEEP:]; self._save(); return True
    elif len(self.gap_only) >= MIN_SAMPLES and len(self.diffs) == 0:
      self.missed.append([round(now, 1), round(_median(self.gap_only), 3)])
      self.missed = self.missed[-KEEP:]; self._save(); return True
    return False

  def summary(self):
    diffs = [s[3] for s in self.stops]
    q1, q3 = _quartiles(diffs)
    seen = [s[1] for s in self.stops]
    return {'n': len(diffs), 'offset_m': None if not diffs else round(_median(diffs), 3),
            'iqr_m': None if q1 is None else round(q3 - q1, 3), 'last_m': diffs[-1] if diffs else None,
            'missed': len(self.missed),
            'missed_max_gap_m': max((m[1] for m in self.missed), default=None),
            'radar_min_seen_gap_m': min(seen) if seen else None}
