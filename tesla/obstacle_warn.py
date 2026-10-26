# Obstacle warning detector (advisory only, never commands the car)
import time

MIN_SPEED = 8.0      # OBSTUNE m/s (about 29 km/h), no warnings when slower
PATH_HALF = 1.0      # OBSTUNE2 m, "in my path" corridor half-width (car half-width ~0.95)
SIDE_MAX = 2.6       # m, side band out to here
PATH_TTC = 2.5       # OBSTUNE2 s, time to contact below this = path warning
PATH_MAXD = 45.0     # m
SIDE_MAXD = 20.0     # m
SIDE_MIN_OBJS = 3    # objects needed in the side band
SIDE_COOLDOWN = 15.0 # s, one side notice per zone
STATIONARY = 1.8     # m/s, |relSpeed + vEgo| below this = stationary
MIN_SEEN = 8         # consecutive frames before an object counts
HOLD = 1.5           # s, keep a warning level this long to avoid flicker


class ObstacleDetector(object):
  def __init__(self):
    self.seen = {}
    self.level = 0
    self.hold_until = 0.
    self.side_quiet_until = 0.

  def update(self, items, v_ego, now=None):
    """items: list of (key, ok, dist, lateral, rel_speed). Returns 0 none, 1 side, 2 path."""
    if now is None:
      now = time.time()
    objs = []
    for key, ok, d, y, v in items:
      n = self.seen.get(key, 0) + 1 if ok else 0
      self.seen[key] = n
      if n >= MIN_SEEN:
        objs.append((d, y, v))
    level = 0
    if v_ego >= MIN_SPEED:
      for d, y, v in objs:
        if abs(y) < PATH_HALF and 1.0 < d < PATH_MAXD and v < -0.5 and d / -v < PATH_TTC:
          level = 2
          break
      if level == 0:
        side = [1 for d, y, v in objs
                if abs(v + v_ego) < STATIONARY and PATH_HALF <= abs(y) <= SIDE_MAX and d < SIDE_MAXD]
        if len(side) >= SIDE_MIN_OBJS and now >= self.side_quiet_until:
          level = 1
    # OBSLOG: record every change of warning level with the objects behind it
    if level != getattr(self, '_logged', 0):
      self._logged = level
      try:
        with open('/data/obstacle_events.log', 'a') as lf:
          lf.write('%s level=%d v=%.1fkph objs=%s\n' % (time.strftime('%H:%M:%S'), level, v_ego * 3.6,
                   [(round(d, 1), round(y, 1), round(v, 1)) for d, y, v in objs][:6]))
      except Exception:
        pass
    if level >= self.level or now >= self.hold_until:
      if level == 1 and self.level == 0:
        self.side_quiet_until = now + SIDE_COOLDOWN
      self.level = level
    if level > 0:
      self.hold_until = now + HOLD
    return self.level
