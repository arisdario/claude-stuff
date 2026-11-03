"""Safety governor for the experimental traffic detector (advisory only, pure logic, no I/O).

The detector may run ONLY while this says allowed. It fails safe: no limits -> it cannot even be built; a missing sensor value,
a thermal warning, a falling driving-model frame rate, or the rest of the system getting busier than ever seen in the baseline run
all switch the detector off. It never asks for more resources and never touches the driving model: it only ever says "stop".

Limits come from a baseline run with the detector OFF (benchmark.py summary --suggest): nothing here is a guessed number."""
import json

REQUIRED = ('others_cpu_limit_pct', 'own_cpu_limit_core_pct', 'temp_limit_c', 'mem_avail_min_mb')
DEFAULTS = {
  'model_fps_min': 19.0,        # the driving model runs at 20 Hz; the MEAN over fps_window_s below this = stop (single seconds jitter: messages arrive in bunches)
  'fps_window_s': 5.0,
  'model_gaps_max': 3,          # missed model frame ids within the window; more = stop
  'sustain_s': 3.0,             # a limit must be exceeded this long (instant for thermal status / hard temperature)
  'hold_s': 60.0,               # stay off at least this long after a trip
  'recover_s': 30.0,            # then everything must be healthy this long (with the margins) before it comes back
  'warmup_s': 20.0,             # healthy time needed after start
  'cpu_margin_pct': 3.0, 'temp_margin_c': 2.0, 'mem_margin_mb': 50.0,
  'hard_temp_extra_c': 5.0,     # temperature this far over the limit = instant trip
  'max_trips_per_hour': 3,      # more than this: latched off until restart
}


class Decision(object):
  def __init__(self, allowed, reason):
    self.allowed, self.reason = allowed, reason

  def __repr__(self):
    return 'Decision(%s, %r)' % (self.allowed, self.reason)


def load_limits(path):
  with open(path) as f:
    return build_limits(json.load(f))


def build_limits(d):
  missing = [k for k in REQUIRED if d.get(k) is None]
  if missing:
    raise ValueError('limits missing %s: run a baseline benchmark and use benchmark.py summary --suggest' % ', '.join(missing))
  out = dict(DEFAULTS)
  out.update(d)
  return out


class Governor(object):
  def __init__(self, limits):
    self.L = build_limits(limits)
    self.allowed = False
    self.reason = 'warming up'
    self.good_since = None       # start of the current healthy streak
    self.bad_since = None        # start of the current violation streak
    self.hold_until = 0.0
    self.trips = []
    self.latched = False
    self.fps_bad_since = None
    self.fps_hist = []           # (time, model_fps, model_gaps)
    self.last_trip = ''          # reason of the most recent trip

  # ---- individual checks, each returns '' when fine, else the reason
  def _hard(self, s, margin=False):
    L = self.L
    if s.get('thermal_status') not in (None, 'green'):
      return 'thermal status %s' % s.get('thermal_status')
    t = s.get('temp_max_c')
    if t is None:
      return 'no temperature reading'
    if t >= L['temp_limit_c'] + L['hard_temp_extra_c']:
      return 'temperature %.1f C (hard limit)' % t
    return ''

  def _soft(self, s, margin=False):
    L = self.L
    cm = L['cpu_margin_pct'] if margin else 0.0
    tm = L['temp_margin_c'] if margin else 0.0
    mm = L['mem_margin_mb'] if margin else 0.0
    o = s.get('others_cpu_pct')
    if o is None:
      return 'no cpu reading'
    if o > L['others_cpu_limit_pct'] - cm:
      return 'rest of system cpu %.1f%% over %.1f%%' % (o, L['others_cpu_limit_pct'] - cm)
    if s.get('self_cpu_core_pct', 0.0) > L['own_cpu_limit_core_pct'] - (cm if margin else 0.0):
      return 'own cpu %.0f%% of a core over %.0f%%' % (s.get('self_cpu_core_pct', 0.0), L['own_cpu_limit_core_pct'])
    if s['temp_max_c'] > L['temp_limit_c'] - tm:
      return 'temperature %.1f C over %.1f C' % (s['temp_max_c'], L['temp_limit_c'] - tm)
    m = s.get('mem_avail_mb')
    if m is None or m < L['mem_avail_min_mb'] + mm:
      return 'memory available %s MB' % ('?' if m is None else '%.0f' % m)
    return ''

  def _fps(self, s, now):
    f = s.get('model_fps')
    if f is None:
      return 'driving model frame rate unknown'
    L = self.L
    self.fps_hist = [h for h in self.fps_hist if now - h[0] <= L['fps_window_s']] + [(now, f, s.get('model_gaps') or 0)]
    if now - self.fps_hist[0][0] < L['fps_window_s'] - 1.5:          # not enough history yet: do not judge on one jittery second
      return ''
    mean = sum(h[1] for h in self.fps_hist) / len(self.fps_hist)
    gaps = sum(h[2] for h in self.fps_hist)
    if mean < L['model_fps_min']:
      return 'driving model at %.1f fps (mean of %.0f s)' % (mean, L['fps_window_s'])
    if gaps > L['model_gaps_max']:
      return 'driving model missed %d frames in %.0f s' % (gaps, L['fps_window_s'])
    return ''

  def update(self, s, now, switch_on=True, switch_off=False):
    """s: one Sampler sample plus optional keys 'thermal_status' ('green'...), 'model_fps'. -> Decision. Call about once a second."""
    L = self.L
    if switch_off:
      return self._deny(now, 'off switch')
    if not switch_on:
      return self._deny(now, 'not enabled')
    if self.latched:
      return self._deny(now, 'latched off after repeated trips (restart to clear)')
    hard = self._hard(s)
    if hard:
      return self._trip(now, hard)
    fps = self._fps(s, now)
    if fps:                                   # a falling model frame rate is the one thing that must never happen: needs 2 samples in a row
      if self.fps_bad_since is None:
        self.fps_bad_since = now
      if 'unknown' not in fps or now - self.fps_bad_since >= L['sustain_s']:
        return self._trip(now, fps)
      return self._deny(now, 'checking ' + fps, keep_streak=False)
    self.fps_bad_since = None
    soft = self._soft(s)
    if soft:
      if self.bad_since is None:
        self.bad_since = now
      self.good_since = None
      if self.allowed and now - self.bad_since >= L['sustain_s']:
        return self._trip(now, soft)
      if not self.allowed:
        return self._deny(now, soft, keep_streak=False)
      return Decision(True, 'ok (watching: %s)' % soft)
    self.bad_since = None
    if self.allowed:
      return Decision(True, 'ok')
    # not allowed and healthy: count the streak (with margins once a trip happened)
    margin_ok = not self._soft(s, margin=True)
    if self.trips and not margin_ok:
      self.good_since = None
      return self._deny(now, 'recovering (not clear of the limits by the margin)', keep_streak=False)
    if self.good_since is None:
      self.good_since = now
    need = L['recover_s'] if self.trips else L['warmup_s']
    if now < self.hold_until:
      return self._deny(now, 'holding off until %.0f s' % (self.hold_until - now), keep_streak=True)
    if now - self.good_since >= need:
      self.allowed, self.reason = True, 'ok'
      return Decision(True, 'ok')
    return self._deny(now, 'waiting %.0f s of healthy readings' % (need - (now - self.good_since)), keep_streak=True)

  def _deny(self, now, reason, keep_streak=False):
    self.allowed, self.reason = False, reason
    if not keep_streak:
      self.good_since = None
    return Decision(False, reason)

  def _trip(self, now, reason):
    self.trips = [t for t in self.trips if now - t < 3600.0] + [now]
    self.hold_until = now + self.L['hold_s']
    self.bad_since = None
    self.fps_bad_since = None
    self.fps_hist = []
    self.good_since = None
    if len(self.trips) > self.L['max_trips_per_hour']:
      self.latched = True
    self.allowed, self.reason = False, 'tripped: ' + reason
    self.last_trip = reason
    return Decision(False, self.reason)
