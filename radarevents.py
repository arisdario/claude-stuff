#!/usr/bin/env python3
"""
radarevents.py - records driver interventions and close passes, for finding out where openpilot drives too close to
walls, curbs and static objects. Used by radarweb (10 Hz); plain Python, no openpilot imports (tests: tests/test_radarevents.py).

An event is written when, with openpilot engaged:
  steer   you correct the steering (carState.steeringPressed) for STEER_HOLD_S or more
  disengage  openpilot turns off: cause brake / gas / steer / cancel (whatever you did in the last 0.6 s)
  close   the road edge (0.8.3 model, see tesla/model083) or a stationary radar object comes within CLOSE_M of the car's
          side for CLOSE_HOLD_S - recorded even if you don't take over
  mark    you press Mark on /dash ("that was too close" without taking over)

Each event file holds PRE_S seconds before and POST_S after, 10 samples per second: speed, steering angle, your
steering torque, pedals, lane lines and path, road edges, nearby radar objects, the clearance on each side (lane line,
road edge, static object) and GPS, plus the openpilot video segments of that moment (/data/media/0/realdata).
Files: /data/interventions/ev_YYYYmmdd_HHMMSS_<type>.json, the newest MAX_FILES kept.
"""
import json
import os
import time

X_IDXS = [0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75, 22.6875, 27., 31.6875, 36.75,
          42.1875, 48., 54.1875, 60.75, 67.6875, 75., 82.6875, 90.75, 99.1875, 108., 117.1875, 126.75, 136.6875, 147.,
          157.6875, 168.75, 180.1875, 192.]          # 0.8.3 distances of the road-edge points
HALF_W = 0.98            # m, half the Model S body width (mirrors folded in: the radar can't see mirror height anyway)
PRE_S, POST_S = 15.0, 5.0
STEER_HOLD_S = 0.3
STEER_TQ_NM = 1.8        # Nm twist on the wheel (EPAS torsion bar) = a correction. Not carState.steeringPressed: in this
                         # fork that is EPAS_handsOnLevel > 0, i.e. simply hands resting on the wheel
CLOSE_M, CLOSE_HOLD_S = 0.3, 0.5
CLEAR_AHEAD_M = 12.0     # look this far ahead for edges / static objects beside the car's path
OBJ_AHEAD_M = 20.0
MIN_GAP_S = 8.0          # no second event of the same type within this time
NEAR_M = 1.0             # an event's side is named only when edge/object clearance on it was below this
MAX_FILES = 500
EVENT_DIR = '/data/interventions'
SEGMENT_DIR = '/data/media/0/realdata'


def poly_y(p, x):
  return ((p[0] * x + p[1]) * x + p[2]) * x + p[3]


def interp(x, xs, ys):
  if x <= xs[0]: return ys[0]
  if x >= xs[-1]: return ys[-1]
  for i in range(1, len(xs)):
    if x <= xs[i]:
      t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]); return ys[i - 1] + t * (ys[i] - ys[i - 1])
  return ys[-1]


def clearances(st):
  """free space (m) between the car's side and, on each side: the lane line, the road edge, the nearest stationary
  radar object beside the path ahead. None when unknown. y is left-positive."""
  out = {'lane_l': None, 'lane_r': None, 'edge_l': None, 'edge_r': None, 'obj_l': None, 'obj_r': None}
  md = st.get('model') or {}
  l, r = md.get('l'), md.get('r')
  if l and (md.get('pl') or 0) > 0.3: out['lane_l'] = round(poly_y(l, 0.0) - HALF_W, 2)
  if r and (md.get('pr') or 0) > 0.3: out['lane_r'] = round(-poly_y(r, 0.0) - HALF_W, 2)
  e = md.get('e')
  if e and len(e.get('l', [])) == len(X_IDXS):
    xs = [x * 1.0 for x in range(0, int(CLEAR_AHEAD_M) + 1)]
    out['edge_l'] = round(min(interp(x, X_IDXS, e['l']) for x in xs) - HALF_W, 2)
    out['edge_r'] = round(min(-interp(x, X_IDXS, e['r']) for x in xs) - HALF_W, 2)
  p = md.get('p')
  for o in st.get('o') or []:
    if not o.get('stationary') or not (0 < (o.get('d') or 0) < OBJ_AHEAD_M): continue
    y = o.get('y') or 0.0
    if p: y -= poly_y(p, o['d'])          # relative to where the car is heading, so a bend doesn't fake a wall
    if y > 0:
      c = round(y - HALF_W, 2); out['obj_l'] = c if out['obj_l'] is None else min(out['obj_l'], c)
    else:
      c = round(-y - HALF_W, 2); out['obj_r'] = c if out['obj_r'] is None else min(out['obj_r'], c)
  return out


def _min_side(c, side):
  vals = [c[k] for k in ('edge_' + side, 'obj_' + side) if c.get(k) is not None]
  return min(vals) if vals else None


def sample(st, now):
  """one compact 10 Hz snapshot of what matters around an intervention"""
  md = st.get('model') or {}
  drv = st.get('drv') or {}
  g = st.get('gps') or {}
  p, l, r, e = md.get('p'), md.get('l'), md.get('r'), md.get('e')
  tq = drv.get('st')
  s = {'t': round(now, 2), 'v': round(st.get('v') or 0.0, 2), 'ang': st.get('ang'), 'op': bool(st.get('op')),
       'acc': bool(st.get('acc')), 'tq': tq, 'sp': bool(drv.get('sp')), 'br': bool(drv.get('br')),
       'gas': bool(drv.get('gas')), 'hands': drv.get('hands'), 'c': clearances(st)}
  s['steer'] = tq is not None and abs(tq) >= STEER_TQ_NM          # a real correction, not resting hands
  if p: s['path'] = [round(poly_y(p, x), 2) for x in (0, 10, 20, 40)]
  if l: s['lane_l'] = [round(poly_y(l, x), 2) for x in (0, 10, 20, 40)] + [md.get('pl')]
  if r: s['lane_r'] = [round(poly_y(r, x), 2) for x in (0, 10, 20, 40)] + [md.get('pr')]
  if e: s['edge'] = [[round(interp(x, X_IDXS, e['l']), 2) for x in (0, 10, 20, 40)],
                     [round(interp(x, X_IDXS, e['r']), 2) for x in (0, 10, 20, 40)]]
  if g.get('lat') is not None: s['gps'] = [g['lat'], g['lon'], g.get('hdg')]
  # radar objects: out to 200 m (a slow truck matters long before it is close), with track id and closing speed
  objs = [o for o in (st.get('o') or []) if 0 < (o.get('d') or 999) < 200 and abs(o.get('y') or 99) < 6]
  objs.sort(key=lambda o: o['d'])
  s['obj'] = [[o.get('d'), o.get('y'), o.get('vy'), o.get('c'), 1 if o.get('stationary') else 0, o.get('s'), o.get('vr')]
              for o in objs[:16]]
  # what openpilot is doing: its lead (radar-confirmed or camera only), planner target, warnings, actual accel
  s['a'] = None if st.get('a') is None else round(st['a'], 2)
  ld = st.get('lead')
  s['lead'] = [ld.get('d'), ld.get('vr'), 1 if ld.get('radar') else 0] if ld else None
  pl = st.get('plan') or {}
  s['aT'] = pl.get('aT')
  s['fcw'] = bool(pl.get('fcw') or (ld and ld.get('fcw')))
  s['alert'] = (st.get('alert') or '')[:40]
  if p: s['pp'] = [round(x, 7) for x in p]
  return s


IN_PATH_M = 1.6          # m either side of the predicted path counts as "in my lane" for the analysis
SLOW_A = -0.5            # m/s2: "openpilot slowed" = the car decelerating this much without the brake pedal
TRACK_GAP_S = 0.6        # a radar track may drop out this long and still be the same object
LATE_S = 1.5             # openpilot slowing less than this before you acted counts as late


def _in_path(o, s):
  pp = s.get('pp')
  return abs((o[1] or 0.0) - (poly_y(pp, o[0]) if pp else 0.0)) < IN_PATH_M


def _ttc(o):
  return o[0] / -o[6] if (o[6] or 0) < -0.5 else None


def analyze(samples, t0, kind):
  """What happened before you acted. Times are seconds BEFORE t0, None = never:
  radar_saw   the radar first saw the hazard (the in-path object most urgent when you acted)
  op_lead     openpilot's lead matched it (op_lead_radar: radar-confirmed, else camera only)
  op_slowed   the car was decelerating (no brake pedal, engaged) - op_max_decel how hard
  warned      forward collision warning or an openpilot alert
  ttc         seconds to contact with the hazard when you acted"""
  pre = [x for x in samples if x['t'] <= t0 + 0.05]
  out = {'hazard': None, 'radar_saw': None, 'radar_saw_d': None, 'op_lead': None, 'op_lead_radar': None,
         'op_slowed': None, 'op_max_decel': None, 'warned': None, 'ttc': None, 'late': None, 'verdict': ''}
  if not pre: return out
  at = pre[-1]
  cand = [o for o in at.get('obj') or [] if len(o) >= 7 and o[5] is not None and _in_path(o, at)]
  hz = min(cand, key=lambda o: (_ttc(o) if _ttc(o) is not None else 1e9, o[0])) if cand else None
  if hz:
    sid = hz[5]
    out['hazard'] = {'d': hz[0], 'vr': hz[6], 'cls': hz[3], 'stationary': bool(hz[4]), 'id': sid}
    if _ttc(hz) is not None: out['ttc'] = round(_ttc(hz), 1)
    first, last_seen, hd = None, at['t'], {}
    for x in reversed(pre):                                   # radar: back while this track id is present
      m = [o for o in x.get('obj') or [] if len(o) >= 7 and o[5] == sid]
      if m:
        first = last_seen = x['t']; hd[x['t']] = m[0][0]
      elif last_seen - x['t'] > TRACK_GAP_S:
        break
    if first is not None:
      out['radar_saw'] = round(t0 - first, 1); out['radar_saw_d'] = hd[first]
    lf = lr = None
    for x in reversed(pre):                                   # openpilot's lead: back while it matches this object
      d, ld = hd.get(x['t']), x.get('lead')
      if d is not None and ld and ld[0] is not None and abs(ld[0] - d) < max(5.0, 0.1 * d):
        lf, lr = x['t'], ld[2]
      elif lf is not None and x['t'] < t0 - 0.3:
        break
    if lf is not None: out['op_lead'] = round(t0 - lf, 1); out['op_lead_radar'] = bool(lr)
  win = [x for x in pre if t0 - x['t'] <= 10.0]
  start, mx = None, 0.0
  for x in reversed(win):                                     # the latest stretch of openpilot slowing before t0
    a = x.get('a')
    if a is not None and a < SLOW_A and x['op'] and not x['br']:
      start = x['t']; mx = min(mx, a)
    elif start is not None and x['t'] < t0 - 0.3:
      break
  if start is not None: out['op_slowed'] = round(t0 - start, 1); out['op_max_decel'] = round(mx, 2)
  w = [x['t'] for x in win if x.get('fcw') or x.get('alert')]
  if w: out['warned'] = round(t0 - min(w), 1)
  if hz: out['late'] = (out['op_slowed'] is None or out['op_slowed'] < LATE_S) and out['warned'] is None
  out['verdict'] = _verdict(out, kind)
  return out


def _verdict(a, kind):
  if a['hazard'] is None:
    return 'no radar object in your path - a steering / lane-position event' if kind in ('steer', 'close', 'mark') \
      else 'no radar object in your path when you acted'
  p = []
  rs, ol, sl, wn = a['radar_saw'], a['op_lead'], a['op_slowed'], a['warned']
  if rs is not None: p.append('radar saw it %.1f s before you (%s m)' % (rs, a['radar_saw_d']))
  if ol is None: p.append('openpilot never took it as its lead')
  else: p.append('openpilot lead %.1f s before (%s)' % (ol, 'radar' if a['op_lead_radar'] else 'camera only'))
  if sl is None: p.append('openpilot did not slow down')
  elif sl < LATE_S: p.append('openpilot slowed only %.1f s before you' % sl)
  else: p.append('openpilot was slowing %.1f s before (max %.1f m/s2)' % (sl, a['op_max_decel']))
  p.append('no warning' if wn is None else 'warned %.1f s before' % wn)
  if a['ttc'] is not None: p.append('%.1f s to contact when you acted' % a['ttc'])
  return '; '.join(p)


class Recorder:
  def __init__(self, out_dir=EVENT_DIR, segment_dir=SEGMENT_DIR, write=True):
    self.out_dir, self.segment_dir, self.write = out_dir, segment_dir, write
    self.buf = []; self.pending = None; self.prev_op = False
    self.steer_since = None; self.steer_fired = False
    self.close_since = {'l': None, 'r': None}; self.close_fired = {'l': False, 'r': False}
    self.last = {}; self.marks = []

  def mark(self, note=''):
    self.marks.append(str(note)[:120])

  def _trigger(self, kind, cause, side, now):
    if self.pending is not None or now - self.last.get(kind, -1e9) < MIN_GAP_S: return
    self.last[kind] = now
    self.pending = {'type': kind, 'cause': cause, 'side': side, 't0': now, 'pre': list(self.buf), 'post': []}

  def update(self, st, now=None):
    """feed one 10 Hz state (radarweb STATE); returns a finished event dict when one completes, else None"""
    if now is None: now = time.time()
    s = sample(st, now)
    self.buf.append(s)
    while self.buf and now - self.buf[0]['t'] > PRE_S: self.buf.pop(0)
    op = s['op']
    # steering correction while engaged
    if op and s['steer']:
      if self.steer_since is None: self.steer_since = now
      if not self.steer_fired and now - self.steer_since >= STEER_HOLD_S:
        self.steer_fired = True
        self._trigger('steer', 'steer', None, now)     # side: from where it was closest (torque sign not assumed)
    else:
      self.steer_since = None; self.steer_fired = False
    # disengage: what did the driver do in the last 0.6 s
    if self.prev_op and not op:
      recent = [x for x in self.buf if now - x['t'] <= 0.6]
      cause = 'brake' if any(x['br'] for x in recent) else 'gas' if any(x['gas'] for x in recent) else \
              'steer' if any(x.get('steer') for x in recent) else 'cancel'
      if self.pending is not None and self.pending['type'] in ('steer', 'close') and now - self.pending['t0'] < POST_S:
        self.pending['disengaged'] = cause            # same moment: keep one event, note that it also disengaged
      else:
        self._trigger('disengage', cause, None, now)
    self.prev_op = op
    # close pass (engaged, moving)
    for side in ('l', 'r'):
      m = _min_side(s['c'], side)
      if op and s['v'] > 3 and m is not None and m < CLOSE_M:
        if self.close_since[side] is None: self.close_since[side] = now
        if not self.close_fired[side] and now - self.close_since[side] >= CLOSE_HOLD_S:
          self.close_fired[side] = True
          self._trigger('close', 'edge or object %.2f m' % m, 'left' if side == 'l' else 'right', now)
      else:
        self.close_since[side] = None; self.close_fired[side] = False
    # manual mark
    while self.marks:
      self._trigger('mark', self.marks.pop(0) or 'marked', None, now)
    # collect after the trigger, then finish
    if self.pending is not None and self.pending['t0'] != now:
      self.pending['post'].append(s)
    if self.pending is not None and now - self.pending['t0'] >= POST_S:
      ev = self._finish(self.pending); self.pending = None
      return ev
    return None

  def _finish(self, p):
    samples = p['pre'] + p['post']
    t0 = p['t0']
    win = [x for x in samples if t0 - 5.0 <= x['t'] <= t0 + 1.0]          # the moments that matter
    mins = {}
    for k in ('lane_l', 'lane_r', 'edge_l', 'edge_r', 'obj_l', 'obj_r'):
      vals = [x['c'][k] for x in win if x['c'].get(k) is not None]
      mins[k] = min(vals) if vals else None
    near = p['side']
    if near is None:
      left, right = _min_side(mins, 'l'), _min_side(mins, 'r')
      m = min(x for x in (left, right) if x is not None) if (left is not None or right is not None) else None
      if m is not None and m < NEAR_M:                   # name a side only if something was actually close
        near = 'left' if (right is None or (left is not None and left < right)) else 'right'
    at = min(samples, key=lambda x: abs(x['t'] - t0)) if samples else {}
    ev = {'format': 1, 'type': p['type'], 'cause': p['cause'], 'side': near, 't': round(t0, 2),
          'time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t0)), 'kmh': round((at.get('v') or 0) * 3.6, 1),
          'gps': at.get('gps') or next((x['gps'] for x in reversed(samples) if x.get('gps')), None),
          'min_clear': mins, 'video': self._segments(t0), 'samples': samples}
    if p.get('disengaged'): ev['disengaged'] = p['disengaged']
    act = t0                     # when you actually started acting (brake / gas / steering), not when openpilot switched off
    if p['type'] in ('disengage', 'steer'):
      for x in reversed([x for x in samples if x['t'] <= t0 + 0.05]):
        if x['br'] or x['gas'] or x.get('steer'): act = x['t']
        elif x['t'] < t0 - 0.05: break
    ev['acted'] = round(act, 2)
    try: ev['analysis'] = analyze(samples, act, p['type'])
    except Exception as e: ev['analysis'] = {'verdict': 'analysis failed: %s' % e}
    if self.write: self._write(ev)
    return ev

  def _segments(self, t0):
    """openpilot's 1-minute video segments that cover the event (newest dirs around that time)"""
    try:
      ds = [d for d in os.listdir(self.segment_dir) if '--' in d]
      ds = [(os.path.getmtime(os.path.join(self.segment_dir, d)), d) for d in ds]
      return [d for m, d in sorted(ds) if t0 - PRE_S - 70 <= m <= t0 + POST_S + 70][-3:]
    except Exception:
      return []

  def _write(self, ev):
    try:
      os.makedirs(self.out_dir, exist_ok=True)
      name = 'ev_%s_%s.json' % (time.strftime('%Y%m%d_%H%M%S', time.localtime(ev['t'])), ev['type'])
      tmp = os.path.join(self.out_dir, '.' + name)
      with open(tmp, 'w') as f: json.dump(ev, f, separators=(',', ':'))
      os.replace(tmp, os.path.join(self.out_dir, name))
      ev['file'] = name
      files = sorted(x for x in os.listdir(self.out_dir) if x.startswith('ev_') and x.endswith('.json'))
      for old in files[:-MAX_FILES]:
        os.remove(os.path.join(self.out_dir, old))
    except Exception as e:
      print('radarevents: could not write event:', e)


def list_events(out_dir=EVENT_DIR, limit=300):
  """summaries for /events (without the samples)"""
  rows = []
  try: files = sorted((x for x in os.listdir(out_dir) if x.startswith('ev_') and x.endswith('.json')), reverse=True)[:limit]
  except Exception: return rows
  for name in files:
    try:
      with open(os.path.join(out_dir, name)) as f: ev = json.load(f)
      ev.pop('samples', None); ev['file'] = name; rows.append(ev)
    except Exception:
      pass
  return rows


def load_event(name, out_dir=EVENT_DIR):
  if not (name.startswith('ev_') and name.endswith('.json')) or '/' in name or '..' in name: return None
  try:
    with open(os.path.join(out_dir, name)) as f: return json.load(f)
  except Exception:
    return None
