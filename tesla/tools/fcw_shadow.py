#!/usr/bin/env python3
"""fcw_shadow.py - SHADOW MODE for fcw_v2: runs the redesigned FCW next to openpilot's stock FCW while you
drive and LOGS every moment either one would warn. It never alerts, never sends anything, never changes a
setting: it only subscribes to radarState / carState / plan and appends to a log file.

  run (on the EON, openpilot running):
    cd /data/openpilot && nohup nice -n 10 python3 /data/claude-stuff/tesla/tools/fcw_shadow.py > /dev/null 2>&1 &
  stop:    pkill -f fcw_shadow.py
  report:  python3 /data/claude-stuff/tesla/tools/fcw_shadow.py --report [/data/fcw_shadow.jsonl]

Log: /data/fcw_shadow.jsonl, one JSON line per event:
  {"ev": "start"|"end", "who": "stock"|"v2_early"|"v2_urgent"|"model", "t": ..., "v_ego": m/s, "d_rel": m,
   "v_lead": m/s, "y_rel": m, "a_needed_g": g, "blinker": bool}
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fcw_v2  # noqa: E402

LOG = os.environ.get('FCW_SHADOW_LOG', '/data/fcw_shadow.jsonl')
MATCH_WINDOW_S = 3.0   # a stock and a v2 warning within this many seconds are "the same event"


class ShadowCompare:
  """Pure logic (no I/O): feed one frame, get back the start/end events of each warning source."""

  def __init__(self):
    self.v2 = fcw_v2.FCWv2()
    self.state = {'stock': False, 'v2_early': False, 'v2_urgent': False, 'model': False}

  def step(self, t, v_ego, lead_status, d_rel, v_lead, y_rel, blinker, stock_fcw, model_fcw=None):
    """model_fcw: the 0.8.12 model's own hard-brake prediction (comma's meta.hardBrakePredicted, passed through the
    adapter in model.freePath); None when the running model has none (0.8.3) - then 'model' never fires."""
    if lead_status:
      out = self.v2.step(t, v_ego, d_rel, v_lead, y_rel, blinker)
    else:   # no lead: nothing to warn about; keep the persistence filters fed with "no danger"
      out = self.v2.step(t, v_ego, 1e3, v_ego, 0.0, blinker)
    now = {'stock': bool(stock_fcw), 'v2_early': out['early'], 'v2_urgent': out['urgent'], 'model': bool(model_fcw)}
    events = []
    for who in ('stock', 'v2_early', 'v2_urgent', 'model'):
      if now[who] != self.state[who]:
        events.append({'ev': 'start' if now[who] else 'end', 'who': who, 't': round(t, 2),
                       'v_ego': round(v_ego, 2), 'd_rel': round(d_rel, 2) if lead_status else None,
                       'v_lead': round(v_lead, 2) if lead_status else None,
                       'y_rel': round(y_rel, 2) if lead_status else None,
                       'a_needed_g': round(out['a_needed'] / fcw_v2.G, 3), 'blinker': bool(blinker)})
    self.state = now
    return events


def report(rows):
  """Pairs warning starts: stock-only, v2-only, both (and which came first)."""
  starts = [r for r in rows if r.get('ev') == 'start' and r.get('who') in ('stock', 'v2_early', 'v2_urgent', 'model')]
  stock = [r for r in starts if r['who'] == 'stock']
  v2 = [r for r in starts if r['who'] == 'v2_early']
  urgent = [r for r in starts if r['who'] == 'v2_urgent']
  both, stock_only, used = [], [], set()
  for s in stock:
    m = next((i for i, v in enumerate(v2) if i not in used and abs(v['t'] - s['t']) <= MATCH_WINDOW_S), None)
    if m is None:
      stock_only.append(s)
    else:
      used.add(m); both.append((s, v2[m]))
  v2_only = [v for i, v in enumerate(v2) if i not in used]
  lines = ['FCW shadow report: %d stock warnings, %d v2 EARLY, %d v2 URGENT' % (len(stock), len(v2), len(urgent)),
           '  both fired:  %d' % len(both)]
  for s, v in both:
    lines.append('    t=%.1f  v2 %s stock by %.2f s  (gap %.1f m, v %.0f km/h, needed %.2f g)' % (
        s['t'], 'BEFORE' if v['t'] < s['t'] else 'after', abs(v['t'] - s['t']), v['d_rel'] or -1,
        v['v_ego'] * 3.6, v['a_needed_g']))
  lines.append('  stock only:  %d   (v2 missed these - check them)' % len(stock_only))
  for s in stock_only:
    lines.append('    t=%.1f  gap %s m, v %.0f km/h' % (s['t'], s['d_rel'], s['v_ego'] * 3.6))
  lines.append('  v2 only:     %d   (v2 extra warnings: real danger stock missed, or false alarms?)' % len(v2_only))
  for v in v2_only:
    lines.append('    t=%.1f  gap %.1f m, lead %.1f km/h, v %.0f km/h, needed %.2f g, y %.1f m%s' % (
        v['t'], v['d_rel'] or -1, (v['v_lead'] or 0) * 3.6, v['v_ego'] * 3.6, v['a_needed_g'], v['y_rel'] or 0,
        ', blinker' if v['blinker'] else ''))
  model = [r for r in starts if r['who'] == 'model']
  lines.append('  model FCW (0.8.12 model hard-brake prediction): %d' % len(model))
  for m in model:
    s = next((x for x in stock if abs(x['t'] - m['t']) <= MATCH_WINDOW_S), None)
    lines.append('    t=%.1f  %s  (gap %s m, v %.0f km/h)' % (m['t'], ('with stock, %s by %.2f s' % ('BEFORE' if m['t'] < s['t'] else 'after',
                 abs(m['t'] - s['t']))) if s else 'model only', m['d_rel'], m['v_ego'] * 3.6))
  return '\n'.join(lines)


def model_fcw_from(model, brake_pressed):
  """the 0.8.12 adapter appends [hard brake predicted 1/0 or -1, P(5 m/s2), P(3 m/s2)] after the 69 road-edge values
  in model.freePath (tesla/model083/model0812_adapter.h model_fcw). Like comma's controlsd: not while braking."""
  try:
    fp = list(model.freePath)
  except Exception:
    return None
  if len(fp) < 72 or fp[0] != 1.0 or fp[69] < 0:
    return None
  return fp[69] == 1.0 and not brake_pressed


def _err(msg):
  sys.stderr.write('%s fcw_shadow: %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)); sys.stderr.flush()


def connect(messaging, names, retry_s=5.0):
  """openpilot's message queue allows a limited number of readers per message; when one is full, connecting raises
  MessagingError. Wait and retry instead of dying (param_switch only starts the logger when the toggle changes)."""
  while True:
    try:
      return messaging.SubMaster(names)
    except Exception as e:
      _err('cannot subscribe to %s yet (%r) - retrying in %.0f s' % (names, e, retry_s))
      time.sleep(retry_s)


def run():
  import cereal.messaging as messaging
  sm = connect(messaging, ['radarState', 'carState', 'plan'])
  model_sock, model_next_try, last_model = None, 0.0, None   # 'model' is the busiest message: optional, retried
  sc = ShadowCompare()
  with open(LOG, 'a') as f:
    f.write(json.dumps({'ev': 'session', 't': round(time.time(), 2)}) + '\n'); f.flush()
    while True:
      if model_sock is None and time.time() >= model_next_try:
        try:
          model_sock = messaging.sub_sock('model', conflate=True)
        except Exception as e:
          _err('model FCW unavailable for now (%r) - stock FCW and fcw_v2 still logged; retry in 30 s' % (e,))
          model_next_try = time.time() + 30.0
      if model_sock is not None:
        m = messaging.recv_one_or_none(model_sock)
        if m is not None:
          last_model = m.model
      sm.update(1000)
      if not sm.updated['radarState']:
        continue
      l1, cs = sm['radarState'].leadOne, sm['carState']
      mf = model_fcw_from(last_model, cs.brakePressed) if last_model is not None else None
      events = sc.step(time.time(), cs.vEgo, l1.status, l1.dRel, l1.vLead, l1.yRel,
                       cs.leftBlinker or cs.rightBlinker, sm['plan'].fcw, mf)
      for e in events:
        f.write(json.dumps(e) + '\n')
      if events:
        f.flush()


if __name__ == '__main__':
  if '--report' in sys.argv:
    args = [a for a in sys.argv[1:] if a != '--report']
    path = args[0] if args else LOG
    with open(path) as fh:
      print(report([json.loads(l) for l in fh if l.strip()]))
  else:
    run()
