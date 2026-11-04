#!/usr/bin/env python3
"""Offline analysis of a steering-diagnostics session (run by stop_diagnostics.sh, or on a PC):

  python3 analyze.py all    SESSION_DIR    # summary.txt + plots/ + root_cause_report.txt
  python3 analyze.py summary|plots|report SESSION_DIR

stdlib only (no numpy/matplotlib needed). Plots are SVG; plots/index.html shows them all.
"""
import collections
import glob
import gzip
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import svgplot  # noqa: E402

GROUPS = ('camera', 'model', 'pose', 'calibration', 'can', 'localization', 'steering', 'performance')
HZ = 20.0


def jl(path):
  op = gzip.open if path.endswith('.gz') else open
  out = []
  try:
    with op(path, 'rt') as f:
      for line in f:
        try:
          out.append(json.loads(line))
        except ValueError:
          pass
  except (OSError, IOError):
    pass
  return out


def load_trace(root):
  recs = []
  for g in GROUPS:
    recs += jl(os.path.join(root, g, 'trace.jsonl'))
  recs.sort(key=lambda r: r.get('rt', 0))
  return recs


def get(recs, svc, fn, addr=None):
  t, v = [], []
  for r in recs:
    if r.get('s') != svc or (addr is not None and r.get('addr') != addr):
      continue
    try:
      x = fn(r)
    except (TypeError, KeyError, IndexError):
      continue
    if x is not None:
      t.append(r['rt']); v.append(x)
  return t, v


# signal name -> (service, extractor, addr, unit)
SIGNALS = collections.OrderedDict([
  ('model.c0 (path offset, m)', ('model', lambda r: r['path'][3], None)),
  ('model.heading c1 (rad)', ('model', lambda r: r['path'][2], None)),
  ('model.curvature 2*c2 (1/m)', ('model', lambda r: 2 * r['path'][1], None)),
  ('pose.yaw rate (rad/s)', ('cameraOdometry', lambda r: r['rot'][2], None)),
  ('pose.pitch rate (rad/s)', ('cameraOdometry', lambda r: r['rot'][1], None)),
  ('pathPlan.desired angle (deg)', ('pathPlan', lambda r: r['des'], None)),
  ('controls.desired angle (deg)', ('controlsState', lambda r: r['des'], None)),
  ('controls.actual angle (deg)', ('controlsState', lambda r: r['act'], None)),
  ('CAN.EPAS angle (deg, openpilot sign)', ('can_raw', lambda r: -r['sig']['EPAS_internalSAS'], 0x370)),
  ('CAN.angle request 0x488 (deg)', ('can_raw', lambda r: r['sig']['DAS_steeringAngleRequest'], 0x488)),
  ('CAN.torsion bar torque (Nm)', ('can_raw', lambda r: r['sig']['EPAS_torsionBarTorque'], 0x370)),
  ('car.measured torque', ('carState', lambda r: r['tq'], None)),
  ('car.yaw rate carState (rad/s)', ('carState', lambda r: r['yaw'], None)),
  ('loc.yaw rate LLK (rad/s)', ('liveLocationKalman', lambda r: r['yawrate'][2], None)),
  ('IMU.gyro z (rad/s)', ('sensorEvents', lambda r: r['gyro'][2], None)),
  ('IMU.accel y (m/s2)', ('sensorEvents', lambda r: r['accel'][1], None)),
])


def grid(t, v, t0, t1, hz=HZ):
  """step-hold resample onto a uniform grid; returns (grid_t, values) (None where no sample yet)."""
  n = int((t1 - t0) * hz)
  out, j = [], 0
  for i in range(n):
    tt = t0 + i / hz
    while j + 1 < len(t) and t[j + 1] <= tt:
      j += 1
    out.append(v[j] if t and t[j] <= tt else None)
  return [t0 + i / hz for i in range(n)], out


def highpass(x, win_s=3.0, hz=HZ):
  k = max(1, int(win_s * hz / 2))
  out = []
  for i in range(len(x)):
    seg = [a for a in x[max(0, i - k):i + k + 1] if a is not None]
    out.append(None if x[i] is None or not seg else x[i] - sum(seg) / len(seg))
  return out


def _clean(x):
  return [a for a in x if a is not None]


def amp_at(t, x, f):
  xs = _clean(x)
  if len(xs) < 10:
    return 0.0
  m = sum(xs) / len(xs)
  re = sum((a - m) * math.cos(2 * math.pi * f * tt) for tt, a in zip(t, x) if a is not None)
  im = sum((a - m) * math.sin(2 * math.pi * f * tt) for tt, a in zip(t, x) if a is not None)
  return 2 * math.hypot(re, im) / len(xs)


def dominant(t, x, fmin=0.15, fmax=3.0):
  best, bf = 0.0, None
  f = fmin
  while f <= fmax:
    a = amp_at(t, x, f)
    if a > best:
      best, bf = a, f
    f += 0.05
  return bf, best


def snr_at(t, x, f):
  a = amp_at(t, x, f)
  nb = [amp_at(t, x, f + d) for d in (-0.6, -0.45, -0.3, 0.3, 0.45, 0.6, 0.9, 1.2) if f + d > 0.05]
  nb.sort()
  base = nb[len(nb) // 2] if nb else 0.0
  return a, (a / base if base > 1e-12 else (99.0 if a > 1e-9 else 0.0))


def xcorr_lag(a, b, max_lag_s=1.0, hz=HZ):
  """lag (s) at which b best matches a (positive: b LAGS a), and the correlation there."""
  pairs = [(x, y) for x, y in zip(a, b)]
  best = (0.0, 0.0)
  K = int(max_lag_s * hz)
  for k in range(-K, K + 1):
    xs, ys = [], []
    for i in range(len(a)):
      j = i + k
      if 0 <= j < len(b) and a[i] is not None and b[j] is not None:
        xs.append(a[i]); ys.append(b[j])
    if len(xs) < 20:
      continue
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs)); sy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if sx < 1e-12 or sy < 1e-12:
      continue
    c = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / (sx * sy)
    if abs(c) > abs(best[1]):
      best = (k / hz, c)
  return best


def onset(t, hx, base_end, floor):
  """first time after base_end where |hp| exceeds max(5*sigma_baseline, floor) for 3 samples in a row."""
  base = [abs(a) for tt, a in zip(t, hx) if a is not None and tt < base_end]
  if len(base) < 20:
    return None
  sigma = math.sqrt(sum(a * a for a in base) / len(base))
  thr = max(5 * sigma, floor)
  run = 0
  for tt, a in zip(t, hx):
    if tt < base_end or a is None:
      continue
    run = run + 1 if abs(a) > thr else 0
    if run >= 3:
      return tt - 2 / HZ
  return None


FLOORS = {'m': 0.05, 'rad': 0.005, 'deg': 1.0, 'Nm': 0.2, '1/m': 0.002, 'torque': 0.3, 'm/s2': 0.2}


def _floor(name):
  for k, v in FLOORS.items():
    if k in name:
      return v
  return 0.05


def analyze_event(ev_dir):
  summ = json.load(open(os.path.join(ev_dir, 'summary.json')))
  recs = jl(os.path.join(ev_dir, 'telemetry_full.jsonl.gz'))
  trig = summ['trigger_rt']
  w0, w1 = summ['window_rt']
  res = {'summary': summ, 'signals': {}, 'n_records': len(recs)}
  grids, hps = {}, {}
  for name, (svc, fn, addr) in SIGNALS.items():
    t, v = get(recs, svc, fn, addr)
    if len(t) < 20:
      continue
    gt, gv = grid(t, v, w0, w1)
    grids[name], hps[name] = gv, highpass(gv)
    res['signals'][name] = {'n': len(t), 'rate_hz': round((len(t) - 1) / max(1e-6, t[-1] - t[0]), 1)}
  gt = [w0 + i / HZ for i in range(int((w1 - w0) * HZ))]
  res['grid_t'] = gt
  base_end = w0 + min(15.0, 0.5 * (trig - w0))
  order = []
  for name in grids:
    o = onset(gt, hps[name], base_end, _floor(name))
    res['signals'][name]['onset_rt'] = o
    if o is not None:
      order.append((o - trig, name))
  res['onset_order'] = sorted(order)
  # oscillation frequency from the steering signals, analysed in a window around the trigger
  kinds = summ['kinds']
  osc = [r for r in summ['reasons'] if r['kind'] in ('steering_oscillation', 'steering_swing')]
  f_star = None
  if osc:
    f_star = osc[0]['detail'].get('freq_hz')
    ta = osc[0]['t']
  else:
    ta = trig
  sel = [i for i, tt in enumerate(gt) if ta - 10 <= tt <= ta + 15]
  tt = [gt[i] for i in sel]
  if f_star is None:
    for name in ('controls.actual angle (deg)', 'CAN.EPAS angle (deg, openpilot sign)'):
      if name in hps:
        f_star, _ = dominant(tt, [hps[name][i] for i in sel])
        break
  res['f_star'] = f_star
  if f_star:
    res['at_f'] = {}
    for name in grids:
      a, s = snr_at(tt, [hps[name][i] for i in sel], f_star)
      res['at_f'][name] = {'amp': round(a, 5), 'snr': round(s, 1)}
  lags = {}
  def L(a, b):
    if a in hps and b in hps:
      lg, c = xcorr_lag([hps[a][i] for i in sel], [hps[b][i] for i in sel])
      lags['%s -> %s' % (a, b)] = {'lag_s': round(lg, 2) if abs(c) >= 0.3 else None, 'corr': round(c, 2)}
  L('model.c0 (path offset, m)', 'controls.desired angle (deg)')
  L('model.curvature 2*c2 (1/m)', 'controls.desired angle (deg)')
  L('pose.yaw rate (rad/s)', 'model.heading c1 (rad)')
  L('controls.desired angle (deg)', 'controls.actual angle (deg)')
  L('controls.desired angle (deg)', 'CAN.EPAS angle (deg, openpilot sign)')
  L('CAN.angle request 0x488 (deg)', 'CAN.EPAS angle (deg, openpilot sign)')
  L('controls.actual angle (deg)', 'loc.yaw rate LLK (rad/s)')
  L('controls.actual angle (deg)', 'IMU.gyro z (rad/s)')
  L('controls.actual angle (deg)', 'model.heading c1 (rad)')
  L('controls.actual angle (deg)', 'model.c0 (path offset, m)')
  res['lags'] = lags
  # timing health inside the window
  anomalies = [r for r in summ['reasons']]
  res['timing_kinds'] = collections.Counter(r['kind'] for r in anomalies if r['kind'].startswith(('camera_', 'model_', 'can_', 'ipc_', 'control_loop', 'pose_')))
  res['verdict'] = verdict(res)
  res['grids'] = grids
  return res


def verdict(res):
  """rule-based reading of one oscillation event; returns (label, [evidence lines])."""
  ev = []
  at = res.get('at_f')
  if not res.get('f_star') or not at:
    return ('inconclusive', ['no steering oscillation frequency could be established in this event'])
  f = res['f_star']
  ev.append('oscillation frequency f* = %.2f Hz (period %.1f s)' % (f, 1 / f))
  path_names = [n for n in at if n.startswith(('model.c0', 'model.heading', 'model.curv'))]
  pose_names = [n for n in at if n.startswith('pose.')]
  steer_names = [n for n in at if n.startswith(('controls.', 'CAN.EPAS', 'CAN.angle'))]
  def best(names):
    return max(((at[n]['snr'], n) for n in names), default=(0, None))
  ps, pn = best(path_names)
  qs, qn = best(pose_names)
  ss, sn = best(steer_names)
  ev.append('SNR at f*: steering %.1f (%s), model path %.1f (%s), pose %.1f (%s)' % (ss, sn, ps, pn, qs, qn))
  on = dict((n, t) for t, n in res['onset_order'])
  first = res['onset_order'][0] if res['onset_order'] else None
  if first:
    ev.append('first signal to leave its baseline: %s at %+.2f s relative to the trigger' % (first[1], first[0]))
  for k, v in res['lags'].items():
    ev.append('lag %s: %s' % (k, ('%+.2f s (corr %.2f)' % (v['lag_s'], v['corr'])) if v['lag_s'] is not None else 'no usable correlation (corr %.2f)' % v['corr']))
  if ss < 3:
    return ('inconclusive', ev + ['steering signals do not show a clear oscillation at f*'])
  if ps < 3 and qs < 3:
    ev.append('model path and pose are NOT oscillating at f* while steering is -> perception/pose is not the source')
    d_a = res['lags'].get('controls.desired angle (deg) -> controls.actual angle (deg)')
    if d_a and d_a['lag_s'] is not None and abs(d_a['corr']) > 0.5 and d_a['lag_s'] > 0.4:
      return ('downstream: actuator/vehicle response lag', ev + ['actual follows desired only after %.2f s' % d_a['lag_s']])
    return ('downstream: controller/vehicle loop (gain, parameters or physical response)', ev)
  # path oscillates too: who is first?
  des_on = on.get('controls.desired angle (deg)') or on.get('pathPlan.desired angle (deg)')
  path_on = min([on[n] for n in path_names if n in on] + [1e9])
  if des_on is not None and path_on < des_on - 0.2:
    return ('perception/pose candidate: model path changed before the controller (check camera/pose evidence)', ev)
  if des_on is not None and path_on > des_on + 0.1:
    return ('closed loop: the path oscillates because the car is steering (controller started first)', ev)
  return ('ambiguous: path and controller change together (cannot separate perception from controller)', ev)


# ---------------------------------------------------------------- plots
def event_plots(ev_dir, out_dir):
  res = analyze_event(ev_dir)
  recs = jl(os.path.join(ev_dir, 'telemetry_full.jsonl.gz'))
  summ = res['summary']
  trig = summ['trigger_rt']
  w0 = summ['window_rt'][0]
  rel = lambda t: [x - trig for x in t]
  def S(label, svc, fn, addr=None):
    t, v = get(recs, svc, fn, addr)
    return (label, rel(t), v)
  def D(label, key, fn=None):
    t, v = [], []
    for r in recs:
      if r.get('s') == 'diag' and key in r:
        try:
          x = fn(r[key]) if fn else r[key]
        except (KeyError, TypeError):
          continue
        if x is not None:
          t.append(r['rt']); v.append(x)
    return (label, rel(t), v)
  def P(label, fn):
    t, v = [], []
    for r in recs:
      if r.get('s') == 'perf':
        try:
          x = fn(r)
        except (KeyError, TypeError):
          continue
        if x is not None:
          t.append(r['rt']); v.append(x)
    return (label, rel(t), v)
  panels = [
    {'ylabel': 'steering deg', 'series': [S('desired', 'controlsState', lambda r: r['des']), S('actual', 'controlsState', lambda r: r['act']),
                                            S('CAN EPAS', 'can_raw', lambda r: -r['sig']['EPAS_internalSAS'], 0x370)]},
    {'ylabel': 'torque', 'series': [S('measured (carState)', 'carState', lambda r: r['tq']),
                                     S('torsion bar Nm (CAN)', 'can_raw', lambda r: r['sig']['EPAS_torsionBarTorque'], 0x370),
                                     S('cmd angle 0x488', 'can_raw', lambda r: r['sig']['DAS_steeringAngleRequest'], 0x488)]},
    {'ylabel': 'model path', 'series': [S('c0 offset m', 'model', lambda r: r['path'][3]), S('heading c1', 'model', lambda r: r['path'][2]),
                                          S('curvature 2c2 x10', 'model', lambda r: 20 * r['path'][1])]},
    {'ylabel': 'yaw / lateral', 'series': [S('yaw LLK', 'liveLocationKalman', lambda r: r['yawrate'][2]),
                                            S('yaw car', 'carState', lambda r: r['yaw']), S('gyro z', 'sensorEvents', lambda r: r['gyro'][2]),
                                            S('accel y', 'sensorEvents', lambda r: r['accel'][1])]},
    {'ylabel': 'pose rate rad/s', 'series': [S('roll', 'cameraOdometry', lambda r: r['rot'][0]), S('pitch', 'cameraOdometry', lambda r: r['rot'][1]),
                                              S('yaw', 'cameraOdometry', lambda r: r['rot'][2])]},
    {'ylabel': 'camera interval ms', 'series': [D('frame max', 'iv', lambda d: d['frame']['max_ms']), D('frame mean', 'iv', lambda d: d['frame']['mean_ms']),
                                                 D('model max', 'iv', lambda d: d['model']['max_ms'])]},
    {'ylabel': 'CAN max gap ms', 'series': [D('0x370 EPAS', 'can_gap_max_ms', lambda d: d['0x370']), D('0xe STW', 'can_gap_max_ms', lambda d: d['0xe']),
                                             D('0x118', 'can_gap_max_ms', lambda d: d['0x118'])]},
    {'ylabel': 'CPU % / temp C', 'series': [P('cpu total %', lambda r: r['cpu_total_pct']), P('temp max C', lambda r: r['temp_max_c']),
                                             P('mem avail /100MB', lambda r: r['mem_avail_mb'] / 100.0)]},
    {'ylabel': 'latency ms', 'series': [D('model e2e max', 'iv', lambda d: d['model_lat']['max_ms']), D('model exec max', 'iv', lambda d: d['model_exec']['max_ms']),
                                         D('ctl ipc max', 'iv', lambda d: d['ipc_controlsState']['max_ms'])]},
  ]
  vl = [(0.0, 'trigger')] + [(r['t'] - trig, r['kind']) for r in summ['reasons'][1:8] if isinstance(r, dict) and 't' in r]
  name = os.path.basename(ev_dir)
  svgplot.figure(os.path.join(out_dir, name + '.svg'), '%s   kinds=%s' % (name, ','.join(summ['kinds'])), panels, vl)
  return res


def session_plots(root, out_dir, recs):
  if not recs:
    return []
  t0 = recs[0]['rt']
  rel = lambda t: [x - t0 for x in t]
  def S(label, svc, fn, addr=None):
    t, v = get(recs, svc, fn, addr)
    return (label, rel(t), v)
  def P(label, fn, svc='perf'):
    t, v = get(recs, svc, fn)
    return (label, rel(t), v)
  anomalies = jl(os.path.join(root, 'events', 'anomalies.jsonl'))
  vl = [(a['rt'] - t0, a['kind']) for a in anomalies[:25]]
  panels = [
    {'ylabel': 'steering deg', 'series': [S('desired', 'controlsState', lambda r: r['des']), S('actual', 'controlsState', lambda r: r['act'])]},
    {'ylabel': 'model path', 'series': [S('c0 m', 'model', lambda r: r['path'][3]), S('heading c1', 'model', lambda r: r['path'][2])]},
    {'ylabel': 'pose / yaw rad/s', 'series': [S('pose yaw', 'cameraOdometry', lambda r: r['rot'][2]), S('LLK yaw', 'liveLocationKalman', lambda r: r['yawrate'][2])]},
    {'ylabel': 'camera/model ms', 'series': [P('frame max', lambda r: r['iv']['frame']['max_ms'], 'diag'), P('model max', lambda r: r['iv']['model']['max_ms'], 'diag'),
                                              P('model e2e max', lambda r: r['iv']['model_lat']['max_ms'], 'diag')]},
    {'ylabel': 'CPU % / temp C', 'series': [P('cpu', lambda r: r['cpu_total_pct']), P('temp', lambda r: r['temp_max_c']), P('logger cpu %core', lambda r: r['self_cpu_core'], 'diag')]},
    {'ylabel': 'learned params', 'series': [S('steerRatio', 'liveParameters', lambda r: r['sr']), S('stiffness x10', 'liveParameters', lambda r: 10 * r['sf'])]},
  ]
  p = os.path.join(out_dir, 'session_overview.svg')
  svgplot.figure(p, 'Session overview (10 Hz trace) - red dashed = anomalies', panels, vl)
  return [p]


# ---------------------------------------------------------------- summary / report
def pct(x, p):
  s = sorted(x)
  return s[min(len(s) - 1, int(p * (len(s) - 1)))] if s else None


def write_summary(root, recs):
  L = []
  A = L.append
  A('STEERING DIAGNOSTICS SUMMARY   %s' % os.path.basename(os.path.abspath(root)))
  sysj = {}
  try:
    sysj = json.load(open(os.path.join(root, 'metadata', 'system.json')))
    A('created %s | python %s | %s' % (sysj.get('created_wall'), sysj.get('python'), sysj.get('uname', '')[:80]))
  except Exception:
    A('metadata/system.json missing')
  if recs:
    A('trace span: %.1f s of boot-clock time, %d records' % (recs[-1]['rt'] - recs[0]['rt'], len(recs)))
  cnt = collections.Counter(r['s'] for r in recs)
  A('records per service in the decimated trace: ' + ', '.join('%s=%d' % kv for kv in sorted(cnt.items())))
  diag = [r for r in recs if r['s'] == 'diag']
  if diag:
    A('')
    A('LOGGER OVERHEAD / HEALTH')
    A('  logger CPU (%% of one core): median %.2f  p95 %.2f  max %.2f' % tuple(pct([d['self_cpu_core'] for d in diag if d.get('self_cpu_core') is not None], q) or 0 for q in (.5, .95, 1.0)))
    A('  write queue max %d | dropped records %d | write errors %d | disk write p95 %s ms | fsync probe max %s ms' % (
      max(d['queue_max'] for d in diag), diag[-1]['dropped'], diag[-1]['write_errors'],
      pct([d['write_ms_p95'] for d in diag if d.get('write_ms_p95') is not None], .95),
      max([d['fsync_ms'] for d in diag if d.get('fsync_ms') is not None] or [None])))
    A('  free disk at end: %s MB (state %s)' % (diag[-1]['free_mb'], diag[-1]['disk_state']))
    A('')
    A('TIMING (per-second stats, worst seconds)')
    for k, lab in (('frame', 'camera frame interval'), ('model', 'model output interval'), ('controlsState', 'controlsd cycle'),
                   ('cameraOdometry', 'pose interval'), ('model_lat', 'model end-to-end latency (capture->output)'),
                   ('model_exec', 'model execution'), ('frame_age', 'camera frame age at receive'),
                   ('ipc_controlsState', 'IPC delay controlsState'), ('ipc_model', 'IPC delay model')):
      mx = [d['iv'][k]['max_ms'] for d in diag if k in d.get('iv', {})]
      mn = [d['iv'][k]['mean_ms'] for d in diag if k in d.get('iv', {})]
      if mx:
        A('  %-42s mean %.1f ms | p99 of per-second max %.1f ms | worst %.1f ms' % (lab, sum(mn) / len(mn), pct(mx, .99), max(mx)))
    ch = collections.defaultdict(list)
    for d in diag:
      for a, g in d.get('can_gap_max_ms', {}).items():
        ch[a].append(g)
    for a, g in sorted(ch.items()):
      hz = [d['can_hz'].get(a) for d in diag if d.get('can_hz', {}).get(a) is not None]
      A('  CAN %-6s rate median %s Hz | longest gap %.0f ms' % (a, pct(hz, .5), max(g)))
  perf = [r for r in recs if r['s'] == 'perf']
  if perf:
    A('')
    A('SYSTEM')
    A('  CPU total %%: median %.0f p95 %.0f max %.0f | max temp C: median %s max %s | mem avail MB min %s' % (
      pct([p['cpu_total_pct'] for p in perf if p.get('cpu_total_pct') is not None], .5) or 0, pct([p['cpu_total_pct'] for p in perf if p.get('cpu_total_pct') is not None], .95) or 0,
      max([p['cpu_total_pct'] for p in perf if p.get('cpu_total_pct') is not None] or [0]),
      pct([p['temp_max_c'] for p in perf if p.get('temp_max_c') is not None], .5), max([p['temp_max_c'] for p in perf if p.get('temp_max_c') is not None] or [None]),
      min([p['mem_avail_mb'] for p in perf if p.get('mem_avail_mb') is not None] or [None])))
    run_delay = [(p['sched'][k].get('run_delay_ms_per_s'), k) for p in perf for k in p.get('sched', {}) if p['sched'][k].get('run_delay_ms_per_s') is not None]
    if run_delay:
      A('  worst scheduler run-queue delay: %.1f ms/s in %s' % max(run_delay))
  usb = [r for r in recs if r['s'] == 'usb']
  if usb and not any(u.get('present') for u in usb):
    A('  C920 NOT FOUND on USB in any sample (sysfs vid 046d pid 082d)')
  elif usb:
    speeds = collections.Counter((u.get('c920') or {}).get('speed_mbps') for u in usb)
    A('  C920 USB speed seen (Mbit/s): %s | present in %d/%d samples' % (dict(speeds), sum(1 for u in usb if u.get('present')), len(usb)))
  cq = [r for r in recs if r['s'] == 'cam_quality']
  if cq:
    A('  camera image: brightness mean %.0f (min %.0f), contrast %.0f, sharpness(laplacian var) median %.0f min %.0f, overexposed max %.1f%%, underexposed max %.1f%%' % (
      sum(c['mean'] for c in cq) / len(cq), min(c['mean'] for c in cq), sum(c['contrast'] for c in cq) / len(cq), pct([c['sharp'] for c in cq], .5),
      min(c['sharp'] for c in cq), 100 * max(c['over_frac'] for c in cq), 100 * max(c['under_frac'] for c in cq)))
  anomalies = jl(os.path.join(root, 'events', 'anomalies.jsonl'))
  A('')
  A('ANOMALIES: %d triggers' % len(anomalies))
  for k, n in collections.Counter(a['kind'] for a in anomalies).most_common():
    A('  %-34s %d' % (k, n))
  evs = sorted(glob.glob(os.path.join(root, 'events', 'event_*')))
  A('EVENT RECORDINGS: %d' % len(evs))
  for e in evs:
    try:
      s = json.load(open(os.path.join(e, 'summary.json')))
      A('  %s: %s | %d records, %d frames | window %.0f s' % (s['event'], ','.join(s['kinds']), s.get('records', 0), s.get('frames', 0), s['window_rt'][1] - s['window_rt'][0]))
    except Exception:
      A('  %s: summary missing' % os.path.basename(e))
  for name in ('param_changes.jsonl',):
    ch = jl(os.path.join(root, 'localization', name))
    if ch:
      A('')
      A('LEARNED PARAMETER CHANGES (%d)  first/last:' % len(ch))
      for c in (ch[0], ch[-1]):
        A('  rt %.1f %s valid=%s' % (c['rt'], c['change'], c.get('valid')))
  cc = jl(os.path.join(root, 'calibration', 'changes.jsonl'))
  if cc:
    A('')
    A('CALIBRATION CHANGES (%d), last: %s' % (len(cc), {k: cc[-1].get(k) for k in ('event', 'rpy', 'status', 'perc', 'vp_px', 'pitch_deg', 'yaw_deg', 'roll_deg')}))
  text = '\n'.join(L) + '\n'
  open(os.path.join(root, 'summary.txt'), 'w').write(text)
  return text


def write_report(root, results):
  L = []
  A = L.append
  A('ROOT-CAUSE REPORT (automatic, rule-based)  %s' % os.path.basename(os.path.abspath(root)))
  A('Question: where does the left-right oscillation start?  camera -> model -> pose/path -> localization -> controller -> CAN -> physical steering')
  A('This report is evidence, not a verdict by itself: read the numbers. It never proposes parameter changes.')
  osc = [r for r in results if {'steering_oscillation', 'steering_swing'} & set(r['summary']['kinds'])]
  if not osc:
    A('')
    A('No steering_oscillation event was recorded in this session. Other events: %s' % ', '.join(sorted({k for r in results for k in r['summary']['kinds']}) or ['none']))
  for r in osc:
    s = r['summary']
    label, ev = r['verdict']
    A('')
    A('=' * 78)
    A('%s   trigger at boot-clock %.1f s' % (s['event'], s['trigger_rt']))
    A('VERDICT: %s' % label)
    for e in ev:
      A('  - ' + e)
    if r['onset_order']:
      A('  onset order (s relative to trigger):')
      for t, n in r['onset_order']:
        A('      %+7.2f  %s' % (t, n))
    if r.get('at_f'):
      A('  amplitude / SNR at f* per signal:')
      for n, v in r['at_f'].items():
        A('      %-40s amp %-10g snr %s' % (n, v['amp'], v['snr']))
    if r['timing_kinds']:
      A('  timing anomalies inside the event window: %s' % dict(r['timing_kinds']))
  A('')
  A('HOW TO READ: SNR = amplitude at the oscillation frequency relative to neighbouring frequencies in the same window (>4 = a real peak).')
  A('Closed-loop caveat: once the car steers, the model path moves because the car moved; the ORDER of onset and the lag table separate cause from effect.')
  text = '\n'.join(L) + '\n'
  open(os.path.join(root, 'root_cause_report.txt'), 'w').write(text)
  return text


def write_index(out_dir):
  figs = sorted(f for f in os.listdir(out_dir) if f.endswith('.svg'))
  with open(os.path.join(out_dir, 'index.html'), 'w') as f:
    f.write('<!doctype html><meta charset=utf-8><title>diagnostic plots</title><body style="font-family:sans-serif">')
    for n in figs:
      f.write('<h3>%s</h3><img src="%s" style="max-width:100%%">' % (n, n))


def run_all(root, do_plots=True):
  root = os.path.abspath(root)
  recs = load_trace(root)
  out_dir = os.path.join(root, 'plots')
  os.makedirs(out_dir, exist_ok=True)
  results = []
  for e in sorted(glob.glob(os.path.join(root, 'events', 'event_*'))):
    if not os.path.exists(os.path.join(e, 'telemetry_full.jsonl.gz')):
      continue
    try:
      if do_plots:
        results.append(event_plots(e, out_dir))
      else:
        results.append(analyze_event(e))
    except Exception as ex:                                         # one bad event must not stop the rest
      open(os.path.join(root, 'events', 'analysis_errors.log'), 'a').write('%s: %r\n' % (e, ex))
  if do_plots:
    session_plots(root, out_dir, recs)
    write_index(out_dir)
  write_summary(root, recs)
  write_report(root, results)
  return results


if __name__ == '__main__':
  if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(1)
  cmd, root = sys.argv[1], sys.argv[2]
  if cmd == 'all':
    run_all(root)
  elif cmd == 'summary':
    print(write_summary(root, load_trace(root)))
  elif cmd == 'plots':
    run_all(root, True)
  elif cmd == 'report':
    r = run_all(root, False)
    print(open(os.path.join(root, 'root_cause_report.txt')).read())
