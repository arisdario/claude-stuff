#!/usr/bin/env python3
"""lightlog: a small steering/parameter logger for the EON (one process, ~20 Hz, plain CSV, nice 19).

Records only what is needed to answer "what are the learned vehicle parameters, and does the wiggle follow them":
  carState (measured angle, speed, yaw rate), carControl (commanded angle, enabled), controlsState (active, desired
  angle), pathPlan (planner angle, lane probs, paramsValid), liveParameters (steerRatio, stiffnessFactor, angleOffset,
  valid), model (lane probs). It never sends anything and never touches control. Analysis is offline:
  python3 lightlog_report.py /data/lightlog/*.csv

  python3 lightlog.py                  # log until Ctrl-C / SIGTERM (suggested: in tmux)
  python3 lightlog.py --hz 50          # finer timing for the delay estimate (still light)
Files: /data/lightlog/ll_YYYYmmdd_HHMMSS.csv, a new one every hour, oldest deleted above 150 MB in total.
"""
import argparse
import glob
import os
import signal
import sys
import time

LOGDIR = '/data/lightlog'
MAX_TOTAL_MB = 150
COLS = ['t', 'v', 'ang', 'ang_cmd', 'yaw', 'pressed', 'enabled', 'active', 'ang_des', 'pp_ang', 'pp_valid', 'lprob',
        'rprob', 'sR', 'sF', 'ao', 'lp_valid', 'm_lprob', 'm_rprob', 'age_cs', 'age_cc']


def _g(o, *names, default=float('nan')):
  for n in names:
    try:
      v = getattr(o, n)
      if v is not None:
        return float(v)
    except Exception:
      pass
  return default


def _trim(logdir, max_mb):
  files = sorted(glob.glob(os.path.join(logdir, 'll_*.csv')))
  total = sum(os.path.getsize(f) for f in files)
  while files[:-1] and total > max_mb * 1e6:     # never delete the newest file
    total -= os.path.getsize(files[0]); os.remove(files[0]); files.pop(0)


def _open(logdir):
  name = os.path.join(logdir, 'll_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
  f = open(name, 'w')
  meta = ''
  try:
    from common.params import Params
    p = Params()
    for k in ('LiveParameters', 'CalibrationParams'):
      v = p.get(k)
      if v:
        meta += '# %s %s\n' % (k, v.decode('utf8', 'replace').replace('\n', ' '))
  except Exception:
    pass
  f.write(meta + ','.join(COLS) + '\n')
  return f


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--hz', type=float, default=20.)
  ap.add_argument('--dir', default=LOGDIR)
  a = ap.parse_args()
  try:
    os.nice(19)
  except Exception:
    pass
  import cereal.messaging as messaging
  os.makedirs(a.dir, exist_ok=True)
  S = {n: messaging.sub_sock(n, conflate=True) for n in ('carState', 'carControl', 'controlsState', 'pathPlan',
                                                         'liveParameters', 'model')}
  stop = []
  signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
  signal.signal(signal.SIGINT, lambda *_: stop.append(1))
  v = dict.fromkeys(COLS, float('nan'))
  recv = {'carState': 0., 'carControl': 0.}
  f = _open(a.dir); opened = time.time(); buf = []; last_flush = last_sync = time.time()
  dt = 1. / a.hz; nxt = time.monotonic()
  while not stop:
    now = time.monotonic()
    for name, sock in S.items():
      m = messaging.recv_one_or_none(sock)
      if m is None:
        continue
      if name == 'carState':
        c = m.carState; v['v'] = _g(c, 'vEgo'); v['ang'] = _g(c, 'steeringAngle', 'steeringAngleDeg')
        v['yaw'] = _g(c, 'yawRate'); v['pressed'] = _g(c, 'steeringPressed'); recv[name] = now
      elif name == 'carControl':
        c = m.carControl; v['enabled'] = _g(c, 'enabled'); v['ang_cmd'] = _g(c.actuators, 'steerAngle', 'steeringAngleDeg')
        recv[name] = now
      elif name == 'controlsState':
        c = m.controlsState; v['active'] = _g(c, 'active'); v['ang_des'] = _g(c, 'angleSteersDes')
      elif name == 'pathPlan':
        c = m.pathPlan; v['pp_ang'] = _g(c, 'angleSteers'); v['pp_valid'] = _g(c, 'paramsValid')
        v['lprob'] = _g(c, 'lProb'); v['rprob'] = _g(c, 'rProb')
      elif name == 'liveParameters':
        c = m.liveParameters; v['sR'] = _g(c, 'steerRatio'); v['sF'] = _g(c, 'stiffnessFactor')
        v['ao'] = _g(c, 'angleOffsetAverage', 'angleOffset'); v['lp_valid'] = _g(c, 'valid')
      elif name == 'model':
        c = m.model; v['m_lprob'] = _g(c.leftLane, 'prob'); v['m_rprob'] = _g(c.rightLane, 'prob')
    v['t'] = now
    v['age_cs'] = now - recv['carState'] if recv['carState'] else float('nan')
    v['age_cc'] = now - recv['carControl'] if recv['carControl'] else float('nan')
    buf.append(','.join('%.4f' % v[c] if v[c] == v[c] else '' for c in COLS))
    t = time.time()
    if t - last_flush >= 5:
      f.write('\n'.join(buf) + '\n'); f.flush(); buf = []; last_flush = t
      if t - last_sync >= 30:
        try: os.fsync(f.fileno())
        except Exception: pass
        last_sync = t
      if t - opened >= 3600:
        f.close(); _trim(a.dir, MAX_TOTAL_MB); f = _open(a.dir); opened = t
    nxt += dt
    sl = nxt - time.monotonic()
    if sl > 0:
      time.sleep(sl)
    else:
      nxt = time.monotonic()
  if buf:
    f.write('\n'.join(buf) + '\n')
  f.close()


if __name__ == '__main__':
  main()
