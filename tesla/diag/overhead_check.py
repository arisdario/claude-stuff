#!/usr/bin/env python3
"""Does the logger disturb openpilot?  Run on the EON with openpilot running (driving NOT required, e.g. parked on the bench):

  python3 overhead_check.py [--baseline 40] [--logging 60] [--base /data/steering_diagnostics_overhead]

Phase 1 measures openpilot alone, phase 2 repeats with the collector running. Compared: CPU of camerad/modeld/controlsd/plannerd,
total CPU, temperature, model output rate and its worst interval, controlsd cycle worst interval. PASS = no meaningful change.
Read-only: it starts the same collector as start_diagnostics.sh and stops it afterwards."""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'traffic'))
sys.path.insert(0, '/data/openpilot')
import sysstats  # noqa: E402


def phase(seconds, label):
  import cereal.messaging as m
  socks = {n: m.sub_sock(n, conflate=False) for n in ('model', 'controlsState')}
  S = sysstats.Sampler(proc_every=2)
  t_last = {}
  worst = {'model': 0.0, 'controlsState': 0.0}
  cnt = {'model': 0, 'controlsState': 0}
  cpu, temps, per = [], [], {}
  end = time.time() + seconds
  nxt = time.time() + 2.0
  S.sample()
  while time.time() < end:
    for n, s in socks.items():
      for ev in m.drain_sock(s, wait_for_one=False):
        t = ev.logMonoTime / 1e9
        if n in t_last:
          worst[n] = max(worst[n], t - t_last[n])
        t_last[n] = t
        cnt[n] += 1
    if time.time() >= nxt:
      nxt += 2.0
      x = S.sample()
      if x:
        cpu.append(x['cpu_total_pct'])
        if x.get('temp_max_c') is not None:
          temps.append(x['temp_max_c'])
        for k, v in x['procs'].items():
          per.setdefault(k, []).append(v)
    time.sleep(0.003)
  avg = lambda v: sum(v) / len(v) if v else None
  out = {'label': label, 'model_hz': cnt['model'] / seconds, 'model_worst_ms': 1e3 * worst['model'], 'ctl_hz': cnt['controlsState'] / seconds,
         'ctl_worst_ms': 1e3 * worst['controlsState'], 'cpu_total_pct': avg(cpu), 'temp_max_c': max(temps) if temps else None,
         'procs': {k: round(avg(v), 1) for k, v in per.items() if any(key in k for key in ('camerad', 'modeld', 'controlsd', 'plannerd', 'collector'))}}
  print(json.dumps(out))
  return out


def main():
  a = sys.argv
  bl = float(a[a.index('--baseline') + 1]) if '--baseline' in a else 40
  lg = float(a[a.index('--logging') + 1]) if '--logging' in a else 60
  base = a[a.index('--base') + 1] if '--base' in a else '/data/steering_diagnostics_overhead'
  A = phase(bl, 'openpilot alone')
  session = os.path.join(base, 'overhead_%d' % int(time.time()))
  os.makedirs(session)
  p = subprocess.Popen(['nice', '-n', '10', sys.executable, os.path.join(HERE, 'collector.py'), '--root', session],
                       env=dict(os.environ, PYTHONPATH='/data/openpilot:' + HERE), stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
  time.sleep(8)
  B = phase(lg, 'with collector')
  p.terminate()
  try:
    p.wait(30)
  except subprocess.TimeoutExpired:
    p.kill()
  print('\n%-26s %12s %12s %10s' % ('', 'alone', 'collector', 'delta'))
  bad = []
  for k, tol in (('model_hz', 0.3), ('model_worst_ms', 25), ('ctl_worst_ms', 20), ('cpu_total_pct', 6), ('temp_max_c', 3)):
    x, y = A[k], B[k]
    if x is None or y is None:
      continue
    d = y - x
    print('%-26s %12.2f %12.2f %+10.2f' % (k, x, y, d))
    if (k == 'model_hz' and d < -tol) or (k != 'model_hz' and d > tol):
      bad.append(k)
  for k in sorted(set(A['procs']) | set(B['procs'])):
    print('%-26s %12s %12s' % ('cpu% ' + k, A['procs'].get(k), B['procs'].get(k)))
  print('\nRESULT: %s' % ('PASS - no meaningful effect on camerad/modeld/controlsd' if not bad else 'CHECK: %s changed beyond tolerance' % bad))
  return 1 if bad else 0


if __name__ == '__main__':
  sys.exit(main())
