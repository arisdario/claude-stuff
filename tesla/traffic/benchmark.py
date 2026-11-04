#!/usr/bin/env python3
"""Resource benchmark for the EON. Advisory tooling: it only READS /proc, /sys and (optionally) cereal; it writes only its own CSV/JSON.

  python3 benchmark.py hw                                   what hardware/runtimes/libraries this EON has (read-only probe)
  python3 benchmark.py run --out /data/traffic_bench/base1.csv [--label drive] [--duration 3600]    sample every second until Ctrl+C
  python3 benchmark.py summary /data/traffic_bench/base1.csv [--suggest /data/traffic_limits.json] [--own-budget 40]

`run` records CPU (total, per core, rest-of-system), per-process CPU, free RAM, every thermal zone, GPU load and clock, CPU clocks,
battery temperature and - when cereal is importable - the driving model's and camerad's real message rates, frame-id gaps and thermald's
thermal status (this SUBSCRIBES to cereal; if it prints "evicting all subscribers" another reader may have been dropped: use --no-cereal). Run it with the detector OFF, onroad (bench or car), to measure what the driving stack already uses. Needs no openpilot code.
`summary --suggest` turns a baseline run into the limits file the governor requires (it refuses a run that is too short or lacks readings).
"""
import argparse
import csv
import json
import os
import platform
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sysstats as S  # noqa: E402

MIN_BASELINE_S = 600


class CerealProbe(object):
  """message rates of the driving model / camerad and thermald's status; silently absent when cereal cannot be imported"""

  def __init__(self, enabled=True):
    self.ok = False
    if not enabled:
      self.err = 'disabled with --no-cereal'
      return
    try:
      import cereal.messaging as messaging
      self.m = messaging
      self.socks = {n: messaging.sub_sock(n, conflate=False) for n in ('model', 'frame', 'thermal')}
      self.ok = True
    except Exception as e:                       # no cereal (PC) or no sockets yet
      self.err = str(e)
    self.last_model_id = None

  def poll(self, dt):
    out = {'model_fps': None, 'frame_fps': None, 'model_gaps': None, 'thermal_status': None}
    if not self.ok:
      return out
    try:
      n_model = n_frame = gaps = 0
      for ev in self.m.drain_sock(self.socks['model']):
        n_model += 1
        fid = ev.model.frameId
        if self.last_model_id is not None and fid > self.last_model_id + 1:
          gaps += fid - self.last_model_id - 1
        self.last_model_id = fid
      n_frame = len(self.m.drain_sock(self.socks['frame']))
      th = self.m.drain_sock(self.socks['thermal'])
      if th:
        out['thermal_status'] = str(th[-1].thermal.thermalStatus)
      out['model_fps'] = n_model / dt
      out['frame_fps'] = n_frame / dt
      out['model_gaps'] = gaps
    except Exception:
      pass
    return out


def hw_probe():
  info = {'python': sys.version.split()[0], 'machine': platform.machine(), 'kernel': platform.release()}
  try:
    cpu = open('/proc/cpuinfo').read()
    info['cpuinfo_hardware'] = [l.split(':', 1)[1].strip() for l in cpu.splitlines() if l.lower().startswith('hardware')][:1]
    info['cpus'] = sum(1 for l in cpu.splitlines() if l.startswith('processor'))
  except OSError:
    pass
  info['mem'] = S.parse_meminfo(open('/proc/meminfo').read()) if os.path.exists('/proc/meminfo') else None
  mods = {}
  for m in ('numpy', 'cv2', 'onnxruntime', 'tflite_runtime', 'torch', 'cereal', 'snpe'):
    try:
      __import__(m)
      mods[m] = True
    except Exception:
      mods[m] = False
  info['python_modules'] = mods
  libs = []
  for root in ('/data/openpilot', '/system/lib64', '/system/lib', '/vendor/lib64', '/vendor/lib', '/data/data/com.termux/files/usr/lib', '/usr/lib'):
    if not os.path.isdir(root):
      continue
    for dp, dn, fn in os.walk(root):
      if dp.count(os.sep) - root.count(os.sep) > 4:
        dn[:] = []
        continue
      for f in fn:
        lf = f.lower()
        if any(k in lf for k in ('snpe', 'cdsprpc', 'adsprpc', 'libopencv', 'libncnn', 'tflite', 'onnxruntime', 'libgsl', 'libopencl', 'libopenvx')):
          libs.append(os.path.join(dp, f))
      if len(libs) > 80:
        break
  info['accelerator_libs'] = sorted(set(libs))[:80]
  info['device_nodes'] = {n: os.path.exists(n) for n in ('/dev/kgsl-3d0', '/dev/adsprpc-smd', '/dev/ion', '/dev/video0')}
  info['thermal_zones'] = S.read_thermal()
  info['gpu'] = {'busy_pct': S.read_gpu()[0], 'mhz': S.read_gpu()[1]}
  models = {}
  for d in ('/data/openpilot/models', '/data/openpilot/selfdrive/modeld/models'):
    if os.path.isdir(d):
      models[d] = {f: os.path.getsize(os.path.join(d, f)) for f in sorted(os.listdir(d))[:40]}
  info['model_files'] = models
  return info


COLS_FIXED = ['t', 'label', 'cpu_total_pct', 'others_cpu_pct', 'self_cpu_core_pct', 'mem_avail_mb', 'temp_max_c', 'temp_soc_max_c', 'temp_board_max_c', 'gpu_busy_pct', 'gpu_mhz',
              'batt_c', 'model_fps', 'frame_fps', 'model_gaps', 'thermal_status', 'cpu_mhz_max']


def run(a):
  os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
  smp, cer = S.Sampler(proc_every=a.proc_every), CerealProbe(not a.no_cereal)
  print('cereal: %s' % ('on (model/frame/thermal)' if cer.ok else 'off (%s) - model fps and thermal status will be empty' % getattr(cer, 'err', '?')))
  smp.sample()
  cer.poll(1.0)
  t0 = time.time()
  with open(a.out, 'w', newline='') as f:
    w = None
    n = 0
    try:
      while a.duration <= 0 or time.time() - t0 < a.duration:
        time.sleep(max(0.05, a.interval - ((time.time() - t0) % a.interval)))
        s = smp.sample()
        if s is None:
          continue
        c = cer.poll(s['dt'])
        row = {k: s.get(k) for k in COLS_FIXED if k in s}
        row.update(c)
        row['label'] = a.label
        row['cpu_mhz_max'] = max(s['cpu_mhz'].values()) if s['cpu_mhz'] else ''
        for i, v in enumerate(s['cpu_core_pct']):
          row['core%d' % i] = '' if v is None else round(v, 1)
        top = sorted(s['procs'].items(), key=lambda kv: -kv[1])[:12]
        row['procs_json'] = json.dumps({k: round(v, 1) for k, v in top if v >= 0.5})
        if w is None:
          cols = COLS_FIXED + ['core%d' % i for i in range(len(s['cpu_core_pct']))] + ['procs_json']
          w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
          w.writeheader()
        w.writerow({k: ('' if v is None else (round(v, 2) if isinstance(v, float) else v)) for k, v in row.items()})
        f.flush()
        n += 1
        if n % 10 == 0:
          print('%5ds  cpu %5.1f%%  rest %5.1f%%  temp %s  model %s fps  free %.0f MB' % (
            time.time() - t0, s['cpu_total_pct'] or 0, s['others_cpu_pct'] or 0,
            '%.1f' % s['temp_max_c'] if s['temp_max_c'] is not None else '?', '%.1f' % c['model_fps'] if c['model_fps'] is not None else '?', s['mem_avail_mb']))
    except KeyboardInterrupt:
      pass
  print('wrote %d samples to %s' % (n, a.out))


def q(vals, p):
  v = sorted(x for x in vals if x is not None)
  if not v:
    return None
  k = (len(v) - 1) * p / 100.0
  lo, hi = int(k), min(int(k) + 1, len(v) - 1)
  return v[lo] + (v[hi] - v[lo]) * (k - lo)


def load(path):
  rows = []
  with open(path) as f:
    for r in csv.DictReader(f):
      d = {}
      for k, v in r.items():
        if k in ('label', 'thermal_status', 'procs_json'):
          d[k] = v
        else:
          try:
            d[k] = float(v) if v != '' else None
          except ValueError:
            d[k] = None
      rows.append(d)
  return rows


def summarize(rows, own_budget=40.0):
  col = lambda k: [r.get(k) for r in rows]
  res = {'samples': len(rows), 'seconds': (rows[-1]['t'] - rows[0]['t']) if rows else 0, 'metrics': {}, 'warnings': []}
  keys = ['cpu_total_pct', 'others_cpu_pct', 'mem_avail_mb', 'temp_max_c', 'temp_soc_max_c', 'temp_board_max_c', 'gpu_busy_pct', 'gpu_mhz', 'batt_c', 'model_fps', 'frame_fps', 'cpu_mhz_max'] + \
         sorted(k for k in (rows[0] if rows else {}) if k.startswith('core'))
  for k in keys:
    v = col(k)
    if any(x is not None for x in v):
      res['metrics'][k] = {'mean': sum(x for x in v if x is not None) / max(1, sum(1 for x in v if x is not None)), 'p5': q(v, 5), 'p50': q(v, 50),
                           'p95': q(v, 95), 'p99': q(v, 99), 'min': q(v, 0), 'max': q(v, 100)}
  procs = {}
  for r in rows:
    try:
      for k, v in json.loads(r.get('procs_json') or '{}').items():
        procs.setdefault(k, []).append(v)
    except ValueError:
      pass
  res['processes_pct_of_one_core'] = {k: {'mean_when_seen': sum(v) / len(v), 'p95': q(v, 95), 'max': max(v), 'seen_in_samples': len(v)}
                                      for k, v in sorted(procs.items(), key=lambda kv: -sum(kv[1]))[:15]}
  gaps = [x for x in col('model_gaps') if x is not None]
  res['model_frame_gaps_total'] = sum(gaps) if gaps else None
  ts = [r['thermal_status'] for r in rows if r.get('thermal_status')]
  res['thermal_status_not_green_samples'] = sum(1 for x in ts if x != 'green') if ts else None
  if res['seconds'] < MIN_BASELINE_S:
    res['warnings'].append('run is only %.0f s; a baseline needs at least %d s (more, over a real drive and a hot parked spell)' % (res['seconds'], MIN_BASELINE_S))
  if 'model_fps' not in res['metrics']:
    res['warnings'].append('no driving-model frame rate recorded (cereal not available): the governor cannot protect the model without it')
  if 'temp_max_c' not in res['metrics']:
    res['warnings'].append('no temperature readings')
  if 'cpu_total_pct' in res['metrics']:
    idle = [100.0 - x for x in col('cpu_total_pct') if x is not None]
    ncores = max(1, sum(1 for k in (rows[0] if rows else {}) if k.startswith('core')))
    need_all = own_budget / ncores + 10.0                  # the detector's budget plus a 10-point safety margin, in percent of ALL cores
    res['headroom'] = {'idle_pct_p5': q(idle, 5), 'idle_pct_p50': q(idle, 50), 'idle_pct_p95': q(idle, 95), 'ncores': ncores,
                       'idle_needed_pct': round(need_all, 1), 'fraction_of_time_enough_idle': round(sum(1 for x in idle if x >= need_all) / len(idle), 3)}
  if 'others_cpu_pct' in res['metrics']:
    m = res['metrics']
    ncores = res.get('headroom', {}).get('ncores', 4)
    res['suggested_limits'] = {
      'others_cpu_limit_pct': round(min(m['others_cpu_pct']['p99'] + 5.0, 100.0 - own_budget / ncores - 10.0), 1),
      'own_cpu_limit_core_pct': own_budget,
      'temp_limit_c': round(m['temp_max_c']['p99'] + 3.0, 1) if 'temp_max_c' in m else None,
      'mem_avail_min_mb': round(max(50.0, m['mem_avail_mb']['min'] - 100.0), 0) if 'mem_avail_mb' in m else None,
      '_from': 'baseline run, detector OFF: rest-of-system cpu p99 + 5 points, never above 100 minus the detector budget minus 10; temperature p99 + 3 C; free RAM min - 100 MB. Review before use.',
    }
  return res


def print_summary(res):
  print('%d samples, %.0f s' % (res['samples'], res['seconds']))
  print('%-18s %8s %8s %8s %8s %8s %8s' % ('metric', 'mean', 'p5', 'p50', 'p95', 'p99', 'max'))
  for k, m in res['metrics'].items():
    print('%-18s %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f' % (k, m['mean'], m['p5'], m['p50'], m['p95'], m['p99'], m['max']))
  print('\nper process (percent of ONE core):')
  for k, m in res['processes_pct_of_one_core'].items():
    print('  %-28s mean %6.1f  p95 %6.1f  max %6.1f  (in %d samples)' % (k, m['mean_when_seen'], m['p95'], m['max'], m['seen_in_samples']))
  h = res.get('headroom')
  if h:
    print('\nHEADROOM: idle CPU p5/p50/p95 = %.1f / %.1f / %.1f %% of all %d cores; the detector needs %.1f %% idle; that much was free %.0f %% of the time' % (
      h['idle_pct_p5'], h['idle_pct_p50'], h['idle_pct_p95'], h['ncores'], h['idle_needed_pct'], 100.0 * h['fraction_of_time_enough_idle']))
  print('\ndriving-model frame-id gaps: %s   thermal status not green: %s' % (res['model_frame_gaps_total'], res['thermal_status_not_green_samples']))
  for w in res['warnings']:
    print('WARNING:', w)
  if 'suggested_limits' in res:
    print('\nsuggested limits:', json.dumps(res['suggested_limits'], indent=1))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest='cmd')
  sub.add_parser('hw')
  r = sub.add_parser('run')
  r.add_argument('--out', required=True)
  r.add_argument('--label', default='')
  r.add_argument('--interval', type=float, default=2.0)
  r.add_argument('--proc-every', type=int, default=10, help='scan the process list only every N samples (a scan costs real CPU on the EON)')
  r.add_argument('--duration', type=float, default=0)
  r.add_argument('--no-cereal', action='store_true', help='do not subscribe to openpilot message queues (no model fps / thermal status). Use if the "evicting all subscribers" warning appears')
  s = sub.add_parser('summary')
  s.add_argument('csv')
  s.add_argument('--suggest')
  s.add_argument('--own-budget', type=float, default=40.0, help='percent of ONE core the detector may use (default 40)')
  s.add_argument('--force', action='store_true', help='write limits even from a short/incomplete run (not recommended)')
  a = ap.parse_args()
  if a.cmd == 'hw':
    print(json.dumps(hw_probe(), indent=1, default=str))
  elif a.cmd == 'run':
    if a.interval < 1.0:
      sys.exit('interval below 1 s makes the benchmark itself a load; use 1 or more')
    if a.interval > 5.0:
      sys.exit('interval above 5 s is too coarse for the model frame-rate check')
    run(a)
  elif a.cmd == 'summary':
    rows = load(a.csv)
    if not rows:
      sys.exit('no samples')
    res = summarize(rows, a.own_budget)
    print_summary(res)
    if a.suggest:
      if res['warnings'] and not a.force:
        sys.exit('not writing limits: %d warning(s) above (use --force only for experiments)' % len(res['warnings']))
      lim = res.get('suggested_limits')
      if not lim or lim.get('temp_limit_c') is None or lim.get('mem_avail_min_mb') is None:
        sys.exit('not writing limits: temperature or memory readings missing')
      with open(a.suggest, 'w') as f:
        json.dump({k: v for k, v in lim.items() if not k.startswith('_')}, f, indent=1)
      print('wrote', a.suggest)
  else:
    ap.print_help()


if __name__ == '__main__':
  main()
