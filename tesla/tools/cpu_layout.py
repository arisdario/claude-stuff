#!/usr/bin/env python3
"""cpu_layout.py - CPU affinity analyzer for the EON: discover, measure, project, recommend, and (only if you ask) apply with a saved original and automatic rollback.
Like openpilot 0.8.3, but checked against YOUR measured load first. Nothing is changed unless you run `apply`.

  python3 cpu_layout.py discover             read-only: online CPUs, frequencies, Android cpusets, interrupt assignments (which core takes which IRQ)
  python3 cpu_layout.py show                 read-only: which cores each openpilot process may use, and where its threads last ran
  python3 cpu_layout.py plan [seconds]       read-only: measure per process and per core (user/system/IRQ) for N s (default 20), project every candidate layout (per-core
                                             busy %, peak, headroom, overload), then recommend one, with warnings and alternatives. "No change" is a valid recommendation.
  python3 cpu_layout.py apply LAYOUT [--trial SECS] [--keep]
        LAYOUT = adapted | stock083 | reset.  The ORIGINAL affinity of every thread is saved first (/data/cpu_layout_saved.json, kept from the first apply).
        --trial SECS (default 60): apply, watch, and ROLL BACK AUTOMATICALLY at the end unless --keep. Rolls back at once if a pinned process exits or a core stays above
        95% for 3 samples, on Ctrl+C, and on any error. --keep leaves the layout in place after a clean trial.
  python3 cpu_layout.py rollback             restore the saved original affinities (also what a reboot does)
  python3 cpu_layout.py system               PRINTS (does not run) the 0.8.3 cpuset and interrupt commands matched to your /proc/interrupts, each with its undo

0.8.3 (launch_chffrplus.sh, modeld.cc, camerad/main.cc, controlsd.py, plannerd.py, radard.py, boardd.cc): modeld, camerad, plannerd, radard -> core 2; controlsd, boardd -> core 3;
Android confined to cores 0-1; USB interrupt -> 3, camera and GPU interrupts -> 2, modem/flash/wifi/display interrupts -> 1. Affinity only moves threads between cores: no
priorities, no code. Apply only with the car parked and openpilot NOT engaged; compare the model's `from last` intervals before and after."""
import json
import os
import re
import sys
import time

NCPU = 4
ALL = {0, 1, 2, 3}
PROCS = {                       # name -> argv[0] basename as it appears in /proc/<pid>/cmdline
  'modeld': '_modeld', 'camerad': 'camerad', 'controlsd': 'selfdrive.controls.controlsd', 'plannerd': 'selfdrive.controls.plannerd',
  'radard': 'selfdrive.controls.radard', 'boardd': 'boardd',
}
LAYOUTS = {
  'reset': {n: set(ALL) for n in PROCS},
  'adapted': {'modeld': {2}, 'controlsd': {3}, 'boardd': {3}},
  'stock083': {'modeld': {2}, 'camerad': {2}, 'plannerd': {2}, 'radard': {2}, 'controlsd': {3}, 'boardd': {3}},
}
OVERLOAD = 85.0               # projected busy % above which a core is called overloaded
CRITICAL = 95.0               # measured busy % that triggers an immediate rollback during a trial
STATE = os.environ.get('CPU_LAYOUT_STATE', '/data/cpu_layout_saved.json')
CPUSETS = ['background', 'system-background', 'foreground', 'foreground/boost', 'android', 'app']


def project(loads, layout, other=0.0, ncpu=NCPU, irq=None):
  """loads: {name: % of one core}; layout: {name: set(cores)} (names not in the layout float); other: % of all remaining processes (spread evenly)
  -> list of projected busy % per core. Floating processes go greedily to the least loaded core they may use."""
  core = [0.0] * ncpu
  for n, c in loads.items():
    cores = layout.get(n)
    if cores and len(cores) == 1:
      core[min(cores)] += c
  for n, c in sorted(loads.items(), key=lambda kv: -kv[1]):
    cores = layout.get(n)
    if cores and len(cores) == 1:
      continue
    allowed = sorted(cores) if cores else list(range(ncpu))
    core[min(allowed, key=lambda i: core[i])] += c
  for i in range(ncpu):
    core[i] += other / ncpu + (irq[i] if irq and i < len(irq) else 0.0)
  return core


def evaluate(loads, layout, other, irq, ncpu=NCPU):
  cores = project(loads, layout, other, ncpu, irq)
  peak = max(cores)
  return {'cores': cores, 'peak': peak, 'headroom': 100.0 - peak, 'overloaded': [i for i, c in enumerate(cores) if c > OVERLOAD]}


CANDIDATES = [
  ('float (no pinning, what runs now)', {}),
  ('adapted: modeld->2, controlsd+boardd->3', LAYOUTS['adapted']),
  ('modeld->2 only', {'modeld': {2}}),
  ('controlsd+boardd->3 only', {'controlsd': {3}, 'boardd': {3}}),
  ('adapted + camerad->1', dict(LAYOUTS['adapted'], camerad={1})),
  ('stock083: modeld, camerad, plannerd, radard->2; controlsd, boardd->3', LAYOUTS['stock083']),
]


def recommend(loads, other, irq, measured_peak=None):
  """-> (verdict text, warnings list, rows [(name, eval)]). Picks the candidate with the lowest projected peak among those with no overloaded core, but only
  if it beats the floating baseline by at least 3 points; otherwise recommends no change (pinning also takes away the scheduler's freedom to rebalance)."""
  rows = [(n, evaluate(loads, lay, other, irq)) for n, lay in CANDIDATES]
  base = rows[0][1]
  ok = [(n, e) for n, e in rows[1:] if not e['overloaded']]
  warnings = [n.split(':')[0] + ' would overload core(s) %s (peak %.0f%%)' % (e['overloaded'], e['peak']) for n, e in rows[1:] if e['overloaded']]
  if loads.get('camerad', 0) > 30:
    warnings.append('camerad costs %.0f%% of a core (software JPEG decode of the C920): it cannot share a core with modeld' % loads['camerad'])
  warnings.append('projection ignores cache effects, memory bandwidth and the time-varying load; interrupts are taken as measured now (the `system` plan would move them)')
  if measured_peak is not None:
    warnings.append('the scheduler measured peak was %.0f%% (the float row is a projection of the same load)' % measured_peak)
  best = min(ok, key=lambda r: r[1]['peak']) if ok else None
  if best and best[1]['peak'] <= base['peak'] - 3.0:
    verdict = 'candidate: %s  (projected peak %.0f%% against %.0f%% floating, headroom %.0f%%)' % (best[0], best[1]['peak'], base['peak'], best[1]['headroom'])
  else:
    verdict = 'NO CHANGE recommended: no candidate beats floating by 3 points or more (floating peak %.0f%%, headroom %.0f%%)' % (base['peak'], base['headroom'])
  return verdict, warnings, rows


def cmd0(pid):
  try:
    c = open('/proc/%s/cmdline' % pid, 'rb').read().replace(b'\0', b' ').decode('utf8', 'replace')
    return c.split(' ')[0].split('/')[-1]
  except OSError:
    return ''


def find_pids():
  out = {}
  me = os.getpid()
  for d in os.listdir('/proc'):
    if d.isdigit() and int(d) != me:
      b = cmd0(d)
      for n, m in PROCS.items():
        if b == m and n not in out:
          out[n] = int(d)
  return out


def tids(pid):
  try:
    return [int(t) for t in os.listdir('/proc/%d/task' % pid)]
  except OSError:
    return []


def ticks(pid):
  try:
    f = open('/proc/%d/stat' % pid).read().rsplit(')', 1)[1].split()
    return int(f[11]) + int(f[12])               # utime + stime
  except (OSError, IndexError, ValueError):
    return 0


def cpu_stat():
  d = {}
  for l in open('/proc/stat'):
    if l.startswith('cpu') and l[3].isdigit():
      f = l.split(); v = list(map(int, f[1:8])); d[int(f[0][3:])] = (sum(v), v[3] + v[4])
  return d


def cpu_split():
  """/proc/stat per core: (total, idle+iowait, user+nice, system, irq+softirq)"""
  d = {}
  for l in open('/proc/stat'):
    if l.startswith('cpu') and l[3].isdigit():
      f = l.split(); v = list(map(int, f[1:8]))
      d[int(f[0][3:])] = (sum(v), v[3] + v[4], v[0] + v[1], v[2], v[5] + v[6])
  return d


def measure(secs):
  """-> ({name: %}, other_total %, measured busy % per core)"""
  hz = os.sysconf('SC_CLK_TCK')
  a = {int(d): ticks(int(d)) for d in os.listdir('/proc') if d.isdigit()}
  c0 = cpu_split(); t0 = time.time()
  time.sleep(secs)
  b = {int(d): ticks(int(d)) for d in os.listdir('/proc') if d.isdigit()}
  c1 = cpu_split(); dt = time.time() - t0
  pct = {pid: 100.0 * (b[pid] - a.get(pid, 0)) / hz / dt for pid in b}
  pids = find_pids()
  known = {n: pct.get(p, 0.0) for n, p in pids.items()}
  other = max(0.0, sum(pct.values()) - sum(known.values()))
  tot = {i: max(1, c1[i][0] - c0[i][0]) for i in c0}
  busy = [100.0 * (tot[i] - (c1[i][1] - c0[i][1])) / tot[i] for i in sorted(c0)]
  split = [{'user': 100.0 * (c1[i][2] - c0[i][2]) / tot[i], 'system': 100.0 * (c1[i][3] - c0[i][3]) / tot[i], 'irq': 100.0 * (c1[i][4] - c0[i][4]) / tot[i]} for i in sorted(c0)]
  return known, other, busy, split


def fmt(cores):
  return '  '.join('cpu%d %5.1f%%%s' % (i, c, ' OVERLOAD' if c > OVERLOAD else '') for i, c in enumerate(cores))


def cmd_plan(secs):
  known, other, busy, split = measure(secs)
  irq = [x['irq'] for x in split]
  print('WORKLOAD, measured over %.0f s (percent of ONE core):' % secs)
  for n, c in sorted(known.items(), key=lambda kv: -kv[1]):
    print('  %-10s %5.1f%%' % (n, c))
  print('  %-10s %5.1f%%   (everything else together, kernel threads included)' % ('other', other))
  print('PER CORE now (the scheduler decides):')
  for i, (bz, sp) in enumerate(zip(busy, split)):
    print('  cpu%d busy %5.1f%%   = user %5.1f%% + system %5.1f%% + irq/softirq %4.1f%%' % (i, bz, sp['user'], sp['system'], sp['irq']))
  verdict, warnings, rows = recommend(known, other, irq, max(busy))
  print('\nPROJECTION (pinned processes where the layout puts them; floaters fill the least loaded allowed core; irq as measured above):')
  for n, e in rows:
    print('  %-70s peak %5.1f%%  headroom %5.1f%%  %s' % (n, e['peak'], e['headroom'], 'OVERLOAD cpu%s' % e['overloaded'] if e['overloaded'] else ''))
    print('      ' + '  '.join('cpu%d %5.1f%%' % (i, c) for i, c in enumerate(e['cores'])))
  print('\nRECOMMENDATION: ' + verdict)
  for w in warnings:
    print('  warning: ' + w)
  print('A core projected above %d%% leaves the realtime processes on it no room. Apply (parked, not engaged): python3 %s apply adapted --trial 120' % (OVERLOAD, sys.argv[0]))
  return 0


def allowed_list(pid):
  try:
    for l in open('/proc/%d/status' % pid):
      if l.startswith('Cpus_allowed_list'):
        return l.split(':')[1].strip()
  except OSError:
    pass
  return '?'


def last_cpu(pid, tid):
  try:
    return int(open('/proc/%d/task/%d/stat' % (pid, tid)).read().rsplit(')', 1)[1].split()[36])
  except (OSError, IndexError, ValueError):
    return -1


def cmd_show():
  pids = find_pids()
  for n in PROCS:
    if n not in pids:
      print('%-10s not running' % n); continue
    pid = pids[n]
    ts = tids(pid)
    where = {}
    for t in ts:
      c = last_cpu(pid, t)
      where[c] = where.get(c, 0) + 1
    print('%-10s pid %-5d allowed cpus %-8s threads %2d, last ran on: %s' % (n, pid, allowed_list(pid), len(ts), ', '.join('cpu%d x%d' % kv for kv in sorted(where.items()))))
  return 0


def read(path, default=''):
  try:
    return open(path).read().strip()
  except OSError:
    return default


def parse_cpulist(text):
  out = set()
  for part in text.split(','):
    part = part.strip()
    if '-' in part:
      lo, hi = part.split('-'); out.update(range(int(lo), int(hi) + 1))
    elif part.isdigit():
      out.add(int(part))
  return sorted(out)


def parse_interrupts(text):
  """/proc/interrupts -> [{'irq': N, 'counts': [per cpu], 'name': str}] for numbered IRQs"""
  rows = []
  for l in text.splitlines()[1:]:
    m = re.match(r'\s*(\d+):\s+((?:\d+\s+)+)(.*)$', l)
    if m:
      rows.append({'irq': int(m.group(1)), 'counts': [int(x) for x in m.group(2).split()], 'name': m.group(3).strip()})
  return rows


def cmd_discover():
  cpus = parse_cpulist(read('/sys/devices/system/cpu/online', '0-%d' % (NCPU - 1)))
  print('CPUS online: %s' % cpus)
  for c in cpus:
    cur = read('/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq' % c, '?'); mx = read('/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq' % c, '?')
    gov = read('/sys/devices/system/cpu/cpu%d/cpufreq/scaling_governor' % c, '?')
    print('  cpu%d  now %s kHz  max %s kHz  governor %s' % (c, cur, mx, gov))
  print('ANDROID/APP CPUSETS (/dev/cpuset):')
  for n in CPUSETS:
    print('  %-18s %s' % (n, read('/dev/cpuset/%s/cpus' % n, '(not present)')))
  print('INTERRUPTS with activity (counts per cpu since boot, and which cores each IRQ may use):')
  rows = [r for r in parse_interrupts(read('/proc/interrupts')) if sum(r['counts']) > 0]
  for r in sorted(rows, key=lambda r: -sum(r['counts']))[:14]:
    print('  irq %-4d %-30s counts %-40s allowed %s' % (r['irq'], r['name'][:30], r['counts'], read('/proc/irq/%d/smp_affinity_list' % r['irq'], '?')))
  return 0


def snapshot(pids):
  """{pid: {tid: [cores]}} - the exact affinity of every thread, to restore later"""
  snap = {}
  for n, pid in pids.items():
    snap[str(pid)] = {'name': n, 'threads': {}}
    for t in tids(pid):
      try:
        snap[str(pid)]['threads'][str(t)] = sorted(os.sched_getaffinity(t))
      except OSError:
        pass
  return snap


def restore(snap):
  ok = bad = 0
  for pid, info in snap.items():
    for t, cores in info['threads'].items():
      try:
        os.sched_setaffinity(int(t), set(cores)); ok += 1
      except OSError:
        bad += 1                                    # thread or process gone: nothing to restore
  return ok, bad


def apply_layout(layout, pids):
  for n, cores in layout.items():
    if n not in pids:
      print('%-10s not running, skipped' % n); continue
    ok = bad = 0
    for t in tids(pids[n]):
      try:
        os.sched_setaffinity(t, cores); ok += 1
      except OSError:
        bad += 1
    print('%-10s -> cpus %s : %d threads set%s' % (n, sorted(cores), ok, (', %d failed' % bad) if bad else ''))


def watch(alive, busy_now, secs, interval=2.0, sleep=time.sleep):
  """-> None if the trial ran clean, else the reason to roll back. alive() -> name of a pinned process that is gone, or None; busy_now() -> busy % per core over the last interval"""
  streak = 0
  waited = 0.0
  while waited < secs:
    sleep(interval); waited += interval
    gone = alive()
    if gone:
      return 'pinned process %s exited' % gone
    hot = [i for i, b in enumerate(busy_now()) if b > CRITICAL]
    streak = streak + 1 if hot else 0
    if streak >= 3:
      return 'cpu%s above %d%% for 3 samples' % (hot, CRITICAL)
  return None


def load_state():
  try:
    return json.load(open(STATE))
  except (OSError, ValueError):
    return None


def cmd_rollback():
  snap = load_state()
  if not snap:
    print('nothing saved (%s): the original affinities are in place, or this reboot reset them' % STATE); return 0
  ok, bad = restore(snap['saved'])
  try:
    os.unlink(STATE)
  except OSError:
    pass
  print('restored the original affinity of %d threads (%d gone)' % (ok, bad))
  return 0


def cmd_apply(args):
  name = args[0] if args else ''
  if name not in LAYOUTS:
    print('unknown layout %s (adapted | stock083 | reset)' % name); return 2
  keep = '--keep' in args
  trial = 60.0
  if '--trial' in args:
    try:
      trial = float(args[args.index('--trial') + 1])
    except (IndexError, ValueError):
      print('--trial needs a number of seconds'); return 2
  pids = find_pids()
  if not pids:
    print('no openpilot processes found: is it running?'); return 1
  st = load_state()
  if st is None:                                    # the first apply saves the true originals; later applies keep them
    st = {'when': time.time(), 'saved': snapshot(pids)}
    json.dump(st, open(STATE, 'w'))
  print('original affinities saved to %s (%d processes)' % (STATE, len(st['saved'])))
  reason = None
  try:
    apply_layout(LAYOUTS[name], pids)
    prev = [cpu_stat()]
    def busy_now():
      cur = cpu_stat(); p = prev[0]; prev[0] = cur
      return [100.0 * ((cur[i][0] - p[i][0]) - (cur[i][1] - p[i][1])) / max(1, cur[i][0] - p[i][0]) for i in sorted(cur)]
    def alive():
      for n in LAYOUTS[name]:
        if n in pids and cmd0(pids[n]) != PROCS[n]:
          return n
      return None
    print('trial for %.0f s (Ctrl+C rolls back)%s ...' % (trial, '' if keep else '; it will roll back by itself at the end unless --keep'))
    reason = watch(alive, busy_now, trial)
  except KeyboardInterrupt:
    reason = 'interrupted'
  except Exception as e:
    reason = 'error: %r' % (e,)
  if reason or not keep:
    ok, bad = restore(st['saved'])
    try:
      os.unlink(STATE)
    except OSError:
      pass
    print('ROLLED BACK (%s): %d threads restored' % (reason or 'trial finished, --keep not given', ok))
    return 1 if reason else 0
  print('trial clean: layout %s is KEPT. Undo with: python3 %s rollback   (or reboot)' % (name, sys.argv[0]))
  return 0


def cmd_system():
  irqs = {}
  for l in open('/proc/interrupts'):
    m = re.match(r'\s*(\d+):', l)
    if m:
      irqs[int(m.group(1))] = l.strip()
  def find(pat):
    return [i for i, l in irqs.items() if re.search(pat, l)]
  plan = [
    ('USB (the C920 and the panda share it): CPU3, as 0.8.3 does', find(r'xhci|dwc3'), 3),
    ('GPU: CPU2', find(r'kgsl'), 2),
    ('camera control bus: CPU2', find(r'\bcci\b'), 2),
    ('display (MDSS): CPU1', find(r'mdss'), 1),
    ('wifi: CPU1', find(r'wlan|wifi'), 1),
    ('flash storage: CPU1', find(r'ufs'), 1),
  ]
  print('# 0.8.3 "shield cores 2-3", matched to this phone. Nothing here has been run. Park the car, openpilot not engaged. Undo for every IRQ line: echo 0-3 > /proc/irq/N/smp_affinity_list')
  for text, nums, cpu in plan:
    for n in nums:
      print('echo %d > /proc/irq/%d/smp_affinity_list   # %s  [now: %s]' % (cpu, n, text, irqs[n][:70]))
    if not nums:
      print('# (no matching interrupt found for: %s)' % text)
  print('# Android confined to cores 0-1 (openpilot keeps all four). Undo: write 0-3 to the same files')
  for d in ('background', 'system-background', 'foreground', 'foreground/boost', 'android'):
    print('echo 0-1 > /dev/cpuset/%s/cpus' % d)
  return 0


def main():
  a = sys.argv[1:]
  if not a:
    print(__doc__); return 2
  if a[0] == 'show':
    return cmd_show()
  if a[0] == 'plan':
    return cmd_plan(float(a[1]) if len(a) > 1 else 20.0)
  if a[0] == 'discover':
    return cmd_discover()
  if a[0] == 'rollback':
    return cmd_rollback()
  if a[0] == 'apply' and len(a) > 1:
    return cmd_apply(a[1:])
  if a[0] == 'system':
    return cmd_system()
  print(__doc__); return 2


if __name__ == '__main__':
  sys.exit(main())
