#!/usr/bin/env python3
"""cpu_layout.py - CPU affinity like openpilot 0.8.3 on the EON, checked against YOUR measured load first. Nothing is applied unless you run `apply`.

  python3 cpu_layout.py show                 read-only: which cores each openpilot process may use, and where its threads last ran
  python3 cpu_layout.py plan [seconds]       read-only: measure every process for N s (default 20), then project the busy % per core for each layout
  python3 cpu_layout.py apply adapted        pin modeld->2, controlsd->3, boardd->3 (what 0.8.3 does for these), everything else floats. Reversible: apply reset (or reboot)
  python3 cpu_layout.py apply reset          all four cores allowed again for every openpilot process listed here
  python3 cpu_layout.py apply stock083       0.8.3 literally (modeld, camerad, plannerd, radard -> 2; controlsd, boardd -> 3). plan shows why that is overloaded with a C920
  python3 cpu_layout.py system               PRINTS (does not run) the 0.8.3 cpuset and interrupt commands matched to your /proc/interrupts, each with its undo

0.8.3 (launch_chffrplus.sh, modeld.cc, camerad/main.cc, controlsd.py, plannerd.py, radard.py, boardd.cc): modeld, camerad, plannerd, radard -> core 2; controlsd, boardd -> core 3;
Android confined to cores 0-1; USB interrupt -> 3, camera and GPU interrupts -> 2, modem/flash/wifi/display interrupts -> 1. Affinity only moves threads between cores; it changes no
priorities and no code. Run `plan` first and apply only with the car parked and openpilot NOT engaged; compare the model's `from last` intervals before and after."""
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
OVERLOAD = 85.0


def project(loads, layout, other=0.0, ncpu=NCPU):
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
    core[i] += other / ncpu
  return core


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


def measure(secs):
  """-> ({name: %}, other_total %, measured busy % per core)"""
  hz = os.sysconf('SC_CLK_TCK')
  a = {int(d): ticks(int(d)) for d in os.listdir('/proc') if d.isdigit()}
  c0 = cpu_stat(); t0 = time.time()
  time.sleep(secs)
  b = {int(d): ticks(int(d)) for d in os.listdir('/proc') if d.isdigit()}
  c1 = cpu_stat(); dt = time.time() - t0
  pct = {pid: 100.0 * (b[pid] - a.get(pid, 0)) / hz / dt for pid in b}
  pids = find_pids()
  known = {n: pct.get(p, 0.0) for n, p in pids.items()}
  other = max(0.0, sum(pct.values()) - sum(known.values()))
  busy = [100.0 * ((c1[i][0] - c0[i][0]) - (c1[i][1] - c0[i][1])) / max(1, c1[i][0] - c0[i][0]) for i in sorted(c0)]
  return known, other, busy


def fmt(cores):
  return '  '.join('cpu%d %5.1f%%%s' % (i, c, ' OVERLOAD' if c > OVERLOAD else '') for i, c in enumerate(cores))


def cmd_plan(secs):
  known, other, busy = measure(secs)
  print('measured over %.0f s (percent of ONE core):' % secs)
  for n, c in sorted(known.items(), key=lambda kv: -kv[1]):
    print('  %-10s %5.1f%%' % (n, c))
  print('  %-10s %5.1f%%   (everything else together)' % ('other', other))
  print('measured busy per core now (the scheduler decides): ' + '  '.join('cpu%d %.0f%%' % (i, v) for i, v in enumerate(busy)))
  print('\nprojected busy per core if the pinned processes stay where the layout puts them (floaters fill the least loaded allowed core; interrupts not included):')
  for name in ('adapted', 'stock083'):
    print('  %-9s %s' % (name, fmt(project(known, LAYOUTS[name], other))))
  print('\nA core projected above %d%% leaves the realtime processes on it no room: do not apply that layout.' % OVERLOAD)
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


def cmd_apply(name):
  if name not in LAYOUTS:
    print('unknown layout %s (reset | adapted | stock083)' % name); return 2
  pids = find_pids()
  for n, cores in LAYOUTS[name].items():
    if n not in pids:
      print('%-10s not running, skipped' % n); continue
    ok = bad = 0
    for t in tids(pids[n]):
      try:
        os.sched_setaffinity(t, cores); ok += 1
      except OSError:
        bad += 1
    print('%-10s -> cpus %s : %d threads set%s' % (n, sorted(cores), ok, (', %d failed' % bad) if bad else ''))
  print('undo: python3 %s apply reset   (a reboot also resets everything)' % sys.argv[0])
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
  if a[0] == 'apply' and len(a) > 1:
    return cmd_apply(a[1])
  if a[0] == 'system':
    return cmd_system()
  print(__doc__); return 2


if __name__ == '__main__':
  sys.exit(main())
