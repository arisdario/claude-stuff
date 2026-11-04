#!/usr/bin/env python3
"""thread_cpu.py - READ-ONLY: where does a running process spend CPU, per thread, split into user and kernel (system) time. Changes nothing.

  python3 thread_cpu.py camerad [seconds]     (default 10 s; any process name or pid; run with the car on and openpilot running)

Reads /proc/<pid>/task/*/stat twice. Per thread: % of one core in user time and in system time. A large SYSTEM share points at the kernel:
USB handling, buffer (ION) cache syncs, GPU driver waits; a large USER share is our own/library code (JPEG decode, copies).
Thread names help: the libuvc streaming thread, camerad's capture thread, OpenCL/GPU driver threads. stdlib only (works on the EON's python 3.7)."""
import os
import sys
import time

TICK = os.sysconf('SC_CLK_TCK')


def find_pid(name):
  if name.isdigit():
    return int(name)
  me = os.getpid()
  best = None
  for d in os.listdir('/proc'):
    if d.isdigit() and int(d) != me:
      try:
        cmd = open('/proc/%s/cmdline' % d, 'rb').read().replace(b'\0', b' ').decode('utf8', 'replace')
        base = cmd.split(' ')[0].split('/')[-1]
      except OSError:
        continue
      if base == name or base.startswith(name) or ('/' + name) in cmd.split(' ')[0]:
        best = int(d)
  return best


def snap(pid):
  out = {}
  for t in os.listdir('/proc/%d/task' % pid):
    try:
      st = open('/proc/%d/task/%s/stat' % (pid, t)).read()
      comm = st[st.index('(') + 1:st.rindex(')')]
      f = st[st.rindex(')') + 2:].split()
      out[int(t)] = (comm, int(f[11]), int(f[12]))          # utime, stime in ticks
    except (OSError, ValueError, IndexError):
      pass
  return out


def main():
  if len(sys.argv) < 2:
    print(__doc__); return 2
  secs = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
  pid = find_pid(sys.argv[1])
  if not pid:
    print('process not found: %s' % sys.argv[1]); return 1
  a = snap(pid); t0 = time.time()
  time.sleep(secs)
  b = snap(pid); dt = time.time() - t0
  rows = []
  for t, (comm, u1, s1) in b.items():
    u0, s0 = (a[t][1], a[t][2]) if t in a else (0, 0)
    rows.append((100.0 * (u1 - u0 + s1 - s0) / TICK / dt, 100.0 * (u1 - u0) / TICK / dt, 100.0 * (s1 - s0) / TICK / dt, t, comm))
  rows.sort(reverse=True)
  tot = sum(r[0] for r in rows); tu = sum(r[1] for r in rows); ts = sum(r[2] for r in rows)
  print('pid %d over %.1f s: total %.1f%% of one core  (user %.1f%%, system/kernel %.1f%%)' % (pid, dt, tot, tu, ts))
  print('%7s %7s %7s  %-8s %s' % ('total%', 'user%', 'sys%', 'tid', 'thread'))
  for r in rows[:12]:
    if r[0] >= 0.2:
      print('%7.1f %7.1f %7.1f  %-8d %s' % r)
  return 0


if __name__ == '__main__':
  sys.exit(main())
