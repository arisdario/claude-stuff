#!/usr/bin/env python3
"""pyprof.py - a tiny sampling profiler for one openpilot Python process (controlsd), about 1 % overhead.

Started from inside the process (tools/patch_controlsd_profile.py adds the hook, active only when the trigger file
exists): a background thread looks 100 times a second at the MAIN thread's stack (Python's thread switch interval is
0.5 ms during the window, so a look can land anywhere, not only at the waits). Looks that land in a waiting call
(WAIT_FUNCS: waiting for CAN data / sleeping) are counted apart; the rest show where the CPU goes. After
`duration` seconds it writes a ranked report and renames the trigger file to *.done (so the next start is normal).

Report: where the CPU goes, by
  * SELF   - the function the CPU was executing (the hot spot itself)
  * TOTAL  - every function on the stack (a function plus everything it calls), e.g. carcontroller.update
  * FILE   - per source file
plus the process's measured CPU use over the window.
"""
import collections
import os
import sys
import threading
import time

SWITCH_S = 0.0005     # Python thread switch interval while sampling (default 0.005)
# the main thread is WAITING (not using CPU) when its innermost Python function is one of these: controlsd waits for
# the next CAN data in cereal.messaging.drain_sock_raw(wait_for_one=True); Ratekeeper / time.sleep pacing
WAIT_FUNCS = {'drain_sock_raw', 'drain_sock', 'recv_one', 'recv_sock', 'keep_time', 'sleep', 'wait', 'poll'}


def _state_and_cpu(pid):
  """('R'|'S'|..., cpu seconds) of the process's main thread (tid == pid), (None, None) if unreadable"""
  try:
    st = open('/proc/self/task/%d/stat' % pid).read().rsplit(')', 1)[1].split()
    return st[0], (int(st[11]) + int(st[12])) / float(os.sysconf('SC_CLK_TCK'))
  except Exception:
    return None, None


def _short(code):
  f = code.co_filename
  for root in ('/data/openpilot/', '/data/pythonpath/'):
    if f.startswith(root):
      f = f[len(root):]
  return '%s:%s' % (f, code.co_name)


class Sampler(object):
  def __init__(self, out, duration=60., hz=100., trigger=None, main_ident=None, state_fn=None, wait_funcs=None):
    self.out, self.duration, self.dt, self.trigger = out, duration, 1. / hz, trigger
    self.main = main_ident if main_ident is not None else threading.main_thread().ident
    self.pid = os.getpid()
    self.state_fn = state_fn or (lambda: _state_and_cpu(self.pid))
    self.self_c, self.total_c, self.file_c = collections.Counter(), collections.Counter(), collections.Counter()
    self.lines_c = collections.Counter()
    self.n_run = self.n_all = 0
    self.n_wait = 0
    self.wait_funcs = set(wait_funcs) if wait_funcs is not None else WAIT_FUNCS

  def sample_once(self):
    """one look at the main thread: counted unless it is inside a known waiting call (WAIT_FUNCS)"""
    self.n_all += 1
    frame = sys._current_frames().get(self.main)
    if frame is None:
      return
    if frame.f_code.co_name in self.wait_funcs:
      self.n_wait += 1
      return
    self.n_run += 1
    seen = set()
    leaf = True
    while frame is not None:
      name = _short(frame.f_code)
      if leaf:
        self.self_c[name] += 1
        self.lines_c['%s:%d' % (name, frame.f_lineno)] += 1
        self.file_c[name.rsplit(':', 1)[0]] += 1
        leaf = False
      if name not in seen:
        self.total_c[name] += 1
        seen.add(name)
      frame = frame.f_back

  def run(self):
    # Python hands its lock to this thread only at switch points: by default every 5 ms or when the main thread waits
    # (sleep / messaging) - so most looks would land on the waits. 0.5 ms during the window makes the looks fair.
    old_switch = sys.getswitchinterval()
    sys.setswitchinterval(SWITCH_S)
    try:
      self._run()
    finally:
      sys.setswitchinterval(old_switch)

  def _run(self):
    _, cpu0 = self.state_fn()
    t0 = time.monotonic()
    nxt = t0
    while time.monotonic() - t0 < self.duration:
      try:
        self.sample_once()
      except Exception:
        pass
      nxt += self.dt
      time.sleep(max(0., nxt - time.monotonic()))
    _, cpu1 = self.state_fn()
    wall = time.monotonic() - t0
    self.write(wall, (cpu1 - cpu0) if (cpu0 is not None and cpu1 is not None) else None)

  def report(self, wall, cpu_s, top=40):
    n = max(1, self.n_run)
    lines = ['controlsd CPU profile, %s, %.0f s: %d looks, %d busy, %d waiting (%s)' % (
             time.strftime('%Y-%m-%d %H:%M:%S'), wall, self.n_all, self.n_run, self.n_wait, ', '.join(sorted(self.wait_funcs)))]
    if cpu_s is not None:
      lines.append('main thread CPU: %.1f s in %.0f s = %.0f %% of one core' % (cpu_s, wall, 100. * cpu_s / max(wall, 1e-6)))
    lines.append('percentages below = share of the busy looks (= how the main thread\'s CPU time is split)')
    for title, c in (('SELF (the hot spot itself)', self.self_c), ('TOTAL (function + everything it calls)', self.total_c),
                     ('FILE (self time per source file)', self.file_c), ('LINES (hottest lines)', self.lines_c)):
      lines += ['', '== %s' % title]
      for k, v in c.most_common(top if 'LINES' not in title else 25):
        lines.append('%5.1f %%  %6d  %s' % (100. * v / n, v, k))
    return '\n'.join(lines) + '\n'

  def write(self, wall, cpu_s):
    try:
      with open(self.out, 'w') as f:
        f.write(self.report(wall, cpu_s))
    except Exception:
      pass
    if self.trigger and os.path.exists(self.trigger):
      try:
        os.rename(self.trigger, self.trigger + '.done')
      except Exception:
        pass


def start(out, duration=60., trigger=None):
  """called from the profiled process's main thread; returns the (daemon) thread"""
  s = Sampler(out, duration, trigger=trigger)
  th = threading.Thread(target=s.run, name='pyprof', daemon=True)
  th.start()
  return th
