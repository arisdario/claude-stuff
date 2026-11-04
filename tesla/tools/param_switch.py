#!/usr/bin/env python3
"""param_switch.py - use toggles that ALREADY exist in the EON settings screen as on/off switches for radarweb and the diagnostics logger.

No change to the EON UI/app: the app writes each toggle as a file in /data/params/d/<Name> ("1"/"0"). This watcher polls two of them every 2 s (idle cost ~0):
  switch_param (default IsUploadRawEnabled = "Upload Raw Logs")        1 -> start radarweb      0 -> stop radarweb
  diag_param   (default RecordFront        = "Record and Upload Driver Camera")   1 -> start diagnostics recording   0 -> stop + build zip
These two toggles only matter to loggerd/uploader, which are blocked on this EON (/data/no_loggerd, /data/no_uploader), so reusing them is harmless. The labels in the app will
not match: that is the price of not touching the app. Change the names in /data/launcher.conf if your settings screen shows others (find them with --watch-names).

  python3 param_switch.py                 run the watcher (single instance; safe to start twice)
  python3 param_switch.py --watch-names   prints every param that changes: press a toggle in the app and see its name

It never touches openpilot processes; start/stop logic is shared with launcher.py (whitelisted process names, SIGTERM to the exact PID).
"""
import fcntl
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import launcher  # noqa: E402

PARAMS = os.environ.get('PARAMS_DIR', '/data/params/d')
LOG = os.environ.get('PARAM_SWITCH_LOG', '/data/param_switch.log')


def log(msg):
  line = '%s %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg)
  try:
    open(LOG, 'a').write(line)
  except OSError:
    pass
  sys.stdout.write(line)
  sys.stdout.flush()


def rd(name, params=None):
  """value as text; params can be binary (capnp blobs): read bytes, decode leniently, never raise."""
  try:
    with open(os.path.join(params or PARAMS, name), 'rb') as f:
      return f.read(512).decode('utf8', 'replace').strip()
  except (OSError, IOError):          # missing, a directory, or removed while reading
    return None


def watch_names(params=None, interval=1.0):
  params = params or PARAMS
  def snap():
    try:
      return {n: rd(n, params) for n in os.listdir(params)}
    except OSError:
      return {}
  prev = snap()
  print('watching %s (%d params) - press a toggle in the app' % (params, len(prev)))
  sys.stdout.flush()
  while True:
    time.sleep(interval)
    cur = snap()
    for n in sorted(set(cur) | set(prev)):
      if cur.get(n) != prev.get(n):
        print('%s: %r -> %r' % (n, (prev.get(n) or '')[:60], (cur.get(n) or '')[:60]))
        sys.stdout.flush()
    prev = cur


class Switch(object):
  """pure state machine: feed it the current values; it calls the actions on changes only."""

  def __init__(self, c, params=None):
    self.c, self.params = c, params
    self.web = self.diag = None            # last seen value; None = not seen yet

  def step(self, start_web=launcher.start_web, stop_web=launcher.stop_web, diag=launcher.diag):
    wn = self.c.get('switch_param', 'IsUploadRawEnabled')
    dn = self.c.get('diag_param', 'RecordFront')
    w, d = rd(wn, self.params), rd(dn, self.params)
    acts = []
    if w is not None and w != self.web:
      if w == '1':
        acts.append(('radarweb on', start_web(self.c)))
      elif self.web is not None:           # a 0 at startup does not stop a radarweb someone started by hand
        acts.append(('radarweb off', stop_web(self.c)))
      self.web = w
    if d is not None and d != self.diag:
      if d == '1':
        acts.append(('diagnostics on', diag(self.c, 'start')))
      elif self.diag is not None:
        acts.append(('diagnostics off', diag(self.c, 'stop')))
      self.diag = d
    return acts


def main():
  if '--watch-names' in sys.argv:
    watch_names()
    return
  lock = open('/tmp/param_switch.lock', 'w')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except OSError:
    print('already running')
    return
  sw = Switch(launcher.conf())
  log('param_switch started: radarweb <- %s, diagnostics <- %s' % (sw.c.get('switch_param', 'IsUploadRawEnabled'), sw.c.get('diag_param', 'RecordFront')))
  while True:
    try:
      sw.c = launcher.conf()
      for what, res in sw.step():
        log('%s: %s' % (what, str(res).replace('\n', ' | ')[:300]))
    except Exception as e:
      log('error: %r' % (e,))
    time.sleep(2.0)


if __name__ == '__main__':
  main()
