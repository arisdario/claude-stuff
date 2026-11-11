#!/usr/bin/env python3
"""param_switch.py - use toggles that ALREADY exist in the EON settings screen as on/off switches for radarweb and the diagnostics logger.

No change to the EON UI/app: the app writes each toggle as a file in /data/params/d/<Name> ("1"/"0"). This watcher polls two of them every 2 s (idle cost ~0):
  switch_param (default /data/web_switch = the on-screen WEB button, see add_web_button.py; or a param name such as IsUploadRawEnabled = "Upload Raw Logs")        1 -> start radarweb      0 -> stop radarweb
  diag_param   (default RecordFront        = "Record and Upload Driver Camera")   1 -> start diagnostics recording   0 -> stop + build zip
  fcw_param    (default IsUploadRawEnabled = "Upload Raw Logs")                    1 -> start the FCW shadow logger   0 -> stop it
               (skipped if it is the same toggle as switch_param or diag_param; set fcw_param=none to disable)
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
    with open(name if name.startswith('/') else os.path.join(params or PARAMS, name), 'rb') as f:
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
  print('watching %s (%d params) - press a toggle in the app; params that change constantly are ignored' % (params, len(prev)))
  sys.stdout.flush()
  hist = {}                                   # name -> change times: anything changing more than 3 times in 10 s is openpilot noise
  while True:
    time.sleep(interval)
    cur = snap()
    now = time.time()
    for n in sorted(set(cur) | set(prev)):
      if cur.get(n) != prev.get(n):
        h = hist.setdefault(n, [])
        h.append(now)
        hist[n] = h = [t for t in h if now - t < 10]
        if len(h) > 3:
          continue
        print('%s: %r -> %r' % (n, (prev.get(n) or '')[:60], (cur.get(n) or '')[:60]))
        sys.stdout.flush()
    prev = cur


def unbind_c920_audio(usb_root='/sys/bus/usb/devices', unbind='/sys/bus/usb/drivers/snd-usb-audio/unbind'):
  """detach snd-usb-audio from the C920's AUDIO interfaces (class 0x01) so its microphone never shows up as an ALSA card (it can take card0 and crash controlsd).
  Only interfaces of device 046d:082d with bInterfaceClass 01 that are bound to snd-usb-audio are touched; the video interfaces (class 0e) are never touched.
  Returns the list of interfaces unbound."""
  done = []
  try:
    devs = os.listdir(usb_root)
  except OSError:
    return done
  for d in devs:
    p = os.path.join(usb_root, d)
    if rd('idVendor', p) != '046d' or rd('idProduct', p) != '082d':
      continue
    try:
      subs = os.listdir(p)
    except OSError:
      continue
    for itf in subs:
      ip = os.path.join(p, itf)
      if ':' not in itf or rd('bInterfaceClass', ip) != '01':
        continue
      drv = os.path.join(ip, 'driver')
      if not os.path.islink(drv) or os.path.basename(os.path.realpath(drv)) != 'snd-usb-audio':
        continue
      try:
        with open(unbind, 'w') as f:
          f.write(itf)
        done.append(itf)
      except OSError:
        pass
  return done


class Switch(object):
  """pure state machine: feed it the current values; it calls the actions on changes only."""

  def __init__(self, c, params=None):
    self.c, self.params = c, params
    self.web = self.diag = self.fcw = None            # last seen value; None = not seen yet

  def step(self, start_web=launcher.start_web, stop_web=launcher.stop_web, diag=launcher.diag,
           start_fcw=launcher.start_fcw, stop_fcw=launcher.stop_fcw):
    wn = self.c.get('switch_param', '/data/web_switch')
    dn = self.c.get('diag_param', 'RecordFront')
    fn = self.c.get('fcw_param', 'IsUploadRawEnabled')
    w, d = rd(wn, self.params), rd(dn, self.params)
    f = rd(fn, self.params) if fn not in ('none', '', wn, dn) else None
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
    if f is not None and f != self.fcw:
      if f == '1':
        acts.append(('FCW logger on', start_fcw(self.c)))
      elif self.fcw is not None:
        acts.append(('FCW logger off', stop_fcw(self.c)))
      self.fcw = f
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
  last_audio = 0.0
  log('param_switch started: radarweb <- %s, diagnostics <- %s, FCW logger <- %s' % (sw.c.get('switch_param', '/data/web_switch'),
      sw.c.get('diag_param', 'RecordFront'), sw.c.get('fcw_param', 'IsUploadRawEnabled')))
  while True:
    try:
      sw.c = launcher.conf()
      for what, res in sw.step():
        log('%s: %s' % (what, str(res).replace('\n', ' | ')[:300]))
      if sw.c.get('c920_audio_off', '0') == '1' and time.time() - last_audio > 5.0:       # opt-in: /data/launcher.conf c920_audio_off=1
        last_audio = time.time()
        for itf in unbind_c920_audio():
          log('c920 audio interface %s unbound from snd-usb-audio (video untouched)' % itf)
    except Exception as e:
      log('error: %r' % (e,))
    time.sleep(2.0)


if __name__ == '__main__':
  main()
