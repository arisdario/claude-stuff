#!/usr/bin/env python3
"""
bench_started.py - BENCH ONLY: makes openpilot go "onroad" on an EON with no car and no panda, so the normal UI shows
the road camera with the model's path and lanes (point the camera at a screen playing a dashcam video).

How: thermald starts the onroad processes when the panda's `health` message says ignition is on. With no panda,
boardd publishes `hwType = unknown` and never ignition (its STARTED switch only works with a panda attached), so this
script publishes `health` itself: ignition on, a black panda type, 2 messages a second like boardd.
openpilot must run with NOBOARD=1 (no pandad/boardd), otherwise two programs would publish `health`.
Ctrl+C sends ignition OFF a few times before exiting (thermald keeps the last state it saw).

Steps (README "Bench test with the camera on screen"):
  tmux kill-session -t comma; sleep 2; pkill -f boardd; pkill -f pandad
  cd /data/openpilot && tmux new -d -s bench 'NOBOARD=1 ./launch_openpilot.sh'
  PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_started.py
  ... Ctrl+C when done (puts your car calibration back; it is also restored at the next start if a run was cut off), then reboot to go back to normal.
Nothing reaches a car: without boardd there is no CAN at all.
"""
import argparse
import glob
import json
import os
import signal
import sys
import time

import cereal.messaging as messaging
from cereal import log


def panda_on_usb():
  for f in glob.glob('/sys/bus/usb/devices/*/idVendor'):
    try:
      d = os.path.dirname(f)
      if open(f).read().strip() == 'bbaa' and open(os.path.join(d, 'idProduct')).read().strip() == 'ddcc':
        return True
    except Exception:
      pass
  return False


def panda_processes():
  """real boardd / pandad processes as (pid, command line). Reads /proc directly: Android's pgrep -f can match its
  own command line, which made the old check report boardd when it was not running."""
  found, me = [], {os.getpid(), os.getppid()}
  for d in glob.glob('/proc/[0-9]*'):
    try:
      pid = int(os.path.basename(d))
      if pid in me:
        continue
      args = [a for a in open(d + '/cmdline', 'rb').read().split(b'\0') if a]
      if not args:
        continue
      argv = [a.decode('utf8', 'replace') for a in args]
      if os.path.basename(argv[0]) in ('boardd', '_boardd') or 'selfdrive.pandad' in argv:
        found.append((pid, ' '.join(argv)))
    except Exception:
      pass
  return found


def health_msg(ignition):
  m = messaging.new_message()
  m.init('health')
  h = m.health
  h.ignitionLine = ignition
  h.ignitionCan = False
  h.hwType = log.HealthData.HwType.blackPanda       # not 'unknown': thermald treats that as "panda gone"
  h.usbPowerMode = log.HealthData.UsbPowerMode.cdp  # charging, not 'client'
  h.voltage = 12600                                 # car battery mV: above thermald's low-voltage cut-off
  h.controlsAllowed = False
  return m


CALIB_KEY = 'CalibrationParams'
CALIB_BACKUP = '/data/bench_calibration_backup.json'      # your car's calibration while the bench test runs
NEUTRAL = json.dumps({'vanishing_point': [582.0, 437.0], 'valid_blocks': 50})   # straight ahead (image centre)


def calib_restore(params):
  """put the car's calibration back (also run at start, in case a previous bench run did not finish)"""
  if not os.path.exists(CALIB_BACKUP):
    return
  saved = open(CALIB_BACKUP).read()
  if saved == 'NONE':
    params.delete(CALIB_KEY)
  else:
    params.put(CALIB_KEY, saved.encode('utf8'))
  os.remove(CALIB_BACKUP)
  print('restored your car calibration: %s' % (saved if saved != 'NONE' else '(none)'))


def calib_neutral(params):
  cur = params.get(CALIB_KEY)
  with open(CALIB_BACKUP, 'w') as f:
    f.write(cur.decode('utf8') if cur else 'NONE')
  params.put(CALIB_KEY, NEUTRAL.encode('utf8'))
  print('bench calibration: straight ahead (your car calibration saved in %s, restored on Ctrl+C)' % CALIB_BACKUP)


LIVE_KEY = 'LiveParameters'
LIVE_BACKUP = '/data/bench_liveparameters_backup.json'    # the car's learned steering ratio / tyre stiffness while the bench test runs


def live_restore(params):
  """put the car's learned vehicle parameters back (also run at start, in case a previous bench run did not finish).
  WHY: the bench replays real steering angles and speeds while the camera sees a still scene, so paramsd learns nonsense (a huge
  steering ratio, a tiny tyre stiffness) and SAVES it; the next real drive then starts with a doubled steering gain and oscillates."""
  if not os.path.exists(LIVE_BACKUP):
    return
  saved = open(LIVE_BACKUP).read()
  if saved == 'NONE':
    params.delete(LIVE_KEY)
  else:
    params.put(LIVE_KEY, saved.encode('utf8'))
  os.remove(LIVE_BACKUP)
  print('restored your car\'s learned vehicle parameters: %s' % (saved if saved != 'NONE' else '(none)'))


def live_backup(params):
  cur = params.get(LIVE_KEY)
  with open(LIVE_BACKUP, 'w') as f:
    f.write(cur.decode('utf8') if cur else 'NONE')
  print('your learned vehicle parameters are saved in %s and put back on Ctrl+C (the replay would otherwise corrupt them)' % LIVE_BACKUP)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--car-calib', action='store_true',
                  help="keep the car's calibration (default: straight ahead, right for a video recorded by an EON)")
  ap.add_argument('--restore', action='store_true',
                  help="only put back the saved calibration and learned vehicle parameters, then exit (use it if a bench run ended without Ctrl+C, e.g. after a reboot)")
  args = ap.parse_args()
  if args.restore:
    from common.params import Params
    p_ = Params()
    calib_restore(p_)
    live_restore(p_)
    print('done (nothing was saved from a bench run)' if not os.path.exists(LIVE_BACKUP) else '')
    return
  if panda_on_usb():
    sys.exit('STOP: a panda is connected over USB. This is for the bench only - unplug it.')
  procs = panda_processes()
  if procs:
    for pid, cmd in procs:
      print('  running: pid %d  %s' % (pid, cmd))
    sys.exit('STOP: boardd/pandad is running and publishes `health` too. Stop it (kill %s) and start openpilot with '
             'NOBOARD=1 (see the top of this file).' % ' '.join(str(p) for p, _ in procs))
  from common.params import Params
  params = Params()
  calib_restore(params)                    # left over from an interrupted run
  live_restore(params)                     # left over from an interrupted run
  live_backup(params)                      # paramsd keeps learning during the replay and saves it: protect the car's values
  if not args.car_calib:
    calib_neutral(params)                  # before ignition: calibrationd reads it when it starts
  def _stop(*_):
    raise KeyboardInterrupt
  signal.signal(signal.SIGTERM, _stop)     # SSH dropped / killed: still switch ignition off and restore calibration
  signal.signal(signal.SIGHUP, _stop)
  pm = messaging.PubMaster(['health'])
  print('publishing ignition ON (bench). The EON should switch to the camera view within ~10 s. Ctrl+C to stop.')
  try:
    while True:
      pm.send('health', health_msg(True))
      time.sleep(0.5)
  except KeyboardInterrupt:
    pass
  print('\nignition OFF ...')
  for _ in range(10):
    pm.send('health', health_msg(False))
    time.sleep(0.5)
  calib_restore(params)
  live_restore(params)
  print('done. Reboot to go back to normal (openpilot with boardd).')


if __name__ == '__main__':
  main()
