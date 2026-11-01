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
  ... Ctrl+C when done, then reboot to go back to normal.
Nothing reaches a car: without boardd there is no CAN at all.
"""
import glob
import os
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


def main():
  if panda_on_usb():
    sys.exit('STOP: a panda is connected over USB. This is for the bench only - unplug it.')
  procs = panda_processes()
  if procs:
    for pid, cmd in procs:
      print('  running: pid %d  %s' % (pid, cmd))
    sys.exit('STOP: boardd/pandad is running and publishes `health` too. Stop it (kill %s) and start openpilot with '
             'NOBOARD=1 (see the top of this file).' % ' '.join(str(p) for p, _ in procs))
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
  print('done. Reboot to go back to normal (openpilot with boardd).')


if __name__ == '__main__':
  main()
