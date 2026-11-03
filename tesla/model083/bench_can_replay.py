#!/usr/bin/env python3
"""
bench_can_replay.py - BENCH ONLY: replays a logged CAN capture (the "[CAN->CEREAL] ID=0x.. DLC=n BUS=0,2 DATA=.." lines of the old tesla_can_bridge.log) into openpilot's local `can` channel,
so on an EON with NO car and NO panda openpilot sees a real drive's CAN frames - including vehicle speed (DI_torque2 0x118) - while you point the camera at a monitor (see README "Bench test with the camera on screen").

It REBUILDS the old receive-only bridge's output side from its log; the original code was lost, so this is new and has not been run on an EON yet.
It only publishes to the EON's internal cereal socket. It never opens a CAN interface, never talks to a panda, never publishes `sendcan`. It refuses to run if a panda is on USB or boardd/pandad is running.

Use together with bench_started.py (which publishes the ignition):
  tmux kill-session -t comma; sleep 2; pkill -f boardd; pkill -f pandad
  cd /data/openpilot && tmux new -d -s bench 'NOBOARD=1 ./launch_openpilot.sh'
  PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_started.py          # session 1: ignition
  PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_can_replay.py /data/tesla_can_bridge.log   # session 2: the CAN frames (speed)
  (check the log first, on any computer:  python3 bench_can_replay.py tesla_can_bridge.log --dry)
The log has no timestamps, so the pace is a setting: --fps frames per second (default 100). The speed in the log goes 0 .. ~66 mph over the whole file.
"""
import argparse
import glob
import os
import re
import signal
import sys
import time

LINE = re.compile(r'\[CAN->CEREAL\] ID=0x([0-9A-Fa-f]+) DLC=(\d+) BUS=([\d,]+) DATA=([0-9A-Fa-f]*)')


def parse_log(path):
  """-> [(address, data bytes, [buses])] in file order; malformed lines (and DLC/data mismatches) are skipped and counted"""
  frames, bad = [], 0
  with open(path, errors='replace') as f:
    for line in f:
      m = LINE.search(line)
      if not m:
        continue
      try:
        dat = bytes.fromhex(m.group(4))
        if len(dat) != int(m.group(2)) or len(dat) > 8:
          bad += 1
          continue
        frames.append((int(m.group(1), 16), dat, [int(b) for b in m.group(3).split(',')]))
      except ValueError:
        bad += 1
  return frames, bad


def di_speed_mph(addr, dat):
  """DI_torque2 (0x118) DI_vehicleSpeed: bits 16..27, 0.05 mph/bit, offset -25 mph (from opendbc tesla_can.dbc); None for other frames"""
  if addr != 0x118 or len(dat) < 4:
    return None
  return (dat[2] | ((dat[3] & 0xF) << 8)) * 0.05 - 25.0


def speed_summary(frames):
  sp = [s for s in (di_speed_mph(a, d) for a, d, _ in frames) if s is not None]
  return (min(sp), max(sp), len(sp)) if sp else None


def batches(frames, fps, tick=0.01):
  """yield (frames_for_this_tick) so that on average `fps` frames are published per second, in file order"""
  per_tick = max(1, int(round(fps * tick)))
  for i in range(0, len(frames), per_tick):
    yield frames[i:i + per_tick]


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
  found, me = [], {os.getpid(), os.getppid()}
  for d in glob.glob('/proc/[0-9]*'):
    try:
      pid = int(os.path.basename(d))
      if pid in me:
        continue
      argv = [a.decode('utf8', 'replace') for a in open(d + '/cmdline', 'rb').read().split(b'\0') if a]
      if argv and (os.path.basename(argv[0]) in ('boardd', '_boardd') or 'selfdrive.pandad' in argv):
        found.append((pid, ' '.join(argv)))
    except Exception:
      pass
  return found


def can_msg(messaging, frames, buses_override=None):
  m = messaging.new_message()
  n = sum(len(buses_override or b) for _, _, b in frames)
  m.init('can', n)
  k = 0
  for addr, dat, buses in frames:
    for bus in (buses_override or buses):
      c = m.can[k]
      c.address, c.busTime, c.dat, c.src = addr, 0, dat, bus
      k += 1
  return m


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('log')
  ap.add_argument('--fps', type=float, default=100.0, help='frames per second to publish (the log has no timestamps)')
  ap.add_argument('--once', action='store_true', help='play the log once instead of looping')
  ap.add_argument('--buses', default='', help='override the bus numbers, e.g. 0,2 (default: as in the log)')
  ap.add_argument('--dry', action='store_true', help='only parse the log and print what is in it (no openpilot needed, nothing published)')
  a = ap.parse_args()
  frames, bad = parse_log(a.log)
  if not frames:
    sys.exit('no CAN frames found in %s' % a.log)
  sp = speed_summary(frames)
  print('%d frames, %d distinct ids (%d malformed lines skipped); DI_vehicleSpeed %s' % (
    len(frames), len(set(f[0] for f in frames)), bad, 'none' if sp is None else '%.1f .. %.1f mph in %d frames' % sp))
  print('pace %.0f frames/s -> one pass takes %.0f s' % (a.fps, len(frames) / a.fps))
  if a.dry:
    return
  if panda_on_usb():
    sys.exit('STOP: a panda is connected over USB. This is for the bench only - unplug it.')
  procs = panda_processes()
  if procs:
    for pid, cmd in procs:
      print('  running: pid %d  %s' % (pid, cmd))
    sys.exit('STOP: boardd/pandad is running. Stop it and start openpilot with NOBOARD=1 (see the top of this file).')
  import cereal.messaging as messaging
  override = [int(b) for b in a.buses.split(',')] if a.buses else None
  pm = messaging.PubMaster(['can'])
  signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
  print('publishing the logged CAN frames to the local `can` channel (nothing reaches any car). Ctrl+C to stop.')
  sent, last_print, p = 0, time.time(), 0
  try:
    while True:
      t0 = time.time()
      for i, chunk in enumerate(batches(frames, a.fps)):
        pm.send('can', can_msg(messaging, chunk, override))
        sent += len(chunk)
        for addr, dat, _ in chunk:
          s = di_speed_mph(addr, dat)
          if s is not None:
            p = s
        time.sleep(max(0.0, t0 + (i + 1) * 0.01 - time.time()))
        if time.time() - last_print >= 2.0:
          print('  %d frames sent, last DI_vehicleSpeed %.1f mph (%.0f km/h)' % (sent, p, p * 1.609344))
          last_print = time.time()
      if a.once:
        break
  except KeyboardInterrupt:
    pass
  print('\nstopped after %d frames.' % sent)


if __name__ == '__main__':
  main()
