#!/usr/bin/env python3
"""can_capture.py - record ALL raw CAN frames for a short time (read-only) and, for the high beams, find which bits
follow the light lever.

Why: the radarweb logs (1 sample/s, decoded) showed only the lever itself (STW_ACTN_RQ 0x45, HiBmLvr_Stat) changing;
the body controller's lamp status (BODY_R1 0x283) is frozen on the bus the panda listens to. A full-rate capture on
every bus while you work the lever settles it: is "high beams ON" reported anywhere, and how does the lever behave
(latched? flash? message rate, counter)?

  car on, in P, headlights on, openpilot running (boardd gives the CAN), parked:
  PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/tools/can_capture.py --guide hibeam
  (free-form: --seconds 30, no prompts)

It only LISTENS (openpilot's own 'can' stream). Writes /data/can_capture_<name>_<time>.csv.gz (t, bus, address, data)
and prints the analysis. Copy the file to your computer to keep it: scp -P 8022 root@<eon>:/data/can_capture_*.csv.gz .
"""
import argparse
import collections
import gzip
import os
import sys
import time

GUIDE_HIBEAM = [            # (from s, what to do) - printed when its time comes
  (0, 'lever in its NORMAL position (low beams), do not touch anything'),
  (8, 'PUSH the lever OUT: high beams ON - leave it there'),
  (18, 'bring the lever back to NORMAL: high beams OFF'),
  (26, 'FLASH: pull the lever toward you for about 1 s, then let go'),
  (31, 'FLASH again: pull for about 1 s, let go'),
  (36, 'lever NORMAL, do not touch anything'),
]
GUIDE_HIBEAM_S = 42
LEVER_ADDR, LEVER_BUS = 0x45, 0          # STW_ACTN_RQ; HiBmLvr_Stat = bits 18-19 = byte 2, bits 2-3
SETTLE_S = 0.6                           # ignore this long after each lever change (the lamp follows with a delay)
NAMES = {0: 'NORMAL', 1: 'OUT (high on)', 2: 'PULLED (flash)', 3: 'SNA'}


def lever_of(data):
  return (data[2] >> 2) & 3 if len(data) > 2 else None


def lever_timeline(frames):
  """[(t, lever)] at every change, from the real stalk messages on bus 0"""
  out = []
  for t, bus, addr, data in frames:
    if bus == LEVER_BUS and addr == LEVER_ADDR:
      v = lever_of(data)
      if v is not None and (not out or out[-1][1] != v):
        out.append((t, v))
  return out


def state_at(timeline, t):
  """lever state at time t, or None inside SETTLE_S after a change (or before the first message)"""
  cur = None
  for tc, v in timeline:
    if tc > t:
      break
    cur = (tc, v)
  if cur is None or t - cur[0] < SETTLE_S:
    return None
  return cur[1]


def analyse(frames, out=print):
  tl = lever_timeline(frames)
  out('lever changes (STW_ACTN_RQ 0x45 on bus 0): ' + ', '.join('%.1f s %s' % (t, NAMES.get(v, v)) for t, v in tl))
  lever = [f for f in frames if f[1] == LEVER_BUS and f[2] == LEVER_ADDR]
  if len(lever) > 1:
    rate = (len(lever) - 1) / max(1e-6, lever[-1][0] - lever[0][0])
    cnt = [f[3][6] >> 4 for f in lever if len(f[3]) > 6]
    steps = collections.Counter((b - a) % 16 for a, b in zip(cnt, cnt[1:]))
    out('0x45: %.1f messages/s, counter steps %s' % (rate, dict(steps)))
    # how long each lever position was held (latched vs momentary)
    held = ['%s %.1f s' % (NAMES.get(v, v), (tl[i + 1][0] if i + 1 < len(tl) else frames[-1][0]) - t) for i, (t, v) in enumerate(tl)]
    out('lever held: ' + ', '.join(held))
  # every bit of every (bus, address): share of frames with the bit set, per lever state
  acc = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))   # (bus, addr, byte, bit) -> state -> [set, n]
  for t, bus, addr, data in frames:
    if addr in (LEVER_ADDR, 0x219):                 # the lever itself and the panda's copy of it for the radar
      continue
    st = state_at(tl, t)
    if st is None:
      continue
    for i, b in enumerate(data):
      for k in range(8):
        c = acc[(bus, addr, i, k)][st]
        c[0] += (b >> k) & 1
        c[1] += 1
  found = []
  for key, d in acc.items():
    p = {s: c[0] / c[1] for s, c in d.items() if c[1] >= 3}
    if 0 in p and 1 in p and abs(p[1] - p[0]) >= 0.9:
      found.append(('follows OUT (high on)', key, p))
    elif 0 in p and 2 in p and abs(p[2] - p[0]) >= 0.9:
      found.append(('follows PULL (flash)', key, p))
  out('bits that follow the lever (other than the lever message itself): %d' % len(found))
  for what, (bus, addr, i, k), p in sorted(found, key=lambda x: (x[1][0], x[1][1], x[1][2], x[1][3]))[:40]:
    out('  %-22s bus %d  0x%03x  byte %d bit %d   set: normal %.2f  out %s  pulled %s%s' % (
      what, bus, addr, i, k, p.get(0, float('nan')), '%.2f' % p[1] if 1 in p else '-', '%.2f' % p[2] if 2 in p else '-',
      '   (SENT BY OPENPILOT, not the car)' if bus >= 128 else ''))
  return found


def capture(seconds, guide):
  import cereal.messaging as messaging
  sock = messaging.sub_sock('can')
  frames, t0, said = [], time.monotonic(), -1
  last_lever, last_print = None, 0.
  while True:
    t = time.monotonic() - t0
    if t >= seconds:
      break
    for i, (ts, text) in enumerate(guide):
      if i > said and t >= ts:
        said = i
        print('\n>>> %4.0f s: %s' % (t, text), flush=True)
    for m in messaging.drain_sock(sock, wait_for_one=True):
      tm = time.monotonic() - t0
      for c in m.can:
        frames.append((tm, c.src, c.address, bytes(c.dat)))
        if c.src == LEVER_BUS and c.address == LEVER_ADDR:
          last_lever = lever_of(bytes(c.dat))
    if t - last_print >= 1.0:
      last_print = t
      sys.stdout.write('\r  %4.0f s  frames %7d  lever: %-16s' % (t, len(frames), NAMES.get(last_lever, '-')))
      sys.stdout.flush()
  print()
  return frames


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--guide', choices=['hibeam'], help='print what to do when (hibeam: on, off, flash x2)')
  ap.add_argument('--seconds', type=float, default=None)
  ap.add_argument('--name', default=None)
  a = ap.parse_args()
  guide = GUIDE_HIBEAM if a.guide == 'hibeam' else []
  seconds = a.seconds or (GUIDE_HIBEAM_S if a.guide == 'hibeam' else 30.)
  name = a.name or (a.guide or 'free')
  print('recording ALL CAN frames for %.0f s (read-only) ...' % seconds)
  frames = capture(seconds, guide)
  path = '/data/can_capture_%s_%s.csv.gz' % (name, time.strftime('%Y%m%d_%H%M%S'))
  with gzip.open(path, 'wt') as f:
    f.write('t,bus,addr,data\n')
    for t, bus, addr, data in frames:
      f.write('%.4f,%d,0x%03x,%s\n' % (t, bus, addr, data.hex()))
  buses = collections.Counter(b for _, b, _, _ in frames)
  print('saved %d frames to %s (frames per bus: %s)' % (len(frames), path, dict(sorted(buses.items()))))
  analyse(frames)


if __name__ == '__main__':
  main()
