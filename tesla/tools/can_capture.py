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
import bisect
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


def state_at(timeline, t, _times=None):
  """lever state at time t, or None inside SETTLE_S after a change (or before the first message)"""
  times = _times if _times is not None else [tc for tc, _ in timeline]
  i = bisect.bisect_right(times, t) - 1
  if i < 0 or t - timeline[i][0] < SETTLE_S:
    return None
  return timeline[i][1]


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
  # HIGH-BEAM STATUS (found 16 Nov 2026): 0x30A byte 0 on bus 0 - bit 3 = high beams on, bit 2 = flash - set by the
  # module that acts on the lever (6-31 ms after STW_ACTN_RQ). Its timeline, and the stalk frames openpilot sent
  # (bus 128+: e.g. the ahb_stalk.py dip test), show whether the car accepted an injected lever change.
  hb, last = [], None
  for t, bus, addr, data in frames:
    if bus == 0 and addr == 0x30A and data:
      v = (data[0] >> 2) & 3
      if v != last:
        hb.append((t, v)); last = v
  out('high-beam status 0x30A (on/flash bits): ' + (', '.join('%.2f s %s' % (t, {0: 'off', 2: 'ON', 1: 'FLASH', 3: 'on+flash'}[v]) for t, v in hb) or '-'))
  sent = [(t, lever_of(data)) for t, bus, addr, data in frames if bus >= 128 and addr == LEVER_ADDR]
  if sent:
    out('stalk frames SENT BY OPENPILOT: %d, %.2f-%.2f s, lever values %s' % (len(sent), sent[0][0], sent[-1][0],
        dict(collections.Counter(NAMES.get(v, v) for _, v in sent))))
  else:
    n_tx = sum(1 for f in frames if f[1] >= 128)
    out('stalk frames SENT BY OPENPILOT: none (all frames openpilot sent: %d) - %s' % (n_tx,
        'is controlsd running? ps -ef | grep controlsd' if n_tx < 100 else 'the test did not send'))
  # every bit of every (bus, address): share of frames with the bit set, per lever state. Fast on the EON: first count
  # the DISTINCT payloads per (bus, address, state) - one dict step per frame - then expand those to bits.
  times = [tc for tc, _ in tl]
  payloads = collections.defaultdict(collections.Counter)            # (bus, addr, state) -> Counter(payload)
  for t, bus, addr, data in frames:
    if addr in (LEVER_ADDR, 0x219):                 # the lever itself and the panda's copy of it for the radar
      continue
    st = state_at(tl, t, times)
    if st is not None:
      payloads[(bus, addr, st)][data] += 1
  acc = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))   # (bus, addr, byte, bit) -> state -> [set, n]
  for (bus, addr, st), cnt in payloads.items():
    for data, n in cnt.items():
      for i, b in enumerate(data):
        for k in range(8):
          c = acc[(bus, addr, i, k)][st]
          c[0] += n * ((b >> k) & 1)
          c[1] += n
  found = []
  for key, d in acc.items():
    p = {s: c[0] / c[1] for s, c in d.items() if c[1] >= 3}
    if 0 in p and 1 in p and abs(p[1] - p[0]) >= 0.9:
      found.append(('follows OUT (high on)', key, p))
    elif 0 in p and 2 in p and abs(p[2] - p[0]) >= 0.9:
      found.append(('follows PULL (flash)', key, p))
  out('bits that follow the lever (other than the lever message itself): %d' % len(found))
  # TIMING: how long after each lever change does the bit change too? A status from another module (e.g. the body
  # controller reporting the lamps) lags by tens of ms; a copy of the lever changes within a few ms.
  by_key = collections.defaultdict(list)
  for t, bus, addr, data in frames:
    by_key[(bus, addr)].append((t, data))
  for what, (bus, addr, i, k), p in found:
    if bus >= 128:
      continue
    want = 1 if 'OUT' in what else 2
    seq, delays = by_key[(bus, addr)], []
    for j, (tc, v) in enumerate(tl[1:], 1):
      if v != want and tl[j - 1][1] != want:
        continue                                  # only changes into or out of the state this bit follows
      target = 1 if v == want else 0
      for t, data in seq:
        if t >= tc - 0.5 and len(data) > i and ((data[i] >> k) & 1) == target and t >= tc - 0.05:
          delays.append((t - tc) * 1000.)
          break
    rate = (len(seq) - 1) / max(1e-6, seq[-1][0] - seq[0][0]) if len(seq) > 1 else 0.
    out('  timing  bus %d 0x%03x byte %d bit %d (%.0f messages/s): changes %s ms after the lever' % (
      bus, addr, i, k, rate, ', '.join('%+.0f' % d for d in delays) or '-'))
  for what, (bus, addr, i, k), p in sorted(found, key=lambda x: (x[1][0], x[1][1], x[1][2], x[1][3]))[:40]:
    out('  %-22s bus %d  0x%03x  byte %d bit %d   set: normal %.2f  out %s  pulled %s%s' % (
      what, bus, addr, i, k, p.get(0, float('nan')), '%.2f' % p[1] if 1 in p else '-', '%.2f' % p[2] if 2 in p else '-',
      '   (SENT BY OPENPILOT, not the car)' if bus >= 128 else ''))
  extras(frames, out)
  return found


TURN_NAMES = {0: 'idle', 1: 'LEFT', 2: 'RIGHT', 3: 'SNA'}


def extras(frames, out=print):
  """for the flash-HOLD test: (1) the turn-signal lever (STW_ACTN_RQ byte 2 bits 0-1) - its changes, the bits on any
  bus that follow it (blinking lamps give ~0.5, so >= 0.4 is reported), and what openpilot's own stalk frames carried;
  (2) message IDs that appear only later in the capture (a fault / diagnostic message would)."""
  tl, last = [], None
  for t, bus, addr, data in frames:
    if bus == LEVER_BUS and addr == LEVER_ADDR and len(data) > 2:
      v = data[2] & 3
      if v != last:
        tl.append((t, v)); last = v
  out('turn-signal lever (real): ' + ', '.join('%.2f s %s' % (t, TURN_NAMES[v]) for t, v in tl))
  sent = [(t, data[2] & 3) for t, bus, addr, data in frames if bus >= 128 and addr == LEVER_ADDR and len(data) > 2]
  if sent:
    out('turn-signal values in the stalk frames SENT BY OPENPILOT: %s' % dict(collections.Counter(TURN_NAMES[v] for _, v in sent)))
  if len(tl) > 1:
    times = [tc for tc, _ in tl]
    payloads = collections.defaultdict(collections.Counter)
    for t, bus, addr, data in frames:
      if bus >= 128 or addr in (LEVER_ADDR, 0x219):
        continue
      st = state_at(tl, t, times)
      if st is not None:
        payloads[(bus, addr, st)][data] += 1
    acc = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))
    for (bus, addr, st), cnt in payloads.items():
      for data, n in cnt.items():
        for i, b in enumerate(data):
          for k in range(8):
            c = acc[(bus, addr, i, k)][st]
            c[0] += n * ((b >> k) & 1); c[1] += n
    hits = []
    for (bus, addr, i, k), d in acc.items():
      if bus == 0 and addr == 0x30A and i == 0 and k in (2, 3):
        continue                                     # the known high-beam bits (held during a HOLD test, not the turn signal)
      p = {st: c[0] / c[1] for st, c in d.items() if c[1] >= 3}
      for st in (1, 2):
        if 0 in p and st in p and abs(p[st] - p[0]) >= 0.4:
          hits.append((bus, addr, i, k, st, p))
    out('bits that follow the turn-signal lever: %d' % len(hits))
    for bus, addr, i, k, st, p in sorted(hits)[:20]:
      out('  %-5s bus %d  0x%03x  byte %d bit %d   set: idle %.2f  left %s  right %s%s' % (
        TURN_NAMES[st], bus, addr, i, k, p[0], '%.2f' % p[1] if 1 in p else '-', '%.2f' % p[2] if 2 in p else '-',
        '   (blinks)' if abs(p[st] - p[0]) < 0.9 else ''))
  first = {}
  for t, bus, addr, data in frames:
    if bus < 128 and (bus, addr) not in first:
      first[(bus, addr)] = t
  late = sorted((t, bus, addr) for (bus, addr), t in first.items() if t > 3.0)
  out('message IDs that first appeared after 3 s: ' + (', '.join('bus %d 0x%03x at %.1f s' % (b, a, t) for t, b, a in late[:20]) or 'none'))


def capture(seconds, guide):
  import cereal.messaging as messaging
  sock = messaging.sub_sock('can')
  frames, t0 = [], time.monotonic()
  try:
    _capture_loop(messaging, sock, frames, t0, seconds, guide)
  except KeyboardInterrupt:                        # Ctrl+C: stop early, but keep and analyse what was recorded
    print('\n  stopped early (Ctrl+C) - saving and analysing the %d frames recorded' % len(frames), flush=True)
  print()
  return frames


def _capture_loop(messaging, sock, frames, t0, seconds, guide):
  said, last_lever, last_print, warned = -1, None, 0., False
  while True:
    t = time.monotonic() - t0
    if t >= seconds:
      break
    for i, (ts, text) in enumerate(guide):
      if i > said and t >= ts:
        said = i
        print('\n>>> %4.0f s: %s' % (t, text), flush=True)
    msgs = messaging.drain_sock(sock)              # never blocks (16 Nov 2026: waiting here froze the prompts with no CAN)
    if not msgs:
      time.sleep(0.005)
    if t > 3. and not frames and not warned:
      warned = True
      print('\n  NO CAN DATA after 3 s - is openpilot running? (boardd provides the CAN): ps -A | grep boardd', flush=True)
    for m in msgs:
      tm = time.monotonic() - t0
      for c in m.can:
        frames.append((tm, c.src, c.address, bytes(c.dat)))
        if c.src == LEVER_BUS and c.address == LEVER_ADDR:
          last_lever = lever_of(bytes(c.dat))
    if t - last_print >= 1.0:
      last_print = t
      sys.stdout.write('\r  %4.0f s  frames %7d  lever: %-16s' % (t, len(frames), NAMES.get(last_lever, '-')))
      sys.stdout.flush()


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--guide', choices=['hibeam'], help='print what to do when (hibeam: on, off, flash x2)')
  ap.add_argument('--seconds', type=float, default=None)
  ap.add_argument('--name', default=None)
  ap.add_argument('--analyse', metavar='FILE', help='analyse a saved capture (.csv.gz) again, no recording')
  a = ap.parse_args()
  if a.analyse:
    frames = []
    with gzip.open(a.analyse, 'rt') as f:
      next(f)
      for line in f:
        t, bus, addr, data = line.strip().split(',')
        frames.append((float(t), int(bus), int(addr, 16), bytes.fromhex(data)))
    print('%d frames from %s' % (len(frames), a.analyse))
    analyse(frames)
    return
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
  print('analysing ...', flush=True)
  analyse(frames)


if __name__ == '__main__':
  main()
