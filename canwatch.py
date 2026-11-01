#!/usr/bin/env python3
"""
canwatch.py - find which CAN bits follow something you switch by hand (e.g. high beam on/off). LISTENS ONLY.

  cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/canwatch.py
  (radarweb or openpilot can keep running: this only subscribes to the 'can' messages boardd already publishes)

Start with the thing OFF. Each time you press Enter you switch it (OFF -> ON -> OFF ...): press Enter, then switch it
within a second. Do at least 3 ON and 3 OFF phases of 3-5 s each. Type q + Enter to finish.
Prints every bit (per bus and ID) that was steady in each phase and different between ON and OFF phases - the signals
that follow your switch - plus multi-bit fields whose values never overlap. Saves the result to /data/canwatch_*.json.
Frames sent by the panda itself (bus >= 128) are ignored.
"""
import json
import sys
import threading
import time

SETTLE = 1.0          # s ignored after each Enter (time to move the stalk, light controller reaction)
MIN_PHASES = 2        # each of ON and OFF at least this many times
STEADY = 0.97         # share of frames a bit must keep its value within a phase
LOOSE = 0.75          # 'weaker matches': mostly follows (e.g. the switch was late in a phase)
STALK = (0, 0x45, 18) # STW_ACTN_RQ HiBmLvr_Stat low bit: shown per phase as a check that the phases match the stalk


class Watch(object):
  def __init__(self):
    self.segments = []            # [{'on': bool, 't0': start, 'frames': {(bus, addr): [values...]}}]
    self.new_segment(False, 0.0)

  def new_segment(self, on, t):
    self.segments.append({'on': on, 't0': t, 'frames': {}})

  def toggle(self, t):
    self.new_segment(not self.segments[-1]['on'], t)

  def add(self, bus, addr, dat, t):
    if bus >= 128:
      return
    seg = self.segments[-1]
    if t - seg['t0'] < SETTLE and len(self.segments) > 1:
      return
    e = seg['frames'].get((bus, addr))
    if e is None:
      e = seg['frames'][(bus, addr)] = [0, {}]          # [longest frame, {value: count}] - values repeat a lot
    if len(dat) > e[0]: e[0] = len(dat)
    v = int.from_bytes(dat, 'little')
    e[1][v] = e[1].get(v, 0) + 1

  def analyse(self):
    segs = [s for s in self.segments if s['frames']]
    ons = [s for s in segs if s['on']]; offs = [s for s in segs if not s['on']]
    if len(ons) < MIN_PHASES or len(offs) < MIN_PHASES:
      return {'error': 'need at least %d ON and %d OFF phases (got %d ON, %d OFF)' % (MIN_PHASES, MIN_PHASES, len(ons), len(offs))}
    keys = set(segs[0]['frames'])
    for s in segs[1:]:
      keys &= set(s['frames'])      # only IDs seen in every phase
    bits, fields, loose = [], [], []
    for key in sorted(keys):
      n = max(s['frames'][key][0] for s in segs)
      all_or, all_and = 0, (1 << (8 * n)) - 1
      for s in segs:
        for v in s['frames'][key][1]:
          all_or |= v; all_and &= v
      moving = all_or & ~all_and     # only bits that changed at all are worth counting
      if not moving:
        continue
      cand = [b for b in range(n * 8) if (moving >> b) & 1]
      share = []                     # per segment: share of frames with each candidate bit = 1
      for s in segs:
        vals = s['frames'][key][1]; tot = float(sum(vals.values()))
        share.append(dict((b, sum(c for v, c in vals.items() if (v >> b) & 1) / tot) for b in cand))
      for b in cand:
        on_v = [sh[b] for sh, s in zip(share, segs) if s['on']]
        off_v = [sh[b] for sh, s in zip(share, segs) if not s['on']]
        if min(on_v) >= STEADY and max(off_v) <= 1 - STEADY:
          bits.append({'bus': key[0], 'id': '0x%X' % key[1], 'byte': b // 8, 'bit': b % 8, 'on': 1, 'off': 0})
        elif max(on_v) <= 1 - STEADY and min(off_v) >= STEADY:
          bits.append({'bus': key[0], 'id': '0x%X' % key[1], 'byte': b // 8, 'bit': b % 8, 'on': 0, 'off': 1})
        elif (min(on_v) >= LOOSE and max(off_v) <= 1 - LOOSE) or (max(on_v) <= 1 - LOOSE and min(off_v) >= LOOSE):
          loose.append({'bus': key[0], 'id': '0x%X' % key[1], 'byte': b // 8, 'bit': b % 8,
                        'on_share': [round(x, 2) for x in on_v], 'off_share': [round(x, 2) for x in off_v]})
      for byte in sorted(set(b // 8 for b in cand)):    # whole bytes whose ON values and OFF values never overlap
        on_set = set(); off_set = set()
        for s in segs:
          vs = set((v >> (8 * byte)) & 0xFF for v in s['frames'][key][1])
          (on_set if s['on'] else off_set).update(vs)
        if on_set and off_set and not (on_set & off_set) and len(on_set | off_set) <= 8:
          fields.append({'bus': key[0], 'id': '0x%X' % key[1], 'byte': byte,
                         'on_values': sorted('%02x' % v for v in on_set), 'off_values': sorted('%02x' % v for v in off_set)})
    stalk = []
    for s in segs:
      e = s['frames'].get(STALK[:2])
      if e:
        tot = float(sum(e[1].values()))
        stalk.append(('ON ' if s['on'] else 'OFF') + ' %.0f%%' % (100 * sum(c for v, c in e[1].items() if (v >> STALK[2]) & 1) / tot))
    return {'phases': {'on': len(ons), 'off': len(offs)}, 'ids_compared': len(keys), 'bits': bits, 'bytes': fields,
            'loose': loose, 'stalk_high_share_per_phase': stalk}


def report(r):
  if 'error' in r:
    print(r['error']); return
  print('\n%d ON / %d OFF phases, %d IDs seen in every phase' % (r['phases']['on'], r['phases']['off'], r['ids_compared']))
  if not r['bits'] and not r['bytes']:
    print('nothing followed the switch')
  for b in r['bits']:
    print('  bus %d  %-6s byte %d bit %d   ON=%d OFF=%d   (start bit %d, little-endian count)' % (
        b['bus'], b['id'], b['byte'], b['bit'], b['on'], b['off'], b['byte'] * 8 + b['bit']))
  for f in r['bytes']:
    print('  bus %d  %-6s byte %d   ON values %s   OFF values %s' % (f['bus'], f['id'], f['byte'], ' '.join(f['on_values']), ' '.join(f['off_values'])))
  if r.get('loose'):
    print('weaker matches (share of frames with the bit = 1, per phase):')
    for b in r['loose'][:40]:
      print('  bus %d  %-6s byte %d bit %d   ON %s   OFF %s' % (b['bus'], b['id'], b['byte'], b['bit'], b['on_share'], b['off_share']))
  if r.get('stalk_high_share_per_phase'):
    print('check - stalk in high-beam position (0x45 HiBmLvr_Stat), share of each phase: ' + ', '.join(r['stalk_high_share_per_phase']))


def save(r, t_start):
  fn = '/data/canwatch_%s.json' % time.strftime('%Y%m%d_%H%M%S')
  try:
    with open(fn, 'w') as f:
      json.dump(r, f, indent=1)
    print('saved %s (%.0f s recorded)' % (fn, time.time() - t_start))
  except Exception as e:
    print('NOT saved to %s: %s' % (fn, e))


def main():
  import traceback
  import cereal.messaging as messaging
  w = Watch(); lock = threading.Lock(); done = [False]
  sock = messaging.sub_sock('can')
  def keys():
    while True:
      line = sys.stdin.readline()
      if not line or line.strip().lower() == 'q':
        done[0] = True; return
      with lock:
        w.toggle(time.time())
        print('  -> phase %d: now switch it %s' % (len(w.segments), 'ON' if w.segments[-1]['on'] else 'OFF')); sys.stdout.flush()
  threading.Thread(target=keys, daemon=True).start()
  print('recording, thing is OFF. Enter = switch, q + Enter (or Ctrl+C) = finish'); sys.stdout.flush()
  t_start = time.time(); frames = 0
  try:
    while not done[0]:
      for msg in messaging.drain_sock(sock):
        t = time.time()
        with lock:
          for c in msg.can:
            w.add(c.src, c.address, bytes(c.dat), t); frames += 1
      time.sleep(0.01)
  except KeyboardInterrupt:
    print('\nstopped')
  print('%d CAN frames received' % frames + ('  <- none: is boardd running (openpilot started, panda connected)?' if not frames else ''))
  print('analysing ...'); sys.stdout.flush()
  try:
    with lock:
      r = w.analyse()
    report(r)
  except (Exception, KeyboardInterrupt):
    traceback.print_exc()
    r = {'error': traceback.format_exc()}
  r['frames_received'] = frames
  save(r, t_start)


if __name__ == '__main__':
  main()
