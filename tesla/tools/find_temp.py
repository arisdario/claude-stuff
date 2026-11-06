#!/usr/bin/env python3
"""find_temp.py - READ-ONLY: look for the outside / inside temperature in your CAN logs. Reads log files only; never touches the car, the panda or openpilot.

You tell it what the car's own display showed while each log was recorded; it scans every field of every address for one that matches those readings through a plain scale and offset.

  python3 find_temp.py LOG1.jsonl:outside=5,inside=19.5  LOG2.jsonl:outside=11,inside=23  LOG3.jsonl:outside=12,inside=27  [--top 10] [--tol 0.75]
  python3 find_temp.py LOG.jsonl:outside=9,inside=23.5..26      ONE log in which the inside temperature changed from 23.5 (start) to 26 (end): see below. (Do NOT write > : the shell would send the output to a file)

TWO CAPTURES (the practical way to get an inside temperature that really changes): capture now, warm (or cool) the cabin by 3 degrees or more, capture again, then search both:
  python3 find_temp.py --live 60 outside=9 inside=21 --save /data/capA.jsonl ; (change the cabin) ; python3 find_temp.py --live 60 outside=9 inside=24 --save /data/capB.jsonl
  python3 find_temp.py /data/capA.jsonl:outside=9,inside=21 /data/capB.jsonl:outside=9,inside=24 --steady 0.5
LIVE MODE (no log file needed; openpilot must be running and the car awake): it listens to the CAN messages boardd already publishes for N seconds, then searches exactly as below.
  cd /data/openpilot && python3 /data/find_temp.py --live 180 outside=9 inside=23.5 --ask-end
While it listens, change the cabin temperature (heating or A/C, fan up). At the end, --ask-end asks for the inside value the car shows NOW, so the search has two points. Reads messages only.
LOG = a maplog file (/data/maplog/map_*.jsonl: one row per second with the latest payload of every address under "raw"). A reading is optional per log (give only outside, only inside, or both).
HOW MANY LOGS: with ONE log the tool can only list fields that CAN be a temperature (many random fields fit one number): treat those as hints. With TWO logs at clearly different
temperatures most chance matches disappear; with THREE or more, a field that fits all of them through a round scale (0.5, 0.1, 1 ... and offset 0, -40, -50 ...) is very likely real.
ONE LOG IS ENOUGH FOR THE INSIDE TEMPERATURE if it moved during the recording (heat or cool the cabin by 2 degrees or more while logging; write down the display at the start and at the end and
give them as inside=START..END (two dots)). The tool then fits the first fifth of the log to START and the last fifth to END, which is two points, so a field must actually follow the cabin. Outside cannot
be moved, so for outside you still need logs on other days.
Record the logs a few hours or days apart (or after heating/cooling the cabin) and note the displayed numbers at the same moment. The value must also be steady inside each log (a temperature
changes slowly): fields that jump around are ignored.

What it tries: unsigned and signed fields of 8, 10, 11, 12 and 16 bits at every bit position (little-endian), plus 16-bit big-endian words; scales 1, 0.5, 0.25, 0.1, 0.125, 0.0625, 0.05, 0.2, 2, 0.01
and the usual offsets (0, -40, -50, -30, -20, -10, -25, -60, -100, 10, -273.15). It only finds the temperature if its message is in the log (a different bus segment may carry it).
Stdlib only (Python 3.7)."""
import argparse
import json
import sys

NICE_SCALES = [1.0, 0.5, 0.25, 0.1, 0.125, 0.0625, 0.05, 0.2, 2.0, 0.01]
NICE_OFFSETS = [0.0, -40.0, -50.0, -30.0, -20.0, -10.0, -25.0, -60.0, -100.0, 10.0, -273.15]
WIDTHS = (8, 10, 11, 12, 16)
STEADY = 0.9            # fraction of samples within +-3 raw units of the most common value (lower it with --steady 0.5 for a message that alternates between frame types)
CAP = 60                # samples per address per log (slow signals need no more)


def load(path, cap=CAP):
  """-> ({address: [64-bit little-endian ints]}, rows)"""
  per = {}
  rows = 0
  with open(path) as f:
    for line in f:
      try:
        r = json.loads(line)
      except ValueError:
        continue
      rows += 1
      for k, h in (r.get('raw') or {}).items():
        try:
          addr = int(k, 16)
          b = bytes.fromhex(h.replace(' ', ''))
        except (ValueError, AttributeError):
          continue
        per.setdefault(addr, []).append(int.from_bytes(b[:8].ljust(8, b'\0'), 'little'))
  for a, v in per.items():
    if len(v) > cap:
      step = len(v) / float(cap)
      per[a] = [v[int(i * step)] for i in range(cap)]
  return per, rows


def describe(key):
  kind, w, st, signed = key
  s = 'signed' if signed else 'unsigned'
  if kind == 'M':
    return 'bytes %d-%d big-endian 16-bit %s' % (st, st + 1, s)
  if w == 8 and st % 8 == 0:
    return 'byte %d, 8-bit %s' % (st // 8, s)
  return 'bits %d..%d little-endian, %d-bit %s' % (st, st + w - 1, w, s)


def fields(ints):
  """yields (key, [raw values]) for every candidate field of one address"""
  for w in WIDTHS:
    mask = (1 << w) - 1
    for st in range(0, 64 - w + 1):
      vals = [(v >> st) & mask for v in ints]
      yield ('I', w, st, False), vals
      if w in (8, 12, 16):
        yield ('I', w, st, True), [x - (1 << w) if x >> (w - 1) else x for x in vals]
  for i in range(7):
    vals = [(((v >> (8 * i)) & 0xFF) << 8) | ((v >> (8 * (i + 1))) & 0xFF) for v in ints]
    yield ('M', 16, i, False), vals
    yield ('M', 16, i, True), [x - 65536 if x >> 15 else x for x in vals]


def stats(vals):
  """-> (value, fraction of samples within +-3 of it). Two measures, the better one wins: the MEDIAN (a field that drifts slowly, like a temperature while the heating runs) and the MOST COMMON
  value (a message that alternates between frame types, where only some frames carry the field)."""
  from collections import Counter
  s = sorted(vals)
  med = s[len(s) // 2]
  st_med = sum(1 for x in vals if abs(x - med) <= 3) / float(len(vals))
  mode = Counter(vals).most_common(1)[0][0]
  st_mode = sum(1 for x in vals if abs(x - mode) <= 3) / float(len(vals))
  return (mode, st_mode) if st_mode > st_med else (med, st_med)


def trivial(key, med):
  w, signed = key[1], key[3]
  return med == 0 or (not signed and med == (1 << w) - 1) or (signed and med == -1)


def scan(per, part='all'):
  """-> {(addr, key): (median, steadiness)} for fields that are steady in this log. part = 'all', 'first' (the first fifth of the samples) or 'last' (the last fifth)"""
  out = {}
  for addr, ints in per.items():
    if len(ints) < 3:
      continue
    n = max(3, len(ints) // 5)
    if part == 'first':
      ints = ints[:n]
    elif part == 'last':
      ints = ints[-n:]
    for key, vals in fields(ints):
      med, st = stats(vals)
      if st >= STEADY and not trivial(key, med):
        out[(addr, key)] = (med, st)
  return out


def candidates(points, tol):
  """points: [(median raw, steadiness, reading)] -> list of (scale, offset, niceness, max error) that fit every point"""
  res = []
  for si, s in enumerate(NICE_SCALES):
    for oi, o in enumerate(NICE_OFFSETS):
      err = max(abs(r * s + o - t) for r, _, t in points)
      if err <= tol:
        res.append((s, o, si + oi, err))
  return res


def search(logs, target, tol=0.75):
  """logs: [(scanned fields dict, {target: reading})] -> (rows sorted best first, number of logs used, number of distinct readings)"""
  used = [(sc, rd[target]) for sc, rd in logs if target in rd]
  if not used:
    return [], 0, 0
  distinct = len(set(round(t, 1) for _, t in used))
  keys = set(used[0][0])
  for sc, _ in used[1:]:
    keys &= set(sc)
  rows = []
  for k in keys:
    pts = [(sc[k][0], sc[k][1], t) for sc, t in used]
    if distinct >= 2 and len(set(p[0] for p in pts)) < 2:
      continue                                  # the field never moves although the temperature did
    for s, o, nice, err in candidates(pts, tol):
      rows.append({'addr': k[0], 'key': k[1], 'scale': s, 'offset': o, 'nice': nice, 'err': err, 'raws': [p[0] for p in pts], 'steady': min(p[1] for p in pts)})
  def aligned(k):                                    # whole bytes (8 or 16 bits starting on a byte edge) are far more common in real CAN messages than odd bit positions
    return 0 if (k[1] in (8, 16) and k[2] % (8 if k[0] == 'I' else 1) == 0) or k[0] == 'M' else 1
  rows.sort(key=lambda r: (aligned(r['key']), r['nice'], r['err'], -r['steady'], r['key'][1], r['addr']))
  return rows, len(used), distinct


def parse_readings(tail):
  """'outside=9,inside=23.5..26' -> {'outside': 9.0, 'inside': (23.5, 26.0)}"""
  rd = {}
  for part in tail.split(','):
    k, _, v = part.partition('=')
    if k.strip() not in ('outside', 'inside'):
      raise ValueError('%r: only outside= and inside= are known' % part)
    if '..' in v or '>' in v:
      a, _, b = v.partition('..') if '..' in v else v.partition('>')
      rd[k.strip()] = (float(a), float(b))
    else:
      rd[k.strip()] = float(v)
  return rd


def collect_live(drain, seconds, bus=0, now=None, sleep=None, cap=CAP, progress=None):
  """drain() -> list of events with .can = [objects with .src .address .dat]; returns ({address: [64-bit ints in arrival order]}, frames seen)"""
  import time
  now = now or time.time
  sleep = sleep or time.sleep
  per = {}
  frames = 0
  t0 = now()
  last_tick = t0
  while now() - t0 < seconds:
    for ev in drain():
      for c in ev.can:
        if c.src != bus:
          continue
        frames += 1
        per.setdefault(c.address, []).append(int.from_bytes(bytes(c.dat)[:8].ljust(8, b'\0'), 'little'))
    if progress and now() - last_tick >= 10:
      last_tick = now(); progress(int(now() - t0), seconds, frames)
    sleep(0.02)
  for a, v in per.items():
    if len(v) > cap:
      step = len(v) / float(cap)
      per[a] = [v[int(i * step)] for i in range(cap)]
  return per, frames


def parse_spec(spec):
  path, _, tail = spec.rpartition(':')
  if not path or '=' not in tail:
    raise ValueError('%r: expected FILE:outside=5,inside=20' % spec)
  rd = {}
  for part in tail.split(','):
    k, _, v = part.partition('=')
    if k.strip() not in ('outside', 'inside'):
      raise ValueError('%r: only outside= and inside= are known' % part)
    if '..' in v or '>' in v:                         # START..END (or START>END, which the shell needs quoted): the temperature moved during this log
      a, _, b = v.partition('..') if '..' in v else v.partition('>')
      rd[k.strip()] = (float(a), float(b))
    else:
      rd[k.strip()] = float(v)
  return path, rd


def read_number(prompt, reader=None):
  """asks for a number; keys typed during the capture (arrow keys etc.) are discarded first and anything that is not part of a number is ignored"""
  import re
  try:
    import termios
    termios.tcflush(sys.stdin, termios.TCIFLUSH)
  except Exception:
    pass
  try:
    text = (reader or input)(prompt)
  except EOFError:
    return None
  m = re.search(r'-?\d+(?:[.,]\d+)?', text.replace('\x1b[A', '').replace('\x1b[B', '').replace('\x1b[C', '').replace('\x1b[D', ''))
  return float(m.group(0).replace(',', '.')) if m else None


def save_capture(per, path):
  """writes a live capture as a maplog-like jsonl (row i = the i-th sample of every address), so it can be searched later together with other captures"""
  n = max(len(v) for v in per.values()) if per else 0
  with open(path, 'w') as f:
    for i in range(n):
      raw = {'0x%x' % a: v[i].to_bytes(8, 'little').hex() for a, v in per.items() if i < len(v)}
      f.write(json.dumps({'t': i, 'raw': raw}) + '\n')
  return n


def live_capture(seconds, bus):
  import cereal.messaging as messaging
  sock = messaging.sub_sock('can', timeout=50)
  def progress(done, total, frames):
    print('   ... %d / %d s, %d frames' % (done, total, frames)); sys.stdout.flush()
  print('listening to bus %d for %d s: change the cabin temperature now (heating or A/C, fan up) ...' % (bus, seconds)); sys.stdout.flush()
  return collect_live(lambda: messaging.drain_sock(sock), seconds, bus, progress=progress)


def main(argv=None, capture=None):
  global STEADY
  ap = argparse.ArgumentParser(description='find the outside/inside temperature in CAN logs or the live CAN stream (read-only)')
  ap.add_argument('logs', nargs='+', help='FILE.jsonl:outside=5,inside=20 (one per log); with --live: outside=9 inside=23.5')
  ap.add_argument('--top', type=int, default=10)
  ap.add_argument('--tol', type=float, default=0.75, help='allowed error in degrees C (the display rounds)')
  ap.add_argument('--live', type=int, default=0, metavar='SECONDS', help='listen to the live CAN stream instead of reading a log (openpilot must be running)')
  ap.add_argument('--bus', type=int, default=0, help='CAN bus to listen to in --live mode (0 = chassis)')
  ap.add_argument('--steady', type=float, default=STEADY, help='how steady a field must be (0..1, default %(default)s); 0.5 finds fields of messages that alternate between frame types')
  ap.add_argument('--save', metavar='FILE', help='--live: also write the capture to FILE, to search later together with another capture (FILE:outside=9,inside=24)')
  ap.add_argument('--ask-end', action='store_true', help='--live: ask for the inside temperature shown at the end, as the END of inside=START..END')
  a = ap.parse_args(argv)
  STEADY = a.steady
  logs = []
  specs = []
  if a.live:
    try:
      rd = parse_readings(','.join(a.logs).replace(' ', ''))
    except ValueError as e:
      print(e); return 2
    per, frames = (capture or live_capture)(a.live, a.bus)
    print('captured %d frames from %d addresses on bus %d' % (frames, len(per), a.bus))
    if a.save:
      print('saved %d rows to %s' % (save_capture(per, a.save), a.save))
    if a.ask_end and isinstance(rd.get('inside'), float):
      end = read_number('inside temperature on the display NOW (Enter to skip): ')
      if end is not None:
        rd['inside'] = (rd['inside'], end)
    specs = [('live capture', per, frames, rd)]
  else:
    for spec in a.logs:
      try:
        path, rd = parse_spec(spec)
      except ValueError as e:
        print(e); return 2
      per, rows = load(path)
      specs.append((path, per, rows, rd))
  for path, per, rows, rd in specs:
    sc = scan(per)
    print('%s: %d rows, %d addresses, %d steady fields; readings %s' % (path, rows, len(per), len(sc), rd))
    moving = [t for t, v in rd.items() if isinstance(v, tuple)]
    if moving:
      first, last = scan(per, 'first'), scan(per, 'last')
    for t, v in rd.items():
      if isinstance(v, tuple):
        logs.append((first, {t: v[0]})); logs.append((last, {t: v[1]}))     # two points from one log
      else:
        logs.append((sc, {t: v}))
  rc = 1
  for target in ('outside', 'inside'):
    rows, n, distinct = search(logs, target, a.tol)
    print('\n== %s temperature: %d reading point(s), %d distinct value(s)' % (target, n, distinct))
    if n == 0:
      print('   no reading given'); continue
    if n < 2 or distinct < 2:
      print('   WEAK: with one log (or readings that are all the same) many random fields fit; these are hints only. Add logs at other temperatures.')
    elif n == 2:
      print('   2 logs: some chance matches remain; a third reading at another temperature will remove most of them.')
    if not rows:
      print('   nothing fits: either the message is not in these logs, or the scale is unusual, or a reading is off')
      continue
    rc = 0
    print('   %d candidate(s); best first (rounder scale/offset, smaller error):' % len(rows))
    for r in rows[:a.top]:
      print('   0x%03x  %-46s value = raw * %-6g %+g   raw per log %s  (max error %.2f C)' % (r['addr'], describe(r['key']), r['scale'], r['offset'], r['raws'], r['err']))
  return rc


if __name__ == '__main__':
  sys.exit(main())
