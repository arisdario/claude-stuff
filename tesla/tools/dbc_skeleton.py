#!/usr/bin/env python3
"""dbc_skeleton.py - start YOUR car's own DBC from what it actually sends: every CAN ID seen in your logs, per bus, with the
definitions of existing DBC files merged in where the ID is known, and a placeholder for every unknown ID that shows which
bytes ever change (so you know where to look). Then fill the placeholders in by watching bits while you do one thing at a
time (open a door, press the brake ...), e.g. in SavvyCAN, and check the result with --check.

  python3 dbc_skeleton.py LOGS... [--dbc known.dbc ...] [--out my_car]     -> my_car_bus0.dbc, my_car_radar.dbc, my_car_report.md
  python3 dbc_skeleton.py --check my_car_bus0.dbc LOGS... [--bus 0]       -> min / max / distinct values of every signal (needs cantools)
  python3 dbc_skeleton.py --savvy LOGS... --out drive                     -> drive_savvycan.csv (SavvyCAN "GVRET CSV" import)

LOGS can be any mix of:
  radarweb map logs  /data/maplog/map_*.jsonl   'raw' snapshots: '0xNNN' = chassis bus 0, 'r0xNNN' = radar bus (about 1/s)
  bridge / scan text lines like  ID=0x238 DLC=8 BUS=0,2 DATA=0400007E0863C3CA   (tesla_can_bridge.log, pasted CAN scans)
  candump lines like  (1699999999.123456) can0 238#0400007E0863C3CA
Pure Python 3 (cantools only for --check). Reads logs, writes new files: never touches the car or the logs.

Left out on purpose: the instrument-cluster / DAS telemetry IDs openpilot itself sends (EXCLUDE below) - they are listed in the
report as seen, but no definitions are copied or generated for them.
"""
import json
import os
import re
import sys
from collections import defaultdict

EXCLUDE = {0x239, 0x309, 0x398, 0x488, 0x554, 0x557, 0x559, 0x659}
RE_BRIDGE = re.compile(r'ID=0x([0-9A-Fa-f]+)\s+(?:DLC|LEN)=(\d+)\s+BUS=([0-9,]+)\s+DATA=([0-9A-Fa-f ]*)')
RE_CANDUMP = re.compile(r'\(([\d.]+)\)\s+\S*?(\d+)\s+([0-9A-Fa-f]{1,8})#([0-9A-Fa-f]*)')


class Seen(object):
  """per (bus, id): frame count, lengths, and per byte min / max / distinct values"""
  def __init__(self):
    self.n = 0; self.dlc = defaultdict(int); self.lo = [255] * 8; self.hi = [0] * 8; self.vals = [set() for _ in range(8)]; self.bits = [0] * 8; self.first = None

  def add(self, data):
    self.n += 1; self.dlc[len(data)] += 1
    if self.first is None:
      self.first = data
    for i, b in enumerate(data[:8]):
      self.lo[i] = min(self.lo[i], b); self.hi[i] = max(self.hi[i], b)
      if len(self.vals[i]) < 300:
        self.vals[i].add(b)
      self.bits[i] |= b ^ self.first[i] if i < len(self.first) else b       # bits that ever differed from the first frame


def read_logs(paths):
  seen = defaultdict(Seen)          # (bus, addr) -> Seen;  bus 'radar' for the radar bus
  for p in paths:
    with open(p, errors='replace') as f:
      for line in f:
        if line.startswith('{'):
          try:
            raw = json.loads(line).get('raw') or {}
          except Exception:
            continue
          for k, h in raw.items():
            try:
              bus, a = ('radar', int(k[1:], 16)) if k.startswith('r') else (0, int(k, 16))
              seen[(bus, a)].add(bytes.fromhex(h.replace(' ', '')))
            except Exception:
              pass
          continue
        m = RE_BRIDGE.search(line)
        if m:
          a, data = int(m.group(1), 16), bytes.fromhex(m.group(4).replace(' ', ''))
          seen[(int(m.group(3).split(',')[0]), a)].add(data)
          continue
        m = RE_CANDUMP.search(line)
        if m:
          seen[(int(m.group(2)), int(m.group(3), 16))].add(bytes.fromhex(m.group(4)))
  return seen


def read_dbc_blocks(paths):
  """{id: (name, BO_ block text incl. its SG_ lines, [CM_/VAL_ lines for it])} from existing DBC files; first file wins"""
  out = {}
  for p in paths:
    text = open(p, errors='replace').read()
    blocks = re.findall(r'^(BO_ (\d+) (\w+):.*?\n(?:[ \t]+SG_ .*\n)*)', text, re.M)
    extra = defaultdict(list)
    for line in text.splitlines():
      m = re.match(r'(?:CM_ SG_|VAL_) (\d+) ', line)
      if m and line.rstrip().endswith(';'):            # skip broken multi-line / unterminated entries (cantools rejects them)
        extra[int(m.group(1))].append(line.rstrip())
    for blk, i, name in blocks:
      sigs = set(re.findall(r'SG_ (\w+)', blk))
      ok = [l for l in extra[int(i)] if l.split()[2 if l.startswith('VAL_') else 3] in sigs]   # only labels/comments of signals it has
      i = int(i) & 0x1FFFFFFF
      if i not in out:
        out[i] = (name, blk.rstrip('\n'), ok)
  return out


def dbc_id(a):
  """DBC marks 29-bit (extended) ids with bit 31"""
  return a | 0x80000000 if a > 0x7FF else a


def placeholder(a, s):
  """BO_ for an unknown id: one 8-bit raw signal per byte that ever changed"""
  dlc = max(s.dlc, key=s.dlc.get)
  lines = ['BO_ %d UNKNOWN_%03X: %d Vector__XXX' % (dbc_id(a), a, dlc)]
  for i in range(dlc):
    if s.bits[i]:
      lines.append(' SG_ B%d_%03X : %d|8@1+ (1,0) [0|255] "" Vector__XXX' % (i, a, i * 8))
  return '\n'.join(lines)


def byte_summary(s):
  dlc = max(s.dlc, key=s.dlc.get); parts = []
  for i in range(dlc):
    if not s.bits[i]:
      parts.append('%d:const %02X' % (i, s.first[i] if i < len(s.first) else 0))
    else:
      parts.append('%d:%02X-%02X (%s%d values, bits %s)' % (i, s.lo[i], s.hi[i], '>=' if len(s.vals[i]) >= 300 else '', len(s.vals[i]),
                                                      ''.join('x' if s.bits[i] >> (7 - k) & 1 else '.' for k in range(8))))
  return '; '.join(parts)


def build(seen, known, out):
  files = {}
  report = ['# CAN IDs seen in the logs', '', 'x = bit changed at least once (bit 7 first), const = never changed. Radar bus IDs '
            'overlap chassis IDs, so each bus has its own DBC.', '']
  for bus in sorted({b for b, _ in seen}, key=str):
    ids = sorted(a for b, a in seen if b == bus)
    hdr = ['VERSION ""', '', 'NS_ :', '', 'BS_:', '', 'BU_: Vector__XXX', '']
    body, extra = [], []
    report += ['## bus %s: %d IDs' % (bus, len(ids)), '', '| ID | frames | len | known as | bytes |', '|---|---|---|---|---|']
    for a in ids:
      s = seen[(bus, a)]
      name = '(left out)' if a in EXCLUDE else known[a][0] if a in known else '**unknown**'
      report.append('| 0x%03X | %d | %s | %s | %s |' % (a, s.n, '/'.join(str(k) for k in sorted(s.dlc)), name, byte_summary(s)))
      if a in EXCLUDE:
        continue
      if a in known and bus != 'radar' or (bus == 'radar' and a in known and known[a][0].startswith('M_')):
        body.append(known[a][1]); extra += known[a][2]
      else:
        body.append(placeholder(a, s))
        extra.append('CM_ BO_ %d "not in any DBC given: %d frames, %s";' % (dbc_id(a), s.n, byte_summary(s).replace('"', "'")))
      body.append('')
    fn = '%s_%s.dbc' % (out, 'radar' if bus == 'radar' else 'bus%s' % bus)
    with open(fn, 'w') as f:
      f.write('\n'.join(hdr + body + extra) + '\n')
    files[bus] = fn
    report.append('')
  with open(out + '_report.md', 'w') as f:
    f.write('\n'.join(report) + '\n')
  return files


def check(dbc, logs, bus):
  import cantools
  db = cantools.database.load_file(dbc, strict=False)
  seen_rows = defaultdict(list)
  for p in logs:
    for line in open(p, errors='replace'):
      if not line.startswith('{'):
        m = RE_BRIDGE.search(line) or None
        if m and str(bus) == m.group(3).split(',')[0]:
          seen_rows[int(m.group(1), 16)].append(bytes.fromhex(m.group(4)))
        continue
      try:
        raw = json.loads(line).get('raw') or {}
      except Exception:
        continue
      for k, h in raw.items():
        if (bus == 'radar') == k.startswith('r'):
          seen_rows[int(k.lstrip('r'), 16)].append(bytes.fromhex(h.replace(' ', '')))
  for m in sorted(db.messages, key=lambda m: m.frame_id):
    rows = seen_rows.get(m.frame_id)
    if not rows:
      print('0x%03X %s: not in these logs' % (m.frame_id, m.name)); continue
    vals = defaultdict(list)
    for d in rows:
      try:
        for k, v in m.decode(d.ljust(m.length, b'\0'), decode_choices=False).items():
          vals[k].append(v)
      except Exception:
        pass
    print('0x%03X %s (%d frames)' % (m.frame_id, m.name, len(rows)))
    for sname in sorted(vals):
      v = vals[sname]; sig = m.get_signal_by_name(sname)
      print('   %-34s min %10.3f  max %10.3f  %4d values %s' % (sname, min(v), max(v), len(set(v)), sig.unit or ''))


def savvy(logs, out):
  """SavvyCAN 'GVRET CSV': Time Stamp,ID,Extended,Bus,LEN,D1..D8 (maplog snapshots get 1 s steps)"""
  fn = out + '_savvycan.csv'; t = 0
  with open(fn, 'w') as f:
    f.write('Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n')
    for p in logs:
      for line in open(p, errors='replace'):
        rows = []
        if line.startswith('{'):
          try:
            raw = json.loads(line).get('raw') or {}
          except Exception:
            continue
          rows = [(1 if k.startswith('r') else 0, int(k.lstrip('r'), 16), bytes.fromhex(h.replace(' ', ''))) for k, h in raw.items()]
          t += 1000000
        else:
          m = RE_BRIDGE.search(line)
          if m:
            rows = [(int(m.group(3).split(',')[0]), int(m.group(1), 16), bytes.fromhex(m.group(4)))]; t += 1000
        for b, a, d in rows:
          f.write('%d,%08X,false,%d,%d,%s\n' % (t, a, b, len(d), ','.join('%02X' % x for x in d) + ',' * (8 - len(d))))
  return fn


def main(argv):
  def opt(name, many=False, default=None):
    vals = [argv[i + 1] for i, a in enumerate(argv) if a == name and i + 1 < len(argv)]
    return vals if many else (vals[-1] if vals else default)
  flags = {'--dbc', '--out', '--check', '--bus'}
  logs = [a for i, a in enumerate(argv) if not a.startswith('--') and (i == 0 or argv[i - 1] not in flags)]
  if '--check' in argv:
    b = opt('--bus', default='0'); check(opt('--check'), logs, 'radar' if b == 'radar' else int(b)); return 0
  if not logs:
    print(__doc__); return 1
  out = opt('--out', default='my_car')
  if '--savvy' in argv:
    print('wrote', savvy(logs, out)); return 0
  seen = read_logs(logs)
  known = read_dbc_blocks(opt('--dbc', many=True))
  files = build(seen, known, out)
  for bus, fn in files.items():
    ids = [a for b, a in seen if b == bus]
    print('bus %s: %d IDs (%d already defined, %d unknown placeholders) -> %s' % (
        bus, len(ids), sum(1 for a in ids if a in known and a not in EXCLUDE), sum(1 for a in ids if a not in known and a not in EXCLUDE), fn))
  print('report ->', out + '_report.md')
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
