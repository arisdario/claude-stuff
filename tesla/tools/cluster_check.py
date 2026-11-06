#!/usr/bin/env python3
"""cluster_check: ONE read-only command for the "cluster shows nothing" question. Run it on the EON, parked (not engaged):

    python3 /data/claude-stuff/tesla/tools/cluster_check.py            # newest /data/maplog/map_*.jsonl
    python3 /data/claude-stuff/tesla/tools/cluster_check.py --log FILE
    python3 /data/claude-stuff/tesla/tools/cluster_check.py --no-panda # skip pandatool (e.g. while openpilot runs)

It collects, prints and saves (/data/cluster_check_YYYYmmdd_HHMMSS.txt) three things, then gives a verdict:
  1. the enable_*/use_*/uses_* switches in /data/bb_openpilot.cfg (enable_das_emulation must be True; uses_a_pillar_harness tells the harness type)
  2. `pandatool.py status` (read-only): does the flashed firmware match the source built from the tree?
  3. the CAN-dump check of CLUSTER.md on the newest drive log: are the cluster frames on the bus, and do they follow openpilot?
It changes nothing, sends nothing on CAN, and never flashes. No secrets are printed (only the enable_* lines).
"""
import argparse
import collections
import glob
import json
import os
import re
import subprocess
import sys
import time

CLUSTER_IDS = ('0x399', '0x389', '0x239', '0x309', '0x3e9')
CFG = '/data/bb_openpilot.cfg'
PANDATOOL = '/data/pandatool.py'


def analyze_dump(path):
  """-> dict(rows, seen{id: rows}, pairs{(op_status in 0x659, op_status in 0x399): rows})"""
  seen = collections.Counter(); pairs = collections.Counter(); rows = 0
  lanes = collections.Counter(); objs = collections.Counter()
  with open(path) as f:
    for line in f:
      try:
        r = json.loads(line).get('raw', {})
      except Exception:
        continue
      rows += 1
      for k in r:
        seen[k.lower()] += 1
      if r.get('0x239'): lanes[r['0x239'].replace(' ', '')[:14]] += 1      # payload without the counter byte
      if r.get('0x309'): objs[r['0x309'].replace(' ', '')[:14]] += 1
      a, s = r.get('0x659'), r.get('0x399')
      if a and s:
        try:
          pairs[(int(a[:2], 16) & 15, int(s[:2], 16) & 15)] += 1
        except ValueError:
          pass
  return {'rows': rows, 'seen': dict(seen), 'pairs': dict(pairs), 'lanes': dict(lanes), 'objs': dict(objs)}


def dump_verdict(res):
  """-> (code, text)"""
  if res['rows'] < 50:
    return 'TOO_SHORT', 'dump too short (%d rows): drive a minute with radarweb running, then run again' % res['rows']
  present = {k: res['seen'].get(k, 0) for k in CLUSTER_IDS}
  if not any(present.values()):
    return 'NO_CLUSTER_FRAMES', ('none of %s on the bus: the firmware sends no cluster frames. Most likely enable_das_emulation is not '
                                 'True in /data/bb_openpilot.cfg (or the panda is not in the Tesla safety mode)' % ', '.join(CLUSTER_IDS))
  pairs = res['pairs']
  if not pairs:
    return 'NO_PAIRS', 'cluster frames present but no row has both 0x659 (from openpilot) and 0x399: cannot compare status'
  total = sum(pairs.values()); follow = sum(v for (a, b), v in pairs.items() if a == b)
  engaged = sum(v for (a, b), v in pairs.items() if a == 5)
  engaged_follow = sum(v for (a, b), v in pairs.items() if a == 5 and b == 5)
  if engaged and engaged_follow / float(engaged) > 0.9:
    return 'STATUS_FOLLOWS', ('the cluster status follows openpilot (%d of %d engaged rows). Firmware is passing the status through: if the '
                              'cluster still draws nothing, the car/MCU is not accepting the frames' % (engaged_follow, engaged))
  if not engaged:
    return 'NOT_ENGAGED', 'openpilot was never engaged in this dump (no op_status 5): drive engaged for a minute and run again'
  return 'STATUS_IGNORED', ('openpilot sends op_status 5 in %d rows but the cluster frame carries it in only %d: the panda is not using the '
                            'status bytes (older/different firmware than the source). Run pandatool status and re-flash if DIFFERENT' %
                            (engaged, engaged_follow))


def decode_lanes(hexpayload):
  """DAS_lanes (0x239) payload hex (>= 6 bytes) -> readable fields (DBC scaling)"""
  c = bytes.fromhex(hexpayload)
  return {'width_m': 2 + 0.3125 * (c[0] >> 4), 'left_lane': c[0] & 1, 'right_lane': (c[0] >> 1) & 1, 'range_m': c[1],
          'C0_m': c[2] * 0.035 - 3.5, 'C1': c[3] * 0.0016 - 0.2, 'C2': c[4] * 2e-5 - 0.0025, 'C3': c[5] * 2.4e-7 - 3e-5}


def lanes_note(res):
  """-> text about whether the cluster ever got lane/lead data (None if no 0x239 in the dump)"""
  lanes = res.get('lanes') or {}
  if not lanes:
    return None
  n = sum(lanes.values()); top = max(lanes, key=lanes.get)
  d = decode_lanes(top)
  txt = '0x239 (lanes to the cluster): %d distinct payloads in %d rows; most common: width %.2f m, left=%d right=%d, range %d m, C0 %.2f m' % (
      len(lanes), n, d['width_m'], d['left_lane'], d['right_lane'], d['range_m'], d['C0_m'])
  if len(lanes) <= 2 and not d['left_lane'] and not d['right_lane']:
    txt += '\n   => LANES_NEVER_UPDATED: the same "no lane" frame the whole drive (a width of 6.06 m / range 161 m is the firmware default overflow, see CLUSTER.md)'
  objs = res.get('objs') or {}
  if objs and len(objs) <= 2:
    txt += '\n   0x309 (lead cars): %d distinct payloads: no lead was ever drawn' % len(objs)
  return txt


def read_cfg():
  out = []
  try:
    for line in open(CFG):
      if re.match(r'^\s*(enable_|use_|uses_)[a-z_]*\s*=', line):
        out.append(line.strip())
  except Exception as e:
    out.append('cannot read %s: %s' % (CFG, e))
  return out


def pandatool_status(timeout=40):
  if not os.path.exists(PANDATOOL):
    return ['%s not found' % PANDATOOL]
  try:
    p = subprocess.run([sys.executable, PANDATOOL, 'status'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout,
                       universal_newlines=True)
    return p.stdout.strip().splitlines()[-40:]
  except Exception as e:
    return ['pandatool status failed: %s' % e]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--log'); ap.add_argument('--no-panda', action='store_true')
  a = ap.parse_args()
  lines = ['cluster_check %s' % time.strftime('%Y-%m-%d %H:%M:%S')]
  lines.append('\n=== 1. switches in %s' % CFG)
  lines += read_cfg() or ['(no enable_/use_/uses_ lines found)']
  cfg_text = '\n'.join(read_cfg())
  emu = re.search(r'enable_das_emulation\s*=\s*(\w+)', cfg_text)
  lines.append('=> enable_das_emulation = %s' % (emu.group(1) if emu else 'not set (default False)'))
  lines.append('\n=== 2. pandatool status (read-only)')
  if a.no_panda:
    lines.append('(skipped: --no-panda)')
  else:
    st = pandatool_status(); lines += st
    txt = '\n'.join(st)
    lines.append('=> firmware: %s' % ('MATCHES the built source' if 'MATCH' in txt and 'DIFFERENT' not in txt else
                                      'DIFFERENT from the built source (needs flash)' if 'DIFFERENT' in txt else 'unknown (see output)'))
  lines.append('\n=== 3. CAN dump check')
  log = a.log
  if not log:
    cands = sorted(glob.glob('/data/maplog/map_*.jsonl'))
    log = cands[-1] if cands else None
  if not log or not os.path.exists(log):
    lines.append('no dump found: drive with radarweb running first (it writes /data/maplog/map_*.jsonl)')
    code = 'NO_DUMP'
  else:
    res = analyze_dump(log)
    lines.append('file: %s  rows: %d' % (log, res['rows']))
    lines.append('frames seen (rows): %s' % {k: res['seen'].get(k, 0) for k in ('0x659', '0x488') + CLUSTER_IDS})
    lines.append('(op_status in 0x659 from openpilot, op_status in 0x399 to the cluster): rows %s' % res['pairs'])
    note = lanes_note(res)
    if note: lines.append(note)
    code, text = dump_verdict(res)
    lines.append('=> %s: %s' % (code, text))
  out = '/data/cluster_check_%s.txt' % time.strftime('%Y%m%d_%H%M%S')
  body = '\n'.join(lines)
  print(body)
  try:
    open(out, 'w').write(body + '\n'); print('\nsaved: %s' % out)
  except Exception:
    pass
  return 0


if __name__ == '__main__':
  sys.exit(main())
