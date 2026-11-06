#!/usr/bin/env python3
"""cluster_check verdicts on synthetic dumps.  python3 tesla/tests/test_cluster_check.py"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import cluster_check as C  # noqa: E402


def dump(rows):
  f = tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False)
  for r in rows: f.write(json.dumps({'raw': r}) + '\n')
  f.close(); return f.name


def row(op, cl=None, cluster=True):
  r = {'0x659': '%02x00000000000000' % op, '0x488': '00' * 4}
  if cluster:
    r.update({'0x399': '%02x00000000000000' % (cl if cl is not None else op), '0x239': '00', '0x389': '00', '0x309': '00', '0x3e9': '00'})
  return r


class T(unittest.TestCase):
  def v(self, rows):
    p = dump(rows)
    try: return C.dump_verdict(C.analyze_dump(p))[0]
    finally: os.remove(p)

  def test_no_cluster_frames(self): self.assertEqual(self.v([row(5, cluster=False)] * 100), 'NO_CLUSTER_FRAMES')
  def test_status_ignored(self): self.assertEqual(self.v([row(5, cl=1)] * 100), 'STATUS_IGNORED')
  def test_status_follows(self): self.assertEqual(self.v([row(5)] * 100), 'STATUS_FOLLOWS')
  def test_not_engaged(self): self.assertEqual(self.v([row(2, cl=1)] * 100), 'NOT_ENGAGED')
  def test_too_short(self): self.assertEqual(self.v([row(5)] * 5), 'TOO_SHORT')
  def test_lanes_never_updated_is_reported(self):
    rows = [dict(row(5, cl=1), **{'0x239': 'd0 a1 64 7f 7f 7f f0 %02x' % (i % 16), '0x309': '03 ff ff ff 03 00 00 00'}) for i in range(100)]
    p = dump(rows)
    try:
      note = C.lanes_note(C.analyze_dump(p))
      self.assertIn('LANES_NEVER_UPDATED', note); self.assertIn('width 6.06 m', note)
    finally: os.remove(p)

  def test_decode_lanes(self):
    d = C.decode_lanes('d0a1647f7f7f')
    self.assertAlmostEqual(d['width_m'], 6.0625); self.assertEqual((d['left_lane'], d['right_lane'], d['range_m']), (0, 0, 161))

  def test_bad_lines_ignored(self):
    p = dump([row(5)] * 80)
    open(p, 'a').write('not json\n')
    try: self.assertEqual(C.analyze_dump(p)['rows'], 80)
    finally: os.remove(p)


if __name__ == '__main__':
  unittest.main()
