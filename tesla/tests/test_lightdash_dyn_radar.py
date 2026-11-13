#!/usr/bin/env python3
"""lightdash drive-unit/brake/airbag/MCU decode (class Dyn) and the radar object list (class Radar): checked against frames
logged on this car (tests/data/lightdash_real_frames.json, 3 moments of one drive) and an independent bit-by-bit decode
written from the DBCs; the C++ twin (DYN-/RADAR- blocks of lightdash.cc) must give the same JSON on the same frames."""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
import lightdash as L  # noqa: E402

CC = open(os.path.join(HERE, '..', 'tools', 'lightdash.cc')).read()
REAL = json.load(open(os.path.join(HERE, 'data', 'lightdash_real_frames.json')))


def hx(s):
  return bytes.fromhex(s.replace(' ', ''))


def fed(row, t=1.0):
  dyn, rad = L.Dyn(), L.Radar()
  for k, h in row['dyn'].items():
    dyn.feed(int(k, 16), hx(h), t)
  for k, h in row['rad'].items():
    rad.feed(int(k[1:], 16), hx(h), t)
  return dyn, rad


def ref_bits(b, start, n, signed=False):            # independent: bit i of a little-endian DBC signal = bit i%8 of byte i//8
  x = sum(((b[(start + i) // 8] >> ((start + i) % 8)) & 1) << i for i in range(n))
  return x - (1 << n) if signed and x >> (n - 1) else x


class TestDyn(unittest.TestCase):
  def test_real_frames_match_reference(self):
    for row in REAL:
      o = fed(row)[0].snapshot(1.5)
      b = hx(row['dyn']['0x108'])
      kw = ref_bits(b, 16, 13, True) * 0.25 * ref_bits(b, 32, 16, True) * 2 * 3.14159265 / 60 / 1000
      self.assertAlmostEqual(o['kw'], kw, delta=0.06)
      b = hx(row['dyn']['0x145'])
      self.assertAlmostEqual(o['ax'], ref_bits(b, 0, 10) * 0.05 - 25.6, places=6)
      self.assertAlmostEqual(o['ay'], ref_bits(b, 16, 10) * 0.05 - 25.6, places=6)
      self.assertEqual(o['gear'], 'D')                                   # all three moments: driving
      self.assertEqual(o['belt'], 1)
      self.assertIn(o['su'], ('km/h', 'mph'))

  def test_stale_and_zero_frames_are_unknown(self):
    d = L.Dyn()
    d.feed(0x368, bytes(8), 1.0)                                         # all zero: not awake, not "cruise off"
    self.assertIsNone(d.snapshot(1.1)['cs'])
    d.feed(0x118, bytes([0, 0x40, 0, 0, 0, 0, 0, 0]), 1.0)              # gear 4 = D
    self.assertEqual(d.snapshot(1.5)['gear'], 'D')
    self.assertIsNone(d.snapshot(2.5)['gear'])                           # older than 1 s

  def test_power_sign(self):
    d = L.Dyn()
    v = ((-100 * 4) & 0x1FFF) << 16 | (3000 & 0xFFFF) << 32             # -100 Nm at 3000 rpm = regen
    d.feed(0x108, v.to_bytes(8, 'little'), 1.0)
    self.assertAlmostEqual(d.snapshot(1.0)['kw'], -31.4, delta=0.1)


class TestRadar(unittest.TestCase):
  def test_real_frames(self):
    for row in REAL:
      objs = fed(row)[1].snapshot(1.1, row['v'])
      for k, d, y, gs, cls, ms in objs:
        a, b = hx(row['rad']['r0x%x' % (0x310 + 3 * k)]), hx(row['rad']['r0x%x' % (0x311 + 3 * k)])
        self.assertAlmostEqual(d, ref_bits(a, 0, 12) * 0.0625, delta=0.051)
        self.assertAlmostEqual(y, ref_bits(a, 24, 11) * 0.125 - 128, delta=0.006)
        self.assertTrue(abs(gs) >= 3 or ms in (1, 2) or cls)            # standing things are left out
        self.assertIn(cls, (0, 1, 2, 3))

  def test_rules(self):
    def pair(d, vr, y, pe=31, cls=0, pc=0, ms=1, valid=1):
      a = int(d / 0.0625) | int((vr + 128) / 0.0625) << 12 | int((y + 128) / 0.125) << 24 | pe << 35 | valid << 55 | 1 << 62
      b = ms << 22 | pc << 48 | cls << 53
      return a.to_bytes(8, 'little'), b.to_bytes(8, 'little')
    r = L.Radar()
    for k, p in enumerate([pair(50, -20, 0, ms=3),             # standing post while driving 20 m/s: left out
                           pair(60, 2, 0.5),                    # moving car
                           pair(20, -18, 3, cls=3, pc=31),      # radar "pedestrian" moving 2 m/s: kept as pedestrian
                           pair(30, -10, 3, cls=3, pc=31),      # "pedestrian" at 10 m/s over ground: a vehicle
                           pair(40, 0, 0, pe=10),               # unlikely to exist: left out
                           pair(249, 1, 0)]):                   # far moving car (lead range to 250 m)
      r.feed(0x310 + 3 * k, p[0], 1.0); r.feed(0x311 + 3 * k, p[1], 1.0)
    o = {x[0]: x for x in r.snapshot(1.1, 20.0)}
    self.assertEqual(sorted(o), [1, 2, 3, 5])
    self.assertEqual(o[2][4], 3); self.assertEqual(o[3][4], 1); self.assertAlmostEqual(o[5][1], 249.0, delta=0.07)
    self.assertEqual(r.snapshot(1.5, 20.0), [])                         # stale after 0.3 s


@unittest.skipUnless(shutil.which('g++'), 'no g++')
class TestCppTwin(unittest.TestCase):
  def test_cpp_matches_python(self):
    block = CC[CC.index('// DYN-BEGIN'):CC.index('// RADAR-END')]
    appendf = re.search(r'static void appendf\(.*?\n}\n', CC, re.S).group(0)
    lines = []
    for i, row in enumerate(REAL):
      lines.append('{ Dyn y; Radar r;')
      for obj, k, h in [('y', k, h) for k, h in row['dyn'].items()] + [('r', k[1:], h) for k, h in row['rad'].items()]:   # chassis bus vs radar bus (0x368 is on both)
        lines.append('{ const uint8_t d[] = {%s}; %s.feed(0x%X, d, sizeof d, 1.0); }' % (','.join(str(x) for x in hx(h)), obj, int(k, 16)))
      lines.append(' std::string s="{"; json_dyn(s, y, 1.1); s+=","; json_radar(s, r, 1.1, %r); s+="}"; printf("%%s\\n", s.c_str()); }' % row['v'])
    src = ('#include <cstdio>\n#include <cstring>\n#include <cstdint>\n#include <cstdarg>\n#include <cmath>\n#include <string>\n' + appendf + block +
           '\nint main(){\n' + '\n'.join(lines) + '\nreturn 0; }\n')
    d = tempfile.mkdtemp()
    with open(os.path.join(d, 't.cc'), 'w') as f:
      f.write(src)
    subprocess.check_call(['g++', '-std=c++14', '-Wall', '-o', os.path.join(d, 't'), os.path.join(d, 't.cc')])
    out = subprocess.check_output([os.path.join(d, 't')]).decode().splitlines()
    for row, line in zip(REAL, out):
      cj = json.loads(line); dyn, rad = fed(row)
      self.assertEqual(cj['dyn'], dyn.snapshot(1.1))
      self.assertEqual(cj['rad'], rad.snapshot(1.1, row['v']))
      self.assertGreater(len(cj['rad']), 0)


if __name__ == '__main__':
  unittest.main()
