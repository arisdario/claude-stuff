#!/usr/bin/env python3
"""lightdash body/gateway decode (0x318 doors, 0x283 lamps/sensors, 0x238 speed limit, 0x2F8 units): Python class, and
the C++ twin compiled on its own (the BODY-BEGIN..BODY-END block + appendf) and compared field for field on the same frames."""
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
REAL_318 = bytes.fromhex('9a0b091101354001')          # captured on the car (also the clock test sample): all doors closed

FRAMES = [   # (time, addr, payload)
  (1.0, 0x318, REAL_318),
  (1.0, 0x283, bytes([0, 0, 0x40, 0x20, 0x01, 86, 0, 0x5F])),       # night, wipers out of park, low beam, inside 86*0.25 = 21.5 C
  (1.0, 0x238, bytes([3, 56, 0, 0, 0, 0, 0, 0])),                    # 56 * 0.25 = 14 m/s = 50.4 km/h -> 50
  (1.0, 0x2F8, bytes([0, 0, 0, 0, 0, 0x40, 0, 0])),                  # units bit 46 = 1 -> km/h
]


class TestPython(unittest.TestCase):
  def test_nothing_seen_is_unknown_not_closed(self):
    o = L.Body().snapshot(1.0)
    self.assertEqual(o['doors'], [None] * 4)
    self.assertIsNone(o['lo']); self.assertIsNone(o['limit']); self.assertIsNone(o['light'])

  def test_real_frame_and_sensors(self):
    b = L.Body()
    for t, a, d in FRAMES:
      b.feed(a, d, t)
    o = b.snapshot(1.5)
    self.assertEqual(o['doors'], [False] * 4)
    self.assertEqual((o['frunk'], o['trunk']), (False, False))
    self.assertEqual((o['lo'], o['hi'], o['wiper'], o['pbrake'], o['rev']), (True, False, True, False, False))
    self.assertEqual(o['light'], 'night'); self.assertEqual(o['inside'], 21.5)
    self.assertEqual((o['limit'], o['unit']), (50, 'km/h'))

  def test_open_door_and_stale(self):
    b = L.Body()
    d = bytearray(REAL_318); d[1] |= 1 << 4                          # front-left open
    b.feed(0x318, bytes(d), 1.0)
    self.assertEqual(b.snapshot(1.1)['doors'][0], True)
    self.assertIsNone(b.snapshot(4.5)['doors'][0])                    # older than 3 s: unknown

  def test_limit_record_only_and_mph(self):
    b = L.Body()
    b.feed(0x238, bytes([4, 200, 0, 0, 0, 0, 0, 0]), 1.0)            # record 4 is not a speed limit
    self.assertIsNone(b.snapshot(1.1)['limit'])
    b.feed(0x2F8, bytes(8), 1.0); b.feed(0x238, bytes([3, 54, 0, 0, 0, 0, 0, 0]), 1.0)   # 13.5 m/s = 30.2 mph -> 30
    self.assertEqual((b.snapshot(1.1)['limit'], b.snapshot(1.1)['unit']), (30, 'mph'))
    self.assertIsNone(b.snapshot(12.0)['limit'])


@unittest.skipUnless(shutil.which('g++'), 'no g++')
class TestCppTwin(unittest.TestCase):
  def test_cpp_matches_python(self):
    block = CC[CC.index('// BODY-BEGIN'):CC.index('// BODY-END')]
    appendf = re.search(r'static void appendf\(.*?\n}\n', CC, re.S).group(0)
    feeds = '\n'.join('{ const uint8_t d[] = {%s}; b.feed(0x%X, d, sizeof d, %.1f); }' % (','.join(str(x) for x in d), a, t) for t, a, d in FRAMES)
    src = ('#include <cstdio>\n#include <cstdint>\n#include <cstdarg>\n#include <cmath>\n#include <string>\n' + appendf + block +
           '\nint main(){ Body b;\n' + feeds + '\n std::string s="{"; json_body(s, b, 1.5); s+="}"; printf("%s\\n", s.c_str());\n'
           ' std::string q; json_str(q, "a \\"quoted\\" \\\\ alert\\nline"); printf("%s\\n", q.c_str()); return 0; }\n')
    d = tempfile.mkdtemp()
    open(os.path.join(d, 't.cc'), 'w').write(src)
    subprocess.check_call(['g++', '-std=c++14', '-Wall', '-o', os.path.join(d, 't'), os.path.join(d, 't.cc')])
    out = subprocess.check_output([os.path.join(d, 't')]).decode().splitlines()
    py = L.Body()
    for t, a, dd in FRAMES:
      py.feed(a, dd, t)
    self.assertEqual(json.loads(out[0])['body'], py.snapshot(1.5))
    self.assertEqual(json.loads(out[1]), 'a "quoted" \\ alert line')     # escaped, control chars blanked


if __name__ == '__main__':
  unittest.main()
