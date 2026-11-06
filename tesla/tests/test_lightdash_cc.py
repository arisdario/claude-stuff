#!/usr/bin/env python3
"""lightdash.cc and lightdash.py must stay twins (same page, same constants, no panda/USB).  python3 tesla/tests/test_lightdash_cc.py"""
import os
import re
import unittest

T = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools')
PY = open(os.path.join(T, 'lightdash.py')).read()
CC = open(os.path.join(T, 'lightdash.cc')).read()
SH = open(os.path.join(T, 'build_lightdash.sh')).read()


class T(unittest.TestCase):
  def test_same_page(self):
    py = re.search(r'PAGE = r"""(.*?)"""\n', PY, re.S).group(1)
    cc = re.search(r'R"LIGHTDASHPAGE\((.*?)\)LIGHTDASHPAGE"', CC, re.S).group(1)
    self.assertEqual(py, cc)

  def test_same_firmware_constants(self):
    for needle in ('0x00F00000', '0x03FFFF83', '0xFFFFFF00', '2.0'):
      self.assertIn(needle, CC, needle)
    self.assertIn('lWidth = 29', CC); self.assertIn('laneRange = 160', CC)
    self.assertIn('PANDA_STALE_S = 2.0', CC); self.assertIn('PANDA_STALE_S = 2.0', PY)

  def test_cpp_never_opens_the_panda(self):
    for bad in ('libusb', 'usb.h', 'Panda', 'PubSocket', 'sendMessage'):
      self.assertNotIn(bad, CC, bad)
    import re      # the only send() calls are TCP writes to the browser (MSG_NOSIGNAL), nothing else
    for m in re.finditer(r'\bsend\(([^;]*);', CC):
      self.assertIn('MSG_NOSIGNAL', m.group(0))
    for want in ('"sendcan"', '"can"', '"model"', '"carState"', '"controlsState"'):
      self.assertIn(want, CC)

  def test_build_script_is_compile_only(self):
    self.assertIn('--selftest', SH); self.assertNotIn('flash', SH.lower()); self.assertNotIn('pandatool', SH)

  def test_car_state_twins(self):
    for needle in ('0x30A', '0x283', '0xFE'):
      self.assertIn(needle, CC); self.assertIn(needle, PY)
    self.assertIn('CAR_HOLD_S = 3.0', CC); self.assertIn('CAR_HOLD_S = 3.0', PY)
    self.assertIn('d[7] * 0.5 - 40', CC); self.assertIn('d[7] * 0.5 - 40', PY)
    self.assertIn('c.getSrc() == 0', CC); self.assertIn('c.src == 0', PY)             # bus 0 only


if __name__ == '__main__':
  unittest.main()
