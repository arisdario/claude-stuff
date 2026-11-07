#!/usr/bin/env python3
"""Guards how applyVisionSignDetect() is wired into carcontroller.py. carcontroller.py itself needs
the full openpilot environment to import (CANPacker, ALCA/PCC/ACC modules, etc.) so it isn't
instantiated here - these are structural checks on the source, in the same spirit as
test_sign_detect_build_gate.py, plus a real round-trip test of the CAN packer it calls
(teslacan.create_fake_DAS_sign_msg, which only needs the stdlib-ish stub common.numpy_fast.clip).
"""
import os
import struct
import sys
import types
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
CC = open(os.path.join(ROOT, 'carcontroller.py')).read()


def func(src, head):
  a = src.index(head)
  return src[a:src.index('\n  def ', a + 1)]


class TestWiringIsAdditiveAndIsolated(unittest.TestCase):
  def setUp(self):
    self.body = func(CC, '  def applyVisionSignDetect(self):')

  def test_never_touches_the_dormant_trafficEvents_state(self):
    # handleTrafficEvents/checkWhichSign's own fields - applyVisionSignDetect must not read or set any
    for name in ('self.roadSignType', 'self.stopSign_distance', 'self.stopLight_distance',
                 'self.stopSignWarning', 'self.stopLightWarning', 'self.roadSignColor'):
      self.assertNotIn(name, self.body)

  def test_reads_are_wrapped_so_a_bad_file_cannot_raise(self):
    self.assertIn('try:', self.body); self.assertIn('except Exception', self.body)

  def test_called_unconditionally_not_gated_on_trafficeventsMsgs(self):
    line_start = CC.rindex('\n', 0, CC.index('if trafficeventsMsgs is not None:')) + 1
    after = CC[line_start:line_start + 400]
    self.assertIn('applyVisionSignDetect()', after)
    # specifically NOT inside that if-block's indentation (one level deeper than the call line below it)
    call_line = [l for l in after.splitlines() if 'applyVisionSignDetect()' in l][0]
    guard_line = [l for l in after.splitlines() if 'if trafficeventsMsgs is not None' in l][0]
    self.assertEqual(len(call_line) - len(call_line.lstrip()), len(guard_line) - len(guard_line.lstrip()),
                      'applyVisionSignDetect() call must be at the same indent as the trafficeventsMsgs guard, not inside it')

  def test_imports_the_reader_module(self):
    self.assertIn('from selfdrive.car.tesla import sign_detect_reader', CC)


class TestCanPackerRoundTrip(unittest.TestCase):
  """the actual CAN encoding applyVisionSignDetect() would send for a picked stop sign."""
  def setUp(self):
    try:
      import common.numpy_fast  # noqa: F401
    except Exception:
      m = types.ModuleType('common.numpy_fast'); m.clip = lambda x, a, b: max(a, min(b, x))
      c = types.ModuleType('common'); c.numpy_fast = m
      sys.modules['common'] = c; sys.modules['common.numpy_fast'] = m
    sys.path.insert(0, ROOT)
    import teslacan
    self.teslacan = teslacan

  def test_stop_sign_message_packs_without_raising(self):
    msg_id, bus, raw, _ = self.teslacan.create_fake_DAS_sign_msg(0x00, 15.0, 0, 0)
    self.assertEqual(msg_id, 0x556)
    self.assertEqual(len(raw), 4)

  def test_clear_message_uses_the_sentinel_type(self):
    msg_id, bus, raw, _ = self.teslacan.create_fake_DAS_sign_msg(0xFF, 1000., 0, 0)
    self.assertEqual(msg_id, 0x556)
    b0 = raw[0] if isinstance(raw[0], int) else ord(raw[0])
    self.assertEqual((b0 >> 6) & 0x03, 0xFF & 0x03)   # roadSignType's low 2 bits land in sign1's top bits

  def test_distance_round_trips_through_the_same_decode_the_IC_would_do(self):
    # mirrors create_fake_DAS_sign_msg's own encode (roadSignStopDist_t = (dist+20)/0.2, 10 bits
    # split across sign2/sign3) - decode it back and check it recovers the input distance
    msg_id, bus, raw, _ = self.teslacan.create_fake_DAS_sign_msg(0x00, 23.4, 0, 0)
    b = [x if isinstance(x, int) else ord(x) for x in raw]
    sign2, sign3 = b[1], b[2]
    t = ((sign2 >> 6) & 0x03) | (sign3 << 2)
    dist = t * 0.2 - 20
    self.assertAlmostEqual(dist, 23.4, delta=0.2)   # 0.2 m quantization


if __name__ == '__main__':
  unittest.main()
