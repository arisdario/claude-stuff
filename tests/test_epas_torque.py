#!/usr/bin/env python3
"""radarweb reads the real EPAS torsion-bar torque from 0x370, because carState.steeringTorque is hardcoded 0 in
this fork. This guards the decode the intervention recorder depends on.  python3 tests/test_epas_torque.py"""
import os
import sys
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')


def load_dec():
  src = open(os.path.join(ROOT, 'radarweb.py')).read()
  a = src.index('def _dec('); b = src.index('\ndef decode_one(')
  ns = {}; exec(src[a:b], ns)                     # the real _dec from radarweb.py
  return ns['_dec']


_dec = load_dec()


def encode_0x370(torque_nm):
  """inverse of _dec's Motorola walk for EPAS_torsionBarTorque 19|12@0+ (0.01, -20.5)"""
  raw = int(round((torque_nm + 20.5) / 0.01)); v = 0; i = 19
  for k in range(12):
    v |= ((raw >> (11 - k)) & 1) << ((i // 8) * 8 + i % 8)
    i = i + 15 if i % 8 == 0 else i - 1
  return v.to_bytes(8, 'little')


def inline_decode(frame):
  """the exact formula radarweb.py uses in the CAN loop for 0x370"""
  vb = int.from_bytes(frame, 'little'); raw = int(round(_dec(vb, 8, 19, 12, False, 1, 0, 0)))
  return None if raw >= 4094 else round(raw * 0.01 - 20.5, 2)


class TestEpasTorque(unittest.TestCase):
  def test_round_trip(self):
    for nm in (0.0, 2.0, -5.0, 15.3, -20.5, 18.75):
      self.assertAlmostEqual(inline_decode(encode_0x370(nm)), nm, places=2)

  def test_sna_is_none(self):
    self.assertIsNone(inline_decode(encode_0x370(20.44)))     # raw 4094/4095 = SNA/undefined

  def test_crosses_the_correction_threshold(self):
    import radarevents as E
    self.assertLess(abs(inline_decode(encode_0x370(1.0))), E.STEER_TQ_NM)    # light touch: not a correction
    self.assertGreaterEqual(abs(inline_decode(encode_0x370(-3.0))), E.STEER_TQ_NM)  # real pull


if __name__ == '__main__':
  sys.path.insert(0, ROOT)
  unittest.main()
