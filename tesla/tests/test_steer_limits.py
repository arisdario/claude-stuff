#!/usr/bin/env python3
"""steer_limits: opt-in angle limit layer.  python3 tesla/tests/test_steer_limits.py"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import steer_limits as S  # noqa: E402


class T(unittest.TestCase):
  def test_off_is_identity(self):
    for req in (-300., 0., 7.3, 120.):
      self.assertEqual(S.limit_angle(req, 0., 0., 30., 0.0), req)

  def test_full_strength_rate_and_unit_scaling(self):
    # 30 m/s windup 0.75 deg per 20 ms step -> 0.375 deg per 10 ms call
    self.assertAlmostEqual(S.limit_angle(10., 0., 0., 30., 1.0), 0.375)
    # unwinding (towards zero) is allowed faster: 2.4 per step -> 1.2 per call
    self.assertAlmostEqual(S.limit_angle(0., 10., 10., 30., 1.0), 10. - 1.2)
    # crossing zero counts as unwind
    self.assertAlmostEqual(S.limit_angle(-5., 3., 3., 30., 1.0), 3. - 1.2)

  def test_speed_interpolation(self):
    a = S.limit_angle(100., 0., 0., 5., 1.0); self.assertAlmostEqual(a, 2.0)         # 4 deg/step * 0.5
    b = S.limit_angle(100., 0., 0., 10., 1.0); self.assertAlmostEqual(b, (4. + (0.75 - 4.) * 0.5) * 0.5)

  def test_clip_to_measured(self):
    self.assertAlmostEqual(S.limit_angle(100., 99., 50., 0., 1.0), 70.)               # 50 + 20, rate allows it at 0 m/s

  def test_partial_strength_is_a_blend(self):
    full = S.limit_angle(10., 0., 0., 30., 1.0); part = S.limit_angle(10., 0., 0., 30., 0.25)
    self.assertAlmostEqual(part, 10. + 0.25 * (full - 10.))
    self.assertGreater(part, full); self.assertLess(part, 10.)

  def test_flag_file(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'on')
    self.assertEqual(S.read_strength(p), 0.0)                                          # absent = off
    open(p, 'w').write('')
    self.assertEqual(S.read_strength(p), 0.25)                                         # empty = default
    open(p, 'w').write('0.6\n'); self.assertEqual(S.read_strength(p), 0.6)
    open(p, 'w').write('7'); self.assertEqual(S.read_strength(p), 1.0)
    open(p, 'w').write('-1'); self.assertEqual(S.read_strength(p), 0.0)
    open(p, 'w').write('abc'); self.assertEqual(S.read_strength(p), 0.25)
    os.remove(p); os.rmdir(d)


if __name__ == '__main__':
  unittest.main()
