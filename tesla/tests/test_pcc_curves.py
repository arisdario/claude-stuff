#!/usr/bin/env python3
"""Unit tests for pcc_curves.py (no openpilot needed):  python3 -m unittest tesla/tests/test_pcc_curves.py"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import pcc_curves as P  # noqa: E402


def path_straight_then_bend(straight_m, radius_m, n=192):
  """y offset per metre: straight for straight_m, then a circular arc of radius_m (approximated)"""
  y = []
  for x in range(n):
    s = max(0., x - straight_m)
    y.append(radius_m - math.sqrt(max(0., radius_m ** 2 - min(s, radius_m * 0.9) ** 2)))
  return y


class TestCurves(unittest.TestCase):
  def test_approach_speed_is_curve_speed_at_the_curve(self):
    self.assertAlmostEqual(P.approach_speed(10., 0., 0.), 10., places=6)

  def test_approach_speed_grows_with_distance(self):
    v = [P.approach_speed(10., d, 20.) for d in (0, 50, 100, 200, 400)]
    self.assertEqual(v, sorted(v))
    # 200 m ahead at 20 m/s: 20 m/s x CURVE_REACTION_S of margin, then the rest at A_CURVE_DECEL
    d = 200. - 20. * P.CURVE_REACTION_S
    self.assertAlmostEqual(P.approach_speed(10., 200., 20.), math.sqrt(100 + 2 * P.A_CURVE_DECEL * d), places=6)

  def test_map_limit_not_reversed(self):
    # the old code allowed full speed AT the turn; now the limit at the turn is the curve speed itself
    k = P.A_Y_MAP / 12. ** 2                      # stored curve speed 12 m/s
    self.assertAlmostEqual(P.map_curve_limit(k, 0., 0.), 12., places=4)
    self.assertGreater(P.map_curve_limit(k, 300., 20.), P.map_curve_limit(k, 50., 20.))
    self.assertIsNone(P.map_curve_limit(0., 100., 20.))

  def test_vision_straight_road_no_limit_below_cruise(self):
    v = P.vision_curve_limit(np, [0.] * 192, 25.)
    self.assertGreater(v, 40.)

  def test_vision_near_bend_limits_more_than_far_bend(self):
    near = P.vision_curve_limit(np, path_straight_then_bend(20, 40.), 20.)
    far = P.vision_curve_limit(np, path_straight_then_bend(90, 40.), 20.)
    self.assertLess(near, far)
    # a 40 m radius bend: curve speed sqrt(a_y * R) ~ 9.5 m/s; 20 m ahead at 20 m/s it is still inside the reaction margin
    self.assertLess(near, 12.)

  def test_vision_noise_does_not_spike(self):
    rs = np.random.RandomState(0)
    noisy = list(rs.normal(0, 0.05, 192))         # 5 cm jitter on a straight road
    self.assertGreater(P.vision_curve_limit(np, noisy, 25.), 20.)

  def test_vision_too_short(self):
    self.assertIsNone(P.vision_curve_limit(np, [0.] * 10, 20.))

  def test_lateral_accel(self):
    self.assertAlmostEqual(P.lateral_accel(20., 0., 11.5, 2.96), 0.)
    self.assertGreater(P.lateral_accel(20., 30., 11.5, 2.96), 5.)


class TestAccelAndPedal(unittest.TestCase):
  def test_accel_profile(self):
    self.assertAlmostEqual(P.accel_max_no_lead(0.), P.ACCEL_V[0])
    self.assertAlmostEqual(P.accel_max_no_lead(100.), P.ACCEL_V[-1])

  def test_pedal_map_starts_at_old_constant(self):
    m = P.PedalMap()
    self.assertAlmostEqual(m.gain(20.), P.PEDAL_K_DEFAULT)

  def test_pedal_map_learns_and_saves(self):
    path = os.path.join(tempfile.mkdtemp(), 'pm.json')
    m = P.PedalMap(path)
    for _ in range(3000):                         # at 25 m/s, 0.5 pedal gives 0.75 m/s2 -> k = 1.5
      m.update(0.5, 0.75, 25., True)
    self.assertLess(abs(m.gain(25.) - 1.5), 0.1)
    self.assertAlmostEqual(m.gain(5.), P.PEDAL_K_DEFAULT, places=3)   # other speed bands untouched
    m.save()
    self.assertLess(abs(P.PedalMap(path).gain(25.) - 1.5), 0.1)

  def test_pedal_map_ignores_bad_samples(self):
    m = P.PedalMap()
    for _ in range(500): m.update(0.02, 2.0, 20., True)     # pedal too small
    for _ in range(500): m.update(0.5, 0.05, 20., True)     # acceleration too small
    for _ in range(500): m.update(0.5, 3.0, 20., False)     # conditions not ok
    self.assertAlmostEqual(m.gain(20.), P.PEDAL_K_DEFAULT)

  def test_pedal_map_needs_conditions_for_whole_delay(self):
    m = P.PedalMap()
    for i in range(600):                          # ok only 1 step in 3: never ok for the whole 0.3 s window
      m.update(0.5, 0.75, 25., i % 3 == 0)
    self.assertAlmostEqual(m.gain(25.), P.PEDAL_K_DEFAULT)


if __name__ == '__main__':
  unittest.main()
