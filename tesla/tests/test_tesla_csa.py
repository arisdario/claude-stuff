#!/usr/bin/env python3
"""Tests for tesla_csa validation/fallback:  python3 -m unittest tesla.tests.test_tesla_csa"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tesla_csa as T  # noqa: E402


class TestValidation(unittest.TestCase):
  def test_zero_frame_is_unavailable(self):          # pre-AP car: all zeros
    c, why = T.csa_status(0.0, 0.0, 150.0)
    self.assertIsNone(c); self.assertIn('zero', why)

  def test_non_finite_and_sna(self):
    for bad in (float('nan'), float('inf'), 1e9, None, 'x'):
      self.assertIsNone(T.csa_curve(bad, 0.0, 150.0))
      self.assertIsNone(T.csa_curve(0.01, 0.0, bad))

  def test_range_out_of_band(self):
    self.assertIsNone(T.csa_curve(0.01, 0.0, 0.0))
    self.assertIsNone(T.csa_curve(0.01, 0.0, 5.0))
    self.assertIsNone(T.csa_curve(0.01, 0.0, 5000.0))

  def test_implausible_radius_rejected(self):        # R = 2 m is garbage, not a road
    c, why = T.csa_status(0.5, 0.0, 150.0)
    self.assertIsNone(c); self.assertIn('implausible', why)

  def test_straight_is_no_constraint(self):
    c, why = T.csa_status(1e-5, 0.0, 150.0)
    self.assertIsNone(c); self.assertIn('straight', why)


class TestSpeed(unittest.TestCase):
  def test_constant_curvature(self):                 # R=100 m -> sqrt(1.85*100) = 13.6 m/s
    c = T.csa_curve(0.01, 0.0, 150.0)
    self.assertAlmostEqual(c.speed, math.sqrt(1.85 * 100.0), places=3)
    self.assertEqual(c.dist, 0.0); self.assertEqual(c.source, 'tesla_csa')

  def test_curvature_grows_ahead(self):              # tightest point is at Range
    c = T.csa_curve(0.002, 0.0001, 150.0)            # k(150)=0.017
    self.assertEqual(c.dist, 150.0)
    self.assertAlmostEqual(c.curvature, 0.017, places=6)

  def test_sign_does_not_change_speed(self):
    self.assertAlmostEqual(T.csa_curve(0.01, 0, 150).speed, T.csa_curve(-0.01, 0, 150).speed)
    self.assertLess(T.csa_curve(-0.01, 0, 150).curvature, 0)

  def test_confidence_capped_and_spline_higher(self):
    self.assertLess(T.csa_curve(0.01, 0, 150, 0).conf, T.csa_curve(0.01, 0, 150, 1).conf)
    self.assertLess(T.csa_curve(0.01, 0, 150, 1).conf, 1.0)


class TestBest(unittest.TestCase):
  def test_more_restrictive_wins_single_result(self):  # no double counting: ONE result
    c, why = T.best_csa((0.005, 0, 150, 0), (0.02, 0, 100, 0))
    self.assertEqual(c.source, 'tesla_csa_offramp')

  def test_falls_back_when_all_invalid(self):
    c, why = T.best_csa((0, 0, 0, 0), (0, 0, 0, 0))
    self.assertIsNone(c); self.assertIn('tesla_csa', why)

  def test_one_valid_one_invalid(self):
    c, _ = T.best_csa((0, 0, 0, 0), (0.01, 0, 100, 0))
    self.assertEqual(c.source, 'tesla_csa_offramp')


class TestWiring(unittest.TestCase):
  def test_pcc_uses_csa_only_as_floor_with_killswitch(self):
    src = open(os.path.join(os.path.dirname(__file__), '..', 'PCC_module.py')).read()
    self.assertIn('tesla_csa.best_csa', src)
    self.assertIn('v_curve = min(v_curve, v_csa)', src)      # floor only
    self.assertIn('tesla_csa.OFF_FILE', src)
    self.assertIn("v_map = None                      # CSA is the map-curve source", src)   # no double count


if __name__ == '__main__':
  unittest.main()
