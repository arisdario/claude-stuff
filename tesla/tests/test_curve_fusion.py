#!/usr/bin/env python3
"""curve_fusion: the 10 predictive-speed scenarios + safety invariants.  python3 tesla/tests/test_curve_fusion.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import curve_fusion as F  # noqa: E402

KMH = 1 / 3.6


def vis(speed, conf=0.9, valid=True): return {'name': 'vision', 'kind': 'vision', 'speed': speed, 'conf': conf, 'valid': valid}
def mp(speed, conf=0.7, valid=True):  return {'name': 'map', 'kind': 'map', 'speed': speed, 'conf': conf, 'valid': valid}
def route(speed, conf=0.8, valid=True): return {'name': 'route', 'kind': 'route', 'speed': speed, 'conf': conf, 'valid': valid}
def lead(speed, conf=1.0, valid=True): return {'name': 'lead', 'kind': 'lead', 'speed': speed, 'conf': conf, 'valid': valid}


class TestScenarios(unittest.TestCase):
  def test_1_highway_sharp_right_exit_agreement(self):
    # vision + map both see ~60 km/h exit while cruising 130 -> target 60, corroborated -> high decel authority
    r = F.fuse([vis(60 * KMH), mp(62 * KMH)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 60 * KMH, places=2)
    self.assertGreater(r.conf, 0.9); self.assertGreater(r.decel_authority, 0.9)
    self.assertIn('vision', r.binding + r.note)

  def test_2_sharp_left_curve(self):
    r = F.fuse([vis(70 * KMH, conf=0.95)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 70 * KMH, places=2); self.assertEqual(r.binding, 'vision')

  def test_3_gps_curve_vision_straight_disagreement(self):
    # map says 55 (curve), vision says 130 (straight). Restrictive (map) still acts, but single-source -> reduced authority.
    r = F.fuse([mp(55 * KMH, conf=0.6), vis(130 * KMH, conf=0.9)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 55 * KMH, places=2)
    self.assertEqual(r.binding, 'map'); self.assertIn('single source', r.note)
    self.assertLess(r.decel_authority, 0.9)   # not full-send on an uncorroborated map curve

  def test_4_vision_curve_no_map(self):
    # vision sees a curve, map has no data -> vision alone triggers the reduction
    r = F.fuse([vis(50 * KMH, conf=0.9), mp(0, valid=False)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 50 * KMH, places=2); self.assertEqual(r.binding, 'vision')
    self.assertIn('vision', r.used); self.assertNotIn('map', r.used)

  def test_5_slower_lead_before_curve(self):
    # lead wants 80, curve wants 95 -> lead is more restrictive -> lead binds, full authority
    r = F.fuse([vis(95 * KMH), lead(80 * KMH)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 80 * KMH, places=2)
    self.assertEqual(r.binding, 'lead'); self.assertEqual(r.decel_authority, 1.0)

  def test_6_gps_lost_vision_continues(self):
    r = F.fuse([vis(60 * KMH, conf=0.9), mp(60 * KMH, valid=False)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 60 * KMH, places=2); self.assertEqual(r.binding, 'vision')

  def test_7_route_and_vision_agree_on_exit(self):
    r = F.fuse([route(55 * KMH, conf=0.8), vis(58 * KMH, conf=0.9)], 130 * KMH, 130 * KMH)
    self.assertLessEqual(r.target, 56 * KMH); self.assertGreater(r.conf, 0.9)   # corroborated

  def test_8_uncertain_road_low_conf_reduces_authority(self):
    high = F.fuse([vis(60 * KMH, conf=0.95)], 130 * KMH, 130 * KMH).decel_authority
    low = F.fuse([vis(60 * KMH, conf=0.3)], 130 * KMH, 130 * KMH).decel_authority
    self.assertLess(low, high)                       # shaky reading -> gentler braking authority

  def test_9_multiple_curves_most_restrictive(self):
    r = F.fuse([vis(70 * KMH, conf=0.9), mp(45 * KMH, conf=0.8)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 45 * KMH, places=2)   # tightest of the consecutive curves wins

  def test_10_short_distance_high_decel(self):
    urgent = F.approach_decel(130 * KMH, 60 * KMH, dist_m=40, reaction_s=1.0, a_max=3.0)
    early = F.approach_decel(130 * KMH, 60 * KMH, dist_m=400, reaction_s=1.0, a_max=3.0)
    self.assertGreater(urgent, early); self.assertLessEqual(urgent, 3.0)   # clamped to a_max
    self.assertEqual(F.approach_decel(60 * KMH, 70 * KMH, 100, 1.0, 3.0), 0.0)  # already slow enough


class TestSafetyInvariants(unittest.TestCase):
  def test_source_never_raises_speed(self):
    # a fast, confident source must not override a slower restrictive one
    r = F.fuse([vis(40 * KMH, conf=0.9), mp(120 * KMH, conf=0.95)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 40 * KMH, places=2)

  def test_all_invalid_returns_none(self):
    r = F.fuse([vis(50 * KMH, valid=False), mp(50 * KMH, valid=False)], 130 * KMH, 130 * KMH)
    self.assertIsNone(r.target); self.assertEqual(r.decel_authority, 0.0)

  def test_below_conf_min_dropped(self):
    r = F.fuse([vis(50 * KMH, conf=0.05)], 130 * KMH, 130 * KMH)
    self.assertIsNone(r.target)

  def test_lead_always_hard_even_with_bad_curve(self):
    # curve source invalid, lead valid -> lead still enforced
    r = F.fuse([vis(50 * KMH, valid=False), lead(70 * KMH)], 130 * KMH, 130 * KMH)
    self.assertAlmostEqual(r.target, 70 * KMH, places=2); self.assertEqual(r.binding, 'lead')

  def test_decel_authority_bounded(self):
    for conf in (0.0, 0.2, 0.5, 1.0):
      a = F.fuse([vis(60 * KMH, conf=max(conf, F.CONF_MIN))], 130 * KMH, 130 * KMH).decel_authority
      self.assertGreaterEqual(a, 0.0); self.assertLessEqual(a, 1.0)


if __name__ == '__main__':
  unittest.main()
