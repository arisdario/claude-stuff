#!/usr/bin/env python3
"""Tests for tesla_csa validation, shadow default, fallback:  python3 -m unittest tesla.tests.test_tesla_csa"""
import json
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import tesla_csa as T  # noqa: E402


class TestRejection(unittest.TestCase):
  def test_zero_frame_is_unavailable(self):          # pre-AP car: all zeros
    c, why = T.csa_status(0.0, 0.0, 150.0)
    self.assertIsNone(c); self.assertIn('zero', why)

  def test_nan_and_infinity(self):
    for bad in (float('nan'), float('inf'), float('-inf')):
      for args in ((bad, 0.0, 150.0), (0.01, bad, 150.0), (0.01, 0.0, bad)):
        c, why = T.csa_status(*args)
        self.assertIsNone(c, args); self.assertIn('non-finite', why)

  def test_sna_saturation_and_wrong_types(self):
    for bad in (1e9, -1e9, None, 'x', True):
      self.assertIsNone(T.csa_curve(bad, 0.0, 150.0))
      self.assertIsNone(T.csa_curve(0.01, 0.0, bad))

  def test_invalid_ranges(self):
    for r in (-10.0, 0.0, 5.0, 19.9, 400.1, 5000.0):
      c, why = T.csa_status(0.01, 0.0, r)
      self.assertIsNone(c, r); self.assertIn('range', why)

  def test_implausible_curvature(self):              # R = 2 m is garbage, not a road
    for model in T.MODELS:
      c, why = T.csa_status(0.5, 0.0, 150.0, model=model)
      self.assertIsNone(c); self.assertIn('implausible', why)

  def test_straight_is_no_constraint(self):
    c, why = T.csa_status(1e-5, 0.0, 150.0)
    self.assertIsNone(c); self.assertIn('straight', why)

  def test_unknown_model_raises(self):
    with self.assertRaises(ValueError):
      T.csa_status(0.01, 0.0, 150.0, model='bogus')


class TestValid(unittest.TestCase):
  def test_clothoid_constant_curvature(self):        # R=100 m -> sqrt(1.85*100)
    c = T.csa_curve(0.01, 0.0, 150.0)
    self.assertAlmostEqual(c.speed, math.sqrt(1.85 * 100.0), places=3)
    self.assertEqual(c.dist, 0.0); self.assertEqual((c.source, c.model), ('tesla_csa', 'clothoid'))

  def test_clothoid_tightens_ahead(self):            # k(150)=0.002+150*0.0001=0.017, tightest at Range
    c = T.csa_curve(0.002, 0.0001, 150.0)
    self.assertEqual(c.dist, 150.0); self.assertAlmostEqual(c.curvature, 0.017, places=6)

  def test_offset_hypothesis_is_different(self):     # y=C2 x^2 -> k=2*C2: the same numbers mean a different curve
    a = T.csa_curve(0.005, 0.0, 150.0, model='clothoid')   # k=0.005
    b = T.csa_curve(0.005, 0.0, 150.0, model='offset')     # k=0.010
    self.assertAlmostEqual(b.curvature, 2 * a.curvature)
    self.assertLess(b.speed, a.speed)

  def test_sign_does_not_change_speed(self):
    self.assertAlmostEqual(T.csa_curve(0.01, 0, 150).speed, T.csa_curve(-0.01, 0, 150).speed)
    self.assertLess(T.csa_curve(-0.01, 0, 150).curvature, 0)

  def test_confidence_is_capped_low(self):           # units unverified -> never high
    self.assertLess(T.csa_curve(0.01, 0, 150, 0).conf, T.csa_curve(0.01, 0, 150, 1).conf)
    self.assertLessEqual(T.csa_curve(0.01, 0, 150, 1).conf, 0.5)


class TestMissing(unittest.TestCase):
  def test_missing_signals(self):
    c, why = T.best_csa(None, None)
    self.assertIsNone(c); self.assertIn('missing', why)

  def test_read_inputs_missing_attributes(self):
    class CS(object): pass
    self.assertEqual(T.read_inputs(CS()), (None, None))

  def test_read_inputs_present(self):
    class CS(object):
      csaRoadCurvC2 = 0.01; csaRoadCurvC3 = 0.0; csaRoadCurvRange = 100.0; csaRoadCurvUsingTspline = 1
    road, off = T.read_inputs(CS())
    self.assertEqual(road, (0.01, 0.0, 100.0, 1)); self.assertIsNone(off)

  def test_best_more_restrictive_wins_one_result(self):   # alternatives, never summed
    c, _ = T.best_csa((0.005, 0, 150, 0), (0.02, 0, 100, 0))
    self.assertEqual(c.source, 'tesla_csa_offramp')

  def test_falls_back_when_all_invalid(self):
    c, why = T.best_csa((0, 0, 0, 0), (0, 0, 0, 0))
    self.assertIsNone(c); self.assertIn('tesla_csa', why)

  def test_one_valid_one_invalid(self):
    c, _ = T.best_csa((0, 0, 0, 0), (0.01, 0, 100, 0))
    self.assertEqual(c.source, 'tesla_csa_offramp')


class TestShadowDefault(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.en = os.path.join(self.d, 'enable'); self.off = os.path.join(self.d, 'off')

  def test_default_is_shadow(self):                  # no enable file -> no control influence
    self.assertIsNone(T.enabled_model(self.en, self.off))

  def test_blank_or_unknown_enable_is_shadow(self):
    for content in ('', '  \n', 'yes', 'true', 'CURVATURE'):
      open(self.en, 'w').write(content)
      self.assertIsNone(T.enabled_model(self.en, self.off), content)

  def test_explicit_model_enables(self):
    open(self.en, 'w').write('Clothoid\n')
    self.assertEqual(T.enabled_model(self.en, self.off), 'clothoid')
    open(self.en, 'w').write('offset')
    self.assertEqual(T.enabled_model(self.en, self.off), 'offset')

  def test_off_switch_beats_enable(self):
    open(self.en, 'w').write('clothoid'); open(self.off, 'w').close()
    self.assertIsNone(T.enabled_model(self.en, self.off))

  def test_shadow_record_side_by_side(self):
    rec = T.shadow_record((0.01, 0, 150, 1), (0, 0, 0, 0), v_sqlite=14.0, sqlite_dist=120.0, v_ego=25.0)
    self.assertEqual(rec['sqlite']['speed'], 14.0)
    self.assertIn('speed', rec['csa']['clothoid']); self.assertIn('speed', rec['csa']['offset'])
    zero = T.shadow_record((0, 0, 0, 0), None)
    self.assertIn('rejected', zero['csa']['clothoid'])      # rejection reason logged, not hidden
    self.assertIn('missing', zero['csa']['offset']['rejected'])
    json.dumps(rec)                                          # serialisable

  def test_log_shadow_appends_and_caps(self):
    p = os.path.join(self.d, 'log')
    self.assertTrue(T.log_shadow({'a': 1}, p, 1000))
    self.assertEqual(json.loads(open(p).read().splitlines()[0]), {'a': 1})
    open(p, 'a').write('x' * 2000)
    self.assertFalse(T.log_shadow({'a': 2}, p, 1000))        # over the cap: stops, never fills /data
    self.assertFalse(T.log_shadow({'a': 3}, os.path.join(self.d, 'nodir', 'x'), 1000))   # never raises


class TestWiring(unittest.TestCase):
  def setUp(self):
    self.src = open(os.path.join(os.path.dirname(__file__), '..', 'PCC_module.py')).read()

  def test_control_only_via_explicit_enable_and_floor(self):
    self.assertIn('tesla_csa.enabled_model()', self.src)
    self.assertIn('if model is not None:', self.src)
    self.assertIn('v_curve = min(v_curve, v_csa)', self.src)                       # floor only
    self.assertIn("v_map = None                      # CSA is the map-curve source", self.src)   # no double count

  def test_shadow_logging_wired(self):
    self.assertIn('tesla_csa.log_shadow', self.src)


if __name__ == '__main__':
  unittest.main()
