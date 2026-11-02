#!/usr/bin/env python3
"""Tests for tesla_csa validation, shadow default, fallback:  python3 -m unittest tesla.tests.test_tesla_csa"""
import ast
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
    self.en = os.path.join(self.d, 'enable'); self.off = os.path.join(self.d, 'off'); self.ev = os.path.join(self.d, 'evidence')

  def test_default_is_shadow(self):                  # no enable file -> no control influence
    self.assertIsNone(T.enabled_model(self.en, self.off, self.ev))

  def test_blank_or_unknown_enable_is_shadow(self):
    open(self.ev, 'w').write('x' * 40)
    for content in ('', '  \n', 'yes', 'true', 'CURVATURE'):
      open(self.en, 'w').write(content)
      self.assertIsNone(T.enabled_model(self.en, self.off, self.ev), content)

  def test_enable_file_alone_stays_shadow(self):               # the evidence file is required too
    open(self.en, 'w').write('clothoid')
    self.assertIsNone(T.enabled_model(self.en, self.off, self.ev))

  def test_trivial_evidence_is_not_enough(self):
    open(self.en, 'w').write('clothoid')
    for content in ('', '  \n', 'ok', 'verified'):
      open(self.ev, 'w').write(content)
      self.assertIsNone(T.enabled_model(self.en, self.off, self.ev), content)

  def test_enable_plus_evidence_enables_and_off_wins(self):
    open(self.ev, 'w').write('opendbc tesla_can.dbc @ abc1234: UI_csaRoadCurvC2 scale/unit/sign confirmed')
    open(self.en, 'w').write('Clothoid\n')
    self.assertEqual(T.enabled_model(self.en, self.off, self.ev), 'clothoid')
    open(self.en, 'w').write('offset')
    self.assertEqual(T.enabled_model(self.en, self.off, self.ev), 'offset')
    open(self.off, 'w').close()
    self.assertIsNone(T.enabled_model(self.en, self.off, self.ev))

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


class TestAuthorityCap(unittest.TestCase):
  """Review point 1: an unverified CSA curve must never cause full curve braking, in any fusion mode."""
  def test_cap_applies_when_csa_binds(self):
    self.assertEqual(T.limit_authority(1.0, True), T.AUTHORITY_CAP)      # fusion OFF default authority is 1.0
    self.assertLess(T.AUTHORITY_CAP, 1.0)
    self.assertEqual(T.limit_authority(0.95, True), T.AUTHORITY_CAP)     # high fusion authority is capped too

  def test_cap_never_raises_authority(self):
    self.assertEqual(T.limit_authority(0.1, True), 0.1)

  def test_other_sources_unchanged_when_csa_not_binding(self):
    self.assertEqual(T.limit_authority(1.0, False), 1.0)                 # vision/SQLite braking is not weakened

  def test_pcc_applies_cap_in_both_modes(self):
    src = open(os.path.join(os.path.dirname(__file__), '..', 'PCC_module.py')).read()
    self.assertIn('tesla_csa.combine_nonfusion(v_curve, v_map, v_csa, vis_cs)', src)   # fusion off
    self.assertIn('csa_binding = v_csa is not None and fus.binding == csa_src', src)  # fusion on
    cap = src.index('curve_authority = tesla_csa.limit_authority(curve_authority, csa_binding)')
    use = src.index('-pcc_curves.CURVE_BRAKE * curve_authority')
    self.assertLess(cap, use)                                                      # applied BEFORE it scales the brake
    import inspect
    self.assertIn('v_curve = min(v_curve, v_csa)', inspect.getsource(T.combine_nonfusion))   # floor only (min)
    self.assertIn('v_map = None', inspect.getsource(T.resolve))                                # no double count


class TestNotVerified(unittest.TestCase):
  """Review point 2: plausibility is not verification; control needs documented evidence."""
  def test_default_is_shadow_with_no_switch_files(self):
    d = tempfile.mkdtemp()
    self.assertIsNone(T.enabled_model(os.path.join(d, 'e'), os.path.join(d, 'o'), os.path.join(d, 'v')))
    self.assertEqual((T.ENABLE_FILE, T.EVIDENCE_FILE, T.OFF_FILE),
                     ('/data/tesla_csa_enable', '/data/tesla_csa_evidence', '/data/tesla_csa_off'))

  def test_docs_state_nothing_is_verified(self):
    d = T.__doc__
    self.assertIn('NEITHER', d.upper()); self.assertIn('NOT DONE', d)
    self.assertIn('Agreement with SQLite is NOT verification', d)
    self.assertIn('DBC', d); self.assertIn('validated vehicle data', d)

  def test_docs_name_units_reference_and_range(self):
    d = T.__doc__
    for needle in ('clothoid', 'offset', 'Range', 'distance AHEAD', 'reference point', '1/m'):
      self.assertIn(needle, d)

  def test_pcc_uses_csa_only_through_resolve(self):
    src = open(os.path.join(os.path.dirname(__file__), '..', 'PCC_module.py')).read()
    self.assertIn('tesla_csa.resolve(', src)                       # the only control-side use, and it is shadow unless enabled


class TestShadowLoggerFrames(unittest.TestCase):
  """Review point 3: zero/SNA/missing frames are logged too, with or without a SQLite curve and with PCC disengaged."""
  def setUp(self):
    self.path = os.path.join(tempfile.mkdtemp(), 'csa.log')
    self.L = T.ShadowLogger(self.path, 1000000)

  def lines(self):
    return [json.loads(x) for x in open(self.path).read().splitlines()]

  def test_zero_frame_logged_without_sqlite_and_disengaged(self):
    self.assertTrue(self.L.maybe_log((0.0, 0.0, 0.0, 0), (0.0, 0.0, 0.0, 0), None, None, 20.0, False, now=100.0))
    r = self.lines()[0]
    self.assertEqual(r['class'], {'road': 'zero', 'offramp': 'zero'})
    self.assertFalse(r['sqlite_present']); self.assertIsNone(r['sqlite']); self.assertFalse(r['pcc_engaged'])
    self.assertIn('rejected', r['csa']['clothoid'])                  # the rejection reason is recorded

  def test_missing_frame_logged(self):
    self.assertTrue(self.L.maybe_log(None, None, None, None, 0.0, False, now=1.0))
    r = self.lines()[0]
    self.assertEqual(r['class'], {'road': 'missing', 'offramp': 'missing'})
    self.assertIn('missing', r['csa']['offset']['rejected'])

  def test_sna_frame_logged(self):
    self.assertTrue(self.L.maybe_log((1e9, float('nan'), 150.0, 0), None, None, None, 5.0, True, now=1.0))
    self.assertEqual(self.lines()[0]['class']['road'], 'sna')

  def test_sqlite_present_next_to_zero_csa(self):
    self.L.maybe_log((0, 0, 0, 0), (0, 0, 0, 0), 14.0, 120.0, 25.0, True, now=1.0)
    r = self.lines()[0]
    self.assertTrue(r['sqlite_present']); self.assertEqual(r['sqlite']['speed'], 14.0)

  def test_unchanged_state_is_rate_limited_but_heartbeats(self):
    z = ((0, 0, 0, 0), (0, 0, 0, 0), None, None, 0.0, False)
    self.assertTrue(self.L.maybe_log(*z, now=0.0))
    self.assertFalse(self.L.maybe_log(*z, now=2.0))                  # same state, < heartbeat: no new line
    self.assertTrue(self.L.maybe_log(*z, now=11.0))                  # 10 s heartbeat
    self.assertEqual(len(self.lines()), 2)

  def test_state_change_logs_at_most_once_a_second(self):
    self.L.maybe_log((0, 0, 0, 0), None, None, None, 0.0, False, now=0.0)
    self.assertFalse(self.L.maybe_log((0.01, 0, 150, 0), None, None, None, 0.0, False, now=0.5))   # too soon
    self.assertTrue(self.L.maybe_log((0.01, 0, 150, 0), None, None, None, 0.0, False, now=1.2))

  def test_cumulative_counts_show_an_all_zero_drive(self):
    for t in range(0, 60, 2):
      self.L.maybe_log((0, 0, 0, 0), (0, 0, 0, 0), None, None, 20.0, False, now=float(t))
    last = self.lines()[-1]
    self.assertEqual(set(last['seen']['road']), {'zero'})            # never non-zero -> evidence of a pre-AP car
    self.assertEqual(last['seen']['road']['zero'], len(range(0, 51, 2)))   # counts as of the last heartbeat (t=50)

  def test_never_raises_on_unwritable_path(self):
    L = T.ShadowLogger(os.path.join(self.path, 'nodir', 'x'), 1000)
    self.assertFalse(L.maybe_log(None, None, None, None, 0.0, False, now=1.0))


PCC_SRC = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'PCC_module.py')).read()


class FakeCS(object):
  def __init__(self, c2=0.0, c3=0.0, rng=0.0, tsp=0, o2=0.0, o3=0.0, orng=0.0):
    self.csaRoadCurvC2, self.csaRoadCurvC3, self.csaRoadCurvRange, self.csaRoadCurvUsingTspline = c2, c3, rng, tsp
    self.csaOfframpCurvC2, self.csaOfframpCurvC3, self.csaOfframpCurvRange, self.csaOfframpCurvUsingTspline = o2, o3, orng, 0


VALID = dict(c2=0.01, c3=0.0, rng=150.0, tsp=1)


class TestShadowWhileDisengaged(unittest.TestCase):
  """Logging works with PCC disengaged, logs zero/SNA, and is not suppressed by SQLite having no curve."""
  def setUp(self):
    self.path = os.path.join(tempfile.mkdtemp(), 'csa.log')
    self.L = T.ShadowLogger(self.path, 1000000)

  def last(self):
    return json.loads(open(self.path).read().splitlines()[-1])

  def test_logs_while_disengaged(self):
    self.assertTrue(T.shadow_tick(self.L, FakeCS(**VALID), None, 25.0, engaged=False, now=10.0))
    r = self.last()
    self.assertFalse(r['pcc_engaged']); self.assertEqual(r['class']['road'], 'nonzero')
    self.assertIn('speed', r['csa']['clothoid'])

  def test_zero_frame_logged(self):
    self.assertTrue(T.shadow_tick(self.L, FakeCS(), None, 0.0, engaged=False, now=10.0))
    self.assertEqual(self.last()['class'], {'road': 'zero', 'offramp': 'zero'})

  def test_sna_frame_logged(self):
    self.assertTrue(T.shadow_tick(self.L, FakeCS(c2=1e9, c3=float('nan'), rng=150.0), None, 5.0, engaged=True, now=10.0))
    self.assertEqual(self.last()['class']['road'], 'sna')

  def test_missing_signals_logged(self):
    self.assertTrue(T.shadow_tick(self.L, object(), None, 0.0, engaged=False, now=10.0))     # CarState lacks the fields
    self.assertEqual(self.last()['class'], {'road': 'missing', 'offramp': 'missing'})

  def test_sqlite_absence_does_not_suppress_logging(self):
    self.assertTrue(T.shadow_tick(self.L, FakeCS(**VALID), None, 25.0, engaged=False, now=10.0))      # no SQLite state at all
    self.assertFalse(self.last()['sqlite_present'])
    # a stale / zero-curvature SQLite state counts as 'no curve', and logging still happens
    self.assertTrue(T.shadow_tick(self.L, FakeCS(), (0.0, 100.0, 20.0), 25.0, engaged=False, now=20.0))
    self.assertFalse(self.last()['sqlite_present'])
    self.assertTrue(T.shadow_tick(self.L, FakeCS(), (0.01, 100.0, 1.0), 25.0, engaged=False, now=40.0))   # 39 s old = stale
    self.assertFalse(self.last()['sqlite_present'])

  def test_sqlite_present_is_logged_next_to_csa(self):
    T.shadow_tick(self.L, FakeCS(**VALID), (0.0054, 200.0, 10.0), 25.0, engaged=False, now=10.5)
    r = self.last()
    self.assertTrue(r['sqlite_present']); self.assertAlmostEqual(r['sqlite']['speed'], math.sqrt(1.85 / 0.0054), places=1)

  def test_shadow_tick_never_raises(self):
    self.assertFalse(T.shadow_tick(T.ShadowLogger(os.path.join(self.path, 'nodir', 'x'), 10), FakeCS(), None, 0.0, False, now=1.0))
    self.assertFalse(T.shadow_tick(None, FakeCS(), None, 0.0, False))


class TestShadowLeavesControlUnchanged(unittest.TestCase):
  """With the lock closed (the repo default) CSA cannot alter any control input, even if an enable file is present."""
  GRID = [(vm, vc) for vm in (None, 9.0, 15.0, 40.0) for vc in (255.0, 30.0, 12.0)]

  def setUp(self):
    d = tempfile.mkdtemp()
    self.en = os.path.join(d, 'enable'); self.off = os.path.join(d, 'off'); self.ev = os.path.join(d, 'evidence')
    open(self.en, 'w').write('clothoid')                           # enable file present but NO evidence file => still shadow

  def approach(self, v, d, e):
    return math.sqrt(v * v + 2.0 * (max(0.0, d - e * 2.0)))

  def test_resolve_is_a_no_op_in_shadow(self):
    for cs in (FakeCS(**VALID), FakeCS(), object()):
      for vm, _ in self.GRID:
        v_csa, src, why, csa, v_map_out = T.resolve(cs, 25.0, vm, self.approach, self.en, self.off, self.ev)
        self.assertIsNone(v_csa); self.assertIsNone(csa)
        self.assertEqual(v_map_out, vm)                            # the SQLite curve passes through UNCHANGED
        self.assertEqual(src, 'sqlite' if vm else 'none')

  def test_curve_limit_identical_to_legacy_min(self):
    for vm, vc in self.GRID:
      v_csa, _s, _w, _c, v_map = T.resolve(FakeCS(**VALID), 25.0, vm, self.approach, self.en, self.off, self.ev)
      got, binding = T.combine_nonfusion(255.0, v_map, v_csa, vc)
      legacy = min(255.0, vm if vm else 255.0, vc)               # the pre-CSA formula
      self.assertEqual(got, legacy); self.assertFalse(binding)
      self.assertEqual(T.limit_authority(1.0, binding), 1.0)     # braking authority untouched (vision/SQLite not weakened)

  def test_same_outputs_as_a_car_with_no_csa_at_all(self):
    for vm, vc in self.GRID:
      a = T.resolve(FakeCS(**VALID), 25.0, vm, self.approach, self.en, self.off, self.ev)
      b = T.resolve(object(), 25.0, vm, self.approach, self.en, self.off, self.ev)
      self.assertEqual((a[0], a[4]), (b[0], b[4]))
      self.assertEqual(T.combine_nonfusion(255.0, a[4], a[0], vc), T.combine_nonfusion(255.0, b[4], b[0], vc))


class TestUnverifiedNeverFullBraking(unittest.TestCase):
  def setUp(self):
    d = tempfile.mkdtemp()
    self.en = os.path.join(d, 'enable'); self.off = os.path.join(d, 'off'); self.ev = os.path.join(d, 'evidence')
    open(self.en, 'w').write('clothoid')
    open(self.ev, 'w').write('operator-supplied evidence reference for the test')   # control enabled (both files)

  def approach(self, v, d, e):
    return v

  def test_even_when_enabled_authority_is_capped(self):
    v_csa, src, _w, csa, v_map = T.resolve(FakeCS(**VALID), 25.0, 15.0, self.approach, self.en, self.off, self.ev)
    self.assertIsNotNone(v_csa); self.assertEqual(src, 'tesla_csa'); self.assertIsNone(v_map)    # replaces SQLite
    self.assertLessEqual(csa.conf, 0.5)
    _v, binding = T.combine_nonfusion(255.0, v_map, v_csa, 255.0)
    self.assertTrue(binding)
    for fusion_authority in (1.0, 0.95, 0.5):                   # fusion off => 1.0; fusion on => up to ~1.0
      self.assertLessEqual(T.limit_authority(fusion_authority, binding), T.AUTHORITY_CAP)
      self.assertLess(T.limit_authority(fusion_authority, binding), 1.0)

  def test_enabled_csa_is_a_floor_never_raises_the_limit(self):
    v_csa, _s, _w, _c, v_map = T.resolve(FakeCS(**VALID), 25.0, None, lambda v, d, e: 40.0, self.en, self.off, self.ev)
    v_curve, _b = T.combine_nonfusion(255.0, v_map, v_csa, 20.0)     # vision says 20, CSA says 40
    self.assertEqual(v_curve, 20.0)                                  # the stricter source still wins

  def test_pcc_applies_the_cap_before_scaling_the_brake(self):
    cap = PCC_SRC.index('curve_authority = tesla_csa.limit_authority(curve_authority, csa_binding)')
    use = PCC_SRC.index('-pcc_curves.CURVE_BRAKE * curve_authority')
    self.assertLess(cap, use)
    self.assertIn('csa_binding = v_csa is not None and fus.binding == csa_src', PCC_SRC)


class TestOffSwitchOverridesEverything(unittest.TestCase):
  def test_every_enable_configuration(self):
    d = tempfile.mkdtemp()
    en = os.path.join(d, 'enable'); off = os.path.join(d, 'off'); ev = os.path.join(d, 'evidence')
    open(off, 'w').close()
    for evidence in (None, '', 'short', 'a full evidence citation for opendbc commit abc1234'):
      if evidence is not None:
        open(ev, 'w').write(evidence)
      elif os.path.exists(ev):
        os.remove(ev)
      for content in ('clothoid', 'offset', 'Clothoid\n', 'OFFSET', 'yes', ''):
        open(en, 'w').write(content)
        self.assertIsNone(T.enabled_model(en, off, ev), (evidence, content))
        v_csa, _s, _w, csa, v_map = T.resolve(FakeCS(**VALID), 25.0, 15.0, lambda v, dd, e: v, en, off, ev)
        self.assertIsNone(v_csa); self.assertIsNone(csa); self.assertEqual(v_map, 15.0)

  def test_removing_the_off_file_restores_enable(self):             # proves the off file was the only thing blocking it
    d = tempfile.mkdtemp()
    en = os.path.join(d, 'enable'); off = os.path.join(d, 'off'); ev = os.path.join(d, 'evidence')
    open(en, 'w').write('offset'); open(ev, 'w').write('x' * 30); open(off, 'w').close()
    self.assertIsNone(T.enabled_model(en, off, ev))
    os.remove(off)
    self.assertEqual(T.enabled_model(en, off, ev), 'offset')

  def test_off_switch_does_not_stop_shadow_logging(self):             # off disables control influence, not observation
    d = tempfile.mkdtemp(); off = os.path.join(d, 'off'); open(off, 'w').close()
    L = T.ShadowLogger(os.path.join(d, 'log'), 10 ** 6)
    self.assertTrue(T.shadow_tick(L, FakeCS(), None, 0.0, False, now=1.0))


class TestPccTickCannotTouchControlState(unittest.TestCase):
  """Structural proof on the real PCC_module source: the tick that runs before the disengaged early return reads signals,
  validates and logs only."""
  FORBIDDEN = {'v_pid', 'v_curve', 'v_map', 'v_follow', 'accel_limits', 'jerk_limits', 'curve_authority', 'csa_binding',
               '_map_curve', 'fleet_speed', 'pedal_speed_kph', 'prev_tesla_accel', 'apply_accel', 'accel_needed'}

  def tick_node(self):
    for n in ast.walk(ast.parse(PCC_SRC)):
      if isinstance(n, ast.FunctionDef) and n.name == '_csa_shadow_tick':
        return n
    self.fail('_csa_shadow_tick not found')

  def test_tick_references_no_control_variable(self):
    names = set()
    for n in ast.walk(self.tick_node()):
      if isinstance(n, ast.Name): names.add(n.id)
      if isinstance(n, ast.Attribute): names.add(n.attr)
    self.assertFalse(names & self.FORBIDDEN, names & self.FORBIDDEN)

  def test_tick_assigns_only_its_own_state(self):
    targets = set()
    for n in ast.walk(self.tick_node()):
      if isinstance(n, (ast.Assign, ast.AugAssign, ast.AnnAssign)):
        for tg in (n.targets if isinstance(n, ast.Assign) else [n.target]):
          for x in ast.walk(tg):
            if isinstance(x, ast.Attribute): targets.add('self.' + x.attr)
            elif isinstance(x, ast.Name): targets.add(x.id)
    self.assertLessEqual(targets, {'self._csa_sql', 'm', 'lmd', 'self'})

  def test_only_the_tick_call_sits_above_the_early_return(self):
    tick = PCC_SRC.index('self._csa_shadow_tick(CS, enabled)')
    early = PCC_SRC.index('if not self.pcc_available or not enabled:')
    between = PCC_SRC[tick:early].strip().splitlines()
    self.assertEqual(len([l for l in between if l.strip() and not l.strip().startswith('#')]), 1)   # just the call

  def test_control_path_message_handling_is_unmoved(self):
    early = PCC_SRC.index('if not self.pcc_available or not enabled:')
    self.assertGreater(PCC_SRC.index('mapd = messaging.recv_one_or_none(self.live_map_data)'), early)   # still after, as originally
    self.assertGreater(PCC_SRC.index('self._map_curve = (lmd.curvature'), early)
    self.assertIn("self._csa_map_sock = messaging.sub_sock('liveMapData', conflate=True)", PCC_SRC)      # the tick's own subscriber


if __name__ == '__main__':
  unittest.main()
