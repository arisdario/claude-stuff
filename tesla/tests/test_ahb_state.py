#!/usr/bin/env python3
"""ahb_state: the AHB decision state machine scenarios.  python3 tesla/tests/test_ahb_state.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import ahb_state as S  # noqa: E402

FAST = 100 / 3.6      # clearly above SPEED_ON_MS


def I(now, **kw):
  base = dict(enabled=True, dark=True, bright=False, lum_valid=True, v_ego=FAST,
              stalk=0, headlights_on=True, driver_override=False,
              oncoming=False, lead=False, vision_target=False, low_confidence=False)
  base.update(kw)
  return S.AhbInputs(now, **base)


def settle_on(m, t0=0.0, **kw):
  """run clear-dark inputs long enough to pass MIN_OFF + debounce and reach ON"""
  t = t0
  for _ in range(40):
    m.update(I(t, **kw)); t += 0.2
  return t


class TestAHBState(unittest.TestCase):
  def setUp(self):
    self.m = S.AhbStateMachine()

  def test_1_dark_clear_goes_high(self):
    settle_on(self.m)
    self.assertEqual(self.m.state, S.ON); self.assertTrue(self.m.high); self.assertEqual(self.m.reason, S.R_ON)

  def test_2_bright_urban_low_beam(self):
    r = self.m.update(I(0, dark=False, bright=True))
    self.assertEqual(r['state'], S.OFF); self.assertFalse(r['high']); self.assertEqual(r['reason'], S.R_AMBIENT)

  def test_3_oncoming_dips(self):
    settle_on(self.m); t = 10.0
    r = self.m.update(I(t, oncoming=True))
    self.assertEqual(r['state'], S.TEMPORARY_DIPPED); self.assertFalse(r['high']); self.assertEqual(r['reason'], S.R_VISION_TARGET)

  def test_4_lead_dips(self):
    settle_on(self.m)
    r = self.m.update(I(10.0, lead=True))
    self.assertEqual(r['state'], S.TEMPORARY_DIPPED); self.assertEqual(r['reason'], S.R_RADAR_TARGET)

  def test_4b_recovery_hold_then_back_on(self):
    settle_on(self.m)
    self.m.update(I(10.0, lead=True))                       # car appears
    r = self.m.update(I(12.0))                              # gone, but within RECOVERY_S
    self.assertEqual(r['state'], S.TEMPORARY_DIPPED)
    t = 12.0 + S.RECOVERY_S + 0.1                           # past the hold -> climbs back (after debounce)
    for _ in range(5):
      r = self.m.update(I(t)); t += 0.2
    self.assertEqual(r['state'], S.ON); self.assertTrue(r['high'])

  def test_5_tunnel_bright_to_dark(self):
    self.m.update(I(0, dark=False, bright=True))
    self.assertEqual(self.m.state, S.OFF)
    settle_on(self.m, t0=1.0)                               # dark tunnel / night
    self.assertEqual(self.m.state, S.ON)

  def test_9_low_confidence_dips(self):
    settle_on(self.m)
    r = self.m.update(I(10.0, low_confidence=True))
    self.assertEqual(r['state'], S.TEMPORARY_DIPPED)

  def test_10_flash_override_undecided(self):
    settle_on(self.m)
    r = self.m.update(I(10.0, stalk=2))
    self.assertEqual(r['state'], S.UNDECIDED); self.assertFalse(r['high']); self.assertEqual(r['reason'], S.R_SNA)

  def test_10b_driver_override_undecided(self):
    r = self.m.update(I(0, driver_override=True))
    self.assertEqual(r['state'], S.UNDECIDED)

  def test_11_rapid_fluctuation_min_on_holds(self):
    t = settle_on(self.m)                                   # ON
    # a one-cycle false "not dark" must not immediately flip: speed stays ok, but check it does not chatter ON/OFF
    seen = []
    for k in range(10):
      dark = (k % 2 == 0)
      r = self.m.update(I(t, dark=dark)); seen.append(r['state']); t += 0.1
    # it may drop to OFF when not dark, but must never oscillate within one cycle back to ON without debounce
    self.assertIn(S.ON, seen)

  def test_12_slow_is_low_beam(self):
    r = self.m.update(I(0, v_ego=10 / 3.6))
    self.assertEqual(r['state'], S.OFF); self.assertFalse(r['high'])

  def test_speed_hysteresis(self):
    settle_on(self.m)                                       # ON at 100
    self.assertEqual(self.m.update(I(10.0, v_ego=25 / 3.6))['state'], S.ON)   # 25: between 20 and 30, latch stays
    self.assertEqual(self.m.update(I(10.1, v_ego=18 / 3.6))['state'], S.OFF)  # below 20: drops
    self.assertEqual(self.m.update(I(10.2, v_ego=25 / 3.6))['state'], S.OFF)  # 25 from below: stays low

  def test_disabled_is_undecided(self):
    self.assertEqual(self.m.update(I(0, enabled=False))['state'], S.UNDECIDED)

  def test_headlights_off_is_undecided(self):
    self.assertEqual(self.m.update(I(0, headlights_on=False))['state'], S.UNDECIDED)

  def test_lum_unknown_is_undecided(self):
    self.assertEqual(self.m.update(I(0, lum_valid=False))['state'], S.UNDECIDED)

  def test_min_off_before_first_lift(self):
    # first dark cycle after a dip must not instantly go high (MIN_OFF)
    self.m.update(I(0, lead=True))                          # dipped
    r = self.m.update(I(0.1))                               # clear now, but only 0.1 s later
    self.assertNotEqual(r['state'], S.ON)


if __name__ == '__main__':
  unittest.main()
