#!/usr/bin/env python3
"""Tests for fcw_v2.py, targeting the exact two real failure modes replay_fcw.py found in stock
fcw.py (stopped-lead exclusion, single-frame-glitch reset) plus the new design's own safety rails
(blinker suppression, uncertain-track suppression, persistence requirement):
    python3 -m unittest tesla/tests/test_fcw_v2.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import fcw_v2 as F2  # noqa: E402


class TestRequiredDecel(unittest.TestCase):
  def test_zero_for_non_closing_lead(self):
    self.assertEqual(F2.required_decel(v_ego=20.0, x_lead=10.0, v_lead=20.0), 0.0)
    self.assertEqual(F2.required_decel(v_ego=20.0, x_lead=10.0, v_lead=25.0), 0.0)

  def test_well_defined_for_a_stopped_lead(self):
    # this is the whole point: v_lead=0 must work, not get excluded by some v_lead_max-style gate
    a = F2.required_decel(v_ego=15.0, x_lead=20.0, v_lead=0.0)
    self.assertGreater(a, 0.0)

  def test_higher_closing_speed_needs_more_deceleration(self):
    slow = F2.required_decel(v_ego=15.0, x_lead=20.0, v_lead=10.0)
    fast = F2.required_decel(v_ego=25.0, x_lead=20.0, v_lead=10.0)
    self.assertGreater(fast, slow)

  def test_closer_gap_needs_more_deceleration(self):
    far = F2.required_decel(v_ego=20.0, x_lead=40.0, v_lead=5.0)
    near = F2.required_decel(v_ego=20.0, x_lead=10.0, v_lead=5.0)
    self.assertGreater(near, far)


class TestStoppedLeadNoLongerExcluded(unittest.TestCase):
  """The exact gap stock fcw.py had: v_lead_max > 2.5 can never pass for a stopped/slow lead."""

  def test_fires_on_a_genuinely_stopped_lead(self):
    rep = F2.FCWv2()
    t = 0.0
    fired = False
    # ego closing on a dead-stopped lead at a dangerous rate, lead speed is 0.0 throughout
    for _ in range(10):
      out = rep.step(t, v_ego=15.0, x_lead=8.0, v_lead=0.0, y_lead=0.0, blinkers=False)
      if out['urgent']:
        fired = True
      t += 0.05
    self.assertTrue(fired)
    self.assertIsNotNone(rep.urgent_fired_at)


class TestRobustToSingleFrameGlitch(unittest.TestCase):
  def test_one_noisy_safe_frame_does_not_erase_the_whole_history(self):
    rep = F2.FCWv2()
    t = 0.0
    for _ in range(4):  # 3/5 of the persistence window already dangerous
      rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=2.0, y_lead=0.0, blinkers=False)
      t += 0.05
    # one single glitch frame where the lead suddenly looks far/safe (track reacquisition noise)
    rep.step(t, v_ego=20.0, x_lead=40.0, v_lead=20.0, y_lead=0.0, blinkers=False)
    t += 0.05
    # back to real danger - should only need to re-establish majority, not restart from zero
    out = None
    for _ in range(3):
      out = rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=2.0, y_lead=0.0, blinkers=False)
      t += 0.05
    self.assertTrue(out['early'])


class TestBlinkerSuppressionIsScopedNotBlanket(unittest.TestCase):
  def test_suppresses_right_after_a_blinker(self):
    rep = F2.FCWv2()
    t = 0.0
    rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=2.0, y_lead=0.0, blinkers=True)
    t += 0.1
    out = None
    for _ in range(5):
      out = rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=2.0, y_lead=0.0, blinkers=False)
      t += 0.05
    self.assertFalse(out['urgent'])  # still within BLINKER_SUPPRESS_S of the blinker

  def test_does_not_suppress_with_no_recent_blinker_at_all(self):
    rep = F2.FCWv2()
    t = 0.0
    out = None
    for _ in range(10):
      out = rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=0.0, y_lead=0.0, blinkers=False)
      t += 0.05
    self.assertTrue(out['urgent'])

  def test_suppression_expires_after_the_window(self):
    rep = F2.FCWv2()
    t = 0.0
    rep.step(t, v_ego=20.0, x_lead=30.0, v_lead=20.0, y_lead=0.0, blinkers=True)
    t = F2.BLINKER_SUPPRESS_S + 1.0  # well past the suppression window
    out = None
    for _ in range(10):
      out = rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=0.0, y_lead=0.0, blinkers=False)
      t += 0.05
    self.assertTrue(out['urgent'])


class TestUncertainTrackSuppressesButDoesNotCrashOtherLogic(unittest.TestCase):
  def test_large_lateral_offset_suppresses_the_alert(self):
    rep = F2.FCWv2()
    t = 0.0
    out = None
    for _ in range(10):
      out = rep.step(t, v_ego=20.0, x_lead=6.0, v_lead=0.0, y_lead=4.0, blinkers=False)
      t += 0.05
    self.assertFalse(out['urgent'])
    self.assertTrue(out['track_too_uncertain'])


class TestEarlyFiresBeforeUrgent(unittest.TestCase):
  def test_early_threshold_is_more_sensitive_than_urgent(self):
    rep = F2.FCWv2()
    t = 0.0
    # a moderate (not extreme) closing scenario - should eventually trip EARLY, not necessarily URGENT
    early_t = urgent_t = None
    for _ in range(20):
      out = rep.step(t, v_ego=20.0, x_lead=12.5, v_lead=10.0, y_lead=0.0, blinkers=False)
      if out['early'] and early_t is None:
        early_t = t
      if out['urgent'] and urgent_t is None:
        urgent_t = t
      t += 0.05
    self.assertIsNotNone(early_t)
    self.assertIsNone(urgent_t)


if __name__ == '__main__':
  unittest.main()
