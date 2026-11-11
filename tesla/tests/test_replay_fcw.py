#!/usr/bin/env python3
"""Tests for replay_fcw.py, against synthetic drivelog-shaped rows (no real log needed):
    python3 -m unittest tesla/tests/test_replay_fcw.py
"""
import os
import sys
import tempfile
import unittest
import json

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import replay_fcw as RF  # noqa: E402


def row(t, v_ego, a_ego, dRel, vRel, vLeadK, aLeadK=0.0, yRel=0.0, blinker=False):
  return {
      't': t, 'v_ego': v_ego, 'a_ego': a_ego, 'left_blinker': blinker, 'right_blinker': False,
      'lead1': {'status': True, 'dRel': dRel, 'vRel': vRel, 'yRel': yRel, 'aRel': 0.0,
                'vLeadK': vLeadK, 'aLeadK': aLeadK},
  }


class TestCalcTTC(unittest.TestCase):
  def test_no_closing_gives_max_ttc(self):
    # lead matches ego speed exactly - no danger, ttc should be the max cap
    ttc = RF.calc_ttc(v_ego=20.0, a_ego=0.0, x_lead=50.0, v_lead=20.0, a_lead=0.0)
    self.assertEqual(ttc, RF.MAX_TTC)

  def test_fast_closing_short_gap_gives_low_ttc(self):
    ttc = RF.calc_ttc(v_ego=20.0, a_ego=0.0, x_lead=5.0, v_lead=5.0, a_lead=0.0)
    self.assertLess(ttc, 2.5)

  def test_ttc_decreases_as_gap_shrinks(self):
    far = RF.calc_ttc(v_ego=20.0, a_ego=0.0, x_lead=40.0, v_lead=10.0, a_lead=0.0)
    near = RF.calc_ttc(v_ego=20.0, a_ego=0.0, x_lead=10.0, v_lead=10.0, a_lead=0.0)
    self.assertLess(near, far)


class TestFCWReplaySustainedDanger(unittest.TestCase):
  def test_arms_after_sustained_danger_with_no_blinker_history(self):
    # 4 continuous seconds of hard, unambiguous danger at 20Hz, no blinker activity at all -
    # long enough for every counter (including the 3s blinker ramp) to saturate.
    rep = RF.FCWReplay()
    t = 0.0
    armed = False
    for _ in range(4 * 20):
      ttc, blocking = rep.step(t, v_ego=20.0, a_ego=0.0, x_lead=8.0, v_lead=5.0, a_lead=0.0,
                                y_lead=0.0, blinkers=False)
      if not blocking:
        armed = True
      t += 0.05
    self.assertTrue(armed)
    self.assertIsNotNone(rep.armed_at)

  def test_does_not_arm_instantly_on_first_dangerous_frame(self):
    rep = RF.FCWReplay()
    ttc, blocking = rep.step(0.0, v_ego=20.0, a_ego=0.0, x_lead=8.0, v_lead=5.0, a_lead=0.0,
                              y_lead=0.0, blinkers=False)
    self.assertTrue(blocking)
    self.assertIsNone(rep.armed_at)

  def test_recent_blinker_use_blocks_arming_even_with_sustained_danger(self):
    rep = RF.FCWReplay()
    t = 0.0
    for _ in range(4 * 20):
      ttc, blocking = rep.step(t, v_ego=20.0, a_ego=0.0, x_lead=8.0, v_lead=5.0, a_lead=0.0,
                                y_lead=0.0, blinkers=True)  # blinker held the whole time
      t += 0.05
    self.assertIsNone(rep.armed_at)
    self.assertIn('common:blinkers', blocking)

  def test_ttc_counter_resets_on_a_single_safe_frame(self):
    rep = RF.FCWReplay()
    t = 0.0
    for _ in range(15):  # 0.75s of sustained danger - not yet enough to arm ttc counter alone
      rep.step(t, v_ego=20.0, a_ego=0.0, x_lead=8.0, v_lead=5.0, a_lead=0.0, y_lead=0.0, blinkers=False)
      t += 0.05
    before_reset = rep.counters['ttc']
    self.assertGreater(before_reset, 0)
    # one single frame where the lead suddenly looks safe (simulating a track glitch/reacquisition)
    rep.step(t, v_ego=20.0, a_ego=0.0, x_lead=40.0, v_lead=20.0, a_lead=0.0, y_lead=0.0, blinkers=False)
    self.assertEqual(rep.counters['ttc'], 0.0)

  def test_lateral_lead_position_blocks_the_y_lead_counter(self):
    rep = RF.FCWReplay()
    t = 0.0
    for _ in range(4 * 20):
      ttc, blocking = rep.step(t, v_ego=20.0, a_ego=0.0, x_lead=8.0, v_lead=5.0, a_lead=0.0,
                                y_lead=2.0, blinkers=False)  # lead is 2m off to the side
      t += 0.05
    self.assertIn('y_lead', blocking)
    self.assertIsNone(rep.armed_at)


class TestFindLeadEpisodes(unittest.TestCase):
  def _long_close_run(self, t0, n=25, dt=0.05, dRel=4.0):
    rows = []
    t = t0
    for _ in range(n):
      rows.append(row(t, 20.0, 0.0, dRel, -5.0, 15.0))
      t += dt
    return rows, t  # returns rows and the next free timestamp

  def test_brief_dropout_is_bridged_into_one_episode(self):
    run1, t_after = self._long_close_run(0.0)
    # a single brief (0.3s) track dropout - should NOT split the episode
    run2, _ = self._long_close_run(t_after + 0.3)
    rows = run1 + run2
    episodes = RF.find_lead_episodes(rows, min_gap_m=6.0)
    self.assertEqual(len(episodes), 1)
    self.assertEqual(len(episodes[0]), len(run1) + len(run2))

  def test_long_gap_splits_into_separate_episodes(self):
    run1, t_after = self._long_close_run(0.0)
    # a 5 second gap - a genuinely separate encounter, should split
    run2, _ = self._long_close_run(t_after + 5.0)
    rows = run1 + run2
    episodes = RF.find_lead_episodes(rows, min_gap_m=6.0, max_dropout_s=1.0)
    self.assertEqual(len(episodes), 2)

  def test_ignores_episodes_that_never_get_close(self):
    rows, _ = self._long_close_run(0.0, dRel=30.0)
    episodes = RF.find_lead_episodes(rows, min_gap_m=6.0)
    self.assertEqual(episodes, [])

  def test_ignores_episodes_shorter_than_min_duration(self):
    rows = [row(0.0, 20.0, 0.0, 4.0, -5.0, 15.0), row(0.1, 20.0, 0.0, 4.0, -5.0, 15.0)]
    episodes = RF.find_lead_episodes(rows, min_gap_m=6.0, min_duration_s=1.0)
    self.assertEqual(episodes, [])


class TestReplayEpisodeAndScanLog(unittest.TestCase):
  def test_replay_episode_reports_min_gap_and_closing_rate(self):
    rows = [
        row(0.0, 20.0, 0.0, 10.0, -5.0, 15.0),
        row(0.05, 20.0, 0.0, 9.5, -5.0, 15.0),
        row(0.10, 20.0, 0.0, 9.0, -5.0, 15.0),
    ]
    result = RF.replay_episode(rows)
    self.assertAlmostEqual(result['min_gap_m'], 9.0)
    self.assertAlmostEqual(result['closing_rate_at_min_gap'], 5.0)

  def test_replay_episode_tracks_min_ttc_separately_from_min_gap(self):
    # the closest raw gap and the worst (lowest) TTC happen on different frames here -
    # lead is nearer at t=0.05 (gap=5.0) but closing faster (hence worse ttc) at t=0.10 (gap=6.0)
    rows = [
        row(0.0, 20.0, 0.0, 20.0, -2.0, 18.0),
        row(0.05, 20.0, 0.0, 5.0, -1.0, 19.0),
        row(0.10, 20.0, 0.0, 6.0, -15.0, 5.0),
    ]
    result = RF.replay_episode(rows)
    self.assertAlmostEqual(result['min_gap_m'], 5.0)
    self.assertAlmostEqual(result['gap_at_min_ttc'], 6.0)
    self.assertNotAlmostEqual(result['min_gap_t_rel'], result['min_ttc_t_rel'])

  def test_scan_log_end_to_end_against_a_real_file(self):
    d = tempfile.mkdtemp()
    path = os.path.join(d, 'fake_drivelog.jsonl')
    with open(path, 'w') as f:
      f.write(json.dumps({'meta': 'driving_logger start', 't': 0.0, 'hz': 20.0}) + '\n')
      t = 0.0
      for _ in range(100):
        f.write(json.dumps(row(t, 20.0, 0.0, 4.0, -6.0, 5.0)) + '\n')
        t += 0.05
    results = RF.scan_log(path, min_gap_m=6.0)
    self.assertEqual(len(results), 1)
    self.assertIsNotNone(results[0]['armed_t_rel'])
    self.assertIn('min_ttc_s', results[0])


if __name__ == '__main__':
  unittest.main()
