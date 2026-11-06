#!/usr/bin/env python3
"""Tests for analyze_drivelog.py (synthetic JSONL, no cereal/car needed):
    python3 -m unittest tesla/tests/test_analyze_drivelog.py
"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import analyze_drivelog as AD  # noqa: E402


def sample(t, state, a_ego=0., lead_present=True, standstill=False, torque=0., rate=0.,
           cmd=0., meas=0., steer_en=True, sl20=None, sl25=None):
  return {
    't': t, 'state': state, 'a_ego': a_ego, 'standstill': standstill,
    'lead1': {'status': lead_present}, 'steering_torque': torque, 'steering_rate': rate,
    'das_commanded_angle': cmd, 'steering_angle': meas, 'das_steer_enabled': steer_en,
    'sl20_angle': cmd if sl20 is None else sl20, 'sl25_angle': cmd if sl25 is None else sl25,
  }


def write_jsonl(path, rows):
  with open(path, 'w') as f:
    f.write(json.dumps({'meta': 'start'}) + '\n')
    for r in rows:
      f.write(json.dumps(r) + '\n')


class TestBasics(unittest.TestCase):
  def test_empty_file_does_not_crash(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    write_jsonl(p, [])
    a = AD.analyze(p)
    self.assertEqual(a.n, 0)
    self.assertIn('samples: 0', a.report())

  def test_malformed_lines_are_skipped_not_fatal(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    with open(p, 'w') as f:
      f.write('not json at all\n')
      f.write(json.dumps(sample(1.0, 'HUMAN_DRIVING')) + '\n')
    a = AD.analyze(p)
    self.assertEqual(a.n, 1)

  def test_time_per_state_and_transitions(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(0.0, 'HUMAN_DRIVING'), sample(1.0, 'HUMAN_DRIVING'), sample(2.0, 'OPENPILOT_ENGAGED'),
            sample(3.0, 'OPENPILOT_ENGAGED')]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.n, 4)
    self.assertAlmostEqual(a.state_dt['HUMAN_DRIVING'], 1.0, places=3)   # dt accrues on arrival, so only the 0->1 step counts
    self.assertAlmostEqual(a.state_dt['OPENPILOT_ENGAGED'], 2.0, places=3)
    self.assertEqual(a.transitions, 1)


class TestAccelDecel(unittest.TestCase):
  def test_buckets_by_human_vs_openpilot_and_lead(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [
      sample(0.0, 'HUMAN_DRIVING', a_ego=0.0),
      sample(1.0, 'HUMAN_DRIVING', a_ego=1.5, lead_present=True),     # human accel, lead
      sample(2.0, 'HUMAN_DRIVING', a_ego=-1.5, lead_present=False),   # human decel, no lead
      sample(3.0, 'OPENPILOT_ENGAGED', a_ego=0.8, lead_present=True), # openpilot accel, lead
      sample(4.0, 'OPENPILOT_LATERAL_ONLY', a_ego=-0.6, lead_present=True),  # collapses into OPENPILOT bucket
    ]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.accel[('HUMAN', True)], [1.5])
    self.assertEqual(a.decel[('HUMAN', False)], [-1.5])
    self.assertEqual(a.accel[('OPENPILOT', True)], [0.8])
    self.assertEqual(a.decel[('OPENPILOT', True)], [-0.6])

  def test_standstill_and_small_accel_excluded(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(0.0, 'HUMAN_DRIVING', a_ego=2.0, standstill=True),   # excluded: standstill
            sample(1.0, 'HUMAN_DRIVING', a_ego=0.05)]                   # excluded: below the 0.1 noise floor
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.accel, {})
    self.assertEqual(a.decel, {})

  def test_transition_and_unknown_excluded_from_accel_decel(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(0.0, 'TRANSITION', a_ego=2.0), sample(1.0, 'UNKNOWN', a_ego=2.0)]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.accel, {})


class TestSteering(unittest.TestCase):
  def test_torque_and_rate_collected(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    write_jsonl(p, [sample(0.0, 'HUMAN_DRIVING', torque=0.3, rate=1.2)])
    a = AD.analyze(p)
    self.assertEqual(a.torque, [0.3]); self.assertEqual(a.rate, [1.2])

  def test_cmd_vs_meas_only_when_steer_enabled(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(0.0, 'OPENPILOT_ENGAGED', cmd=10.0, meas=8.0, steer_en=True),
            sample(1.0, 'OPENPILOT_ENGAGED', cmd=10.0, meas=8.0, steer_en=False)]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.cmd_vs_meas, [2.0])   # only the steer_en=True row counts

  def test_oscillation_detector_flags_rapid_sign_flips(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    # rate flips sign every 0.1s for a while - should trip the detector
    rows = [sample(i * 0.1, 'OPENPILOT_ENGAGED', rate=(3.0 if i % 2 == 0 else -3.0)) for i in range(10)]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertGreater(a.osc_events, 0)

  def test_no_oscillation_when_rate_is_steady(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(i * 0.1, 'OPENPILOT_ENGAGED', rate=2.0) for i in range(10)]
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.osc_events, 0)

  def test_sl_deviation_collected(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    write_jsonl(p, [sample(0.0, 'OPENPILOT_ENGAGED', cmd=10.0, sl20=8.0, sl25=7.0)])
    a = AD.analyze(p)
    self.assertEqual(a.sl20_dev, [2.0]); self.assertEqual(a.sl25_dev, [3.0])


class TestReport(unittest.TestCase):
  def test_report_runs_end_to_end_and_mentions_sections(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(i * 0.1, 'HUMAN_DRIVING' if i < 5 else 'OPENPILOT_ENGAGED', a_ego=1.0 if i % 2 else -1.0,
                    torque=0.2, rate=0.5, cmd=5.0, meas=4.5) for i in range(10)]
    write_jsonl(p, rows)
    r = AD.analyze(p).report()
    for needle in ('time per state', 'acceleration / deceleration', 'wiggle diagnostics', 'steer_limits hypothetical'):
      self.assertIn(needle, r)

  def test_lead_null_is_counted_and_noted(self):
    d = tempfile.mkdtemp(); p = os.path.join(d, 'x.jsonl')
    rows = [sample(0.0, 'HUMAN_DRIVING')]
    rows[0]['lead1'] = {'status': None}
    write_jsonl(p, rows)
    a = AD.analyze(p)
    self.assertEqual(a.lead_null_n, 1)
    self.assertIn('null/unavailable on 1 samples', a.report())


if __name__ == '__main__':
  unittest.main()
