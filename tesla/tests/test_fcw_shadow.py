#!/usr/bin/env python3
"""Tests for tools/fcw_shadow.py (pure logic only - no cereal needed)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import fcw_shadow as S  # noqa: E402

DT = 0.05


def drive(sc, frames, t0, **kw):
  ev = []
  for i in range(frames):
    a = dict(v_ego=20.0, lead_status=True, d_rel=50.0, v_lead=20.0, y_rel=0.0, blinker=False, stock_fcw=False)
    a.update(kw)
    ev += sc.step(t0 + i * DT, **a)
  return ev


class T(unittest.TestCase):
  def test_cruising_behind_a_lead_logs_nothing(self):
    self.assertEqual(drive(S.ShadowCompare(), 100, 0.0), [])

  def test_stopped_lead_close_fires_v2_but_not_stock(self):
    sc = S.ShadowCompare()
    drive(sc, 20, 0.0)
    ev = drive(sc, 20, 1.0, v_ego=15.0, d_rel=20.0, v_lead=0.0)   # needs 15^2/40 = 5.6 m/s^2 = 0.57 g
    whos = [(e['ev'], e['who']) for e in ev]
    self.assertIn(('start', 'v2_early'), whos)
    self.assertNotIn(('start', 'stock'), whos)

  def test_stock_fcw_edges_are_logged(self):
    sc = S.ShadowCompare()
    ev = drive(sc, 3, 0.0, stock_fcw=True) + drive(sc, 3, 1.0, stock_fcw=False)
    self.assertEqual([(e['ev'], e['who']) for e in ev], [('start', 'stock'), ('end', 'stock')])

  def test_no_lead_never_fires(self):
    self.assertEqual(drive(S.ShadowCompare(), 50, 0.0, lead_status=False, d_rel=1.0, v_lead=0.0), [])

  def test_report_pairs_events(self):
    rows = [{'ev': 'start', 'who': 'stock', 't': 10.0, 'v_ego': 20, 'd_rel': 10, 'v_lead': 5, 'y_rel': 0, 'a_needed_g': .5, 'blinker': False},
            {'ev': 'start', 'who': 'v2_early', 't': 9.2, 'v_ego': 20, 'd_rel': 12, 'v_lead': 5, 'y_rel': 0, 'a_needed_g': .4, 'blinker': False},
            {'ev': 'start', 'who': 'v2_early', 't': 50.0, 'v_ego': 10, 'd_rel': 6, 'v_lead': 0, 'y_rel': 0.3, 'a_needed_g': .8, 'blinker': False}]
    r = S.report(rows)
    self.assertIn('both fired:  1', r)
    self.assertIn('BEFORE stock by 0.80 s', r)
    self.assertIn('v2 only:     1', r)
    self.assertIn('stock only:  0', r)

  def test_model_fcw_read_from_freepath(self):
    class M: pass
    m = M(); m.freePath = [1.0] + [0.0] * 68 + [1.0, 0.3, 0.8]
    self.assertTrue(S.model_fcw_from(m, False))
    self.assertFalse(S.model_fcw_from(m, True))                        # not while the driver brakes (comma controlsd)
    m.freePath = [1.0] + [0.0] * 68
    self.assertIsNone(S.model_fcw_from(m, False))                      # road edges only (older adapter)
    m.freePath = [1.0] + [0.0] * 68 + [-1.0, -1.0, -1.0]
    self.assertIsNone(S.model_fcw_from(m, False))                      # 0.8.3 adapter: not available

  def test_model_fcw_events_and_report(self):
    sc = S.ShadowCompare()
    ev = []
    for i in range(5):
      ev += sc.step(i * DT, 20.0, True, 50.0, 20.0, 0.0, False, False, model_fcw=(i >= 2))
    self.assertEqual([(e['ev'], e['who']) for e in ev], [('start', 'model')])
    rows = [{'ev': 'start', 'who': 'model', 't': 9.0, 'v_ego': 20, 'd_rel': 12, 'v_lead': 0, 'y_rel': 0, 'a_needed_g': .5, 'blinker': False},
            {'ev': 'start', 'who': 'stock', 't': 10.0, 'v_ego': 20, 'd_rel': 10, 'v_lead': 0, 'y_rel': 0, 'a_needed_g': .6, 'blinker': False}]
    self.assertIn('BEFORE by 1.00 s', S.report(rows))


if __name__ == '__main__':
  unittest.main()
