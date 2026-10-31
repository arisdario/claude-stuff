#!/usr/bin/env python3
"""Tests for radarpark.py (no openpilot needed):  python3 tests/test_radarpark.py"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import radarpark as RP  # noqa: E402


def park(fl, fr=255):
  return {'f': {'l': fl, 'r': fr, 'ss': 1}, 'r': {'l': 255, 'r': 255, 'ss': 1}}


def car_ahead(d, y=0.1):
  return [{'d': d, 'y': y}]


def stop(m, t, secs, gap_cm, radar_d, objs_fn=None):
  """drive in, stand still for secs at 10 Hz, drive off; returns the time after"""
  m.update(t, 3.0, park(255), []); t += 0.1
  for i in range(int(secs * 10)):
    objs = objs_fn(i) if objs_fn else (car_ahead(radar_d) if radar_d is not None else [])
    m.update(t, 0.0, park(gap_cm), objs); t += 0.1
  m.update(t, 2.0, park(255), []); t += 0.1
  return t


class TestOffset(unittest.TestCase):
  def setUp(self):
    self.f = os.path.join(tempfile.mkdtemp(), 'off.json'); self.m = RP.OffsetMeter(self.f)

  def test_one_stop_gives_the_offset(self):
    stop(self.m, 0, 6, 120, 1.55)
    s = self.m.summary()
    self.assertEqual(s['n'], 1); self.assertAlmostEqual(s['offset_m'], 0.35, places=3)

  def test_median_over_stops(self):
    t = 0
    for gap, rad in ((100, 1.40), (150, 1.92), (80, 1.18), (130, 1.66), (60, 2.5)):   # last one: a different object
      t = stop(self.m, t, 5, gap, rad)
    s = self.m.summary()
    self.assertEqual(s['n'], 5); self.assertAlmostEqual(s['offset_m'], 0.40, places=2)
    self.assertIsNotNone(s['iqr_m'])

  def test_radar_blind_close_in_front_counts_as_missed(self):
    stop(self.m, 0, 6, 70, None)
    s = self.m.summary()
    self.assertEqual((s['n'], s['missed']), (0, 1)); self.assertAlmostEqual(s['missed_max_gap_m'], 0.70)

  def test_short_stop_ignored(self):
    stop(self.m, 0, 1.5, 120, 1.55)
    self.assertEqual(self.m.summary()['n'], 0)

  def test_flickering_radar_rejected(self):
    stop(self.m, 0, 6, 120, None, objs_fn=lambda i: car_ahead(1.55 if i % 2 else 2.4))
    self.assertEqual(self.m.summary()['n'], 0)

  def test_object_beside_is_not_ahead(self):
    stop(self.m, 0, 6, 120, None, objs_fn=lambda i: car_ahead(1.5, y=2.5))
    s = self.m.summary()
    self.assertEqual((s['n'], s['missed']), (0, 1))

  def test_nothing_in_sensor_range_records_nothing(self):
    stop(self.m, 0, 6, 255, 1.5)
    s = self.m.summary(); self.assertEqual((s['n'], s['missed']), (0, 0))

  def test_saved_and_reloaded(self):
    stop(self.m, 0, 6, 120, 1.55)
    again = RP.OffsetMeter(self.f)
    self.assertEqual(again.summary()['n'], 1)

  def test_front_gap_uses_the_closer_corner(self):
    self.assertEqual(RP.front_gap_cm(park(140, 95)), 95)
    self.assertIsNone(RP.front_gap_cm(park(255, 255)))
    self.assertIsNone(RP.front_gap_cm(None))


if __name__ == '__main__':
  unittest.main()
