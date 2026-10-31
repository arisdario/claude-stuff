#!/usr/bin/env python3
"""Tests for radarsens.py (no openpilot needed):  python3 tests/test_radarsens.py"""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import radarsens as RS  # noqa: E402


class Wheels(object):
  """simulated ESP_B: four 8-bit counters at 50 Hz; rate = pulses per second per wheel"""
  def __init__(self):
    self.c = [0.0] * 4

  def frame(self, rates, dt=0.02):
    for i in range(4): self.c[i] += rates[i] * dt
    return bytes(int(x) & 0xFF for x in self.c) + bytes(4)


def drive(wm, sim, secs, rates, v, steer=0.0, accel=0.0, t0=0.0):
  """feed frames at 50 Hz and windows at 10 Hz; returns (events, end time)"""
  evs = []; t = t0
  for i in range(int(secs * 50)):
    wm.feed(sim.frame(rates)); t += 0.02
    if i % 5 == 4:
      e = wm.window(t, v, steer, accel)
      if e: evs.append(e)
  return evs, t


class TestSun(unittest.TestCase):
  def test_night_and_glare(self):
    self.assertTrue(RS.sun_state(-10, 0)['night'])
    self.assertFalse(RS.sun_state(20, 0)['night'])
    self.assertTrue(RS.sun_state(8, -12)['glare'])          # low sun just left of straight ahead
    self.assertFalse(RS.sun_state(8, 120)['glare'])         # low sun to the side
    self.assertFalse(RS.sun_state(40, 0)['glare'])          # high sun
    self.assertIsNone(RS.sun_state(None, 0))


class TestFleet(unittest.TestCase):
  def test_fleet_and_faster(self):
    f = RS.fleet_state({'UI_medianFleetSpeedMPS': 20.0, 'UI_topQrtlFleetSpeedMPS': 22.0, 'UI_bottomQrtlFleetSpeedMPS': 18.0})
    self.assertEqual(f['median'], 20.0)
    self.assertTrue(RS.faster_than_fleet(25.0, f)); self.assertFalse(RS.faster_than_fleet(23.0, f))
    self.assertIsNone(RS.fleet_state({'UI_medianFleetSpeedMPS': 0}))
    self.assertFalse(RS.faster_than_fleet(30.0, None))


class TestWheels(unittest.TestCase):
  def setUp(self):
    self.f = os.path.join(tempfile.mkdtemp(), 'w.json'); self.w = RS.WheelMonitor(self.f); self.sim = Wheels()

  def test_counts_survive_the_8bit_wrap(self):
    drive(self.w, self.sim, 2, [1000] * 4, 25.0)          # 1000 pulses/s wraps 256 four times a second
    self.assertAlmostEqual(self.w.current['fl'], 1.0, places=2)

  def test_wheelspin_detected_once(self):
    evs, t = drive(self.w, self.sim, 5, [1000] * 4, 20.0)          # learn the normal front/rear ratio
    evs, t = drive(self.w, self.sim, 1.0, [1000, 1000, 1150, 1150], 20.0, t0=t)   # rear 15% faster
    self.assertEqual(len(evs), 1); self.assertIn('rear faster', evs[0]['kind'])
    evs, t = drive(self.w, self.sim, 1.0, [1000] * 4, 20.0, t0=t)
    self.assertEqual(evs, [])

  def test_cornering_is_not_slip(self):
    evs, t = drive(self.w, self.sim, 5, [1000] * 4, 20.0)
    evs, _ = drive(self.w, self.sim, 2, [920, 1080, 920, 1080], 15.0, steer=60.0, t0=t)   # inner/outer differ, axles equal
    self.assertEqual(evs, [])

  def test_soft_tyre_flagged_against_the_baseline(self):
    old = RS.TYRE_MIN_KM; RS.TYRE_MIN_KM = 1.0
    try:
      _, t = drive(self.w, self.sim, 60, [1000, 1003, 997, 1000], 25.0)    # this car's normal (tyres not identical)
      self.assertTrue(self.w.set_baseline())
      _, t = drive(self.w, self.sim, 60, [1000, 1003, 997 * 1.005, 1000], 25.0, t0=t)   # rear-left now 0.5% faster
      s = self.w.summary()
    finally:
      RS.TYRE_MIN_KM = old
    self.assertTrue(s['ready']); self.assertEqual(s['check'], ['rl'])
    self.assertAlmostEqual(s['dev_pct']['rl'], 0.5, delta=0.1)
    self.assertLess(abs(s['dev_pct']['fl']), 0.1)

  def test_turns_and_low_speed_do_not_count_for_tyres(self):
    drive(self.w, self.sim, 10, [1000] * 4, 8.0)                   # too slow
    drive(self.w, self.sim, 10, [1000] * 4, 25.0, steer=20.0)      # turning
    self.assertEqual(self.w.summary()['km'], 0.0)

  def test_saved_and_reloaded(self):
    drive(self.w, self.sim, 20, [1000] * 4, 25.0); self.w.set_baseline()
    again = RS.WheelMonitor(self.f)
    self.assertTrue(again.summary()['baseline_set'])


class TestCurve(unittest.TestCase):
  def test_matching_map(self):
    c = RS.CurveCheck(); h = 0.0; t = 0.0; v = 20.0
    for i in range(200):
      k = 0.004 * math.sin(i / 15.0)                              # the road's curvature, 1/m
      c.update(t, v, h, k / 2.0)                                  # the map reports C2 = curvature / 2
      h = (h + math.degrees(k * v * 1.0)) % 360.0; t += 1.0
    s = c.summary()
    self.assertGreater(s['r'], 0.9); self.assertEqual(s['verdict'], 'matches the road')

  def test_unrelated_map(self):
    c = RS.CurveCheck(); h = 0.0; t = 0.0
    for i in range(200):
      c.update(t, 20.0, h, 0.002 * math.sin(i * 1.7)); h = (h + 3.0 * math.sin(i / 9.0)) % 360.0; t += 1.0
    self.assertLess(abs(c.summary()['r']), 0.4)


if __name__ == '__main__':
  unittest.main()
