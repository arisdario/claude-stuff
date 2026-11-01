#!/usr/bin/env python3
"""ahb_luminance: exposure->index, smoothing, spike rejection, staleness, classification. numpy only for the ROI test.
  python3 tesla/tests/test_ahb_luminance.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import ahb_luminance as L  # noqa: E402


class TestIndex(unittest.TestCase):
  def test_darkest_is_zero(self):
    self.assertAlmostEqual(L.relative_luminance_index(L.GAIN_MAX, L.INTEG_MAX), 0.0, places=3)

  def test_brighter_is_higher(self):
    dark = L.relative_luminance_index(510, 5408)
    oncoming = L.relative_luminance_index(510, 4019)     # from the real capture
    day = L.relative_luminance_index(20, 300)
    self.assertGreater(oncoming, dark)
    self.assertGreater(day, oncoming)

  def test_bad_inputs(self):
    for g, il in ((None, 100), (0, 100), (100, 0), (-1, 5), ('x', 5)):
      self.assertIsNone(L.relative_luminance_index(g, il))


class TestEstimator(unittest.TestCase):
  def feed(self, est, gain, integ, t0, secs, step=0.1):
    t = t0
    while t <= t0 + secs:
      est.update_exposure(gain, integ, t); t += step
    return t

  def test_dark_road_classifies_dark(self):
    e = L.LuminanceEstimator(); self.feed(e, 510, 5408, 0, 5)
    self.assertEqual(e.level, L.DARK); self.assertTrue(e.is_dark()); self.assertTrue(e.needs_headlights())

  def test_bright_urban_classifies_bright(self):
    e = L.LuminanceEstimator(); self.feed(e, 30, 200, 0, 5)
    self.assertEqual(e.level, L.BRIGHT); self.assertFalse(e.is_dark())

  def test_dim_is_headlights_but_not_high_beam(self):
    e = L.LuminanceEstimator()
    # choose an exposure whose index lands between DARK_EXIT and BRIGHT_EXIT
    import math
    target = (L.DARK_EXIT + L.BRIGHT_EXIT) / 2
    integ = L.E_DARK / (2 ** target) / 100.
    self.feed(e, 100, integ, 0, 5)
    self.assertEqual(e.level, L.DIM); self.assertFalse(e.is_dark()); self.assertTrue(e.needs_headlights())

  def test_hysteresis_no_chatter_on_streetlight(self):
    e = L.LuminanceEstimator(); self.feed(e, 510, 5408, 0, 5)       # settle dark
    # a brief streetlight: index rises a little for 1 s, then back. Must not reach BRIGHT or leave DARK permanently.
    seen = set()
    t = 5.0
    for _ in range(10):
      e.update_exposure(510, 4600, t); seen.add(e.level); t += 0.1
    for _ in range(40):
      e.update_exposure(510, 5408, t); seen.add(e.level); t += 0.1
    self.assertNotIn(L.BRIGHT, seen)
    self.assertEqual(e.level, L.DARK)

  def test_spike_rejected(self):
    e = L.LuminanceEstimator(); self.feed(e, 510, 5408, 0, 5)
    before = e.bi
    e.update_exposure(10, 50, 5.1)        # one daylight-bright frame (camera glitch / flash): index jumps ~8 stops
    self.assertLess(e.bi - before, 2.0)   # spike weight keeps the smoothed value from leaping
    self.assertEqual(e.level, L.DARK)

  def test_tunnel_entry_exit(self):
    e = L.LuminanceEstimator(); self.feed(e, 30, 200, 0, 5)         # daylight before tunnel
    self.assertEqual(e.level, L.BRIGHT)
    self.feed(e, 510, 5408, 5, 6)                                   # inside a dark tunnel for 6 s
    self.assertEqual(e.level, L.DARK)
    self.feed(e, 30, 200, 11, 6)                                    # back into daylight
    self.assertEqual(e.level, L.BRIGHT)

  def test_camera_stall_goes_unknown(self):
    e = L.LuminanceEstimator(); self.feed(e, 510, 5408, 0, 5)
    self.assertEqual(e.level, L.DARK)
    e.tick(5.0 + L.STALE_S + 0.5)                                   # no fresh frame
    self.assertEqual(e.level, L.UNKNOWN); self.assertFalse(e.is_dark()); self.assertFalse(e.needs_headlights())

  def test_missing_frame_values(self):
    e = L.LuminanceEstimator()
    e.update_exposure(None, None, 0.0)
    self.assertFalse(e.valid); self.assertEqual(e.level, L.UNKNOWN)


class TestPixels(unittest.TestCase):
  def test_region_luminance_roi(self):
    try:
      import numpy as np
    except ImportError:
      self.skipTest('numpy not available')
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[:40] = 255               # bright sky at the top (should be excluded by the ROI)
    img[45:85, 20:80] = 40       # dim road band
    mean, med = L.region_luminance(np, img)
    self.assertLess(mean, 80)    # sky excluded -> road is dim
    self.assertAlmostEqual(med, 40, delta=5)

  def test_pixel_path_feeds_classifier(self):
    e = L.LuminanceEstimator()
    t = 0.0
    while t <= 5:
      e.update_pixels(2.0, t); t += 0.1      # near-black road
    self.assertTrue(e.needs_headlights())


if __name__ == '__main__':
  unittest.main()
