#!/usr/bin/env python3
"""yt_replay pure functions (geometry, packing, detectors, intake errors).  python3 tesla/tests/test_yt_replay.py"""
import os
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'model083', 'tools'))
import yt_replay as Y  # noqa: E402


class Geometry(unittest.TestCase):
  def test_16x9_no_fov(self):
    g = Y.source_geometry(1920, 1080)
    self.assertEqual((g['cw'], g['ch'], g['cx'], g['cy']), (1920, 1080, 0, 0))
    self.assertFalse(g['fov_applied']); self.assertTrue(any('UNKNOWN' in n for n in g['notes']))

  def test_fov_crop(self):
    g = Y.source_geometry(1920, 1080, src_hfov=120.)
    self.assertTrue(g['fov_applied'])
    self.assertAlmostEqual(g['cw'] / 1920., np.tan(np.radians(35.21)) / np.tan(np.radians(60.)), delta=0.01)
    self.assertEqual(g['cw'] * 9, g['ch'] * 16 + (g['cw'] * 9 - g['ch'] * 16))      # still ~16:9
    self.assertAlmostEqual(g['cw'] / float(g['ch']), 16 / 9., delta=0.02)

  def test_narrower_source_not_widened_and_resolution_flag(self):
    g = Y.source_geometry(1280, 720, src_hfov=50.)
    self.assertFalse(g['fov_applied']); self.assertTrue(g['res_ok'])
    self.assertFalse(Y.source_geometry(640, 360)['res_ok'])

  def test_4x3_is_cropped_and_reported(self):
    g = Y.source_geometry(1164, 874)
    self.assertTrue(any('not 16:9' in n for n in g['notes'])); self.assertEqual(g['cw'], 1164)

  def test_canvas_mapping(self):
    g = Y.source_geometry(1920, 1080)
    x, y = Y.canvas_to_source(582., 437., g)               # canvas centre = middle of the 1280x720 picture
    self.assertAlmostEqual(x, 960., delta=1.); self.assertAlmostEqual(y, 540., delta=1.)
    self.assertEqual(Y.vp_canvas((0.5, 0.5)), (582., 437.))


class Packing(unittest.TestCase):
  def test_pack_is_crop_and_pad_without_scaling(self):
    rgb = np.random.RandomState(0).randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    cv = Y.c920_canvas(rgb)
    self.assertEqual(cv.shape, (874, 1164, 3))
    self.assertTrue((cv[:77] == 0).all() and (cv[797:] == 0).all())
    self.assertTrue((cv[77:797] == rgb[:, 58:58 + 1164]).all())

  def test_i420_layout_and_range(self):
    cv = np.zeros((874, 1164, 3), np.uint8); cv[:, :, 0] = 255                # pure red
    yuv = Y.rgb_to_i420(cv)
    self.assertEqual(len(yuv), 1164 * 874 * 3 // 2)
    self.assertEqual(int(yuv[0]), 82)                                            # BT.601 limited red luma
    self.assertLess(int(yuv[1164 * 874 + 5]), 100)                               # U plane (low for red)
    self.assertGreater(int(yuv[1164 * 874 * 5 // 4 + 5]), 200)                  # V plane (high for red)
    white = Y.rgb_to_i420(np.full((874, 1164, 3), 255, np.uint8)); self.assertEqual(int(white[0]), 235)


def _cols(n, hz=20.):
  t = np.arange(n) / hz
  return t, {'path_y_10': np.zeros(n), 'path_y_30': np.zeros(n), 'path_std_30': np.full(n, 0.3),
             'plan_reach_m': np.full(n, 150.), 'lane_prob_l': np.full(n, 0.9), 'lane_prob_r': np.full(n, 0.9)}


class Detectors(unittest.TestCase):
  def test_clean_signal_has_no_events(self):
    t, c = _cols(200); self.assertEqual(Y.detect(t, c), [])

  def test_path_shift(self):
    t, c = _cols(200); c['path_y_30'][100:] = 1.2
    ev = Y.detect(t, c); self.assertEqual([e['kind'] for e in ev], ['path_shift_30m']); self.assertAlmostEqual(ev[0]['t_start'], 5.0, delta=0.06)

  def test_slow_drift_is_not_a_shift(self):
    t, c = _cols(200); c['path_y_30'] = np.linspace(0, 2, 200); self.assertEqual(Y.detect(t, c), [])

  def test_shift_is_normalised_to_frame_time(self):
    t, c = _cols(200, hz=40.); c['path_y_30'][100:] = 0.3          # 0.3 m in 25 ms = 0.6 m per 50 ms
    self.assertIn('path_shift_30m', [e['kind'] for e in Y.detect(t, c)])

  def test_prob_jump_and_flicker_hysteresis(self):
    t, c = _cols(400)
    c['lane_prob_l'] = 0.5 + 0.05 * np.sin(np.arange(400))          # hovering around 0.5: no flicker
    self.assertNotIn('lane_prob_flicker', [e['kind'] for e in Y.detect(t, c)])
    c['lane_prob_l'] = np.where((np.arange(400) // 10) % 2 == 0, 0.9, 0.1)  # real flip-flopping
    kinds = [e['kind'] for e in Y.detect(t, c)]
    self.assertIn('lane_prob_flicker', kinds); self.assertIn('lane_prob_jump', kinds)

  def test_uncertainty_reach_and_lost_lanes(self):
    t, c = _cols(200)
    c['path_std_30'][50:80] = 2.0; c['plan_reach_m'][50:80] = 10.; c['lane_prob_l'][120:150] = 0.1; c['lane_prob_r'][120:150] = 0.1
    kinds = {e['kind'] for e in Y.detect(t, c)}
    self.assertTrue({'high_path_uncertainty', 'short_plan_reach', 'both_lanes_lost'} <= kinds)


class Intake(unittest.TestCase):
  def test_bot_check_is_reported_not_bypassed(self):
    fake = mock.Mock(returncode=1, stdout='ERROR: [youtube] x: Sign in to confirm you are not a bot')
    with mock.patch('subprocess.run', return_value=fake) as run:
      with self.assertRaises(Y.DownloadBlocked) as cm:
        Y.download('https://www.youtube.com/watch?v=x', '/tmp/yt_replay_test_dl', 0, 10)
    self.assertIn('--cookies', str(cm.exception)); self.assertEqual(run.call_count, 1)    # exactly one attempt
    self.assertNotIn('player_client', ' '.join(run.call_args[0][0]))


if __name__ == '__main__':
  unittest.main()
