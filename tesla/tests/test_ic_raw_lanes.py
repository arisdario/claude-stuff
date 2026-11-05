import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import ic_raw_lanes as R


class TestRawLanes(unittest.TestCase):
  def test_levels(self):
    self.assertEqual([R.level(p) for p in (0.1, 0.25, 0.3, 0.5, 0.6, 0.75, 0.9)], [0, 0, 1, 1, 2, 2, 3])

  def test_both_lanes(self):
    l = [0., 0., 0.01, 1.8]; r = [0., 0., 0.03, -1.6]
    lL, rL, c3, c2, c1, c0, w = R.raw_lane_ic(l, r, 0.9, 0.6, 3.7)
    self.assertEqual((lL, rL), (3, 2))
    self.assertAlmostEqual(c0, 0.1); self.assertAlmostEqual(c1, 0.02); self.assertAlmostEqual(w, 3.4)

  def test_one_lane_uses_fallback_width(self):
    l = [0., 0., 0., 1.8]
    r = R.raw_lane_ic(l, [0., 0., 0., -9.], 0.9, 0.1, 3.6)       # right unseen: its polynomial is ignored
    self.assertAlmostEqual(r[5], 0.0); self.assertAlmostEqual(r[6], 3.6); self.assertEqual((r[0], r[1]), (3, 0))
    r = R.raw_lane_ic([0., 0., 0., 9.], [0., 0., 0., -1.8], 0.1, 0.9, 3.6)
    self.assertAlmostEqual(r[5], 0.0)

  def test_no_lane_or_bad_input(self):
    self.assertIsNone(R.raw_lane_ic([0] * 4, [0] * 4, 0.1, 0.2, 3.7))
    self.assertIsNone(R.raw_lane_ic(None, [0] * 4, 0.9, 0.9, 3.7))
    self.assertIsNone(R.raw_lane_ic([0] * 3, [0] * 4, 0.9, 0.9, 3.7))

  def test_off_by_default_flag_path(self):
    self.assertEqual(R.RAW_LANES_FILE, '/data/ic_raw_lanes')


if __name__ == '__main__':
  unittest.main()
