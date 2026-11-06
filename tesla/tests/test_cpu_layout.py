import os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import cpu_layout as C

LOADS = {'controlsd': 61, 'modeld': 62, 'camerad': 45, 'plannerd': 16, 'radard': 14, 'boardd': 3}   # measured on the EON with the C920 (MJPEG), percent of one core


class T(unittest.TestCase):
  def test_stock083_overloads_core2_with_a_c920(self):
    core = C.project(LOADS, C.LAYOUTS['stock083'], other=60)
    self.assertGreater(core[2], C.OVERLOAD)
    self.assertEqual(core[2] - 60 / 4.0, 62 + 45 + 16 + 14)

  def test_adapted_stays_under_the_limit(self):
    core = C.project(LOADS, C.LAYOUTS['adapted'], other=60)
    self.assertTrue(all(c < C.OVERLOAD for c in core), core)
    self.assertGreaterEqual(core[2], 62); self.assertGreaterEqual(core[3], 61 + 3)

  def test_totals_are_conserved(self):
    for name in C.LAYOUTS:
      self.assertAlmostEqual(sum(C.project(LOADS, C.LAYOUTS[name], other=60)), sum(LOADS.values()) + 60, places=6)

  def test_reset_floats_everything(self):
    core = C.project(LOADS, C.LAYOUTS['reset'], other=0)
    self.assertLess(max(core) - min(core), 62)                    # nothing pinned: no single core takes everything


if __name__ == '__main__':
  unittest.main()
