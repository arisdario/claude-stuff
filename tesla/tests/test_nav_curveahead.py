"""Curve-ahead detection and announcement (advisory only; nothing here touches vehicle control)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from nav import curveahead as C, geo  # noqa: E402

M_LAT = 111320.0


def line_with_arc(radius, straight=1500.0, turn_deg=60.0):
    """east along y=0 for `straight` m, then an arc of `radius` turning left by turn_deg, then straight on. -> (points (lat, lon), cum)"""
    k = 1.0 / M_LAT
    pts, x, y, th = [(0.0, 0.0)], 0.0, 0.0, 0.0
    for _ in range(int(straight / 10)):
        x += 10.0
        pts.append((y * k, x * k))
    steps = int(math.radians(turn_deg) * radius / 10.0)
    for _ in range(steps):
        th += 10.0 / radius
        x += 10.0 * math.cos(th)
        y += 10.0 * math.sin(th)
        pts.append((y * k, x * k))
    for _ in range(100):
        x += 10.0 * math.cos(th)
        y += 10.0 * math.sin(th)
        pts.append((y * k, x * k))
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
    return pts, cum


class TestFindCurves(unittest.TestCase):
    def test_straight_road_has_no_curve(self):
        pts, cum = line_with_arc(150.0, turn_deg=0.0)
        self.assertEqual(C.find_curves(pts, cum, 0.0), [])

    def test_arc_is_found_with_its_radius_speed_and_distance(self):
        pts, cum = line_with_arc(100.0)
        cs = C.find_curves(pts, cum, 800.0)                 # 700 m before the arc starts (at ~1500 m): inside the 800 m horizon
        self.assertEqual(len(cs), 1)
        c = cs[0]
        self.assertAlmostEqual(c['radius_m'], 100.0, delta=15.0)
        self.assertAlmostEqual(c['v_curve'], math.sqrt(C.A_LAT * 100.0), delta=1.5)
        self.assertAlmostEqual(c['start_m'], 700.0, delta=30.0)

    def test_a_curve_beyond_the_horizon_is_not_announced(self):
        pts, cum = line_with_arc(100.0)
        self.assertEqual(C.find_curves(pts, cum, 0.0), [])

    def test_gentle_bend_is_not_a_curve(self):
        pts, cum = line_with_arc(900.0, turn_deg=20.0)
        self.assertEqual(C.find_curves(pts, cum, 800.0), [])


class TestAnnouncer(unittest.TestCase):
    def drive(self, conf='high'):
        pts, cum = line_with_arc(100.0)
        ann, new_events, dists = C.CurveAnnouncer(), 0, []
        for s in range(0, 1700, 10):
            r = ann.update(float(s), conf, C.find_curves(pts, cum, float(s)))
            new_events += len(r['new'])
            for a in r['active']:
                dists.append(a['distance_m'])
        return new_events, dists, ann

    def test_announced_once_distance_shrinks_and_curve_is_dropped_when_passed(self):
        new_events, dists, ann = self.drive()
        self.assertEqual(new_events, 1)
        self.assertEqual(dists, sorted(dists, reverse=True))          # never grows while approaching
        self.assertEqual(dists[0] > 600.0, True)                      # announced far ahead
        self.assertEqual(ann.update(2500.0, 'high', [])['active'], [])

    def test_low_confidence_never_announces_a_new_curve(self):
        new_events, dists, _ = self.drive(conf='low')
        self.assertEqual((new_events, dists), (0, []))


if __name__ == '__main__':
    unittest.main()
