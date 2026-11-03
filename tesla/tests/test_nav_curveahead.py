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


class TestTargetSpeed(unittest.TestCase):
    def test_target_is_the_braking_envelope_and_shrinks_as_the_curve_approaches(self):
        vc = 12.0
        tg = [C.target_speed([{'distance_m': d, 'speed_ms': vc}])[0] for d in (500.0, 300.0, 100.0, 0.0)]
        self.assertEqual(tg, sorted(tg, reverse=True))
        self.assertAlmostEqual(tg[-1], vc)
        self.assertAlmostEqual(tg[0], math.sqrt(vc * vc + 2 * C.DECEL * 500.0))
        self.assertEqual(C.target_speed([]), (None, None))

    def test_following_the_target_reaches_every_curve_at_curve_speed_with_gentle_braking(self):
        pts, cum = line_with_arc(60.0, turn_deg=90.0)
        r = C.simulate_follow(pts, cum, 30.0)
        self.assertEqual(len(r['entries']), 1)
        start, vc, v_entry = r['entries'][0]
        self.assertLessEqual(v_entry, vc + 0.75)
        self.assertLessEqual(r['max_decel'], C.DECEL + 1e-6)
        self.assertGreater(r['min_speed'], 0.9 * vc)               # does not brake far below what the curve needs (no false slowdown)

    def test_no_slowdown_on_a_straight_road(self):
        pts, cum = line_with_arc(150.0, turn_deg=0.0)
        r = C.simulate_follow(pts, cum, 30.0)
        self.assertEqual((r['entries'], r['min_speed']), ([], 30.0))


class TestLateEntryIsOnlyPhysicallyUnavoidable(unittest.TestCase):
    """33 m/s on Bucharest->Fundulea: the first bend (sharp, R=29 m, 8.9 m/s) starts 413 m after the replay's first fix; slowing from 33 to 8.9 m/s at 1.0 m/s^2 needs 505 m.
    Entered at 15.5 m/s = the best the advisory can do (sqrt(33^2 - 2*413) = 15.4). Not an announcement-timing problem: the bend is announced at the very first fix (inside the 800 m horizon).
    Invariant: any entry above curve speed + 0.75 m/s must be a bend that starts closer than its braking distance from a cruise-speed start."""
    def test_every_late_entry_is_unavoidable(self):
        import json
        for name in ('bf', 'fb'):
            pts = [tuple(p) for p in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'route_%s.json' % name)))]
            cum = [0.0]
            for a, b in zip(pts, pts[1:]):
                cum.append(cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
            for v in (15.0, 25.0, 33.0):
                late = [e for e in C.simulate_follow(pts, cum, v)['entries'] if e[2] > e[1] + 0.75]
                for start, vc, ve in late:
                    need = (v * v - vc * vc) / (2.0 * C.DECEL)
                    self.assertLess(start, need, '%s at %.0f m/s: bend at %.0f m entered at %.1f m/s although %.0f m were enough to slow to %.1f' % (name, v, start, ve, start, vc))
                    self.assertGreaterEqual(ve, math.sqrt(max(0.0, v * v - 2 * C.DECEL * (start + v * 0.5 + 1.0))), 'entered above the best physically possible speed (braked too late)')
                    self.assertLessEqual(ve, math.sqrt(max(0.0, v * v - 2 * C.DECEL * start)) + 1.0, 'braked less than the physical maximum')

    def test_first_bend_is_announced_at_the_first_fix(self):
        import json
        pts = [tuple(p) for p in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'route_bf.json')))]
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
        r = C.CurveAnnouncer().update(0.0, 'high', C.find_curves(pts, cum, 0.0))
        self.assertEqual(len(r['new']), 1 + len([x for x in r['active'] if x['id'] != 1]))     # everything within the horizon is announced at once
        self.assertAlmostEqual(r['active'][0]['distance_m'], 413.0, delta=15.0)


if __name__ == '__main__':
    unittest.main()
