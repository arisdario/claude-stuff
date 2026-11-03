"""Curve-ahead detection and announcement (advisory only; nothing here touches vehicle control)."""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from nav import curveahead as C, geo  # noqa: E402

M_LAT = 111320.0


def line_with_arc(radius, straight=1500.0, turn_deg=60.0, step=10.0):
    """east along y=0 for `straight` m, then an arc of `radius` turning left by turn_deg, then straight on. -> (points (lat, lon), cum)"""
    k = 1.0 / M_LAT
    pts, x, y, th = [(0.0, 0.0)], 0.0, 0.0, 0.0
    for _ in range(int(straight / 10)):
        x += 10.0
        pts.append((y * k, x * k))
    steps = int(math.radians(turn_deg) * radius / step)
    for _ in range(steps):
        th += step / radius
        x += step * math.cos(th)
        y += step * math.sin(th)
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
    """33 m/s on Bucharest->Fundulea: the first bend ends up too close to the replay's first fix for a cruise-speed start (CurveCue detector: R=29 m at 413 m needs 505 m; own detector: R=18 m hairpin at 150 m).
    Entered at the best the advisory can do. Not an announcement-timing problem: the bend is announced at the very first fix (inside the 800 m horizon).
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
                    self.assertGreaterEqual(ve, math.sqrt(max(0.0, v * v - 2 * C.DECEL * (start + v * 1.0 + 1.0))), 'entered above the best physically possible speed (braked too late)')       # tolerance = two simulation steps (dt 0.5 s)
                    self.assertLessEqual(ve, math.sqrt(max(0.0, v * v - 2 * C.DECEL * start)) + 1.0, 'braked less than the physical maximum')

    def test_first_bend_is_announced_at_the_first_fix(self):
        import json
        pts = [tuple(p) for p in json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', 'route_bf.json')))]
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
        r = C.CurveAnnouncer().update(0.0, 'high', C.find_curves(pts, cum, 0.0))
        first = [c for c in C.find_curves(pts, cum, 0.0) if C.SEVERITY_ORDER.index(c['severity']) >= 1][0]       # the announcer's standard verbosity: moderate and tighter
        self.assertEqual(len(r['new']), len(r['active']))                      # everything within the horizon is announced at once, at the first fix
        self.assertAlmostEqual(r['active'][0]['distance_m'], max(0.0, first['start_m']), delta=1.0)


def sparse_road():
    """the arc road with realistic edge lengths: 100 m edges on the straights, 10 m edges only through the bend (a tracker cannot follow 10 m edges at 25 m/s with 1 fix/s)"""
    pts, _cum = line_with_arc(100.0, straight=2500.0, turn_deg=90.0)
    n_straight = 250
    keep = [i for i in range(len(pts)) if (i <= n_straight and i % 10 == 0) or (n_straight < i < len(pts) - 100) or (i >= len(pts) - 100 and (i - (len(pts) - 100)) % 10 == 0) or i == len(pts) - 1]
    return [pts[i] for i in keep]


def _along(points, cum, s):
    j = max(i for i in range(len(cum) - 1) if cum[i] <= s)
    f = (s - cum[j]) / max(cum[j + 1] - cum[j], 1e-9)
    return (points[j][0] + f * (points[j + 1][0] - points[j][0]), points[j][1] + f * (points[j + 1][1] - points[j][1]),
            geo.bearing(points[j][0], points[j][1], points[j + 1][0], points[j + 1][1]))


class TestDeadReckoning(unittest.TestCase):
    """The logged MCU GPS position refreshes only every ~14 s: between updates the progress must advance with the car's speed, not freeze."""
    def drive(self, dead_reckon, refresh_s=14.0, v=8.0, seed=1):
        import random
        from nav import scenarios as SC, router as RT
        pts, _c = line_with_arc(100.0, straight=2500.0, turn_deg=90.0, step=5.0)       # vertices no further apart than the engine's 5 m resampling (see HANDOVER: engine quirk with long chords)
        net = SC.Net()
        ids = [net.node() for _ in pts]
        net.way('Main', [(ids[i], pts[i][0] * M_LAT, pts[i][1] * M_LAT) for i in range(len(pts))], highway='secondary')
        g = net.build()
        goal = SC.at(pts[-1][0] * M_LAT, pts[-1][1] * M_LAT)
        rt = RT.find_route(g, SC.at(0, 0), goal)
        ca = C.CurveAhead(g, goal, dead_reckon=dead_reckon)
        ca.start(rt.points[0][0], rt.points[0][1], None)
        rnd, errs, news, last, dr, hi, n, t, s = random.Random(seed), [], 0, None, 0, 0, 0, 0.0, 0.0
        while s < 2600.0:
            if last is None or t - last[0] >= refresh_s:
                lat, lon, h = _along(rt.points, rt.cum, s)
                last = (t, lat + rnd.gauss(0, 3) / M_LAT, lon + rnd.gauss(0, 3) / M_LAT, h)
            msg = ca.update(t, last[1], last[2], last[3], v, 3.0)
            n += 1
            hi += msg['confidence'] == 'high'
            if msg['route_progress_m'] is not None:
                errs.append(abs(msg['route_progress_m'] - s))
            if 'announce' in msg:
                news += len(msg['announce']['new'])
                dr += msg['advisory']['dead_reckoned']
            t += 1.0
            s += v
        self.high_fraction = hi / float(n)
        return max(errs), news, dr

    def test_progress_follows_the_car_between_gps_updates(self):
        err_dr, news_dr, n_dr = self.drive(True)
        err_no, _news_no, _ = self.drive(False)
        self.assertGreater(n_dr, 0)
        self.assertLess(err_dr, 30.0)                       # GPS noise + route snapping, not 14 s x 8 m/s = 112 m
        self.assertGreater(err_no, 60.0)                    # the control: without dead reckoning the progress freezes between updates
        self.assertEqual(news_dr, 1)                        # the one bend announced exactly once
        self.assertGreater(self.high_fraction, 0.6)         # and the road stays confidently identified between GPS updates

    def test_without_a_new_position_for_too_long_confidence_drops(self):
        from nav import scenarios as SC, router as RT
        pts, _ = line_with_arc(100.0, straight=2500.0, turn_deg=90.0)
        net = SC.Net()
        ids = [net.node() for _ in pts]
        net.way('Main', [(ids[i], pts[i][0] * M_LAT, pts[i][1] * M_LAT) for i in range(len(pts))], highway='secondary')
        g = net.build()
        goal = SC.at(pts[-1][0] * M_LAT, pts[-1][1] * M_LAT)
        rt = RT.find_route(g, SC.at(0, 0), goal)
        ca = C.CurveAhead(g, goal)
        ca.start(rt.points[0][0], rt.points[0][1], None)
        lat, lon, h = _along(rt.points, rt.cum, 100.0)
        for t in range(0, 5):
            ca.update(float(t), lat, lon, h, 20.0, 3.0)
        late = ca.update(5.0 + C.DR_MAX_AGE_S + 5.0, lat, lon, h, 20.0, 3.0)
        self.assertNotEqual(late['confidence'], 'high')        # a position that never changed for > 60 s is not trusted for progress


if __name__ == '__main__':
    unittest.main()
