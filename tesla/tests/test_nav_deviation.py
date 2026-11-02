#!/usr/bin/env python3
"""Wrong-turn / parallel-road / switchback regression tests for the navigator, judged by an INDEPENDENT oracle (nav/oracle.py: its own
SQL, its own oneway/access rules, own BFS - no router, no snapping). Every failure message names the wrong-road edge, the junction
node, the oracle's reachability verdict and the navigator's status.    python3 -m unittest tesla.tests.test_nav_deviation"""
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import test_nav as TN  # noqa: E402
from nav import geo, matcher as M, router as RT, validate as V  # noqa: E402
from nav.oracle import Oracle  # noqa: E402

B, at, nid = TN.B, TN.at, TN.nid


def chain_from(g, junction, way_name):
    """the wrong road as the exact directed edges a car would drive, in order, starting at the junction"""
    e = next(x for x in g.out_edges(junction) if x.name == way_name)
    chain = [e]
    while True:
        nxt = [x for x in g.out_edges(chain[-1].v) if x.name == way_name and not (x.u == chain[-1].v and x.v == chain[-1].u)]
        if not nxt:
            return chain
        chain.append(nxt[0])


def wrong_path(g, route, junction, chain):
    i = next(k for k, e in enumerate(route.edges) if e.v == junction)
    cut = max(j for j in range(len(route.points)) if route.cum[j] <= route.edge_offsets[i + 1] + 1e-6)
    pts = list(route.points[:cut + 1])
    for e in chain:
        gm = g.geom(e.eid)
        pts.extend(gm[1:] if pts[-1] == gm[0] else gm)
    return pts, route.cum[cut]


class Drive(object):
    """Drive route -> junction -> wrong road with GPS noise and record every status."""
    def __init__(self, g, route, junction, way_name, noise=5.0, seed=1, speed=15.0):
        self.g, self.route, self.junction, self.speed = g, route, junction, speed
        self.chain = chain_from(g, junction, way_name)
        self.path, self.junction_s = wrong_path(g, route, junction, self.chain)
        self.dest = (route.goal.lat, route.goal.lon)
        self.nav = M.Navigator(g, self.dest)
        self.nav.plan(route.points[0][0], route.points[0][1], None)
        self.rows = []
        for k, f in enumerate(V.fixes_along(self.path, speed, noise_m=noise, seed=seed)):
            st = self.nav.update(*f)
            self.rows.append({'k': k, 's_on_wrong': f[0] * speed - self.junction_s, 'status': st.status, 'reroutes': self.nav.reroutes,
                              'off': st.off_route_fixes, 'msg': st.message})

    def after(self, metres):
        return [r for r in self.rows if r['s_on_wrong'] >= metres]

    def first(self, statuses):
        return next((r for r in self.rows if r['status'] in statuses), None)

    def tag(self, oracle, probe_m=40.0):
        """'wrong-road edge E (way W), junction node J, oracle reachable=R, navigator status=S' - for every failure message"""
        run, k = 0.0, 0
        while k < len(self.chain) - 1 and run < probe_m:
            run += self.chain[k].length_m; k += 1
        e = self.chain[k]
        reach, _ = oracle.reachable(e.eid, oracle.targets_for(self.route.edges[-1].eid, self.route.goal.along_m))
        return 'wrong-road edge %d (way %s), junction node %d, oracle reachable=%s, navigator status=%s (reroutes=%d)' % (
            e.eid, e.way_id, self.junction, reach, self.rows[-1]['status'], self.nav.reroutes), reach

    def verdict(self, oracle):
        return self.tag(oracle)[1]


def build_parallel(sep, connected, graph_name='par'):
    """Main road 0..800 m east. At node M2 (200 m) a parallel road leaves, runs `sep` m north of Main to 700 m. connected: it joins Main
    at node M7 (a legitimate connection by a SHARED node id); otherwise it dead-ends 40 m earlier."""
    b = B()
    M_ = [nid(0, c) for c in range(9)]                                    # Main nodes at east = 0,100,...,800 (node ids 1..9)
    b.way('Main', [(M_[c], 0, 100 * c) for c in range(9)])
    pts = [(M_[2], 0, 200), (5001, sep, 200 + sep * 2)]
    pts += [(5002 + i, sep, 200 + sep * 2 + 100 * (i + 1)) for i in range(4)]
    if connected:
        pts += [(M_[7], 0, 700)]
    b.way('Par', pts)
    return b, M_


class TestOracleIndependence(unittest.TestCase):
    """The oracle is the referee, so first make sure it is right and agrees with the production builder's legality decisions."""
    def test_no_reversal_dead_end_is_a_trap(self):
        b = B(); b.way('Main', [(1, 0, 0), (2, 0, 200)]); b.way('Stub', [(2, 0, 200), (3, 100, 200)])
        g = b.build(); o = Oracle(g.path)
        stub = next(e for e in g.out_edges(2) if e.name == 'Stub')
        main_back = next(e for e in g.out_edges(1))
        self.assertFalse(o.reachable(stub.eid, {main_back.eid})[0])          # no U-turn at the end of the stub
        self.assertTrue(o.reachable(main_back.eid, {stub.eid})[0])

    def test_oneway_and_access_respected(self):
        b = B(); b.way('A', [(1, 0, 0), (2, 0, 100)], oneway='yes'); b.way('P', [(2, 0, 100), (3, 0, 200)], access='private')
        b.way('C', [(2, 0, 100), (4, 100, 100), (5, 100, 0), (1, 0, 0)])
        g = b.build(); o = Oracle(g.path)
        a_fwd = next(e for e in g.out_edges(1) if e.name == 'A')
        tgt = {e.eid for n in (5,) for e in g.out_edges(n)}
        self.assertTrue(o.reachable(a_fwd.eid, tgt)[0])
        self.assertEqual(o.illegal, [])                                        # builder emitted nothing the raw tags forbid

    def test_oracle_agrees_with_builder_on_every_directed_edge_of_a_rich_graph(self):
        g = TN.grid(row_tags={1: {'oneway': 'yes'}, 2: {'oneway': '-1'}, 3: {'access': 'private'}}).build()
        o = Oracle(g.path)
        self.assertEqual(o.illegal, [])
        n_db = g.db.execute('SELECT COUNT(*) FROM edges').fetchone()[0]
        self.assertEqual(len(o.edges), n_db)

    def test_oracle_matches_a_brute_force_search_on_the_grid(self):
        g = TN.grid(row_tags={1: {'oneway': 'yes'}}).build(); o = Oracle(g.path)
        edges = {e.eid: e for n in range(1, 500) for e in g.out_edges(n)}
        import itertools
        for src, dst in list(itertools.product(list(edges)[:12], list(edges)[20:26])):
            # independent DFS with its own bookkeeping
            seen, stack, ok = set(), [src], False
            while stack and not ok:
                x = stack.pop()
                if x in seen:
                    continue
                seen.add(x)
                for y in edges.values():
                    if y.u == edges[x].v and not (y.v == edges[x].u and y.u == edges[x].v):
                        if y.eid == dst:
                            ok = True
                        stack.append(y.eid)
            self.assertEqual(o.reachable(src, {dst})[0], ok or src == dst, (src, dst))


class TestRouteThatDoublesBack(unittest.TestCase):
    """A loop ramp: the route passes within metres of a segment ~250 m further along. Progress must stay continuous, not jump to the later pass."""
    def test_progress_does_not_jump_to_a_later_pass(self):
        class R(object):
            pass
        r = R()
        # geometry in metres (k = degrees per metre of latitude)
        m_lat = 111320.0
        k = 1.0 / m_lat
        # east 150 m along y=0; north 30 m; west back to x=0; south to y=3 m; east again along y=3 m (a later pass ~357 m on, 2.5 m from the fix below)
        pts = [(0.0, 0.0), (0.0, 150 * k), (20 * k, 150 * k), (20 * k, 0.0), (3 * k, 0.0), (3 * k, 400 * k)]
        cum = [0.0]
        for a, b in zip(pts, pts[1:]):
            cum.append(cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
        r.points, r.cum, r.distance_m = pts, cum, cum[-1]
        rm = M.RouteMatcher(r)
        rm.commit(140.0)
        got = rm.match(2.5 * k, 140.0 * k, 90.0, M.AHEAD_M + 45.0)       # fix 2.5 m north of leg 1, 0.5 m from the later pass (leg 5)
        self.assertTrue(got.ok)
        self.assertLess(got.progress_m, 140.0 + M.NEAR_AHEAD_M + 45.0 + 1.0, 'jumped to a later pass at %.0f m' % got.progress_m)


class TestParallelRoadRegression(unittest.TestCase):
    def setUp(self):
        self.dest_at = at(0, 800)

    def run_case(self, sep, connected, noise=5.0, seed=1):
        b, M_ = build_parallel(sep, connected)
        g = b.build()
        route = RT.find_route(g, at(0, 0), self.dest_at)
        self.assertEqual({e.name for e in route.edges}, {'Main'})
        d = Drive(g, route, M_[2], 'Par', noise=noise, seed=seed)
        return g, route, d, Oracle(g.path), M_

    def test_20m_parallel_road_without_connection_is_detected_and_never_snaps_back(self):
        for seed in (1, 2, 3):
            g, route, d, o, _ = self.run_case(20.0, False, seed=seed)
            tag, reach = d.tag(o)
            self.assertFalse(reach, tag)                                                  # scenario sanity: the oracle says unreachable
            late = d.after(60.0)
            self.assertTrue(late, tag)
            first = d.first(('off_route', 'no_route', 'rerouted'))
            self.assertIsNotNone(first, 'deviation NOT detected: ' + tag)
            self.assertLess(first['s_on_wrong'], 120.0, 'detected too late (%.0f m): %s' % (first['s_on_wrong'], tag))
            self.assertEqual(d.nav.reroutes, 0, 'snapped back to the original route without a connection: ' + tag)
            self.assertNotIn('on_route', {r['status'] for r in d.after(first['s_on_wrong'] + 5.0)}, 'reported on_route while on the wrong road: ' + tag)

    def test_20m_parallel_road_with_a_legitimate_connection_reroutes_along_that_road(self):
        for seed in (1, 2, 3):
            g, route, d, o, M_ = self.run_case(20.0, True, seed=seed)
            tag, reach = d.tag(o)
            self.assertTrue(reach, tag)
            self.assertTrue(d.first(('rerouted',)), 'reachable but no recalculation: ' + tag)
            new = d.nav.route
            self.assertEqual(new.edges[0].name, 'Par', 'new route starts on the original road (snap-back): ' + tag)
            self.assertIn(M_[7], [e.v for e in new.edges], 'new route does not use the connection node: ' + tag)
            self.assertEqual(V.check_connectivity(g, new), [], tag)
            self.assertEqual(d.rows[-1]['status'] in ('arrived', 'on_route'), True, tag)

    def test_8m_parallel_road_without_connection_never_snaps_back_to_the_route(self):
        # at 8 m the roads are inside one GPS error: detection is not guaranteed, but a recalculation that STARTS ON MAIN would mean the
        # navigator teleported back to the original route although no connection exists - that must never happen
        for seed in (1, 2, 3, 4, 5):
            g, route, d, o, _ = self.run_case(8.0, False, noise=5.0, seed=seed)
            tag, reach = d.tag(o)
            self.assertFalse(reach, tag)
            self.assertEqual(d.nav.reroutes, 0, 'snapped back to the original route without a connection: ' + tag)

    def test_8m_parallel_road_with_good_gps_is_detected(self):
        g, route, d, o, _ = self.run_case(8.0, False, noise=1.5, seed=1)
        tag, reach = d.tag(o)
        first = d.first(('off_route', 'no_route', 'rerouted'))
        self.assertIsNotNone(first, 'not detected with 1.5 m GPS noise at 8 m separation: ' + tag)
        self.assertEqual(d.nav.reroutes, 0, tag)

    def test_on_route_drive_next_to_a_parallel_road_does_not_false_alarm(self):
        for sep in (8.0, 20.0):
            for seed in range(1, 7):
                b, M_ = build_parallel(sep, True)
                g = b.build()
                route = RT.find_route(g, at(0, 0), self.dest_at)
                nav = M.Navigator(g, (route.goal.lat, route.goal.lon)); nav.plan(route.points[0][0], route.points[0][1], None)
                states = [nav.update(*f) for f in V.fixes_along(route.points, 15.0, noise_m=5.0, seed=seed)]
                self.assertEqual(nav.reroutes, 0, 'false reroute driving the main road, separation %s seed %d' % (sep, seed))
                self.assertNotIn('off_route', {s.status for s in states}, 'false off_route, separation %s seed %d' % (sep, seed))


def build_switchback(spur):
    """Hairpin: up-leg north 0..300 at east 0, apex, down-leg back at east 25 (legs 25 m apart). Route = whole hairpin. 'spur' = None |
    'dead' (leaves the apex outward and ends) | 'loop' (leaves the apex, comes back and joins the down-leg by a shared node)."""
    b = B()
    up = [(1, 0, 0), (2, 100, 0), (3, 200, 0), (4, 300, 0)]
    apex = [(5, 315, 6), (6, 315, 19)]
    down = [(7, 300, 25), (8, 200, 25), (9, 100, 25), (10, 0, 25)]
    b.way('Hairpin', up + apex + down)
    if spur == 'dead':
        b.way('Spur', [(5, 315, 6), (50, 400, 6), (51, 480, 6)])
    elif spur == 'loop':
        b.way('Spur', [(5, 315, 6), (50, 400, 6), (51, 400, 60), (52, 250, 60), (8, 200, 25)])
    return b


class TestSwitchbackRegression(unittest.TestCase):
    def run_case(self, spur, seed=1, noise=5.0):
        g = build_switchback(spur).build()
        route = RT.find_route(g, at(0, 0), at(0, 25), heading=0.0)
        self.assertGreater(route.distance_m, 600.0)
        d = Drive(g, route, 5, 'Spur', noise=noise, seed=seed, speed=12.0)
        return g, route, d, Oracle(g.path)

    def test_spur_leaving_the_apex_dead_end_never_teleports_to_the_adjacent_leg(self):
        for seed in (1, 2, 3):
            g, route, d, o = self.run_case('dead', seed)
            tag, reach = d.tag(o)
            self.assertFalse(reach, tag)
            self.assertTrue(d.first(('off_route', 'no_route', 'rerouted')), 'deviation NOT detected: ' + tag)
            self.assertEqual(d.nav.reroutes, 0, 'teleported onto the adjacent leg 25 m away: ' + tag)

    def test_spur_that_rejoins_the_down_leg_reroutes_along_the_spur(self):
        for seed in (1, 2, 3):
            g, route, d, o = self.run_case('loop', seed)
            tag, reach = d.tag(o)
            self.assertTrue(reach, tag)
            self.assertTrue(d.first(('rerouted',)), 'reachable but no recalculation: ' + tag)
            self.assertEqual(d.nav.route.edges[0].name, 'Spur', 'new route starts on the hairpin, not on the spur the car is on: ' + tag)
            self.assertEqual(V.check_connectivity(g, d.nav.route), [], tag)

    def test_driving_the_hairpin_itself_is_never_off_route_and_progress_never_jumps_legs(self):
        g = build_switchback(None).build()
        route = RT.find_route(g, at(0, 0), at(0, 25), heading=0.0)
        for seed in range(1, 8):
            nav = M.Navigator(g, (route.goal.lat, route.goal.lon)); nav.plan(route.points[0][0], route.points[0][1], 0.0)
            for k, f in enumerate(V.fixes_along(route.points, 12.0, noise_m=5.0, seed=seed)):
                st = nav.update(*f)
                self.assertNotIn(st.status, ('off_route', 'no_route', 'rerouted'), 'seed %d fix %d: %s' % (seed, k, st.message))
                if st.progress_m is not None:
                    self.assertLess(abs(st.progress_m - min(route.cum[-1], 12.0 * f[0])), 45.0, 'progress jumped legs: seed %d fix %d' % (seed, k))
            self.assertEqual(nav.reroutes, 0)


class TestMidEdgeParity(unittest.TestCase):
    """Planner vs the independent oracle on every directed edge near the route, judged at MID-EDGE: from the edge itself and from a GPS fix
    at its midpoint (skipping genuinely ambiguous fixes). Zero disagreements required."""
    def check(self, g, a, b):
        route = RT.find_route(g, a, b)
        pc = V.parity_check(g, route)
        self.assertGreater(pc['edges'], 5)
        self.assertGreater(pc['fix_checks'], 0)
        self.assertEqual(pc['disagreements'], [], 'parity disagreements: %r' % (pc['disagreements'][:5],))
        return pc

    def test_grid_with_oneway_private_and_reverse_only_roads(self):
        g = TN.grid(row_tags={1: {'oneway': 'yes'}, 2: {'oneway': '-1'}, 3: {'access': 'private'}}).build()
        self.check(g, at(0, 0), at(400, 400))

    def test_parallel_roads_both_variants(self):
        for connected in (False, True):
            g = build_parallel(20.0, connected)[0].build()
            self.check(g, at(0, 0), at(0, 800))

    def test_switchback_variants(self):
        for spur in (None, 'dead', 'loop'):
            g = build_switchback(spur).build()
            self.check(g, at(0, 0), at(0, 25))

    def test_the_check_has_teeth_a_planner_that_teleports_to_a_nearby_road_is_caught(self):
        g = build_parallel(20.0, False)[0].build()                      # Par dead-ends; Main is 20 m away
        route = RT.find_route(g, at(0, 0), at(0, 800))
        old = RT.SNAP_WINDOW_M
        RT.SNAP_WINDOW_M = 50.0                                        # the earlier bug: a start-candidate pool wide enough to include Main
        try:
            pc = V.parity_check(g, route)
        finally:
            RT.SNAP_WINDOW_M = old
        self.assertTrue(pc['disagreements'], 'a teleporting planner was not caught by the parity check')
        self.assertTrue(all(d['oracle'] is False and d['planner'] is True for d in pc['disagreements']))     # always: found a route that cannot exist
        caught = {g.edge(d['edge']).name for d in pc['disagreements'] if d['kind'] == 'from-fix'}
        self.assertIn('Par', caught)                                      # including the dead-end parallel road itself


class TestFailureReportingHasTeeth(unittest.TestCase):
    """Break the navigator on purpose in each way the oracle is meant to catch; each must be classified correctly and reported with the
    wrong-road edge id, junction node id, oracle verdict and navigator status."""
    def setUp(self):
        self._saved = (M.Navigator.update, M.Navigator.plan_from_edge)

    def tearDown(self):
        M.Navigator.update, M.Navigator.plan_from_edge = self._saved

    def run_pair(self, connected):
        g = build_parallel(20.0, connected)[0].build()
        return g, V.validate_pair(g, at(0, 0), at(0, 800))

    def assert_identifiers(self, text):
        for needle in ('wrong-road edge', 'way ', 'junction node', 'oracle reachable=', 'navigator status='):
            self.assertIn(needle, text)

    def test_baseline_is_clean(self):
        for connected in (False, True):
            g, (r, rt) = self.run_pair(connected)
            self.assertEqual(r['problems'], [], r['problems'])
            self.assertEqual(r['sim_deviation']['outcome'], 'REROUTED' if connected else 'NO_ROUTE_CORRECT')

    def test_detection_failure_is_reported_as_such(self):
        real = M.Navigator.update
        def blind(self, *a, **k):                                        # never notices anything
            st = real(self, *a, **k)
            if st.status in ('off_route', 'no_route', 'rerouted', 'checking'):
                st.status = 'on_route'
            return st
        M.Navigator.update = blind
        g, (r, rt) = self.run_pair(True)
        self.assertEqual(r['sim_deviation']['outcome'], 'DETECTION_FAILURE')
        msg = next(p for p in r['problems'] if 'DETECTION FAILURE' in p)
        self.assert_identifiers(msg)
        self.assertIn('oracle reachable=True', msg); self.assertIn('navigator status=on_route', msg)

    def test_recalculation_failure_is_distinguished_from_detection_failure(self):
        def cannot_replan(self, edge, along):
            raise RT.NoRoute('planner broken')
        M.Navigator.plan_from_edge = cannot_replan
        real_plan = M.Navigator.plan
        def no_fallback(self, lat, lon, heading=None):
            if self.route is not None:
                raise RT.NoRoute('planner broken')
            return real_plan(self, lat, lon, heading)
        M.Navigator.plan = no_fallback
        try:
            g, (r, rt) = self.run_pair(True)
        finally:
            M.Navigator.plan = real_plan
        sd = r['sim_deviation']
        self.assertTrue(sd['detected']); self.assertFalse(sd['rerouted']); self.assertTrue(sd['oracle_reachable'])
        self.assertEqual(sd['outcome'], 'RECALC_FAILURE')
        msg = next(p for p in r['problems'] if 'RECALCULATION FAILURE' in p)
        self.assert_identifiers(msg); self.assertIn('oracle reachable=True', msg)

    def test_snap_back_to_the_original_route_is_caught(self):
        def teleport(self, edge, along):                                  # plans from the ORIGINAL first route edge, not the tracked one
            first = self.route.edges[0]
            return self._adopt(RT.find_route_from_edge(self.graph, first, 1.0, self.dest))
        M.Navigator.plan_from_edge = teleport
        g, (r, rt) = self.run_pair(True)
        self.assertEqual(r['sim_deviation']['outcome'], 'SNAP_BACK')
        msg = next(p for p in r['problems'] if 'SNAP-BACK' in p)
        self.assert_identifiers(msg)

    def test_reroute_when_the_oracle_says_unreachable_is_caught(self):
        def teleport(self, edge, along):
            first = self.route.edges[0]
            return self._adopt(RT.find_route_from_edge(self.graph, first, 1.0, self.dest))
        M.Navigator.plan_from_edge = teleport
        g, (r, rt) = self.run_pair(False)                                 # Par dead-ends: nothing legitimate exists
        sd = r['sim_deviation']
        self.assertEqual(sd['outcome'], 'REROUTE_UNREACHABLE')
        self.assertFalse(sd['oracle_reachable'])
        msg = next(p for p in r['problems'] if 'SNAP-BACK' in p)
        self.assert_identifiers(msg); self.assertIn('oracle reachable=False', msg)


class TestTrackerNeverReturnsToTheOriginalRoadWithoutAConnection(unittest.TestCase):
    def test_tracker_edge_sequence_on_a_dead_end_parallel_road(self):
        b, M_ = build_parallel(20.0, False); g = b.build()
        route = RT.find_route(g, at(0, 0), at(0, 800))
        for seed in range(1, 6):
            nav = M.Navigator(g, (route.goal.lat, route.goal.lon)); nav.plan(route.points[0][0], route.points[0][1], None)
            ch = chain_from(g, M_[2], 'Par')
            path, js = wrong_path(g, route, M_[2], ch)
            names, on_par = [], False
            for f in V.fixes_along(path, 15.0, noise_m=2.0, seed=seed):
                nav.update(*f)
                names.append(nav.tracker.edge.name)
            first_par = names.index('Par')
            self.assertNotIn('Main', names[first_par:], 'seed %d: tracker returned to Main with no connection after entering Par' % seed)


if __name__ == '__main__':
    unittest.main()
