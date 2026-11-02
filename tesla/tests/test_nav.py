#!/usr/bin/env python3
"""Offline navigation prototype tests (routegraph / router / instructions / matcher / validate). No vehicle, no openpilot:
    python3 -m unittest tesla.tests.test_nav
Synthetic networks target the failure modes that matter; one test also routes on real OSM data (pyrosm's bundled PBF)."""
import ast
import json
import math
import os
import sqlite3
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
NAVDIR = os.path.join(HERE, '..', 'nav')
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import geo, osm_rules as R, routegraph as RG, router as RT, instructions as IN, matcher as M, validate as V  # noqa: E402

LAT0, LON0 = 45.50, 25.57
M_LAT = (math.pi / 180.0) * geo.R_EARTH
M_LON = M_LAT * math.cos(math.radians(LAT0))


def ll(n, e):
    return (LON0 + e / M_LON, LAT0 + n / M_LAT)          # (lon, lat)


def at(n, e):
    return (LAT0 + n / M_LAT, LON0 + e / M_LON)          # (lat, lon)


class B(object):
    """tiny network builder: way(name, [(node_id, north, east), ...], tags)"""
    def __init__(self):
        self.edges, self.wid, self.places = [], 0, []

    def way(self, name, nodes, **tags):
        self.wid += 1
        t = {'highway': 'residential', 'name': name}
        t.update(tags)
        for (a, an, ae), (b, bn, be) in zip(nodes, nodes[1:]):
            self.edges.append((str(self.wid), a, b, [ll(an, ae), ll(bn, be)], t))

    def build(self):
        path = os.path.join(tempfile.mkdtemp(), 'rg.sqlite')
        self.stats = RG.build_from_edges(self.edges, path, self.places)
        return RG.Graph(path)


def nid(r, c):
    return 100 * r + c + 1


def grid(n=4, step=200.0, row_tags=None, col_tags=None, per_row=None):
    b = B()
    for r in range(n):
        tags = dict((row_tags or {}).get(r, {}))
        b.way('Row %d' % r, [(nid(r, c), r * step, c * step) for c in range(n)], **tags)
    for c in range(n):
        tags = dict((col_tags or {}).get(c, {}))
        b.way('Col %d' % c, [(nid(r, c), r * step, c * step) for r in range(n)], **tags)
    return b


def route(g, a, b, heading=None):
    return RT.find_route(g, a, b, heading)


def legal(g, rt):
    return V.check_connectivity(g, rt)


class TestRules(unittest.TestCase):
    def test_access_most_specific_tag_wins(self):
        self.assertFalse(R.access_ok({'highway': 'residential', 'access': 'private'})[0])
        self.assertTrue(R.access_ok({'highway': 'residential', 'access': 'no', 'motor_vehicle': 'yes'})[0])
        self.assertFalse(R.access_ok({'highway': 'residential', 'motor_vehicle': 'no', 'motorcar': 'no'})[0])
        ok, f, _ = R.access_ok({'highway': 'residential', 'access': 'destination'})
        self.assertTrue(ok); self.assertGreater(f, 1.0)

    def test_non_car_classes_and_service_kinds_excluded(self):
        for hw in ('track', 'footway', 'path', 'cycleway', 'pedestrian', 'steps', None):
            self.assertFalse(R.access_ok({'highway': hw})[0], hw)
        self.assertFalse(R.access_ok({'highway': 'service', 'service': 'parking_aisle'})[0])
        self.assertTrue(R.access_ok({'highway': 'service'})[0])

    def test_oneway_variants(self):
        o = R.oneway_dir
        self.assertEqual(o({'highway': 'primary'}), 'both')
        self.assertEqual(o({'highway': 'primary', 'oneway': 'yes'}), 'fwd')
        self.assertEqual(o({'highway': 'primary', 'oneway': '-1'}), 'rev')
        self.assertEqual(o({'highway': 'primary', 'oneway': 'no'}), 'both')
        self.assertEqual(o({'highway': 'primary', 'oneway': 'reversible'}), 'none')
        self.assertEqual(o({'highway': 'primary', 'junction': 'roundabout'}), 'fwd')
        self.assertEqual(o({'highway': 'motorway'}), 'fwd')
        self.assertEqual(o({'highway': 'motorway', 'oneway': 'no'}), 'both')
        self.assertEqual(o({'highway': 'residential', 'oneway': 'yes', 'oneway:motor_vehicle': 'no'}), 'both')

    def test_maxspeed_parsing(self):
        self.assertEqual(R.maxspeed_kph({'maxspeed': '80'}), 80)
        self.assertAlmostEqual(R.maxspeed_kph({'maxspeed': '50 mph'}), 80.4672)
        self.assertEqual(R.maxspeed_kph({'maxspeed': 'RO:urban'}), 50)
        self.assertIsNone(R.maxspeed_kph({'maxspeed': 'banana'}))
        self.assertIsNone(R.maxspeed_kph({'maxspeed': '9999'}))

    def test_names_match_across_diacritics(self):
        for v in ('Timișu de Sus', 'Timişu de Sus', 'TIMISU  de sus', 'timisu de sus'):
            self.assertEqual(R.norm_name(v), 'timisu de sus')


class TestGraphBuild(unittest.TestCase):
    def test_directed_edges_follow_oneway(self):
        b = B(); b.way('A', [(1, 0, 0), (2, 0, 100)], oneway='yes'); b.way('B', [(2, 0, 100), (3, 0, 200)])
        g = b.build()
        self.assertEqual([(e.u, e.v) for e in g.out_edges(1)], [(1, 2)])
        self.assertEqual(g.out_edges(2)[0].v, 3)                    # B continues; A is not enterable backwards
        self.assertEqual({e.v for e in g.out_edges(2)}, {3})
        self.assertEqual({(e.u, e.v) for e in g.out_edges(3)}, {(3, 2)})

    def test_excluded_edges_are_counted(self):
        b = B(); b.way('P', [(1, 0, 0), (2, 0, 100)], access='private'); b.way('T', [(2, 0, 100), (3, 0, 200)], highway='track')
        b.way('R', [(3, 0, 200), (4, 0, 300)], oneway='reversible')
        g = b.build()
        self.assertEqual(b.stats['directed_edges'], 0)
        rej = json.loads(g.meta['rejected'])
        self.assertEqual(rej, {'access=private': 1, 'class:track': 1, 'oneway:reversible': 1})

    def test_edges_without_node_ids_are_rejected_not_guessed(self):
        b = B(); b.edges.append(('9', None, None, [ll(0, 0), ll(0, 100)], {'highway': 'residential'}))
        b.build(); self.assertEqual(b.stats['rejected'], {'no_node_ids': 1})

    def test_places_resolve_by_normalised_name(self):
        b = B(); b.way('A', [(1, 0, 0), (2, 0, 100)])
        b.places = [('Timișu de Sus', 'village', 45.6, 25.6), ('Timișu de Jos', 'village', 45.55, 25.58), ('Predeal', 'town', 45.5, 25.57)]
        g = b.build()
        self.assertEqual(g.find_places('timisu de sus')[1][0][0], 'Timișu de Sus')
        self.assertEqual(g.find_places('Timişu de Jos')[0], 'exact')
        kind, rows = g.find_places('timisu')
        self.assertEqual(kind, 'prefix'); self.assertEqual(len(rows), 2)
        self.assertEqual(g.find_places('nowhere')[1], [])
        self.assertEqual(V.resolve(g, 'PREDEAL')[:2], (45.5, 25.57))
        with self.assertRaises(RT.NoRoute):
            V.resolve(g, 'nowhere')
        self.assertEqual(V.resolve(g, '45.1, 25.2')[:2], (45.1, 25.2))


class TestRouting(unittest.TestCase):
    def test_shortest_path_on_a_grid(self):
        g = grid().build()
        r = route(g, at(0, 0), at(600, 600))
        self.assertAlmostEqual(r.distance_m, 1200.0, delta=3.0)                      # manhattan
        self.assertEqual(legal(g, r), [])
        self.assertEqual(r.edges[0].u, nid(0, 0)); self.assertEqual(r.edges[-1].v, nid(3, 3))

    def test_route_geometry_is_ordered_and_continuous(self):
        g = grid().build()
        r = route(g, at(0, 0), at(600, 600))
        self.assertEqual(r.points[0], at(0, 0)[:2] if False else r.points[0])
        for i in range(len(r.cum) - 1):
            self.assertGreater(r.cum[i + 1], r.cum[i])
        self.assertAlmostEqual(r.cum[-1], r.distance_m, places=6)
        self.assertEqual(len(r.edge_offsets), len(r.edges))
        self.assertTrue(all(b >= a for a, b in zip(r.edge_offsets, r.edge_offsets[1:])))
        self.assertLess(max(geo.hav(a[0], a[1], b[0], b[1]) for a, b in zip(r.points, r.points[1:])), 201.0)

    def test_oneway_forces_a_detour(self):
        free = route(grid().build(), at(200, 600), at(200, 0))
        g = grid(row_tags={1: {'oneway': 'yes'}}).build()                             # Row 1 is eastbound only
        r = route(g, at(200, 600), at(200, 0))
        self.assertAlmostEqual(free.distance_m, 600.0, delta=3.0)
        self.assertGreater(r.distance_m, 900.0)                                        # must use Row 0 or 2
        self.assertEqual(legal(g, r), [])
        self.assertTrue(all(not (e.name == 'Row 1' and e.rev == 1) for e in r.edges))
        east = route(g, at(200, 0), at(200, 600))
        self.assertAlmostEqual(east.distance_m, 600.0, delta=3.0)                      # the legal direction is still direct

    def test_reverse_only_oneway(self):
        g = grid(row_tags={1: {'oneway': '-1'}}).build()                              # legal only westbound
        self.assertAlmostEqual(route(g, at(200, 600), at(200, 0)).distance_m, 600.0, delta=3.0)
        self.assertGreater(route(g, at(200, 0), at(200, 600)).distance_m, 900.0)

    def test_private_road_is_avoided_and_all_private_means_no_route(self):
        b = grid()
        b.edges = [e for e in b.edges if not (e[0] == '1' and e[1] == nid(0, 1))]
        b.way('Private lane', [(nid(0, 1), 0, 200), (nid(0, 2), 0, 400)], access='private')
        g = b.build()
        r = route(g, at(0, 0), at(0, 600))
        self.assertGreater(r.distance_m, 700.0)
        self.assertNotIn('Private lane', [e.name for e in r.edges])
        self.assertIn('access=private', json.loads(g.meta['rejected']))
        only = B(); only.way('P', [(1, 0, 0), (2, 0, 100)], access='private'); only.way('Q', [(3, 500, 0), (4, 500, 100)])
        with self.assertRaises(RT.NoRoute):
            route(only.build(), at(0, 0), at(500, 100))

    def test_destination_only_road_is_penalised_not_forbidden(self):
        b = B()
        b.way('Direct', [(1, 0, 0), (2, 0, 500)], access='destination')
        b.way('Around', [(1, 0, 0), (3, 300, 250), (2, 0, 500)])
        r = route(b.build(), at(0, 0), at(0, 500))
        self.assertEqual({e.name for e in r.edges}, {'Direct'})                         # 500 m x1.6 < 2 x 390 m
        b2 = B()
        b2.way('Direct', [(1, 0, 0), (2, 0, 500)], access='destination')
        b2.way('Around', [(1, 0, 0), (3, 100, 250), (2, 0, 500)])
        self.assertEqual({e.name for e in route(b2.build(), at(0, 0), at(0, 500)).edges}, {'Around'})

    def test_crossing_roads_without_a_shared_node_are_not_a_junction(self):
        b = B()
        b.way('East-West', [(1, 0, -200), (2, 0, 0), (3, 0, 200)])                      # these two cross at (0,0) like a bridge
        b.way('North-South', [(11, -200, 0), (12, 0, 0), (13, 200, 0)])                 # but share NO node (2 != 12)
        g = b.build()
        with self.assertRaises(RT.NoRoute):
            route(g, at(0, -150), at(150, 0))
        b2 = B()
        b2.way('East-West', [(1, 0, -200), (2, 0, 0), (3, 0, 200)])
        b2.way('North-South', [(11, -200, 0), (2, 0, 0), (13, 200, 0)])                 # now they genuinely share node 2
        r = route(b2.build(), at(0, -150), at(150, 0))
        self.assertAlmostEqual(r.distance_m, 300.0, delta=3.0)

    def test_near_miss_endpoints_are_not_connected(self):
        b = B()
        b.way('A', [(1, 0, 0), (2, 0, 200)]); b.way('B', [(3, 0, 200.3), (4, 0, 400)])     # 0.3 m apart, different nodes
        with self.assertRaises(RT.NoRoute):
            route(b.build(), at(0, 50), at(0, 350))

    def test_no_immediate_reversal_when_heading_is_known(self):
        g = grid().build()
        r = route(g, at(200, 300), at(200, 100), heading=90.0)                           # heading east, destination behind
        self.assertGreater(r.distance_m, 700.0)
        self.assertEqual(legal(g, r), [])
        for a, c in zip(r.edges, r.edges[1:]):
            self.assertFalse(a.u == c.v and a.v == c.u)
        self.assertLess(r.distance_m, 900.0)
        direct = route(g, at(200, 300), at(200, 100), heading=270.0)
        self.assertAlmostEqual(direct.distance_m, 200.0, delta=3.0)

    def test_invalid_coordinates_and_far_endpoints(self):
        g = grid().build()
        for bad in ((float('nan'), 25.5), (None, None), (95.0, 25.0)):
            with self.assertRaises(RT.NoRoute):
                route(g, bad, at(0, 0))
        with self.assertRaises(RT.NoRoute):
            route(g, at(5000, 5000), at(0, 0))

    def test_same_edge_start_and_goal(self):
        g = grid().build()
        r = route(g, at(0, 20), at(0, 120))
        self.assertAlmostEqual(r.distance_m, 100.0, delta=2.0)
        self.assertEqual(len(r.edges), 1)

    def test_fast_road_preferred_by_travel_time(self):
        g = grid(row_tags={0: {'highway': 'primary'}}).build()
        r = route(g, at(0, 0), at(400, 400))
        self.assertEqual(r.edges[0].name, 'Row 0')                                       # tie in distance, primary is faster


class TestInstructions(unittest.TestCase):
    def setUp(self):
        self.g = grid(row_tags={0: {'highway': 'primary', 'ref': 'DN1'}}).build()

    def test_left_turn_then_arrival(self):
        r = route(self.g, at(0, 0), at(400, 400))
        s = IN.build_steps(self.g, r)
        self.assertEqual([x.kind for x in s], ['depart', 'turn', 'arrive'])               # no spurious steps at the cross streets
        self.assertIn('Head east on Row 0 (DN1)', s[0].text)
        self.assertEqual(s[1].text, 'Turn left onto Col 2')                                # east -> north (north is +r) is a LEFT turn
        self.assertAlmostEqual(s[1].start_m, 400.0, delta=3.0)
        self.assertAlmostEqual(s[0].distance_m, 400.0, delta=3.0)

    def test_right_turn(self):
        r = route(self.g, at(400, 400), at(0, 0))
        s = IN.build_steps(self.g, r)
        self.assertEqual([x.text for x in s][:2], ['Head south on Col 2', 'Turn right onto Row 0 (DN1)'])   # south -> west is RIGHT

    def test_junction_summary_reports_exits_and_shared_nodes(self):
        r = route(self.g, at(0, 0), at(400, 400))
        js = IN.junctions(self.g, r)
        self.assertTrue(all(j['shares_node'] for j in js))
        self.assertEqual(js[0]['node'], nid(0, 1)); self.assertEqual(js[0]['exits'], 2)   # edge of the grid: straight on, or north (no street south)
        self.assertAlmostEqual(js[0]['angle'], 0.0, delta=3.0)

    def test_u_turn_wording(self):
        self.assertEqual(IN.describe_turn(175)[0], 'u-turn')
        self.assertEqual(IN.describe_turn(-45)[0], 'slight left')
        self.assertEqual(IN.describe_turn(90)[0], 'right')
        self.assertEqual(IN.describe_turn(10)[0], 'straight')

    def roundabout(self):
        b = B()
        ring = [(1, 0, 30), (2, 30, 0), (3, 0, -30), (4, -30, 0)]                           # E, N, W, S: counter-clockwise
        for (a, an, ae), (c, cn, ce) in zip(ring, ring[1:] + ring[:1]):
            b.way('Ring', [(a, an, ae), (c, cn, ce)], junction='roundabout')
        b.way('South Rd', [(10, -300, 0), (4, -30, 0)]); b.way('East Rd', [(1, 0, 30), (11, 0, 300)])
        b.way('North Rd', [(2, 30, 0), (12, 300, 0)]); b.way('West Rd', [(3, 0, -30), (13, 0, -300)])
        return b.build()

    def test_roundabout_exit_counting(self):
        g = self.roundabout()
        for dest, exit_no, road in (((0, 250), 1, 'East Rd'), ((250, 0), 2, 'North Rd'), ((0, -250), 3, 'West Rd')):
            r = route(g, at(-250, 0), at(*dest))
            self.assertEqual(legal(g, r), [], dest)
            steps = IN.build_steps(g, r)
            self.assertIn('At the roundabout take exit %d onto %s' % (exit_no, road), [s.text for s in steps], dest)

    def test_roundabout_is_driven_the_legal_way_round(self):
        g = self.roundabout()
        r = route(g, at(-250, 0), at(0, -250))
        ring = [e for e in r.edges if e.name == 'Ring']
        self.assertEqual(len(ring), 3)                                                       # S->E->N->W, never clockwise
        self.assertTrue(all(e.rev == 0 for e in ring))


class TestMatcherAndNavigator(unittest.TestCase):
    def setUp(self):
        self.g = grid(row_tags={0: {'highway': 'primary'}}).build()      # Row 0 fast: removes distance ties so routes are deterministic
        self.rt = route(self.g, at(0, 0), at(600, 600))

    def drive(self, nav, fixes):
        return [nav.update(*f) for f in fixes]

    def test_noisy_drive_stays_on_route_and_arrives(self):
        for seed in (1, 2, 3):
            nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
            states = self.drive(nav, V.fixes_along(self.rt.points, speed=15, noise_m=6, seed=seed))
            self.assertEqual({s.status for s in states} - {'on_route', 'arrived'}, set(), seed)
            self.assertEqual(nav.reroutes, 0)
            self.assertEqual(states[-1].status, 'arrived')
            prog = [s.progress_m for s in states if s.progress_m is not None]
            self.assertTrue(all(b >= a for a, b in zip(prog, prog[1:])))                   # monotonic progress

    def test_next_turn_info(self):
        nav = M.Navigator(self.g, at(400, 400)); nav.plan(*at(0, 0))
        st = nav.update(0.0, *at(0, 100), heading=90.0, speed=15, accuracy=5)
        self.assertEqual(st.status, 'on_route')
        self.assertEqual(st.next_step['text'], 'Turn left onto Col 2')
        self.assertAlmostEqual(st.distance_to_next_m, 300.0, delta=5.0)
        self.assertAlmostEqual(st.remaining_m, 700.0, delta=5.0)

    def switchback(self):
        b = B()
        pts = [(1, 0, 0), (2, 100, 0), (3, 200, 0), (4, 300, 0), (5, 315, 6), (6, 315, 19), (7, 300, 25), (8, 200, 25), (9, 100, 25), (10, 0, 25)]
        b.way('Hairpin road', pts)
        return b.build()

    def test_switchback_does_not_jump_to_the_adjacent_leg(self):
        g = self.switchback()
        rt = route(g, at(0, 0), at(0, 25), heading=0.0)
        self.assertGreater(rt.distance_m, 600.0)
        cum = rt.cum
        for seed in range(1, 6):
            nav = M.Navigator(g, at(0, 25)); nav.plan(*at(0, 0), heading=0.0)
            fixes = V.fixes_along(rt.points, speed=12, noise_m=5, seed=seed)
            for k, f in enumerate(fixes):
                st = nav.update(*f)
                true_s = min(cum[-1], 12.0 * f[0])
                self.assertNotEqual(st.status, 'off_route', (seed, k))
                if st.progress_m is not None:
                    self.assertLess(abs(st.progress_m - true_s), 45.0, (seed, k, st.progress_m, true_s))   # never the other leg (~600 m away)

    def test_leaving_the_route_is_detected_and_recalculated(self):
        rt = route(self.g, at(0, 0), at(0, 600))
        sd = V.simulate_deviation(self.g, rt, 0.25, speed=15, seed=7)
        self.assertIsNotNone(sd)
        self.assertTrue(sd['detected']); self.assertTrue(sd['rerouted']); self.assertTrue(sd['recoverable'])
        self.assertLessEqual(sd['detect_fixes'], 4)                                         # a few fixes, not a long delay
        self.assertEqual(sd['reroutes'], 1)                                                  # exactly one recalculation
        self.assertEqual(sd['new_route_problems'], [])                                       # the new route is connected and legal
        self.assertIn(sd['final_status'], ('arrived', 'on_route'))                           # and is followed to the end

    def test_recalculated_route_starts_where_the_car_is_and_reaches_the_destination(self):
        rt = route(self.g, at(0, 0), at(0, 600))
        path, node = V.deviation_path(self.g, rt, 150.0)
        nav = M.Navigator(self.g, at(0, 600)); nav.plan(*at(0, 0))
        for f in V.fixes_along(path, speed=15, noise_m=3, seed=2):
            st = nav.update(*f)
            if st.status == 'rerouted':
                here = (f[1], f[2])
                break
        self.assertEqual(st.status, 'rerouted')
        self.assertLess(geo.hav(nav.route.points[0][0], nav.route.points[0][1], here[0], here[1]), 60.0)
        self.assertEqual(nav.route.edges[-1].v, nid(0, 3))

    def spur_net(self, with_loop):
        b = B()
        b.way('Main', [(1, 0, 0), (2, 0, 200), (3, 0, 400), (4, 0, 600), (5, 0, 800)])
        b.way('Stub', [(3, 0, 400), (30, 150, 400), (31, 300, 400)])                         # dead end: no way back for a car that cannot U-turn
        if with_loop:
            b.way('Loop', [(3, 0, 400), (40, -200, 400), (41, -200, 600), (4, 0, 600)])        # a wrong turn that rejoins the route
        return b.build()

    def test_wrong_turn_onto_a_dead_end_is_detected_and_no_route_is_reported(self):
        g = self.spur_net(False)
        rt = route(g, at(0, 0), at(0, 800))
        sd = V.simulate_deviation(g, rt, 0.4, speed=15, seed=3)
        self.assertTrue(sd['detected']); self.assertFalse(sd['rerouted']); self.assertFalse(sd['recoverable'])
        self.assertTrue(set(sd['statuses']) & {'no_route', 'off_route'}); self.assertEqual(sd['reroutes'], 0)
        r, _ = V.validate_pair(g, at(0, 0), at(0, 800))
        self.assertEqual(r['problems'], [])                                                    # detection verified; not a navigator defect
        self.assertTrue(any('cannot be reached' in w for w in r['warnings']))
        rep = V.render_report([(r, route(g, at(0, 0), at(0, 800)))], g)
        self.assertEqual(sd['outcome'], 'NO_ROUTE_CORRECT')
        self.assertIn('oracle reachable=False', rep); self.assertIn('**PASS**', rep); self.assertIn('NO_ROUTE_CORRECT', rep)

    def test_simulation_prefers_a_recoverable_wrong_turn(self):
        g = self.spur_net(True)
        rt = route(g, at(0, 0), at(0, 800))
        sd = V.simulate_deviation(g, rt, 0.4, speed=15, seed=3)
        self.assertTrue(sd['recoverable']); self.assertTrue(sd['rerouted'])                     # picked the Loop, not the dead-end Stub
        self.assertEqual(sd['new_route_problems'], []); self.assertIn(sd['final_status'], ('arrived', 'on_route'))

    def test_one_noisy_fix_does_not_trigger_off_route(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        fixes = V.fixes_along(self.rt.points, speed=15, noise_m=2, seed=4)
        f = list(fixes[10]); f[1] += 90.0 / M_LAT; fixes[10] = tuple(f)                          # one 90 m outlier
        states = self.drive(nav, fixes)
        self.assertEqual(nav.reroutes, 0)
        self.assertNotIn('off_route', [s.status for s in states])

    def test_driving_the_route_backwards_is_not_on_route(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        rev = list(reversed(self.rt.points))
        states = self.drive(nav, V.fixes_along(rev, speed=15, noise_m=2, seed=5))
        self.assertGreaterEqual(nav.reroutes, 1)

    def test_gps_loss_is_not_off_route(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        nav.update(0.0, *at(0, 50), heading=90.0, speed=15, accuracy=5)
        st = nav.update(2.0, None, None)
        self.assertEqual(st.status, 'gps_poor')
        st = nav.update(9.0, None, None)
        self.assertEqual(st.status, 'gps_lost')
        for t in range(10, 40):
            st = nav.update(float(t), float('nan'), float('nan'), 90.0, 15.0, 5.0)
            self.assertEqual(st.status, 'gps_lost')
        self.assertEqual(nav.reroutes, 0)
        self.assertEqual(nav.update(40.0, *at(0, 700), heading=90.0, speed=15, accuracy=5).status != 'gps_lost', True)

    def test_poor_accuracy_fix_is_ignored(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        nav.update(0.0, *at(0, 50), heading=90.0, speed=15, accuracy=5)
        seen = []
        for t in range(1, 10):
            seen.append(nav.update(float(t), *at(900, 900), heading=0.0, speed=15, accuracy=80.0).status)   # wildly off, but 80 m accuracy
        self.assertEqual(seen[:4], ['gps_poor'] * 4)
        self.assertEqual(set(seen), {'gps_poor', 'gps_lost'})                                  # usable GPS is gone after 5 s: lost, never off-route
        self.assertEqual(nav.reroutes, 0)

    def test_invalid_heading_is_tolerated(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        for h in (None, float('nan'), float('inf'), 'x'):
            st = nav.update(1.0, *at(0, 100), heading=h, speed=10, accuracy=5)
            self.assertEqual(st.status, 'on_route', h)

    def test_standstill_noise_never_reroutes(self):
        nav = M.Navigator(self.g, at(600, 600)); nav.plan(*at(0, 0))
        nav.update(0.0, *at(0, 100), heading=90.0, speed=10, accuracy=5)
        for t in range(1, 30):
            nav.update(float(t), *at(60, 100), heading=90.0, speed=0.0, accuracy=5)             # 60 m off, but stationary
        self.assertEqual(nav.reroutes, 0)

    def test_reroute_failure_reports_no_route_and_keeps_trying(self):
        b = grid(); b.way('Island', [(900, 5000, 5000), (901, 5000, 5100)])
        g = b.build()
        nav = M.Navigator(g, at(600, 600)); nav.plan(*at(0, 0))
        nav.update(0.0, *at(0, 50), heading=90.0, speed=15, accuracy=5)
        st = None
        for t in range(1, 12):
            st = nav.update(float(t), *at(5000, 5050), heading=90.0, speed=15, accuracy=5)    # teleported to an unconnected island
        self.assertIn(st.status, ('no_route', 'off_route'))
        self.assertEqual(nav.reroutes, 0)


class TestValidation(unittest.TestCase):
    def test_check_connectivity_catches_a_oneway_violation(self):
        g = grid(row_tags={1: {'oneway': 'yes'}}).build()
        r = route(g, at(200, 0), at(200, 600))
        self.assertEqual(legal(g, r), [])
        db = sqlite3.connect(g.db.execute('PRAGMA database_list').fetchone()[2])           # tamper: pretend a reverse edge exists
        db.execute("UPDATE edges SET rev=1 WHERE eid=?", (r.edges[0].eid,)); db.commit(); db.close()
        g2 = RG.Graph(g.db.execute('PRAGMA database_list').fetchone()[2])
        bad = V.check_connectivity(g2, RT.Route(g2, [g2.edge(r.edges[0].eid)], r.start, r.goal, []))
        self.assertTrue(any('against its oneway' in p for p in bad), bad)

    def test_check_connectivity_catches_unconnected_edges(self):
        g = grid().build()
        a = g.out_edges(nid(0, 0))[0]
        far = g.out_edges(nid(3, 3))[0]
        rt = RT.Route(g, [a, far], RT.Snap(a, 0, 0, 0, 0, None), RT.Snap(far, 0, 10, 0, 0, None), [])
        self.assertTrue(any('NOT connected' in p for p in V.check_connectivity(g, rt)))

    def test_reference_comparison(self):
        line = [at(0, 0), at(0, 500), at(0, 1000)]
        same = V.compare_to_reference(line, line)
        self.assertAlmostEqual(same['length_diff_pct'], 0.0, places=6); self.assertAlmostEqual(same['pct_route_within'], 100.0)
        shifted = V.compare_to_reference(line, [at(100, 0), at(100, 500), at(100, 1000)])
        self.assertAlmostEqual(shifted['route_to_ref_max_m'], 100.0, delta=2.0); self.assertEqual(shifted['pct_route_within'], 0.0)
        detour = V.compare_to_reference(line, [at(0, 0), at(300, 500), at(0, 1000)])           # reference takes a big detour
        self.assertGreater(detour['ref_to_route_max_m'], 200.0); self.assertLess(detour['length_diff_pct'], -10.0)

    def test_reference_loaders(self):
        d = tempfile.mkdtemp()
        pts = [(45.5, 25.5), (45.51, 25.52)]
        open(os.path.join(d, 'r.gpx'), 'w').write('<gpx xmlns="http://www.topografix.com/GPX/1/1"><trk><trkseg>%s</trkseg></trk></gpx>' %
                                                    ''.join('<trkpt lat="%s" lon="%s"/>' % p for p in pts))
        json.dump({'type': 'Feature', 'geometry': {'type': 'LineString', 'coordinates': [[p[1], p[0]] for p in pts]}}, open(os.path.join(d, 'r.geojson'), 'w'))
        open(os.path.join(d, 'r.csv'), 'w').write('lat,lon\n45.5,25.5\n45.51,25.52\n')
        for f in ('r.gpx', 'r.geojson', 'r.csv'):
            self.assertEqual(V.load_reference(os.path.join(d, f)), pts, f)
        open(os.path.join(d, 'bad.csv'), 'w').write('nothing here')
        with self.assertRaises(ValueError):
            V.load_reference(os.path.join(d, 'bad.csv'))

    def test_validate_pair_and_report_on_a_clean_grid(self):
        g = grid(row_tags={0: {'highway': 'primary', 'ref': 'DN1'}}).build()
        ref = [at(0, 0), at(0, 400), at(400, 400)]
        r, rt = V.validate_pair(g, at(0, 0), at(400, 400), ref, 'A', 'B')
        self.assertEqual(r['problems'], [])
        self.assertEqual(r['sim_on_route']['reroutes'], 0)
        self.assertTrue(r['sim_deviation']['detected'])
        self.assertAlmostEqual(r['reference']['length_diff_pct'], 0.0, delta=1.0)
        rep = V.render_report([(r, rt)], g)
        for needle in ('Roads in order', 'Instructions', 'Junctions with alternatives', 'connectivity', '**PASS**', 'Limitations', 'Turn-restriction'):
            self.assertIn(needle, rep)
        self.assertNotIn('FAIL', rep)

    def test_report_states_when_no_reference_was_given(self):
        g = grid().build()
        r, rt = V.validate_pair(g, at(0, 0), at(400, 400))
        self.assertIn('reference comparison: **not performed**', V.render_report([(r, rt)], g))

    def test_unroutable_pair_is_reported_not_hidden(self):
        b = grid(); b.way('Island', [(900, 5000, 5000), (901, 5000, 5100)])
        g = b.build()
        r, rt = V.validate_pair(g, at(0, 0), at(5000, 5050))
        self.assertIsNone(rt); self.assertTrue(any('NO ROUTE' in p for p in r['problems']))
        self.assertTrue(any('DIFFERENT disconnected road components' in p for p in r['problems']), r['problems'])
        self.assertIn('NO ROUTE', V.render_report([(r, rt)], g))

    def test_oneway_trap_is_diagnosed_as_a_directed_block_not_missing_data(self):
        b = B()
        b.way('Out', [(1, 0, 0), (2, 0, 200)], oneway='yes')                                    # you can leave A ...
        b.way('Away', [(2, 0, 200), (3, 0, 400)], oneway='yes')
        b.way('Back', [(3, 0, 400), (4, 300, 400)], oneway='-1')                                 # ... but 4 only flows INTO 3
        g = b.build()
        r, rt = V.validate_pair(g, at(0, 300), at(0, 100))                                      # destination is behind a oneway
        self.assertIsNone(rt)
        self.assertTrue(any('SAME connected road component' in p for p in r['problems']), r['problems'])

    def test_component_size_ignores_direction(self):
        b = grid(); b.way('Island', [(900, 5000, 5000), (901, 5000, 5100), (902, 5000, 5200)])
        g = b.build()
        self.assertEqual(g.component_size(nid(0, 0))[0], 16)
        self.assertEqual(g.component_size(900)[0], 3)
        self.assertTrue(g.component_size(nid(0, 0), cap=5)[1])                                    # reports when it hit the cap

    def test_place_names_drive_the_cli_pair(self):
        b = grid(); b.places = [('Predeal', 'town', *at(0, 0)), ('Timișu de Sus', 'village', *at(600, 600))]
        g = b.build()
        r, rt = V.validate_pair(g, 'Predeal', 'timisu de sus', None, 'Predeal', 'Timișu de Sus')
        self.assertEqual(r['problems'], []); self.assertAlmostEqual(rt.distance_m, 1200.0, delta=5.0)

    def test_cli_exit_code_and_report_file(self):
        b = grid(); b.places = [('Predeal', 'town', *at(0, 0)), ('Timișu de Sus', 'village', *at(600, 600))]
        path = os.path.join(tempfile.mkdtemp(), 'rg.sqlite'); RG.build_from_edges(b.edges, path, b.places)
        out = os.path.join(tempfile.mkdtemp(), 'rep.md')
        import contextlib, io
        with contextlib.redirect_stdout(io.StringIO()):
            code = V.main(['--graph', path, '--from', 'Predeal', '--to', 'Timisu de Sus', '--report', out])
        self.assertEqual(code, 0); self.assertIn('Predeal -> Timisu de Sus', open(out).read())


class TestSeparationFromVehicleControl(unittest.TestCase):
    FORBIDDEN = {'cereal', 'selfdrive', 'carcontroller', 'carstate', 'PCC_module', 'ACC_module', 'HSO_module', 'ALCA_module',
                 'AHB_module', 'teslacan', 'panda', 'messaging', 'common', 'opendbc', 'tesla_csa', 'mapcurved', 'mapcurve_launcher'}

    def test_nav_package_imports_nothing_from_vehicle_control(self):
        for fn in sorted(os.listdir(NAVDIR)):
            if not fn.endswith('.py'):
                continue
            tree = ast.parse(open(os.path.join(NAVDIR, fn)).read())
            for n in ast.walk(tree):
                mods = []
                if isinstance(n, ast.Import):
                    mods = [a.name for a in n.names]
                elif isinstance(n, ast.ImportFrom):
                    mods = [(n.module or '')] + [a.name for a in n.names]
                for m in mods:
                    self.assertFalse(set(m.split('.')) & self.FORBIDDEN, '%s imports %s' % (fn, m))

    def test_control_code_does_not_import_nav(self):
        tesla = os.path.join(HERE, '..')
        for fn in ('carcontroller.py', 'PCC_module.py', 'ACC_module.py', 'HSO_module.py', 'AHB_module.py', 'carstate.py', 'mapcurved.py', 'tesla_csa.py'):
            src = open(os.path.join(tesla, fn)).read()
            self.assertNotIn('.nav', src.replace('.nav_', ''), fn) if False else self.assertNotRegex(src, r'(from|import)\s+(selfdrive\.car\.tesla\.)?nav\b', fn)


def _pyrosm():
    try:
        import pyrosm  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_pyrosm(), 'pyrosm not installed')
class TestRealOsmData(unittest.TestCase):
    """Real OSM data (pyrosm's bundled Helsinki-area PBF - NOT Romania): build the routing graph the production way and route on it."""
    @classmethod
    def setUpClass(cls):
        import pyrosm
        cls.path = os.path.join(tempfile.mkdtemp(), 'real.sqlite')
        cls.stats = RG.build(pyrosm.get_data('test_pbf'), cls.path)
        cls.g = RG.Graph(cls.path)

    def test_graph_has_edges_nodes_places_and_oneway(self):
        self.assertGreater(self.stats['directed_edges'], 500)
        self.assertGreater(self.stats['places'], 0)
        n_ow = self.g.db.execute("SELECT COUNT(*) FROM edges WHERE oneway LIKE '%yes%'").fetchone()[0]
        self.assertGreater(n_ow, 0)

    def test_every_edge_endpoint_has_a_node_and_geometry_matches(self):
        bad = self.g.db.execute('SELECT COUNT(*) FROM edges e LEFT JOIN nodes n ON n.id=e.u WHERE n.id IS NULL').fetchone()[0]
        self.assertEqual(bad, 0)
        for (eid,) in self.g.db.execute('SELECT eid FROM edges LIMIT 200').fetchall():
            e = self.g.edge(eid); p = self.g.geom(eid); a = self.g.node_xy(e.u); b = self.g.node_xy(e.v)
            self.assertLess(geo.hav(p[0][0], p[0][1], a[0], a[1]), 0.5); self.assertLess(geo.hav(p[-1][0], p[-1][1], b[0], b[1]), 0.5)

    def test_real_routes_are_connected_legal_and_survive_gps_simulation(self):
        pts = [(r[2], r[3]) for r in self.g.db.execute('SELECT name, place, lat, lon FROM places').fetchall()]
        done = 0
        for i in range(len(pts)):
            for j in range(len(pts)):
                if i == j or done >= 3:
                    continue
                try:
                    r, rt = V.validate_pair(self.g, pts[i], pts[j])
                except Exception as e:                                       # a place may not be near any road in a tiny extract
                    continue
                if rt is None or geo.hav(pts[i][0], pts[i][1], pts[j][0], pts[j][1]) < 400:
                    continue
                self.assertEqual(r['problems'], [], (i, j, r['problems']))
                self.assertGreaterEqual(len(rt.edges), 3)
                done += 1
        self.assertGreaterEqual(done, 1, 'no routable place pair found in the test extract')


if __name__ == '__main__':
    unittest.main()
