#!/usr/bin/env python3
"""Graph data pipeline: schema v2 tags, v1 compatibility, and tag preservation from a PBF with KNOWN truth (written with pyosmium).
    python3 -m unittest tesla.tests.test_nav_data"""
import math
import os
import sqlite3
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import routegraph as RG, router as RT, scenarios as S  # noqa: E402


def build(edges):
    p = os.path.join(tempfile.mkdtemp(), 'g.sqlite')
    RG.build_from_edges(edges, p, [])
    return p


def E(way, u, v, tags, n0=0.0):
    return (way, u, v, [S.ll(n0, 100.0 * u), S.ll(n0, 100.0 * v)], dict({'highway': 'primary'}, **tags))


class TestSchemaV2(unittest.TestCase):
    def rows(self, edges):
        db = sqlite3.connect(build(edges))
        return {r[0]: r[1:] for r in db.execute('select way_id, bridge, tunnel, layer, maxspeed_kph, roundabout from edges where rev=0')}

    def test_bridge_tunnel_layer_maxspeed_roundabout_are_stored(self):
        r = self.rows([E('b', 1, 2, {'bridge': 'viaduct', 'layer': '1', 'maxspeed': '50'}), E('t', 2, 3, {'tunnel': 'building_passage', 'layer': '-1'}),
                       E('n', 3, 4, {'tunnel': 'no', 'bridge': 'no'}), E('r', 4, 5, {'junction': 'roundabout'}), E('x', 5, 6, {'layer': '99'})])
        self.assertEqual(r['b'], (1, 0, 1, 50.0, 0)); self.assertEqual(r['t'], (0, 1, -1, None, 0))
        self.assertEqual(r['n'], (0, 0, None, None, 0)); self.assertEqual(r['r'][4], 1)
        self.assertIsNone(r['x'][2])                                                        # an implausible layer is not stored

    def test_schema_version_and_columns(self):
        g = RG.Graph(build([E('a', 1, 2, {})]))
        self.assertEqual(g.meta['schema_version'], '2'); self.assertTrue(g.has_v2)
        info = g.edge_info(g.out_edges(1)[0].eid)
        self.assertEqual((info['bridge'], info['tunnel']), (0, 0))

    def test_v1_database_still_reads_and_routes(self):
        net = S.Net()
        ids = [net.node() for _ in range(4)]
        net.way('Main', [(ids[c], 0.0, 100.0 * c) for c in range(4)])
        p = build(net.edges)
        db = sqlite3.connect(p)                                                              # downgrade it to a genuine v1 layout
        db.executescript('CREATE TABLE e1 AS SELECT eid, way_id, seg, u, v, length_m, cost_s, highway, name, ref, roundabout, rev, oneway, access, geom, cell_lat, cell_lon FROM edges;'
                         'DROP TABLE edges; ALTER TABLE e1 RENAME TO edges; CREATE INDEX idx_edges_u ON edges(u); CREATE INDEX idx_edges_v ON edges(v);'
                         'CREATE INDEX idx_edges_cell ON edges(cell_lat, cell_lon); UPDATE meta SET value=\'1\' WHERE key=\'schema_version\';')
        db.commit(); db.close()
        g = RG.Graph(p)
        self.assertFalse(g.has_v2)
        r = RT.find_route(g, S.at(0, 0), S.at(0, 300))
        self.assertAlmostEqual(r.distance_m, 300.0, delta=3.0)
        info = g.edge_info(g.out_edges(ids[0])[0].eid)
        self.assertIsNone(info['bridge']); self.assertIsNone(info['maxspeed_kph'])           # absent on v1, not an error

    def test_unknown_schema_is_refused(self):
        p = build([E('a', 1, 2, {})])
        db = sqlite3.connect(p); db.execute("UPDATE meta SET value='99' WHERE key='schema_version'"); db.commit(); db.close()
        with self.assertRaises(RuntimeError):
            RG.Graph(p)


def _libs():
    try:
        import osmium  # noqa: F401
        import pyrosm  # noqa: F401
        return True
    except ImportError:
        return False


@unittest.skipUnless(_libs(), 'pyosmium / pyrosm not installed')
class TestTagPreservationFromAPbfWithKnownTruth(unittest.TestCase):
    """A 600-byte PBF written here: roundabout, tunnel, bridge, layer, oneway=-1, private lane, motor_vehicle=no, motorway."""
    @classmethod
    def setUpClass(cls):
        import osmium
        cls.pbf = os.path.join(tempfile.mkdtemp(), 'truth.osm.pbf')
        w = osmium.SimpleWriter(cls.pbf)
        lat0, lon0 = 45.50, 25.57
        pt = lambda n, e: (lon0 + e / (111320 * math.cos(math.radians(lat0))), lat0 + n / 111320)
        for i, (n, e) in {1: (0, 0), 2: (0, 200), 3: (0, 400), 4: (0, 600), 5: (0, 800), 6: (0, 1000), 10: (30, 500), 11: (0, 530), 12: (-30, 500), 13: (0, 470),
                          20: (200, 200), 21: (200, 400), 22: (200, 600), 23: (230, 600), 24: (200, 630), 30: (-200, 200), 31: (-200, 400)}.items():
            w.add_node(osmium.osm.mutable.Node(id=i, location=pt(n, e), tags={}))
        way = lambda i, ns, **t: w.add_way(osmium.osm.mutable.Way(id=i, nodes=ns, tags=t))
        way(100, [1, 2], highway='primary', name='Entry Rd', ref='DN1')
        way(101, [2, 3], highway='primary', name='Tunnel St', tunnel='yes', layer='-1', maxspeed='50')
        way(102, [3, 13], highway='primary')
        way(103, [13, 10, 11, 12, 13], highway='primary', junction='roundabout', name='The Roundabout')
        way(104, [11, 4], highway='primary', name='Exit Rd')
        way(105, [4, 5], highway='primary', name='Bridge Ave', bridge='yes', layer='1', oneway='yes')
        way(106, [5, 6], highway='secondary', name='Private Lane', access='private')
        way(107, [2, 20], highway='residential', name='Side A', oneway='-1')
        way(108, [3, 31], highway='tertiary', name='Side B', motor_vehicle='no')
        way(109, [2, 30], highway='motorway', name='Motorway Stub')
        way(110, [20, 21], highway='service', name='Service Lane')                           # a normal service road: must be routable
        way(111, [30, 31], highway='service', service='driveway')                          # a driveway: excluded by OUR rules, not silently by the reader
        way(112, [21, 22], highway='service', service='parking_aisle')
        way(113, [22, 23, 24, 22], highway='service', junction='roundabout', tunnel='yes')   # a service roundabout in a tunnel
        w.close()

    def build(self, raw):
        p = os.path.join(tempfile.mkdtemp(), 'g.sqlite')
        RG.build(self.pbf, p, use_raw_tags=raw)
        return sqlite3.connect(p)

    def test_roundabout_tunnel_bridge_layer_are_preserved_either_way(self):
        for raw in (False, True):
            db = self.build(raw)
            q = lambda sql: db.execute(sql).fetchone()[0]
            self.assertEqual(q('select count(*) from edges where roundabout=1'), 7, raw)             # way 103 (4 edges) + the service roundabout 113 (3), both one-way
            self.assertEqual(q("select count(*) from edges where way_id='101' and tunnel=1 and layer=-1"), 2, raw)
            self.assertEqual(q("select count(*) from edges where way_id='105' and bridge=1 and layer=1"), 1, raw)   # oneway=yes: one direction

    def test_service_roads_are_not_silently_dropped_by_the_reader(self):
        """DEFECT found by auditing a real Romanian extract against the raw PBF: the graph was built with pyrosm network_type='driving', which drops EVERY highway=service
        way (100% of 4,240 routable service ways in the Predeal-Brasov extract), including service tunnels and roundabouts. Our own rules decide which service roads count."""
        for raw in (False, True):
            db = self.build(raw)
            ways = {r[0] for r in db.execute('select distinct way_id from edges')}
            self.assertIn('110', ways, raw)                                                   # an ordinary service road is routable
            self.assertNotIn('111', ways); self.assertNotIn('112', ways)                      # driveway / parking aisle: excluded by osm_rules
            self.assertEqual(db.execute("select count(*) from edges where way_id='113' and roundabout=1 and tunnel=1").fetchone()[0], 3, raw)
            rej = dict(db.execute("select key, value from meta where key='rejected'").fetchall())['rejected']
            self.assertIn('service:driveway', rej)                                            # our rules' exclusions are counted, not silent (pyrosm pre-drops parking aisles itself)

    def test_oneway_access_and_names_are_preserved(self):
        db = self.build(True)
        q = lambda sql: db.execute(sql).fetchall()
        self.assertEqual(q("select count(*), min(rev), max(rev) from edges where way_id='107'"), [(1, 1, 1)])      # oneway=-1: only against the way order
        self.assertEqual(q("select count(*) from edges where way_id='109'"), [(1,)])                                # motorway: implied oneway
        ways = {r[0] for r in q('select distinct way_id from edges')}
        self.assertFalse(ways & {'106', '108'})                                                                    # private / motor_vehicle=no are not routable
        self.assertEqual(q("select name, ref from edges where way_id='100' limit 1"), [('Entry Rd', 'DN1')])
        self.assertEqual(q("select count(*) from edges where way_id='103' and name='The Roundabout'"), [(4,)])

    def test_raw_tags_are_ground_truth_and_provenance_is_recorded(self):
        raw = RG.raw_way_tags(self.pbf)
        self.assertEqual(raw['103']['junction'], 'roundabout'); self.assertEqual(raw['101']['tunnel'], 'yes')
        self.assertNotIn('106x', raw)
        db = self.build(True)
        meta = dict(db.execute('select * from meta'))
        self.assertEqual(meta['source_file'], 'truth.osm.pbf'); self.assertEqual(len(meta['source_md5']), 32)
        self.assertEqual(meta['source_md5'], RG.file_md5(self.pbf)); self.assertIn('pyosmium', meta['tag_source'])
        self.assertEqual(self.build(False).execute("select value from meta where key='tag_source'").fetchone()[0].split(' ')[0], 'pyrosm')


if __name__ == '__main__':
    unittest.main()


class TestSpatialIndex(unittest.TestCase):
    """The R-tree replaces a grid that scanned ~24,000 edges (347 ms) per fix in dense Bucharest and could not see the middle of a long edge."""
    def short_net(self):
        net = S.Net()
        for r in range(6):
            ids = [net.node() for _ in range(8)]
            net.way('Row%d' % r, [(ids[c], 90.0 * r, 70.0 * c) for c in range(8)])
        return net

    def graphs(self, net):
        p = os.path.join(tempfile.mkdtemp(), 'g.sqlite')
        RG.build_from_edges(net.edges, p, [])
        a, b = RG.Graph(p), RG.Graph(p)
        b.has_rtree = False                                           # the same database through the grid fallback
        return a, b

    def test_rtree_is_built_and_recorded(self):
        g, _ = self.graphs(self.short_net())
        self.assertTrue(g.has_rtree); self.assertEqual(g.meta['spatial_index'], 'rtree')
        self.assertEqual(g.db.execute('select count(*) from edge_rtree').fetchone()[0], g.db.execute('select count(*) from edges').fetchone()[0])

    def test_rtree_and_grid_agree_on_short_edges(self):
        import random
        a, b = self.graphs(self.short_net())
        rnd = random.Random(4)
        for _ in range(60):
            la, lo = S.at(rnd.uniform(-40, 500), rnd.uniform(-40, 520))
            for hd in (None, 90.0, 250.0):
                x = [(s.edge.eid, round(s.dist_m, 6)) for s in a.snap(la, lo, hd, 60.0, limit=50)]
                y = [(s.edge.eid, round(s.dist_m, 6)) for s in b.snap(la, lo, hd, 60.0, limit=50)]
                self.assertEqual(x, y, (la, lo, hd))

    def test_a_long_edge_is_found_mid_way_by_the_rtree_but_missed_by_the_grid(self):
        """DEFECT in the grid: an edge is indexed by its START cell only, so a fix more than ~2 cells (1.1 km) from the start of a long rural edge never sees it."""
        net = S.Net()
        a, b = net.node(), net.node()
        net.way('Long Rural Road', [(a, 0.0, 0.0), (b, 0.0, 6000.0)])           # one 6 km edge between two nodes
        g, grid = self.graphs(net)
        la, lo = S.at(4.0, 3000.0)                                              # 4 m from the middle of it
        self.assertTrue(g.snap(la, lo, 90.0, 35.0), 'rtree must find the edge')
        self.assertFalse(grid.snap(la, lo, 90.0, 35.0), 'the grid cannot (this is the defect the rtree fixes)')
        r = RT.find_route(g, S.at(0, 100), S.at(0, 5900))
        self.assertAlmostEqual(r.distance_m, 5800.0, delta=5.0)

    def test_database_without_the_rtree_table_still_works(self):
        net = self.short_net()
        p = os.path.join(tempfile.mkdtemp(), 'g.sqlite')
        RG.build_from_edges(net.edges, p, [])
        db = sqlite3.connect(p); db.execute('DROP TABLE edge_rtree'); db.commit(); db.close()
        g = RG.Graph(p)
        self.assertFalse(g.has_rtree)
        r = RT.find_route(g, S.at(0, 10), S.at(0, 400))                         # within one row (the rows are not connected to each other)
        self.assertAlmostEqual(r.distance_m, 390.0, delta=3.0)
