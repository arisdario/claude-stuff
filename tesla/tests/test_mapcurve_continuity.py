#!/usr/bin/env python3
"""OSM node-ID route continuity: builder (tools/mapcurve_build.py) + runtime (mapcurved.MapCurveLookup), tested end to end
with synthetic networks. No openpilot needed:   python3 -m unittest tesla.tests.test_mapcurve_continuity

Rules under test: roads are joined ONLY through a shared OSM node ID (never because coordinates are close); oneway is
respected; the same OSM way continues through a node whatever its deflection; between DIFFERENT roads heading picks the
continuation only if it is clearly straight ahead and clearly better than the runner-up; otherwise prediction STOPS."""
import contextlib
import io
import math
import os
import sqlite3
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, os.path.join(HERE, '..', 'tools'))
try:
  import numpy  # noqa: F401
  import mapcurve_build as B
except ImportError:                                         # the builder needs numpy (a PC tool, not on the EON)
  B = None
import mapcurved as MC  # noqa: E402

LAT0, LON0 = 45.0, 25.0
M_LAT, M_LON = MC.meters_per_deg(LAT0)


def ll(p):
  """(north_m, east_m) -> (lon, lat)"""
  return (LON0 + p[1] / M_LON, LAT0 + p[0] / M_LAT)


def line(p0, bearing, length, step=20.0):
  b = math.radians(bearing)
  n = int(round(length / step))
  return [(p0[0] + i * step * math.cos(b), p0[1] + i * step * math.sin(b)) for i in range(n + 1)]


def arc(p0, bearing, radius, turn, right=True, step_deg=15.0):
  """points AFTER p0 along a circular arc that turns `turn` degrees (right or left) starting on `bearing`."""
  sgn = 1 if right else -1
  cb = math.radians(bearing + 90 * sgn)
  c = (p0[0] + radius * math.cos(cb), p0[1] + radius * math.sin(cb))
  out = []
  k = int(round(turn / step_deg))
  for i in range(1, k + 1):
    pb = math.radians(bearing - 90 * sgn + sgn * i * step_deg)
    out.append((c[0] + radius * math.cos(pb), c[1] + radius * math.sin(pb)))
  return out


class Net(object):
  def __init__(self):
    self.edges = []
    self.nid = 1000

  def node(self):
    self.nid += 1
    return self.nid

  def way(self, way_id, pts, nodes=None, highway='primary', oneway=None):
    nodes = nodes or [self.node() for _ in pts]
    for i in range(len(pts) - 1):
      self.edges.append((str(way_id), nodes[i], nodes[i + 1], [ll(pts[i]), ll(pts[i + 1])], highway, oneway, None))
    return nodes

  def build(self, with_nodes=True):
    path = os.path.join(tempfile.mkdtemp(), 'mc.sqlite')
    with contextlib.redirect_stdout(io.StringIO()):
      B.build_from_edges(self.edges, path, 1.85, with_nodes)
    return path


def lookup(net, with_nodes=True):
  return MC.MapCurveLookup(net.build(with_nodes))


def at(p):
  lon, lat = ll(p)
  return lat, lon


def q(lk, p, heading, v=25.0):
  lat, lon = at(p)
  return lk.query_ahead(lat, lon, heading, v)


def slow_bend_after(p0, bearing, straight_m, radius=40.0):
  """straight_m of straight road then a 90 degree right bend (speed floor)."""
  s = line(p0, bearing, straight_m)
  return s + arc(s[-1], bearing, radius, 90)


needs_builder = unittest.skipIf(B is None, 'numpy / mapcurve_build not available')


@needs_builder
class TestBuilder(unittest.TestCase):
  def rows(self, path, sql, *a):
    return sqlite3.connect(path).execute(sql, a).fetchall()

  def test_node_ids_recorded_and_schema_v2(self):
    net = Net(); net.way('A', line((0, 0), 0, 100))
    p = net.build()
    cols = {r[1] for r in self.rows(p, 'PRAGMA table_info(segments)')}
    self.assertTrue({'start_node_id', 'end_node_id'} <= cols)
    self.assertEqual(dict(self.rows(p, 'SELECT key, value FROM meta'))['schema_version'], '2')
    self.assertEqual(dict(self.rows(p, 'SELECT key, value FROM meta'))['node_ids'], 'yes')
    s = self.rows(p, 'SELECT start_node_id, end_node_id FROM segments')[0]
    self.assertEqual(s, (1001, 1006))

  def test_ways_split_only_at_shared_nodes(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))                                     # 11 vertices
    net.way('C', line((100, 0), 90, 60), nodes=[a[5]] + [net.node() for _ in range(3)])   # side road meets A's middle node
    p = net.build()
    self.assertEqual(self.rows(p, "SELECT COUNT(*) FROM segments WHERE way_id='A'")[0][0], 2)   # split at the shared node
    self.assertEqual(self.rows(p, "SELECT COUNT(*) FROM segments WHERE way_id='C'")[0][0], 1)
    ends = self.rows(p, "SELECT end_node_id FROM segments WHERE way_id='A' AND segment_id=0")[0][0]
    starts = self.rows(p, "SELECT start_node_id FROM segments WHERE way_id='A' AND segment_id=1")[0][0]
    self.assertEqual(ends, starts)

  def test_way_not_split_without_a_shared_node(self):
    net = Net(); net.way('A', line((0, 0), 0, 200))
    net.way('B', line((100, 0.2), 0, 100))                                     # 0.2 m from A's vertices, different node IDs
    p = net.build()
    self.assertEqual(self.rows(p, "SELECT COUNT(*) FROM segments WHERE way_id='A'")[0][0], 1)   # proximity never splits/joins

  def test_joins_exist_only_where_node_ids_match(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 100))
    net.way('B', line((100, 0), 0, 100), nodes=[a[-1]] + [net.node() for _ in range(5)])        # shares node
    net.way('D', line((100, 0), 0, 100))                                                         # same place, different nodes
    p = net.build()
    joined = self.rows(p, 'SELECT a.way_id, b.way_id FROM segments a JOIN segments b ON a.end_node_id = b.start_node_id '
                          'WHERE a.way_id != b.way_id')
    self.assertEqual(joined, [('A', 'B')])                                    # D is NOT joined although it coincides with A's end

  def test_join_coordinates_agree(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 100))
    net.way('B', line((100, 0), 45, 100), nodes=[a[-1]] + [net.node() for _ in range(5)])
    p = net.build()
    e = self.rows(p, "SELECT lat, lon FROM pts WHERE way_id='A' ORDER BY seq DESC LIMIT 1")[0]
    s = self.rows(p, "SELECT lat, lon FROM pts WHERE way_id='B' ORDER BY seq LIMIT 1")[0]
    self.assertAlmostEqual(e[0], s[0], places=6); self.assertAlmostEqual(e[1], s[1], places=6)

  def test_gap_in_edges_starts_a_new_chain_and_never_joins(self):
    net = Net()
    n = [net.node() for _ in range(4)]
    pts = line((0, 0), 0, 60)
    net.way('A', pts[:2], nodes=[n[0], n[1]])
    net.way('A', pts[2:], nodes=[n[2], n[3]])                                  # edge.u != previous edge.v: a gap
    p = net.build()
    segs = self.rows(p, "SELECT start_node_id, end_node_id FROM segments WHERE way_id='A' ORDER BY segment_id")
    self.assertEqual(segs, [(n[0], n[1]), (n[2], n[3])])                       # two segments, not one invented through the gap

  def test_oneway_codes(self):
    net = Net()
    net.way('M', line((0, 0), 0, 60), highway='motorway')                      # no tag: implied oneway
    net.way('N', line((0, 50), 0, 60), highway='motorway', oneway='no')        # explicit tag wins
    net.way('P', line((0, 100), 0, 60), highway='primary')                     # primary, no tag: both directions
    net.way('Q', line((0, 150), 0, 60), highway='primary', oneway='yes')
    net.way('R', line((0, 200), 0, 60), highway='primary', oneway='-1')
    p = net.build()
    got = dict(self.rows(p, 'SELECT way_id, oneway FROM segments'))
    self.assertEqual(got, {'M': 1, 'N': 0, 'P': 0, 'Q': 1, 'R': -1})

  def test_legacy_build_has_null_node_ids(self):
    net = Net(); net.way('A', line((0, 0), 0, 100))
    p = net.build(with_nodes=False)
    self.assertEqual(self.rows(p, 'SELECT start_node_id, end_node_id FROM segments')[0], (None, None))
    self.assertEqual(dict(self.rows(p, 'SELECT key, value FROM meta'))['node_ids'], 'no')

  def test_curvature_continuous_across_a_split(self):
    net = Net()
    pts = slow_bend_after((0, 0), 0, 100)
    nodes = [net.node() for _ in pts]
    net.way('A', pts, nodes=nodes)
    net.way('X', line(pts[3], 90, 40), nodes=[nodes[3]] + [net.node(), net.node(), net.node()])   # forces a split mid-way
    p = net.build()
    d = self.rows(p, "SELECT speed FROM pts WHERE way_id='A' ORDER BY segment_id, seq")
    self.assertLess(min(r[0] for r in d), 12.0)                               # the bend is still seen after splitting


@needs_builder
class TestConnectedRoads(unittest.TestCase):
  def setUp(self):
    self.net = Net()
    a = self.net.way('A', line((0, 0), 0, 200))
    b_pts = slow_bend_after((200, 0), 0, 100)
    self.net.way('B', b_pts, nodes=[a[-1]] + [self.net.node() for _ in b_pts[1:]])

  def test_curve_in_the_next_way_is_found_across_the_junction(self):
    lk = lookup(self.net)
    speed, dist, m = q(lk, (100, 0), 0.0)
    self.assertLess(speed, 12.0)                                              # B's bend, 100 m past the junction
    self.assertTrue(190 < dist < 280, dist)                                   # along-road distance through the junction
    self.assertEqual([c['way_id'] for c in m['connections']], ['B'])
    self.assertIn(m['truncated'], ('lookahead', 'dead_end'))                  # ran to the end of this small network

  def test_without_node_ids_the_same_network_stops_at_the_segment_end(self):
    lk = lookup(self.net, with_nodes=False)
    speed, dist, m = q(lk, (100, 0), 0.0)
    self.assertGreater(speed, 100.0)                                          # only A's straight samples: B's bend not reached
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'no_node_id')

  def test_opposite_direction_traverses_the_same_junction(self):
    lk = lookup(self.net)
    b_end = self.net.edges[-1][3][-1]                                         # drive B backwards towards A
    pts = slow_bend_after((200, 0), 0, 100)
    lat, lon = at((pts[len(pts) // 2 + 0][0] + 0, pts[len(pts) // 2][1]))
    # car at B's start heading SOUTH (towards the junction and into A): no bend behind it, A is straight
    speed, dist, m = lk.query_ahead(*at((230, 0)), 180.0, 25.0)
    self.assertEqual(m['direction'], -1)
    self.assertEqual([c['way_id'] for c in m['connections']], ['A'])
    self.assertGreater(speed, 100.0)

  def test_lookahead_limit_respected(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    far = slow_bend_after((200, 0), 0, 600)                                   # bend ~600 m past the junction: beyond 400 m
    net.way('B', far, nodes=[a[-1]] + [net.node() for _ in far[1:]])
    speed, dist, m = q(lookup(net), (100, 0), 0.0)
    self.assertGreater(speed, 100.0)                                          # not seen: out of range, and not invented


@needs_builder
class TestDisconnectedNearbyRoads(unittest.TestCase):
  def test_adjacent_road_with_different_node_ids_is_never_followed(self):
    net = Net()
    net.way('A', line((0, 0), 0, 200))
    b = slow_bend_after((200, 0.4), 0, 100)                                   # starts 0.4 m from A's end: DIFFERENT nodes
    net.way('B', b)
    speed, dist, m = q(lookup(net), (100, 0), 0.0)
    self.assertGreater(speed, 100.0)                                          # B's slow bend is NOT taken
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'dead_end')

  def test_coincident_endpoints_still_not_joined(self):
    net = Net()
    net.way('A', line((0, 0), 0, 200))
    net.way('B', slow_bend_after((200, 0), 0, 100))                           # exactly the same coordinate, different nodes
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertGreater(speed, 100.0); self.assertEqual(m['truncated'], 'dead_end')

  def test_parallel_road_is_not_matched_or_joined(self):
    net = Net()
    net.way('A', line((0, 0), 0, 400))
    net.way('P', slow_bend_after((0, 14), 0, 100))                            # parallel road 14 m away, slow bend
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual(m['way_id'], 'A'); self.assertGreater(speed, 100.0)


@needs_builder
class TestIntersections(unittest.TestCase):
  def test_through_road_beats_side_roads_at_a_crossing(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    x = a[-1]
    a2 = slow_bend_after((200, 0), 0, 150)
    net.way('A2', a2, nodes=[x] + [net.node() for _ in a2[1:]])
    for wid, brg in (('E', 90), ('W', 270)):                                  # side roads with tempting slow bends right at the node
      s = [(200, 0)] + arc((200, 0), brg, 30, 90, right=(brg == 90))[:]
      net.way(wid, s, nodes=[x] + [net.node() for _ in s[1:]])
    speed, dist, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual([c['way_id'] for c in m['connections']], ['A2'])         # straight ahead chosen by heading margin
    self.assertEqual(m['connections'][0]['candidates'], 3)
    self.assertTrue(dist > 240, dist)                                         # A2's far bend, not the side roads' near ones

  def test_y_fork_is_ambiguous_and_stops(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    x = a[-1]
    for wid, brg in (('B1', 10), ('B2', 22)):                                 # two options 12 degrees apart
      s = slow_bend_after((200, 0), brg, 100)
      net.way(wid, s, nodes=[x] + [net.node() for _ in s[1:]])
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'ambiguous')
    self.assertGreater(speed, 100.0)                                          # nothing past the fork is guessed

  def test_best_option_not_straight_enough_is_ambiguous(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200)); x = a[-1]
    for wid, brg in (('B1', 40), ('B2', 95)):                                 # best is 40 deg off straight (> 30)
      s = line((200, 0), brg, 100)
      net.way(wid, s, nodes=[x] + [net.node() for _ in s[1:]])
    self.assertEqual(q(lookup(net), (100, 0), 0.0)[2]['truncated'], 'ambiguous')

  def test_single_l_turn_is_followed(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200)); x = a[-1]
    s = slow_bend_after((200, 0), 80, 60)                                     # the only continuation turns 80 deg
    net.way('B', s, nodes=[x] + [net.node() for _ in s[1:]])
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual([c['way_id'] for c in m['connections']], ['B'])
    self.assertAlmostEqual(m['connections'][0]['deviation'], 80.0, delta=5.0)

  def test_reversal_onto_a_different_road_is_refused(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200)); x = a[-1]
    s = slow_bend_after((200, 0), 175, 60)                                    # leaves the node almost back the way we came
    net.way('B', s, nodes=[x] + [net.node() for _ in s[1:]])
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'sharp_turn')

  def test_same_way_continues_through_a_junction_with_a_side_road(self):
    net = Net()
    pts = slow_bend_after((0, 0), 0, 300)
    nodes = [net.node() for _ in pts]
    net.way('A', pts, nodes=nodes)                                            # one way...
    side = [pts[10]] + arc(pts[10], 90, 30, 90)
    net.way('T', side, nodes=[nodes[10]] + [net.node() for _ in side[1:]])    # ...a slow track meets it mid-way
    speed, dist, m = q(lookup(net), (100, 0), 0.0)
    self.assertTrue(m['connections'][0]['same_way'])
    self.assertEqual(m['connections'][0]['way_id'], 'A')
    self.assertTrue(190 < dist < 280, dist)                                   # A's bend (200 m on), not the track's (~100 m)

  def test_hairpin_apex_shared_with_a_track_follows_the_same_way_not_the_track(self):
    net = Net()
    up = line((0, 0), 0, 200)
    hp = arc(up[-1], 0, 15, 180)                                              # 180 degree hairpin
    down = line(hp[-1], 180, 200)[1:]
    pts = up + hp + down
    nodes = [net.node() for _ in pts]
    net.way('A', pts, nodes=nodes)
    apex = nodes[len(up) + len(hp) // 2 - 1]
    apex_pt = pts[len(up) + len(hp) // 2 - 1]
    track = [apex_pt] + line(apex_pt, 90, 80)[1:]
    net.way('T', track, nodes=[apex] + [net.node() for _ in track[1:]])       # a track leaves at ~90 deg from the apex
    speed, dist, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual({c['way_id'] for c in m['connections']} - {'A'}, set())  # never the track
    self.assertLess(speed, 12.0)                                              # the hairpin itself is seen
    self.assertTrue(dist > 90, dist)


@needs_builder
class TestSwitchbacks(unittest.TestCase):
  def setUp(self):
    self.net = Net()
    up = line((0, 0), 0, 200)
    hp = arc(up[-1], 0, 15, 180)
    down = line(hp[-1], 180, 200)[1:]
    self.net.way('A', up + hp + down)                                         # one way folded back on itself, legs 30 m apart

  def test_hairpin_distance_is_along_the_road_not_straight_line(self):
    speed, dist, m = q(lookup(self.net), (100, 0), 0.0)
    self.assertLess(speed, 12.0)
    self.assertTrue(dist > 90, dist)                                          # ~100 m of road, though the down-leg is 30 m away
    self.assertEqual(m['way_id'], 'A'); self.assertEqual(m['direction'], 1)

  def test_after_the_hairpin_the_apex_behind_is_not_reported(self):
    speed, dist, m = q(lookup(self.net), (100, 30), 180.0)                    # on the down-leg heading south
    self.assertEqual(m['way_id'], 'A')
    self.assertGreater(speed, 100.0)                                          # no slow point ahead; the hairpin is behind

  def test_two_ways_on_a_switchback_lock_to_the_one_being_driven(self):
    net = Net()
    net.way('UP', line((0, 0), 0, 200))
    net.way('DN', line((200, 12), 180, 200))                                  # the other leg, 12 m away, slow curvature irrelevant
    net.way('SLOW', slow_bend_after((0, 12), 0, 50))                          # decoy 12 m away heading the same way
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual(m['way_id'], 'UP')
    self.assertGreater(speed, 100.0)


@needs_builder
class TestOneWay(unittest.TestCase):
  def test_continuation_with_the_oneway_is_followed(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200), oneway='yes')
    b = slow_bend_after((200, 0), 0, 60)
    net.way('B', b, nodes=[a[-1]] + [net.node() for _ in b[1:]], oneway='yes')
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual([c['way_id'] for c in m['connections']], ['B'])

  def test_continuation_against_the_oneway_is_refused(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    b = slow_bend_after((200, 0), 0, 60)
    b_rev = list(reversed(b))                                                 # stored so that it ENDS at A's end node
    net.way('B', b_rev, nodes=[net.node() for _ in b_rev[:-1]] + [a[-1]], oneway='yes')   # one-way pointing INTO the node
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'oneway')
    self.assertGreater(speed, 100.0)

  def test_reverse_only_way_is_entered_from_its_end(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    b = slow_bend_after((200, 0), 0, 60)
    b_rev = list(reversed(b))
    net.way('B', b_rev, nodes=[net.node() for _ in b_rev[:-1]] + [a[-1]], oneway='-1')    # -1: legal only against stored order
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual([c['way_id'] for c in m['connections']], ['B'])
    self.assertEqual(m['connections'][0]['same_way'], False)

  def test_driving_against_an_implied_motorway_oneway_is_not_matched(self):
    net = Net(); net.way('M', line((0, 0), 0, 400), highway='motorway')
    self.assertIsNone(q(lookup(net), (200, 0), 180.0))                       # wrong way on a motorway: no match
    self.assertIsNotNone(q(lookup(net), (200, 0), 0.0))

  def test_oneway_blocks_only_the_illegal_option(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200)); x = a[-1]
    good = slow_bend_after((200, 0), 0, 150)
    net.way('G', good, nodes=[x] + [net.node() for _ in good[1:]])
    bad = list(reversed(slow_bend_after((200, 0), 20, 60)))
    net.way('X', bad, nodes=[net.node() for _ in bad[:-1]] + [x], oneway='yes')    # illegal to enter from the node
    speed, _d, m = q(lookup(net), (100, 0), 0.0)
    self.assertEqual([c['way_id'] for c in m['connections']], ['G'])          # the one-way road is not even a candidate


@needs_builder
class TestRobustness(unittest.TestCase):
  def test_loop_of_ways_terminates_and_never_revisits(self):
    net = Net()
    n = [net.node() for _ in range(4)]
    tri = [(0, 0), (0, 150), (150, 150), (0, 0)]
    # three ways around a loop sharing nodes n0,n1,n2 and back to n0
    net.way('W1', line((0, 0), 90, 160, 20), nodes=[n[0]] + [net.node() for _ in range(7)] + [n[1]])
    net.way('W2', line((0, 160), 0, 160, 20), nodes=[n[1]] + [net.node() for _ in range(7)] + [n[2]])
    net.way('W3', line((160, 160), 270, 160, 20), nodes=[n[2]] + [net.node() for _ in range(7)] + [n[3]])
    net.way('W4', line((160, 0), 180, 160, 20), nodes=[n[3]] + [net.node() for _ in range(7)] + [n[0]])
    lat, lon = at((0, 40))
    r = lookup(net).query_ahead(lat, lon, 90.0, 25.0)
    self.assertIsNotNone(r)
    keys = [(c['way_id'], c['segment_id']) for c in r[2]['connections']]
    self.assertEqual(len(keys), len(set(keys)))                               # no segment visited twice
    self.assertLessEqual(len(keys), MC.MapCurveLookup.MAX_HOPS)

  def test_corrupt_segments_table_does_not_crash_the_walk(self):
    net = Net()
    a = net.way('A', line((0, 0), 0, 200))
    p = net.build()
    db = sqlite3.connect(p); db.execute('UPDATE segments SET end_node_id = NULL'); db.commit(); db.close()
    r = MC.MapCurveLookup(p).query_ahead(*at((100, 0)), 0.0, 25.0)
    self.assertIsNotNone(r); self.assertEqual(r[2]['truncated'], 'no_node_id')

  def test_invalid_heading_matches_nothing(self):
    net = Net(); net.way('A', line((0, 0), 0, 200))
    lk = lookup(net)
    for bad in (float('nan'), float('inf'), float('-inf'), None, 'north', True):
      self.assertIsNone(lk.query_ahead(*at((100, 0)), bad, 25.0), bad)

  def test_invalid_position_matches_nothing(self):
    net = Net(); net.way('A', line((0, 0), 0, 200))
    lk = lookup(net)
    lat, lon = at((100, 0))
    for bad in (float('nan'), float('inf'), None, 95.0, 0.0 + 1e9):
      self.assertIsNone(lk.query_ahead(bad, lon, 0.0, 25.0), bad)
      self.assertIsNone(lk.query_ahead(lat, bad, 0.0, 25.0), bad)

  def test_invalid_speed_is_treated_as_zero_not_a_crash(self):
    net = Net(); net.way('A', line((0, 0), 0, 200))
    lk = lookup(net)
    ref = lk.query_ahead(*at((100, 0)), 0.0, 0.0)
    for bad in (float('nan'), float('inf'), -5.0, None):
      self.assertEqual(lk.query_ahead(*at((100, 0)), 0.0, bad)[:2], ref[:2])

  def test_low_confidence_match_far_from_the_road_is_rejected(self):          # GPS 60 m off any mapped road
    net = Net(); net.way('A', line((0, 0), 0, 200))
    self.assertIsNone(q(lookup(net), (100, 60), 0.0))

  def test_heading_far_from_the_road_direction_is_rejected(self):
    net = Net(); net.way('A', line((0, 0), 0, 200))
    self.assertIsNone(q(lookup(net), (100, 0), 90.0))                         # crossing the road, not driving along it

  def test_hop_limit(self):
    net = Net()
    prev = None
    for i in range(MC.MapCurveLookup.MAX_HOPS + 4):                           # a long chain of short ways
      pts = line((i * 60.0, 0), 0, 60, 20)
      nodes = ([prev] if prev else [net.node()]) + [net.node() for _ in pts[1:]]
      net.way('W%d' % i, pts, nodes=nodes)
      prev = nodes[-1]
    lk = lookup(net)
    lk.MAX_HOPS = 3
    m = lk.query_ahead(*at((10, 0)), 0.0, 25.0)[2]
    self.assertEqual(len(m['connections']), 3); self.assertEqual(m['truncated'], 'hop_limit')


class TestOldSchemaCompatibility(unittest.TestCase):
  """A v1 database (no node IDs, as built before this milestone) keeps working and never joins roads by proximity."""
  def make_v1(self):
    path = os.path.join(tempfile.mkdtemp(), 'v1.sqlite')
    db = sqlite3.connect(path)
    db.executescript('''
      CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER, way_id TEXT, segment_id INTEGER,
        seq INTEGER, direction INTEGER, highway TEXT, curvature REAL);
      CREATE TABLE segments (way_id TEXT, segment_id INTEGER, highway TEXT, name TEXT, oneway INTEGER, point_count INTEGER,
        PRIMARY KEY (way_id, segment_id));''')
    for way, p0, speed in (('A', (0, 0), 30.0), ('B', (200, 0), 9.0)):       # B starts at EXACTLY A's end
      pts = [(p0[0] + 20 * i, p0[1]) for i in range(11)]
      db.execute('INSERT INTO segments VALUES (?,?,?,?,?,?)', (way, 0, 'primary', None, 0, len(pts)))
      for seq, p in enumerate(pts):
        lon, lat = ll(p)
        db.execute('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?,?,?)', (lat, lon, speed, int(math.floor(lat / MC.CELL_DEG)),
                   int(math.floor(lon / MC.CELL_DEG)), way, 0, seq, 0, 'primary', 0.0))
    db.commit(); db.close()
    return path

  def test_old_schema_loads_and_is_detected(self):
    lk = MC.MapCurveLookup(self.make_v1())
    self.assertFalse(lk.has_nodes)

  def test_old_schema_stops_at_the_segment_end_even_with_coincident_endpoints(self):
    lk = MC.MapCurveLookup(self.make_v1())
    speed, dist, m = lk.query_ahead(*at((100, 0)), 0.0, 25.0)
    self.assertEqual(speed, 30.0)                                             # B's 9 m/s is NOT reached despite touching A
    self.assertEqual(m['connections'], []); self.assertEqual(m['truncated'], 'no_node_ids'); self.assertFalse(m['node_ids'])

  def test_pts_only_database_still_works(self):                               # the oldest layout (existing mapcurved tests)
    path = os.path.join(tempfile.mkdtemp(), 'old.sqlite')
    db = sqlite3.connect(path)
    db.execute('CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER, way_id TEXT, '
               'segment_id INTEGER, seq INTEGER, direction INTEGER)')
    for seq in range(5):
      lon, lat = ll((20 * seq, 0))
      db.execute('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?)', (lat, lon, 20.0, int(math.floor(lat / MC.CELL_DEG)),
                 int(math.floor(lon / MC.CELL_DEG)), 'A', 0, seq, 0))
    db.commit(); db.close()
    lk = MC.MapCurveLookup(path)
    self.assertFalse(lk.has_nodes)
    self.assertIsNotNone(lk.query_ahead(*at((10, 0)), 0.0, 25.0))


class TestMissingAndCorruptDatabase(unittest.TestCase):
  def test_missing_database_raises_cleanly(self):
    with self.assertRaises(Exception):
      MC.MapCurveLookup(os.path.join(tempfile.mkdtemp(), 'nope', 'x.sqlite'))

  def test_garbage_file_raises_cleanly(self):
    p = os.path.join(tempfile.mkdtemp(), 'bad.sqlite')
    open(p, 'wb').write(b'this is not a sqlite database' * 50)
    with self.assertRaises(Exception):
      MC.MapCurveLookup(p)


if __name__ == '__main__':
  unittest.main()
