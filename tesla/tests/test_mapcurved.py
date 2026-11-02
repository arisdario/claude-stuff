#!/usr/bin/env python3
"""Tests for mapcurved.py's road-aware curve lookup (no openpilot needed):
    python3 -m unittest tesla/tests/test_mapcurved.py

The lookup locks to the road you are on (way_id/segment_id), picks the travel
direction along the stored point order (seq) from your heading, walks forward,
and returns the point that binds NOW (lowest approach_speed). These tests build
tiny road-aware DBs and check that behaviour - including the cases the old
nearest-point lookup got wrong (grabbing an adjacent hairpin leg / a crossing
street, or ignoring a near bend for a tighter far one).
"""
import math
import os
import sqlite3
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import mapcurved as MC  # noqa: E402  (imports cereal only inside main(), so this is clean)

LAT0, LON0 = 45.0, 9.0


def offset(north_m, east_m):
  m_lat, m_lon = MC.meters_per_deg(LAT0)
  return LAT0 + north_m / m_lat, LON0 + east_m / m_lon


def make_db(segments):
  """segments: list of (way_id, segment_id, direction, [(north_m, east_m, speed), ...] in seq order)."""
  path = os.path.join(tempfile.mkdtemp(), 'mc.sqlite')
  db = sqlite3.connect(path)
  db.execute('CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER, '
             'way_id TEXT, segment_id INTEGER, seq INTEGER, direction INTEGER)')
  for way_id, seg_id, direction, pts in segments:
    for seq, (north, east, speed) in enumerate(pts):
      lat, lon = offset(north, east)
      db.execute('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?)',
                 (lat, lon, speed, int(math.floor(lat / MC.CELL_DEG)), int(math.floor(lon / MC.CELL_DEG)),
                  str(way_id), seg_id, seq, direction))
  db.commit()
  db.close()
  return path


def straight_north(speeds_by_m, direction=0):
  """A N-S road sampled every 20 m from 0..(20*(n-1)) m north; speeds_by_m overrides {index: speed}, default 30."""
  n = max(speeds_by_m) + 1
  return (1, 0, direction, [(20 * i, 0, speeds_by_m.get(i, 30.0)) for i in range(n)])


class TestRoadAwareQuery(unittest.TestCase):
  def test_picks_the_curve_that_matters_now(self):
    # heading north at 25 m/s on the road: a 14 m/s bend ~60 m ahead binds now; an 8 m/s corner ~380 m ahead doesn't yet
    lk = MC.MapCurveLookup(make_db([straight_north({3: 14.0, 19: 8.0})]))  # 3*20=60 m, 19*20=380 m
    speed, dist, m = lk.query_ahead(LAT0, LON0, 0.0, 25.0)
    self.assertAlmostEqual(speed, 14.0)
    self.assertLess(abs(dist - 60), 1.0)
    self.assertEqual(m['direction'], 1)

  def test_far_tight_corner_wins_when_tight_enough(self):
    # 6 m/s at 110 m allows ~12.5 m/s now, less than the 14.7 the 14 m/s bend at 60 m allows -> the far tight one binds
    lk = MC.MapCurveLookup(make_db([(1, 0, 0, [(20 * i, 0, {3: 14.0, 5: 6.0, 6: 6.0}.get(i, 30.0)) for i in range(10)])]))
    speed, dist, _m = lk.query_ahead(LAT0, LON0, 0.0, 25.0)
    self.assertAlmostEqual(speed, 6.0)

  def test_follows_travel_direction(self):
    # same road; heading SOUTH picks the curve to the south (low seq), not the one to the north
    lk = MC.MapCurveLookup(make_db([straight_north({2: 10.0, 18: 9.0})]))  # 40 m north, 360 m north
    # stand near the 200 m mark so both curves are reachable; heading south -> walk toward seq 0 (south)
    lat, lon = offset(200, 0)
    speed, _dist, m = lk.query_ahead(lat, lon, 180.0, 25.0)
    self.assertEqual(m['direction'], -1)
    self.assertAlmostEqual(speed, 10.0)   # the southern (seq 2) curve, reached walking south

  def test_crossing_street_rejected_by_heading(self):
    # a slow E-W street crosses within match radius; heading north must NOT read its speed (tangent is E-W, >45 deg off)
    ew = (2, 0, 0, [(0, -40, 5.0), (0, -20, 5.0), (0, 0, 5.0), (0, 20, 5.0), (0, 40, 5.0)])
    ns = straight_north({2: 12.0})     # our road, a 12 m/s bend ahead
    lk = MC.MapCurveLookup(make_db([ew, ns]))
    speed, _dist, m = lk.query_ahead(LAT0, LON0, 0.0, 20.0)
    self.assertEqual(m['way_id'], '1')   # locked to the N-S road, not the crossing street
    self.assertAlmostEqual(speed, 12.0)

  def test_curve_behind_is_not_reported(self):
    # a 5 m/s curve far back along the road (seq 1) is not reached walking forward (north)
    lk = MC.MapCurveLookup(make_db([(1, 0, 0, [(0, 0, 30.0), (20, 0, 5.0)] + [(20 * i, 0, 30.0) for i in range(2, 12)])]))
    lat, lon = offset(200, 0)           # sit at 200 m, heading north; the 5 m/s point is at 20 m, behind
    r = lk.query_ahead(lat, lon, 0.0, 25.0)
    self.assertIsNotNone(r)
    speed, _dist, _m = r
    self.assertAlmostEqual(speed, 30.0)  # only the fast points ahead, never the slow one behind

  def test_oneway_wrong_direction_rejected(self):
    # a forward-only (direction=+1) road: approaching against it (heading south) finds no legal match
    lk = MC.MapCurveLookup(make_db([straight_north({3: 10.0}, direction=1)]))
    self.assertIsNone(lk.query_ahead(LAT0, LON0, 180.0, 10.0))     # against the oneway -> no match
    self.assertIsNotNone(lk.query_ahead(LAT0, LON0, 0.0, 10.0))    # with it -> matches

  def test_far_from_any_road_is_none(self):
    lk = MC.MapCurveLookup(make_db([straight_north({3: 10.0})]))
    lat, lon = offset(0, 200)           # 200 m east of the road, well beyond MATCH_RADIUS_M
    self.assertIsNone(lk.query_ahead(lat, lon, 0.0, 20.0))

  def test_old_schema_db_is_refused(self):
    path = os.path.join(tempfile.mkdtemp(), 'old.sqlite')
    db = sqlite3.connect(path)
    db.execute('CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER)')
    db.commit()
    db.close()
    with self.assertRaises(RuntimeError):
      MC.MapCurveLookup(path)


if __name__ == '__main__':
  unittest.main()
