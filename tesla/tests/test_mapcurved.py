#!/usr/bin/env python3
"""Tests for mapcurved.py's curve choice (no openpilot needed):  python3 -m unittest tesla/tests/test_mapcurved.py"""
import math
import os
import sqlite3
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
for name in ('cereal', 'cereal.messaging'):          # mapcurved imports cereal.messaging at the top
  sys.modules.setdefault(name, types.ModuleType(name))
import mapcurved as MC  # noqa: E402

LAT0, LON0 = 45.0, 9.0


def offset(north_m, east_m):
  m_lat, m_lon = MC.meters_per_deg(LAT0)
  return LAT0 + north_m / m_lat, LON0 + east_m / m_lon


def make_db(points):
  path = os.path.join(tempfile.mkdtemp(), 'mc.sqlite')
  db = sqlite3.connect(path)
  db.execute('CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER)')
  for north, east, speed in points:
    lat, lon = offset(north, east)
    db.execute('INSERT INTO pts VALUES (?,?,?,?,?)', (lat, lon, speed, int(math.floor(lat / MC.CELL_DEG)), int(math.floor(lon / MC.CELL_DEG))))
  db.commit(); db.close()
  return path


class TestQuery(unittest.TestCase):
  def test_picks_the_curve_that_matters_now(self):
    # heading north at 25 m/s: a 14 m/s bend 60 m ahead binds now; an 8 m/s corner 380 m ahead doesn't yet
    lk = MC.MapCurveLookup(make_db([(60, 0, 14.0), (380, 5, 8.0)]))
    speed, dist = lk.query_ahead(LAT0, LON0, 0.0, 25.0)
    self.assertAlmostEqual(speed, 14.0); self.assertLess(abs(dist - 60), 1)

  def test_far_tight_corner_wins_when_slow_or_close_enough(self):
    lk = MC.MapCurveLookup(make_db([(60, 0, 14.0), (110, 0, 6.0)]))   # 6 m/s at 110 m allows 12.5 m/s now, less than 14.7
    speed, dist = lk.query_ahead(LAT0, LON0, 0.0, 25.0)
    self.assertAlmostEqual(speed, 6.0)

  def test_side_street_outside_corridor_ignored(self):
    # 80 m to the east, 100 m ahead: inside the old 50 deg cone, outside the corridor (10 m + 30% of 100 m)
    lk = MC.MapCurveLookup(make_db([(100, 80, 5.0)]))
    self.assertIsNone(lk.query_ahead(LAT0, LON0, 0.0, 25.0))

  def test_behind_ignored(self):
    lk = MC.MapCurveLookup(make_db([(-50, 0, 5.0)]))
    self.assertIsNone(lk.query_ahead(LAT0, LON0, 0.0, 25.0))

  def test_heading_east(self):
    lk = MC.MapCurveLookup(make_db([(0, 80, 9.0), (80, 0, 4.0)]))   # ahead when heading east / to the north side
    speed, dist = lk.query_ahead(LAT0, LON0, 90.0, 20.0)
    self.assertAlmostEqual(speed, 9.0)


if __name__ == '__main__':
  unittest.main()
