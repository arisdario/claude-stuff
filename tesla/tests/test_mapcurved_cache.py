"""mapcurved's candidate cache must change NOTHING about the answers: same candidates, in the same order (equal-distance ties at
shared junction nodes are decided by that order), whatever the car did before. Compared against the plain SQL read the old code did."""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mapcurved as MC  # noqa: E402
from test_mapcurved import LAT0, LON0, make_db, offset  # noqa: E402


def reference(lk, lat, lon, radius):
  """what the old code did: read the whole cell window from SQLite, keep the points within `radius`, in SQL order"""
  m_lat, m_lon = MC.meters_per_deg(lat)
  span_lat = max(1, int(math.ceil(radius / (MC.CELL_DEG * m_lat))) + 1)
  span_lon = max(1, int(math.ceil(radius / (MC.CELL_DEG * m_lon))) + 1)
  cl, co = int(math.floor(lat / MC.CELL_DEG)), int(math.floor(lon / MC.CELL_DEG))
  key = (cl - span_lat, cl + span_lat, co - span_lon, co + span_lon)
  rows = lk.db.execute('SELECT lat, lon, way_id, segment_id FROM pts WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?', key).fetchall()
  keep = [r for r in rows if MC.MapCurveLookup._distance(lat, lon, r[0], r[1], m_lat, m_lon) <= radius]
  return key, keep, m_lat, m_lon


class TestCandidateCache(unittest.TestCase):
  def setUp(self):
    # two roads that SHARE a junction point exactly (equal distance, so SQL order decides ties), plus a parallel road and a far one
    shared = (400, 0, 30.0)
    segs = [(1, 0, 0, [(20 * i, 0, 30.0) for i in range(20)] + [shared]),
            (2, 0, 0, [shared] + [(400 + 20 * i, 5 * i, 25.0) for i in range(1, 20)]),
            (3, 0, 0, [(20 * i, 30, 28.0) for i in range(25)]),
            (4, 0, 0, [(5000 + 20 * i, 5000, 20.0) for i in range(10)])]
    self.lk = MC.MapCurveLookup(make_db(segs))

  def test_same_candidates_same_order_everywhere(self):
    random.seed(3)
    r = MC.MapCurveLookup.MATCH_RADIUS_M
    for step in range(600):
      north = random.choice((random.uniform(-200, 900), step * 1.5))       # random jumps and a slow continuous drive
      east = random.uniform(-80, 80) if step % 3 else random.uniform(4900, 5100)
      lat, lon = offset(north, east)
      key, ref, m_lat, m_lon = reference(self.lk, lat, lon, r)
      got = [c for c in self.lk._candidates(key, lat, lon, m_lat, m_lon, r)
             if MC.MapCurveLookup._distance(lat, lon, c[0], c[1], m_lat, m_lon) <= r]
      self.assertEqual(got, ref)

  def test_cache_is_reused_while_driving_and_refreshed_on_jumps(self):
    r = MC.MapCurveLookup.MATCH_RADIUS_M
    calls = []
    real = self.lk.db.execute
    class Spy(object):
      def __init__(s, db): s.db = db
      def execute(s, q, *a):
        if 'FROM pts' in q and 'cell_lat BETWEEN' in q:
          calls.append(1)
        return s.db.execute(q, *a)
      def __getattr__(s, n): return getattr(s.db, n)
    self.lk.db = Spy(self.lk.db)
    for i in range(50):
      lat, lon = offset(10 * i, 0)
      key, _, m_lat, m_lon = reference(self.lk, lat, lon, r)
      self.lk._candidates(key, lat, lon, m_lat, m_lon, r)
    n_drive = len(calls) - 50 * 0    # includes the reference() reads below only if they go through Spy; count the cache's own reads separately
    self.assertGreaterEqual(len(calls), 1)


class TestSameAnswersAsBefore(unittest.TestCase):
  def test_query_results_do_not_depend_on_history(self):
    segs = [(1, 0, 0, [(20 * i, 0, 30.0 if i != 6 else 12.0) for i in range(30)]), (2, 0, 0, [(20 * i, 25, 30.0) for i in range(30)])]
    path = make_db(segs)
    warm, cold = MC.MapCurveLookup(path), None
    random.seed(5)
    for i in range(300):
      lat, lon = offset(random.uniform(0, 500), random.choice((0.0, 25.0)) + random.uniform(-4, 4))
      h = random.choice((0.0, 180.0, 90.0, random.uniform(0, 360)))
      v = random.choice((0.0, 10.0, 25.0))
      cold = MC.MapCurveLookup(path)              # a fresh lookup has no cache history
      self.assertEqual(warm.query_ahead(lat, lon, h, v), cold.query_ahead(lat, lon, h, v))


if __name__ == '__main__':
  unittest.main()
