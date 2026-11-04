#!/usr/bin/env python3
"""Check the road-aware curve lookup on THIS machine without a car or GPS: walk along a real mapped road like a car would and show what the
lookup returns, with timings. Read-only (opens the map read-only; publishes nothing; touches no car code).

  python3 /data/claude-stuff/tesla/tools/mapcurve_bench.py [--db /data/mapcurve.sqlite] [--near LAT LON] [--steps 300] [--speed 20] [--show 12]

--near picks the mapped road closest to that place (default: a primary road near 44.43782 26.16808, in Bucharest).
Prints: which road it walked, how many steps matched, ms per lookup (first step and the rest), and a sample of results.
"""
import argparse
import math
import os
import sqlite3
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import mapcurved as M  # noqa: E402


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--db', default=M.DB_PATH)
  ap.add_argument('--near', nargs=2, type=float, default=[44.43782, 26.16808], metavar=('LAT', 'LON'))
  ap.add_argument('--steps', type=int, default=300)
  ap.add_argument('--speed', type=float, default=20.0, help='vehicle speed in m/s used for the lookup (default 20 = 72 km/h)')
  ap.add_argument('--show', type=int, default=12)
  a = ap.parse_args()
  db = sqlite3.connect('file:%s?mode=ro' % a.db, uri=True)
  lat0, lon0 = a.near
  m_lat, m_lon = M.meters_per_deg(lat0)
  cl, co = int(math.floor(lat0 / M.CELL_DEG)), int(math.floor(lon0 / M.CELL_DEG))
  rows = db.execute('SELECT lat, lon, way_id, segment_id FROM pts WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?', (cl - 1, cl + 1, co - 1, co + 1)).fetchall()
  if not rows:
    sys.exit('no mapped road points near %.5f %.5f in %s' % (lat0, lon0, a.db))
  d, r = min(((math.hypot((r[0] - lat0) * m_lat, (r[1] - lon0) * m_lon), r) for r in rows), key=lambda x: x[0])
  print('nearest mapped road point: %.0f m from %.5f %.5f (way %s segment %s)' % (d, lat0, lon0, r[2], r[3]))
  walk = db.execute('SELECT lat, lon, seq FROM pts WHERE way_id=? AND segment_id=? ORDER BY seq', (r[2], r[3])).fetchall()
  steps = []
  for i in range(len(walk) - 1):
    h = M.bearing_deg_to(walk[i][0], walk[i][1], walk[i + 1][0], walk[i + 1][1])
    steps.append((walk[i][0], walk[i][1], h))
  steps = steps[:a.steps]
  if not steps:
    sys.exit('that segment has too few points')
  lk = M.MapCurveLookup(a.db)
  res, times = [], []
  for la, lo, h in steps:
    t = time.perf_counter()
    res.append(lk.query_ahead(la, lo, h, a.speed))
    times.append(time.perf_counter() - t)
  ok = sum(1 for x in res if x is not None)
  warm = sorted(times[1:]) if len(times) > 1 else times
  print('walked %d steps along that road at %.0f m/s: %d matched a curve, %d returned nothing' % (len(steps), a.speed, ok, len(steps) - ok))
  print('lookup time: first %.1f ms (includes reading the map cells), then median %.2f ms, 95th percentile %.2f ms, worst %.1f ms' % (
    1000 * times[0], 1000 * warm[len(warm) // 2], 1000 * warm[int(len(warm) * 0.95) - 1], 1000 * warm[-1]))
  print('at 4 lookups per second that is about %.2f%% of one core' % (100 * 4 * warm[len(warm) // 2]))
  print('\nsample of results (binding curve speed, distance ahead):')
  step = max(1, len(res) // a.show)
  for i in range(0, len(res), step):
    x = res[i]
    print('  at %.5f %.5f hdg %5.1f -> %s' % (steps[i][0], steps[i][1], steps[i][2],
          'nothing' if x is None else '%.1f m/s (%.0f km/h) in %.0f m, road %s seg %s' % (x[0], x[0] * 3.6, x[1], x[2]['way_id'], x[2]['segment_id'])))


if __name__ == '__main__':
  main()
