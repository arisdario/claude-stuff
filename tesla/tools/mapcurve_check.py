#!/usr/bin/env python3
"""
mapcurve_check.py - offline validator for mapcurve.sqlite. Runs the SAME lookahead query mapcurved.py does, so you can
check what curve speed the map would command at a given place WITHOUT deploying to the EON. Pure stdlib + the DB.

Two modes:

  # 1. One point: "approaching this spot heading this way, what does the map say?"
  python3 mapcurve_check.py mapcurve.sqlite --at LAT LON HEADING_DEG [--v V_KMH]

  # 2. A recorded GPS trace: replay the driven route and flag every point where you were going faster than the map's
  #    curve speed at the curve ahead (i.e. where it SHOULD have slowed you). Trace = CSV with columns lat,lon,
  #    heading,v_kmh  (or a maplog .jsonl - it pulls MCU_latitude/MCU_longitude/UI_gpsVehicleHeading and v).
  python3 mapcurve_check.py mapcurve.sqlite --trace drive.csv

So: build the DB (mapcurve_build.py), then point --at at the S-curve approach (get its lat/lon/heading off any map app)
and see whether the new DB would have slowed you. If it returns a sensible speed, the DB is good and the on-car failure
is the position/runtime; if it returns nothing or a too-high speed, the DB needs work.
"""
import argparse
import csv
import json
import math
import sqlite3
import sys

CELL_DEG = 0.01
LOOKAHEAD_M = 400.0
HEADING_CONE_DEG = 50.0
A_Y = 1.85
A_CURVE_DECEL = 1.0       # must match pcc_curves.A_CURVE_DECEL
REACTION_S = 2.0          # must match pcc_curves.CURVE_REACTION_S
MIN_SPEED_MS = 1.0


def meters_per_deg(lat):
  r = math.radians(lat)
  return 111320.0, 111320.0 * math.cos(r)


def bearing(lat0, lon0, lat1, lon1):
  mlat, mlon = meters_per_deg(lat0)
  return math.degrees(math.atan2((lon1 - lon0) * mlon, (lat1 - lat0) * mlat)) % 360.0


def ang_diff(a, b):
  d = (a - b) % 360.0
  return d - 360.0 if d > 180 else d


def approach_speed(v_curve, dist, v_ego):
  d = max(0., dist - max(v_ego, 0.) * REACTION_S)
  return math.sqrt(max(v_curve, 0.) ** 2 + 2. * A_CURVE_DECEL * d)


class Lookup(object):
  def __init__(self, db_path):
    self.db = sqlite3.connect(db_path)
    self.db.execute('PRAGMA query_only=ON')

  def query(self, lat, lon, heading, v_ego=0.0):
    """return (binding_point_speed_ms, dist_m, approach_speed_now_ms) or None - the point whose approach speed is the
    lowest within the lookahead cone (same ranking as the repo mapcurved.py)."""
    mlat, mlon = meters_per_deg(lat)
    span_lat = max(1, int(math.ceil(LOOKAHEAD_M / (CELL_DEG * mlat))) + 1)
    span_lon = max(1, int(math.ceil(LOOKAHEAD_M / (CELL_DEG * mlon))) + 1)
    cl = int(math.floor(lat / CELL_DEG)); co = int(math.floor(lon / CELL_DEG))
    rows = self.db.execute('SELECT lat, lon, speed FROM pts WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?',
                           (cl - span_lat, cl + span_lat, co - span_lon, co + span_lon)).fetchall()
    best = None
    for plat, plon, speed in rows:
      dist = math.hypot((plat - lat) * mlat, (plon - lon) * mlon)
      if dist < 1.0 or dist > LOOKAHEAD_M or speed is None or speed < MIN_SPEED_MS:
        continue
      if abs(ang_diff(bearing(lat, lon, plat, plon), heading)) > HEADING_CONE_DEG:
        continue
      v_now = approach_speed(speed, dist, v_ego)
      if best is None or v_now < best[2]:
        best = (speed, dist, v_now)
    return best


def at(db, lat, lon, hdg, v_kmh):
  v = v_kmh / 3.6
  r = Lookup(db).query(lat, lon, hdg, v)
  if r is None:
    print('no curve within %.0f m ahead (cone %.0f deg). Either straight road, or the DB has no points here.' % (LOOKAHEAD_M, HEADING_CONE_DEG))
    return
  sp, dist, vnow = r
  print('curve ahead: safe %.0f km/h at %.0f m.  At %.0f km/h the map would target %.0f km/h NOW (%s).' % (
      sp * 3.6, dist, v_kmh, vnow * 3.6, 'SLOW DOWN' if vnow < v - 0.5 else 'ok, already slow enough'))


def load_trace(path):
  if path.endswith('.jsonl') or path.endswith('.jsonl.gz'):
    import gzip
    op = gzip.open(path, 'rt') if path.endswith('.gz') else open(path)
    for ln in op:
      try: r = json.loads(ln)
      except Exception: continue
      m = r.get('msgs', {})
      loc = m.get('MCU_locationStatus', {}); gps = m.get('UI_gpsVehicleSpeed', {})
      def num(x):
        try: return float(str(x).split()[0])
        except Exception: return None
      lat = num(loc.get('MCU_latitude')); lon = num(loc.get('MCU_longitude')); hdg = num(gps.get('UI_gpsVehicleHeading'))
      v = (r.get('v') or 0) * 3.6
      if lat and lon and hdg is not None:
        yield lat, lon, hdg, v
  else:
    for row in csv.DictReader(open(path)):
      try: yield float(row['lat']), float(row['lon']), float(row['heading']), float(row['v_kmh'])
      except Exception: continue


def trace(db, path):
  lk = Lookup(db); n = 0; should = 0
  print('%-9s %-9s %4s %6s %8s %6s' % ('lat', 'lon', 'hdg', 'v_kmh', 'map_kmh', 'dist'))
  for lat, lon, hdg, v in load_trace(path):
    n += 1
    r = lk.query(lat, lon, hdg, v / 3.6)
    if r is None:
      continue
    sp, dist, vnow = r
    flag = ''
    if vnow < v / 3.6 - 1.0:
      flag = '<- should slow'; should += 1
    print('%-9.5f %-9.5f %4.0f %6.0f %8.0f %6.0f  %s' % (lat, lon, hdg, v, vnow * 3.6, dist, flag))
  print('\n%d trace points, %d where the map would have told you to slow' % (n, should))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('db')
  ap.add_argument('--at', nargs=3, type=float, metavar=('LAT', 'LON', 'HEADING'))
  ap.add_argument('--v', type=float, default=100.0, help='current speed km/h for the --at check')
  ap.add_argument('--trace')
  a = ap.parse_args()
  if a.at:
    at(a.db, a.at[0], a.at[1], a.at[2], a.v)
  elif a.trace:
    trace(a.db, a.trace)
  else:
    ap.error('give --at LAT LON HEADING  or  --trace FILE')


if __name__ == '__main__':
  main()
