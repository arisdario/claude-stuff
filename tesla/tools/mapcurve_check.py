#!/usr/bin/env python3
"""
mapcurve_check.py - offline validator for a road-aware mapcurve.sqlite. Runs the EXACT same road-aware lookup
mapcurved.py uses on the car (imports MapCurveLookup from it), so you can check what curve speed the map would
command at a place WITHOUT deploying to the EON. Pure stdlib + the DB (mapcurved's class imports cereal only inside
its service loop, so importing the lookup needs no openpilot).

Two modes:

  # 1. One point: "on this road heading this way, what does the map say ahead?"
  python3 mapcurve_check.py mapcurve.sqlite --at LAT LON HEADING_DEG [--v V_KMH]

  # 2. A recorded GPS trace: replay the driven route and flag every point where you were going faster than the map's
  #    curve speed at the curve ahead. Trace = CSV with columns lat,lon,heading,v_kmh (or a maplog .jsonl - it pulls
  #    MCU_latitude/MCU_longitude/UI_gpsVehicleHeading and v).
  python3 mapcurve_check.py mapcurve.sqlite --trace drive.csv

Heading is compass degrees: 0=N, 90=E, 180=S, 270=W. NO MATCH means no road sample within MATCH_RADIUS_M whose
tangent is within MATCH_HEADING_DEG of your heading - i.e. you are not on a mapped (motorway/trunk/primary) road in
that direction; that is correct, not a failure, when you are off such a road or pointing across it.
"""
import argparse
import csv
import importlib.util
import json
import math
import os
import sys

A_Y = 1.85

# import the car's road-aware lookup from the sibling runtime so the offline check and the on-car runtime are one code path
_RUNTIME = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'mapcurved.py')
_spec = importlib.util.spec_from_file_location('mapcurved_runtime', _RUNTIME)
_mc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mc)
MapCurveLookup = _mc.MapCurveLookup
approach_speed = _mc.approach_speed


def at(db, lat, lon, hdg, v_kmh):
  lk = MapCurveLookup(db)
  v = v_kmh / 3.6
  r = lk.query_ahead(lat, lon, hdg, v)
  if r is None:
    print('NO MATCH: no mapped road within %.0fm with a tangent within %.0f deg of heading %.0f.\n'
          '          (Off a motorway/trunk/primary road, or pointing across it - not a DB failure.)'
          % (lk.MATCH_RADIUS_M, lk.MATCH_HEADING_DEG, hdg))
    return
  sp, dist, m = r
  vnow = approach_speed(sp, dist, v)
  print('matched road way_id=%s seg=%s dir=%+d  (%.1f m off road, heading err %.1f deg)'
        % (m['way_id'], m['segment_id'], m['direction'], m['match_m'], m['heading_error']))
  print('binding curve ahead: safe %.0f km/h at %.0f m.  At %.0f km/h the map would target %.0f km/h NOW (%s).'
        % (sp * 3.6, dist, v_kmh, vnow * 3.6, 'SLOW DOWN' if vnow < v - 0.5 else 'ok, already slow enough'))


def load_trace(path):
  if path.endswith('.jsonl') or path.endswith('.jsonl.gz'):
    import gzip
    op = gzip.open(path, 'rt') if path.endswith('.gz') else open(path)
    for ln in op:
      try:
        r = json.loads(ln)
      except Exception:
        continue
      msgs = r.get('msgs', {})
      loc = msgs.get('MCU_locationStatus', {})
      gps = msgs.get('UI_gpsVehicleSpeed', {})

      def num(x):
        try:
          return float(str(x).split()[0])
        except Exception:
          return None
      lat = num(loc.get('MCU_latitude'))
      lon = num(loc.get('MCU_longitude'))
      hdg = num(gps.get('UI_gpsVehicleHeading'))
      v = (r.get('v') or 0) * 3.6
      if lat and lon and hdg is not None:
        yield lat, lon, hdg, v
  else:
    for row in csv.DictReader(open(path)):
      try:
        yield float(row['lat']), float(row['lon']), float(row['heading']), float(row['v_kmh'])
      except Exception:
        continue


def trace(db, path):
  lk = MapCurveLookup(db)
  n = 0
  should = 0
  print('%-10s %-10s %4s %6s %8s %6s' % ('lat', 'lon', 'hdg', 'v_kmh', 'map_kmh', 'dist'))
  for lat, lon, hdg, v in load_trace(path):
    n += 1
    r = lk.query_ahead(lat, lon, hdg, v / 3.6)
    if r is None:
      continue
    sp, dist, _m = r
    vnow = approach_speed(sp, dist, v / 3.6)
    flag = ''
    if vnow < v / 3.6 - 1.0:
      flag = '<- should slow'
      should += 1
    print('%-10.5f %-10.5f %4.0f %6.0f %8.0f %6.0f  %s' % (lat, lon, hdg, v, vnow * 3.6, dist, flag))
  print('\n%d trace points, %d where the map would have told you to slow' % (n, should))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('db')
  ap.add_argument('--at', nargs=3, type=float, metavar=('LAT', 'LON', 'HEADING'))
  ap.add_argument('--v', type=float, default=100.0, help='current speed km/h for the approach calc')
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
