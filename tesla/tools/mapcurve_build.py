#!/usr/bin/env python3
"""
mapcurve_build.py (v2) - build the curve-speed database mapcurved.py reads, with ACCURATE curvature.

Why v2: the old builder computed curvature with np.gradient over the raw (sparse, unevenly spaced) OSM nodes, which
SMOOTHS a local bend across the whole span and badly under-reads tight curves - a real S-curve ended up stored as
"safe at 180 km/h", so the car never slowed. v2 computes curvature with MENGER curvature (the exact circumradius
through three consecutive nodes: k = 4*Area / (|AB|*|BC|*|CA|)), the same method the established OSM "curvature" tools
use. It captures the real tightness at every node, and it keeps the apex nodes exactly (no resampling that would blur
them), filling only long gaps between nodes.

Output schema is IDENTICAL to what mapcurved.py expects (do not change):
    pts(lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER), INDEX(cell_lat, cell_lon)
`speed` is the comfortable curve speed in m/s: v = sqrt(A_Y / curvature), clipped to [MIN_MPS, 255]. mapcurved.py
back-derives curvature with A_Y=1.85 and PCC re-applies the same, so A_Y here should stay 1.85 unless you deliberately
want the whole map more/less aggressive (lower A_Y = slower in curves = more slowing).

Run on a PC (NOT the EON):
    pip install pyrosm numpy
    python3 mapcurve_build.py myarea.osm.pbf mapcurve.sqlite            # whole extract
    python3 mapcurve_build.py myarea.osm.pbf mapcurve.sqlite --ay 1.5   # more aggressive (slower in curves)
    python3 mapcurve_build.py myarea.osm.pbf mapcurve.sqlite --bbox minlon,minlat,maxlon,maxlat
Then copy mapcurve.sqlite to the EON at /data/mapcurve.sqlite and restart.

Self-test (no pyrosm needed, verifies the curvature math on a known circle):
    python3 mapcurve_build.py --selftest
"""
import argparse
import math
import sqlite3
import sys

import numpy as np

CELL_DEG = 0.01            # grid cell size; MUST match mapcurved.py / the old builder
A_Y_DEFAULT = 1.85         # m/s^2 lateral; MUST match mapcurved.py A_Y_MAX and PCC A_Y_MAP for a clean round-trip
MIN_MPS = 20 * 0.44704     # 20 mph floor
MAX_MPS = 255.
STEP_M = 6.0               # fill gaps between nodes to at most this spacing (lookup density)
SMOOTH_N = 1              # median window (nodes) to reject single-node GPS spikes; 1 = off

DRIVABLE = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary',
            'secondary_link', 'tertiary', 'tertiary_link', 'unclassified', 'residential', 'living_street', 'service'}


def to_local_xy(lats, lons, lat0):
  R = 6371000.0
  c = math.cos(math.radians(lat0))
  x = np.radians(lons) * R * c
  y = np.radians(lats) * R
  return x, y


def menger_curvature(x, y):
  """exact curvature (1/m) at each point from the circumradius through it and its two neighbours. Endpoints copy the
  nearest interior value. Collinear -> 0."""
  n = len(x)
  k = np.zeros(n)
  if n < 3:
    return k
  for i in range(1, n - 1):
    ax, ay = x[i - 1], y[i - 1]
    bx, by = x[i], y[i]
    cx, cy = x[i + 1], y[i + 1]
    ab = math.hypot(bx - ax, by - ay)
    bc = math.hypot(cx - bx, cy - by)
    ca = math.hypot(ax - cx, ay - cy)
    if ab < 1e-3 or bc < 1e-3 or ca < 1e-3:
      continue
    area2 = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))   # 2 * triangle area
    k[i] = 2.0 * area2 / (ab * bc * ca)                          # = 4*Area/(ab*bc*ca)
  k[0] = k[1]; k[-1] = k[-2]
  return k


def _median_smooth(k, w):
  if w <= 1:
    return k
  h = w // 2
  out = k.copy()
  for i in range(len(k)):
    out[i] = np.median(k[max(0, i - h):i + h + 1])
  return out


def way_points(lats, lons, a_y):
  """given a way's node lats/lons, return [(lat, lon, speed_ms)] with apex nodes kept and long gaps filled."""
  n = len(lats)
  if n < 2:
    return []
  x, y = to_local_xy(np.asarray(lats), np.asarray(lons), float(np.mean(lats)))
  k = _median_smooth(menger_curvature(x, y), SMOOTH_N)
  speed = np.sqrt(a_y / np.clip(k, a_y / (MAX_MPS ** 2), None))   # clip so straight -> MAX_MPS, not inf
  speed = np.clip(speed, MIN_MPS, MAX_MPS)
  out = []
  for i in range(n):
    out.append((float(lats[i]), float(lons[i]), float(speed[i])))
    if i < n - 1:
      seg = math.hypot(x[i + 1] - x[i], y[i + 1] - y[i])
      if seg > STEP_M:                                            # fill a long segment, carrying the tighter speed
        m = int(seg // STEP_M)
        sp = min(speed[i], speed[i + 1])
        for j in range(1, m + 1):
          f = j / (m + 1)
          out.append((float(lats[i] + f * (lats[i + 1] - lats[i])),
                      float(lons[i] + f * (lons[i + 1] - lons[i])), float(sp)))
  return out


def build(pbf, out_path, a_y, bbox=None):
  from pyrosm import OSM
  osm = OSM(pbf, bounding_box=bbox) if bbox else OSM(pbf)
  roads = osm.get_network(network_type='driving')
  if roads is None or len(roads) == 0:
    sys.exit('no roads found - check the extract and bbox')
  db = sqlite3.connect(out_path)
  db.execute('PRAGMA synchronous=OFF'); db.execute('PRAGMA journal_mode=MEMORY')
  db.execute('DROP TABLE IF EXISTS pts')
  db.execute('CREATE TABLE pts (lat REAL, lon REAL, speed REAL, cell_lat INTEGER, cell_lon INTEGER)')
  buf = []; n_ways = 0; n_pts = 0
  for _, row in roads.iterrows():
    if row.get('highway') not in DRIVABLE:
      continue
    geom = row.geometry
    if geom is None or geom.geom_type != 'LineString':
      continue
    coords = list(geom.coords)
    if len(coords) < 2:
      continue
    lons = [c[0] for c in coords]; lats = [c[1] for c in coords]
    for lat, lon, sp in way_points(lats, lons, a_y):
      lat = round(lat, 5); lon = round(lon, 5); sp = round(sp, 1)
      buf.append((lat, lon, sp, int(math.floor(lat / CELL_DEG)), int(math.floor(lon / CELL_DEG))))
    n_ways += 1
    if len(buf) >= 50000:
      db.executemany('INSERT INTO pts VALUES (?,?,?,?,?)', buf); n_pts += len(buf); buf = []
  if buf:
    db.executemany('INSERT INTO pts VALUES (?,?,?,?,?)', buf); n_pts += len(buf)
  db.execute('CREATE INDEX idx_cell ON pts (cell_lat, cell_lon)')
  # a quick quality readout so you can see the curve coverage before trusting it
  tot, b90, b65, b43, tight = db.execute(
      'SELECT count(*), sum(speed<25.0), sum(speed<18.0), sum(speed<12.0), round(min(speed),1) FROM pts').fetchone()
  db.commit(); db.close()
  print('ways %d, points %d -> %s' % (n_ways, n_pts, out_path))
  print('curve coverage: <90km/h=%s  <65km/h=%s  <43km/h=%s  tightest=%s m/s' % (b90, b65, b43, tight))


def selftest():
  # a circle of radius R: Menger curvature must be ~1/R, so speed ~ sqrt(A_Y*R)
  for R in (50., 100., 300.):
    th = np.linspace(0, math.pi / 2, 40)                 # a quarter circle, nodes ~ (R*pi/2/40) apart
    x = R * np.cos(th); y = R * np.sin(th)
    k = menger_curvature(x, y)
    kmid = np.median(k[2:-2])
    v = math.sqrt(A_Y_DEFAULT / kmid)
    print('R=%4.0f m: curvature=%.5f (expect %.5f)  speed=%.1f m/s = %.0f km/h (expect %.0f)' % (
        R, kmid, 1. / R, v, v * 3.6, math.sqrt(A_Y_DEFAULT * R) * 3.6))
    assert abs(kmid - 1. / R) < 0.1 / R, 'curvature off'
  print('selftest OK')


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('pbf', nargs='?'); ap.add_argument('out', nargs='?')
  ap.add_argument('--ay', type=float, default=A_Y_DEFAULT)
  ap.add_argument('--bbox', default=None)
  ap.add_argument('--selftest', action='store_true')
  a = ap.parse_args()
  if a.selftest:
    selftest(); return
  if not a.pbf or not a.out:
    ap.error('need INPUT.osm.pbf OUTPUT.sqlite (or --selftest)')
  bbox = [float(v) for v in a.bbox.split(',')] if a.bbox else None
  build(a.pbf, a.out, a.ay, bbox)


if __name__ == '__main__':
  main()
