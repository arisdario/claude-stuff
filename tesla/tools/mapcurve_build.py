#!/usr/bin/env python3
"""Build a road-aware mapcurve SQLite database from an OSM PBF.

Requires: pip install pyrosm numpy

Road-aware: each point carries way_id + segment_id + seq + direction so the runtime (mapcurved.py / mapcurve_check.py)
can FOLLOW THE ROAD YOU ARE ON instead of grabbing the geometrically-nearest point. This is essential on hairpins /
switchbacks (e.g. DN73A), where the nearest point in a grid can belong to an adjacent hairpin leg or a parallel road.
Curvature is Menger (exact circumradius through consecutive nodes), signed. Major roads only (motorway/trunk/primary,
incl. links) - this is the set used for highway/national-road (DN) driving; secondary/tertiary/residential/service are
excluded on purpose. Self-test:  python3 mapcurve_build.py --selftest

Run on a PC (NOT the EON):
    python3 mapcurve_build.py myarea.osm.pbf mapcurve.sqlite [--ay 1.85] [--bbox minlon,minlat,maxlon,maxlat]
Then copy mapcurve.sqlite to the EON at /data/mapcurve.sqlite and use the road-aware mapcurved.py.
"""
import argparse
import math
import os
import sqlite3
import sys

import numpy as np

CELL_DEG = 0.01
A_Y_DEFAULT = 1.85
MIN_MPS = 20 * 0.44704
MAX_MPS = 255.0
STEP_M = 6.0
DRIVABLE = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link'}


def local_xy(coords):
    lat0 = sum(p[1] for p in coords) / len(coords)
    r = 6371000.0
    c = math.cos(math.radians(lat0))
    return (np.asarray([p[0] for p in coords], dtype=float) * (math.pi / 180) * r * c,
            np.asarray([p[1] for p in coords], dtype=float) * (math.pi / 180) * r)


def curvature_signed(x, y):
    """Menger curvature with sign from ordered triples; units 1/m."""
    n = len(x)
    k = np.zeros(n, dtype=float)
    if n < 3:
        return k
    for i in range(1, n - 1):
        ax, ay = x[i - 1], y[i - 1]; bx, by = x[i], y[i]; cx, cy = x[i + 1], y[i + 1]
        ab = math.hypot(bx - ax, by - ay); bc = math.hypot(cx - bx, cy - by); ca = math.hypot(ax - cx, ay - cy)
        if min(ab, bc, ca) < 1e-3:
            continue
        cross = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        k[i] = 2.0 * cross / (ab * bc * ca)
    k[0] = k[1]; k[-1] = k[-2]
    return k


def way_points(coords, ay):
    """Return ordered (lat, lon, speed_mps, signed_curvature, seq) samples."""
    if len(coords) < 2:
        return []
    x, y = local_xy(coords)
    k = curvature_signed(x, y)
    speed = np.sqrt(ay / np.maximum(np.abs(k), ay / (MAX_MPS ** 2)))
    speed = np.clip(speed, MIN_MPS, MAX_MPS)
    out = []; seq = 0
    for i, (lon, lat) in enumerate(coords):
        out.append((lat, lon, float(speed[i]), float(k[i]), seq)); seq += 1
        if i + 1 < len(coords):
            d = math.hypot(x[i + 1] - x[i], y[i + 1] - y[i])
            if d > STEP_M:
                nfill = int(d // STEP_M)
                sp = min(float(speed[i]), float(speed[i + 1]))
                kk = math.copysign(ay / (sp * sp), k[i] if abs(k[i]) >= abs(k[i + 1]) else k[i + 1])
                for j in range(1, nfill + 1):
                    f = j / (nfill + 1)
                    out.append((lat + f * (coords[i + 1][1] - lat), lon + f * (coords[i + 1][0] - lon), sp, kk, seq)); seq += 1
    return out


def get_way_id(row, index):
    for key in ('id', 'osm_id', 'way_id'):
        try:
            value = row.get(key)
            if value is not None and str(value) not in ('', 'nan', 'None'):
                return str(int(value)) if float(value).is_integer() else str(value)
        except Exception:
            pass
    raise RuntimeError('OSM way ID missing from pyrosm row (index=%r); refusing to build road-unidentified DB' % (index,))


def oneway_code(value):
    s = str(value).strip().lower()
    return 1 if s in ('yes', '1', 'true') else (-1 if s == '-1' else 0)


def iter_lines(geom):
    if geom is None or geom.is_empty:
        return
    if geom.geom_type == 'LineString':
        yield list(geom.coords)
    elif geom.geom_type == 'MultiLineString':
        for line in geom.geoms:
            yield list(line.coords)


def build(pbf, out_path, ay, bbox=None):
    from pyrosm import OSM
    if os.path.exists(out_path):
        os.remove(out_path)
    osm = OSM(pbf, bounding_box=bbox) if bbox else OSM(pbf)
    roads = osm.get_network(network_type='driving')
    if roads is None or len(roads) == 0:
        raise RuntimeError('No drivable roads found in PBF/bbox')
    db = sqlite3.connect(out_path)
    db.execute('PRAGMA journal_mode=OFF'); db.execute('PRAGMA synchronous=OFF'); db.execute('PRAGMA temp_store=MEMORY')
    db.executescript('''
      CREATE TABLE pts (lat REAL NOT NULL, lon REAL NOT NULL, speed REAL NOT NULL,
        cell_lat INTEGER NOT NULL, cell_lon INTEGER NOT NULL, way_id TEXT NOT NULL, segment_id INTEGER NOT NULL,
        seq INTEGER NOT NULL, direction INTEGER NOT NULL, highway TEXT, curvature REAL NOT NULL);
      CREATE TABLE segments (way_id TEXT NOT NULL, segment_id INTEGER NOT NULL, highway TEXT, name TEXT,
        oneway INTEGER NOT NULL, point_count INTEGER NOT NULL, PRIMARY KEY (way_id, segment_id));
    ''')
    buf = []; ways = segments = npts = 0
    for idx, row in roads.iterrows():
        highway = row.get('highway')
        if highway not in DRIVABLE:
            continue
        way_id = get_way_id(row, idx); geom = row.geometry; name = row.get('name'); direction = oneway_code(row.get('oneway'))
        for seg_id, coords in enumerate(iter_lines(geom) or []):
            if len(coords) < 2:
                continue
            samples = way_points(coords, ay)
            if not samples:
                continue
            db.execute('INSERT INTO segments VALUES (?,?,?,?,?,?)', (way_id, seg_id, str(highway),
                       str(name) if name is not None else None, direction, len(samples)))
            for lat, lon, speed, curv, seq in samples:
                lat, lon = round(lat, 7), round(lon, 7)
                buf.append((lat, lon, round(speed, 3), int(math.floor(lat / CELL_DEG)), int(math.floor(lon / CELL_DEG)),
                            way_id, seg_id, seq, direction, str(highway), curv))
            segments += 1
            if len(buf) >= 50000:
                db.executemany('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?,?,?)', buf); npts += len(buf); buf.clear()
        ways += 1
    if buf:
        db.executemany('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?,?,?)', buf); npts += len(buf)
    db.executescript('''
      CREATE INDEX idx_pts_cell ON pts(cell_lat, cell_lon);
      CREATE INDEX idx_pts_road_order ON pts(way_id, segment_id, seq);
      CREATE INDEX idx_pts_way ON pts(way_id);
    ''')
    stats = db.execute('SELECT COUNT(*), SUM(speed<25.0), SUM(speed<18.0), SUM(speed<12.0), MIN(speed) FROM pts').fetchone()
    db.commit(); db.execute('PRAGMA optimize'); db.close()
    print('ways %d, segments %d, points %d -> %s' % (ways, segments, npts, out_path))
    print('coverage: <90km/h=%d <65km/h=%d <43km/h=%d min=%.2f m/s' % (stats[1] or 0, stats[2] or 0, stats[3] or 0, stats[4] or 0.0))


def selftest():
    for R in (50., 100., 300.):
        th = np.linspace(0, math.pi / 2, 40); x = R * np.cos(th); y = R * np.sin(th)
        k = np.abs(curvature_signed(x, y)); kmid = float(np.median(k[2:-2])); v = math.sqrt(A_Y_DEFAULT / kmid)
        print('R=%4.0f m: curvature=%.5f (expect %.5f) speed=%.0f km/h (expect %.0f)' % (
            R, kmid, 1. / R, v * 3.6, math.sqrt(A_Y_DEFAULT * R) * 3.6))
        assert abs(kmid - 1. / R) < 0.1 / R
    print('selftest OK')


def main():
    ap = argparse.ArgumentParser(description='road-aware mapcurve DB builder (motorway/trunk/primary)')
    ap.add_argument('pbf', nargs='?'); ap.add_argument('output', nargs='?')
    ap.add_argument('--ay', type=float, default=A_Y_DEFAULT); ap.add_argument('--bbox')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()
    if a.selftest:
        selftest(); return
    if not a.pbf or not a.output:
        ap.error('need INPUT.osm.pbf OUTPUT.sqlite (or --selftest)')
    if a.ay <= 0:
        ap.error('--ay must be positive')
    bbox = [float(x) for x in a.bbox.split(',')] if a.bbox else None
    if bbox and len(bbox) != 4:
        ap.error('--bbox needs minlon,minlat,maxlon,maxlat')
    build(a.pbf, a.output, a.ay, bbox)


if __name__ == '__main__':
    main()
