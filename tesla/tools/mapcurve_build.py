#!/usr/bin/env python3
"""Build a road-aware mapcurve SQLite database from an OSM PBF.

Requires: pip install pyrosm numpy

Road-aware: each point carries way_id + segment_id + seq + direction so the runtime (mapcurved.py / mapcurve_check.py)
can FOLLOW THE ROAD YOU ARE ON instead of grabbing the geometrically-nearest point. This is essential on hairpins /
switchbacks (e.g. DN73A), where the nearest point in a grid can belong to an adjacent hairpin leg or a parallel road.
Curvature is Menger (exact circumradius through consecutive nodes), signed. Major roads only (motorway/trunk/primary,
incl. links) - this is the set used for highway/national-road (DN) driving; secondary/tertiary/residential/service are
excluded on purpose. Self-test:  python3 mapcurve_build.py --selftest

NODE IDs / CONNECTIVITY (schema v2): the segments table also records start_node_id / end_node_id (OSM node IDs) and a
meta table records schema_version. Ways are split into segments at every node SHARED with another drivable way, so every
junction is a segment endpoint and two segments are connected IF AND ONLY IF they share a node ID - never because their
coordinates are close. The runtime (mapcurved.py) only follows such shared-node links. A database built without node IDs
(old schema, or --no-nodes) simply has no cross-segment links: prediction stops at the segment end.
Oneway: an explicit oneway tag wins; otherwise motorway/motorway_link are oneway (the OSM default).

Run on a PC (NOT the EON):
    python3 mapcurve_build.py myarea.osm.pbf mapcurve.sqlite [--ay 1.85] [--bbox minlon,minlat,maxlon,maxlat] [--no-nodes]
Then copy mapcurve.sqlite to the EON at /data/mapcurve.sqlite and use the road-aware mapcurved.py.
"""
import argparse
import collections
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
SCHEMA_VERSION = 2
IMPLIED_ONEWAY = {'motorway', 'motorway_link'}   # OSM default when there is no oneway tag
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


def way_points_v(coords, ay):
    """Ordered samples (lat, lon, speed_mps, signed_curvature, seq, vidx); vidx is the index of the original vertex in
    `coords`, or -1 for an interpolated filler point."""
    if len(coords) < 2:
        return []
    x, y = local_xy(coords)
    k = curvature_signed(x, y)
    speed = np.sqrt(ay / np.maximum(np.abs(k), ay / (MAX_MPS ** 2)))
    speed = np.clip(speed, MIN_MPS, MAX_MPS)
    out = []; seq = 0
    for i, (lon, lat) in enumerate(coords):
        out.append((lat, lon, float(speed[i]), float(k[i]), seq, i)); seq += 1
        if i + 1 < len(coords):
            d = math.hypot(x[i + 1] - x[i], y[i + 1] - y[i])
            if d > STEP_M:
                nfill = int(d // STEP_M)
                sp = min(float(speed[i]), float(speed[i + 1]))
                kk = math.copysign(ay / (sp * sp), k[i] if abs(k[i]) >= abs(k[i + 1]) else k[i + 1])
                for j in range(1, nfill + 1):
                    f = j / (nfill + 1)
                    out.append((lat + f * (coords[i + 1][1] - lat), lon + f * (coords[i + 1][0] - lon), sp, kk, seq, -1)); seq += 1
    return out


def way_points(coords, ay):
    """Ordered (lat, lon, speed_mps, signed_curvature, seq) samples (legacy shape)."""
    return [s[:5] for s in way_points_v(coords, ay)]


def get_way_id(row, index):
    for key in ('id', 'osm_id', 'way_id'):
        try:
            value = row.get(key)
            if value is not None and str(value) not in ('', 'nan', 'None'):
                return str(int(value)) if float(value).is_integer() else str(value)
        except Exception:
            pass
    raise RuntimeError('OSM way ID missing from pyrosm row (index=%r); refusing to build road-unidentified DB' % (index,))


def oneway_code(value, highway=None):
    """1 = forward only along stored order, -1 = reverse only, 0 = both. An explicit tag wins; with no tag the OSM default
    applies (motorway/motorway_link are oneway)."""
    v = str(value).strip().lower()
    if v in ('yes', '1', 'true'):
        return 1
    if v == '-1':
        return -1
    if v in ('no', '0', 'false'):
        return 0
    return 1 if highway in IMPLIED_ONEWAY else 0


def iter_lines(geom):
    if geom is None or geom.is_empty:
        return
    if geom.geom_type == 'LineString':
        yield list(geom.coords)
    elif geom.geom_type == 'MultiLineString':
        for line in geom.geoms:
            yield list(line.coords)


SCHEMA_SQL = '''
  CREATE TABLE pts (lat REAL NOT NULL, lon REAL NOT NULL, speed REAL NOT NULL,
    cell_lat INTEGER NOT NULL, cell_lon INTEGER NOT NULL, way_id TEXT NOT NULL, segment_id INTEGER NOT NULL,
    seq INTEGER NOT NULL, direction INTEGER NOT NULL, highway TEXT, curvature REAL NOT NULL);
  CREATE TABLE segments (way_id TEXT NOT NULL, segment_id INTEGER NOT NULL, highway TEXT, name TEXT,
    oneway INTEGER NOT NULL, point_count INTEGER NOT NULL, start_node_id INTEGER, end_node_id INTEGER,
    PRIMARY KEY (way_id, segment_id));
  CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT);
'''


def _chains(way_edges):
    """Group one way's edges (u, v, coords) into contiguous chains. Returns [(coords, node_ids)] with node_ids parallel to
    coords (None for a vertex that is not an OSM node boundary). A gap (edge.u != previous edge.v) starts a new chain, so a
    missing/misordered edge never creates a false connection."""
    chains = []; coords = None; nodes = None
    for u, v, ecoords in way_edges:
        if coords is not None and nodes[-1] is not None and nodes[-1] == u:
            coords.extend(ecoords[1:]); nodes.extend([None] * (len(ecoords) - 2) + [v])
        else:
            if coords is not None:
                chains.append((coords, nodes))
            coords = list(ecoords); nodes = [u] + [None] * (len(ecoords) - 2) + [v]
    if coords is not None:
        chains.append((coords, nodes))
    return chains


def build_from_edges(edges, out_path, ay, with_nodes=True):
    """edges: iterable of (way_id, u, v, coords[(lon, lat), ...], highway, oneway_raw, name) in way order; u/v are OSM node
    IDs (None if unknown). Writes the v2 database. Pure function (no pyrosm) so it is unit-testable."""
    if os.path.exists(out_path):
        os.remove(out_path)
    ways = collections.OrderedDict()
    node_ways = {}                                   # node id -> way id, or None when shared by >1 way
    for way_id, u, v, coords, highway, oneway, name in edges:
        w = ways.get(way_id)
        if w is None:
            w = ways[way_id] = {'highway': highway, 'name': name, 'oneway': oneway, 'edges': []}
        w['edges'].append((u if with_nodes else None, v if with_nodes else None, coords))
        if with_nodes:
            for n in (u, v):
                if n is not None:
                    node_ways[n] = way_id if node_ways.get(n, way_id) == way_id else None
    junction = {n for n, w in node_ways.items() if w is None}      # shared with another way => segment boundary

    db = sqlite3.connect(out_path)
    db.execute('PRAGMA journal_mode=OFF'); db.execute('PRAGMA synchronous=OFF'); db.execute('PRAGMA temp_store=MEMORY')
    db.executescript(SCHEMA_SQL)
    buf = []; nways = nseg = npts = 0
    for way_id, w in ways.items():
        highway = str(w['highway']); direction = oneway_code(w['oneway'], w['highway'])
        seg_id = 0
        for coords, nodes in _chains(w['edges']):
            if len(coords) < 2:
                continue
            samples = way_points_v(coords, ay)
            pos = {}
            for si, smp in enumerate(samples):
                if smp[5] >= 0 and smp[5] not in pos:
                    pos[smp[5]] = si
            cuts = [0] + [i for i in range(1, len(coords) - 1) if nodes[i] is not None and nodes[i] in junction] + [len(coords) - 1]
            for ca, cb in zip(cuts[:-1], cuts[1:]):
                piece = samples[pos[ca]:pos[cb] + 1]
                if len(piece) < 2:
                    continue
                db.execute('INSERT INTO segments VALUES (?,?,?,?,?,?,?,?)', (way_id, seg_id, highway, w['name'], direction,
                           len(piece), nodes[ca], nodes[cb]))
                for seq, (lat, lon, speed, curv, _s, _v) in enumerate(piece):
                    lat, lon = round(lat, 7), round(lon, 7)
                    buf.append((lat, lon, round(speed, 3), int(math.floor(lat / CELL_DEG)), int(math.floor(lon / CELL_DEG)),
                                way_id, seg_id, seq, direction, highway, curv))
                seg_id += 1; nseg += 1
                if len(buf) >= 50000:
                    db.executemany('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?,?,?)', buf); npts += len(buf); buf.clear()
        nways += 1
    if buf:
        db.executemany('INSERT INTO pts VALUES (?,?,?,?,?,?,?,?,?,?,?)', buf); npts += len(buf)
    db.executemany('INSERT INTO meta VALUES (?,?)', [('schema_version', str(SCHEMA_VERSION)),
                   ('node_ids', 'yes' if with_nodes else 'no'), ('a_y', str(ay))])
    db.executescript('''
      CREATE INDEX idx_pts_cell ON pts(cell_lat, cell_lon);
      CREATE INDEX idx_pts_road_order ON pts(way_id, segment_id, seq);
      CREATE INDEX idx_pts_way ON pts(way_id);
      CREATE INDEX idx_seg_start ON segments(start_node_id);
      CREATE INDEX idx_seg_end ON segments(end_node_id);
    ''')
    stats = db.execute('SELECT COUNT(*), SUM(speed<25.0), SUM(speed<18.0), SUM(speed<12.0), MIN(speed) FROM pts').fetchone()
    db.commit(); db.execute('PRAGMA optimize'); db.close()
    print('ways %d, segments %d, points %d, node_ids=%s -> %s' % (nways, nseg, npts, 'yes' if with_nodes else 'no', out_path))
    print('coverage: <90km/h=%d <65km/h=%d <43km/h=%d min=%.2f m/s' % (stats[1] or 0, stats[2] or 0, stats[3] or 0, stats[4] or 0.0))


def _clean(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else x


def edges_from_pyrosm(osm):
    """Yield build_from_edges() tuples from pyrosm's node-to-node edges (get_network(nodes=True)): u/v are the OSM node IDs of
    the edge's first/last coordinate (verified against pyrosm output: u -> coords[0], v -> coords[-1])."""
    nodes_gdf, edges = osm.get_network(network_type='driving', nodes=True,
                                       custom_filter={'highway': sorted(DRIVABLE)}, filter_type='keep')
    for row in edges.itertuples():
        if row.highway not in DRIVABLE:
            continue
        for coords in iter_lines(row.geometry) or []:
            yield (str(int(row.id)), int(row.u), int(row.v), coords, row.highway, _clean(getattr(row, 'oneway', None)),
                   _clean(getattr(row, 'name', None)))


def build(pbf, out_path, ay, bbox=None, with_nodes=True):
    from pyrosm import OSM
    osm = OSM(pbf, bounding_box=bbox) if bbox else OSM(pbf)
    if with_nodes:
        build_from_edges(edges_from_pyrosm(osm), out_path, ay, True)
        return
    roads = osm.get_network(network_type='driving')           # legacy: whole ways, no node IDs (no cross-segment links)
    if roads is None or len(roads) == 0:
        raise RuntimeError('No drivable roads found in PBF/bbox')
    def legacy():
        for idx, row in roads.iterrows():
            if row.get('highway') in DRIVABLE:
                for coords in iter_lines(row.geometry) or []:
                    yield (get_way_id(row, idx), None, None, coords, row.get('highway'), _clean(row.get('oneway')), _clean(row.get('name')))
    build_from_edges(legacy(), out_path, ay, False)


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
    ap.add_argument('--no-nodes', action='store_true', help='legacy whole-way build without node IDs (no cross-segment links)')
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
    build(a.pbf, a.output, a.ay, bbox, not a.no_nodes)


if __name__ == '__main__':
    main()
