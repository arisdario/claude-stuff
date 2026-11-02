"""Routing graph database (routegraph.sqlite): directed road edges keyed by OSM node IDs. Built from an OSM PBF on a PC, read
by the router. Separate from mapcurve.sqlite (which has 6 road classes, no access tags and v1 has no node IDs - see
SCHEMA_V2_AND_RUNTIME.md); this file never touches it.

Schema:
  nodes(id, lat, lon)                     OSM node id -> position (edge endpoints only)
  edges(eid, way_id, seg, u, v, length_m, cost_s, highway, name, ref, roundabout, rev, oneway, access, geom, cell_lat, cell_lon,
        bridge, tunnel, layer, maxspeed_kph)                    (the last four: schema v2)
        one row per LEGAL DIRECTION of each node-to-node OSM edge. rev=1: runs against the OSM way order. oneway/access keep
        the raw tags so the validator can re-apply the rules independently. geom = 'lat,lon;lat,lon;...' in travel order.
  places(name, norm, place, lat, lon)     place=* nodes, to resolve 'Predeal' etc. from the data itself
  meta(key, value)
Two edges are connected iff they share an OSM node ID (v of one == u of the next). Never by coordinates."""
import collections
import hashlib
import json
import math
import os
import sqlite3
import time

try:
    from . import geo, osm_rules as R
except ImportError:                                           # run as a plain script / from tests with the dir on sys.path
    import geo
    import osm_rules as R

CELL_DEG = 0.01
SCHEMA_VERSION = 2          # v2 adds bridge / tunnel / layer / maxspeed_kph to edges; v1 databases are still readable
READABLE_VERSIONS = ('1', '2')
TAG_KEYS = ('highway', 'name', 'ref', 'oneway', 'oneway:motorcar', 'oneway:motor_vehicle', 'junction', 'maxspeed', 'access',
            'motor_vehicle', 'motorcar', 'vehicle', 'service', 'bridge', 'tunnel', 'layer')

SCHEMA_SQL = '''
CREATE TABLE nodes (id INTEGER PRIMARY KEY, lat REAL NOT NULL, lon REAL NOT NULL);
CREATE TABLE edges (eid INTEGER PRIMARY KEY, way_id TEXT NOT NULL, seg INTEGER NOT NULL, u INTEGER NOT NULL, v INTEGER NOT NULL,
  length_m REAL NOT NULL, cost_s REAL NOT NULL, highway TEXT, name TEXT, ref TEXT, roundabout INTEGER NOT NULL, rev INTEGER NOT NULL,
  oneway TEXT, access TEXT, geom TEXT NOT NULL, cell_lat INTEGER NOT NULL, cell_lon INTEGER NOT NULL,
  bridge INTEGER NOT NULL DEFAULT 0, tunnel INTEGER NOT NULL DEFAULT 0, layer INTEGER, maxspeed_kph REAL);
CREATE TABLE places (name TEXT, norm TEXT, place TEXT, lat REAL, lon REAL);
CREATE TABLE meta (key TEXT PRIMARY KEY, value TEXT);
'''


def _flag(v, no=('no', 'false', '0', '')):
    return 0 if v is None or str(v).strip().lower() in no or str(v).strip().lower() == 'nan' else 1


def _layer(v):
    try:
        x = int(float(str(v)))
        return x if -5 <= x <= 5 else None
    except (TypeError, ValueError):
        return None


def _geom_text(pts):
    return ';'.join('%.7f,%.7f' % (a, b) for a, b in pts)


def build_from_edges(edges, out_path, places=(), meta_extra=None):
    """edges: iterable of (way_id, u, v, coords[(lon, lat), ...], tags dict) - node-to-node pieces of ways in way order.
    places: iterable of (name, place, lat, lon). Pure function (no pyrosm): unit-testable."""
    if os.path.exists(out_path):
        os.remove(out_path)
    db = sqlite3.connect(out_path)
    db.execute('PRAGMA journal_mode=OFF'); db.execute('PRAGMA synchronous=OFF')
    db.executescript(SCHEMA_SQL)
    rejected = collections.Counter()
    nodes = {}
    seg_of_way = collections.Counter()
    n_dir = 0
    for way_id, u, v, coords, tags in edges:
        ok, factor, why = R.access_ok(tags)
        if not ok:
            rejected[why] += 1
            continue
        d = R.oneway_dir(tags)
        if d == 'none':
            rejected['oneway:reversible'] += 1
            continue
        if u is None or v is None or len(coords) < 2:
            rejected['no_node_ids'] += 1
            continue
        pts = [(c[1], c[0]) for c in coords]                                  # (lat, lon)
        length = geo.polyline_length(pts)
        if length <= 0.0:
            rejected['zero_length'] += 1
            continue
        nodes[int(u)] = pts[0]; nodes[int(v)] = pts[-1]
        cost = length / (R.speed_kph(tags) / 3.6) * factor
        name = str(tags.get('name')) if tags.get('name') not in (None, '', 'nan') else None
        ref = str(tags.get('ref')) if tags.get('ref') not in (None, '', 'nan') else None
        raw_ow = json.dumps({k: tags[k] for k in ('oneway', 'oneway:motorcar', 'oneway:motor_vehicle', 'junction', 'highway') if tags.get(k) not in (None, '', 'nan')})
        raw_ac = json.dumps({k: tags[k] for k in ('access', 'motor_vehicle', 'motorcar', 'vehicle', 'service', 'highway') if tags.get(k) not in (None, '', 'nan')})
        seg = seg_of_way[way_id]; seg_of_way[way_id] += 1
        for rev, (a, b, p) in ((0, (u, v, pts)), (1, (v, u, pts[::-1]))):
            if (rev == 0 and d in ('both', 'fwd')) or (rev == 1 and d in ('both', 'rev')):
                db.execute('INSERT INTO edges (way_id, seg, u, v, length_m, cost_s, highway, name, ref, roundabout, rev, oneway, access, geom, '
                           'cell_lat, cell_lon, bridge, tunnel, layer, maxspeed_kph) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                           (str(way_id), seg, int(a), int(b), length, cost, str(tags.get('highway')), name, ref,
                            1 if R.is_roundabout(tags) else 0, rev, raw_ow, raw_ac, _geom_text(p),
                            int(math.floor(p[0][0] / CELL_DEG)), int(math.floor(p[0][1] / CELL_DEG)),
                            _flag(tags.get('bridge')), _flag(tags.get('tunnel')), _layer(tags.get('layer')), R.maxspeed_kph(tags)))
                n_dir += 1
    db.executemany('INSERT INTO nodes VALUES (?,?,?)', [(k, p[0], p[1]) for k, p in nodes.items()])
    n_pl = 0
    for name, place, lat, lon in places:
        if name:
            db.execute('INSERT INTO places VALUES (?,?,?,?,?)', (name, R.norm_name(name), place, lat, lon)); n_pl += 1
    meta_rows = [('schema_version', str(SCHEMA_VERSION)), ('rejected', json.dumps(dict(rejected))), ('turn_restrictions', 'not modelled')]
    meta_rows += [(str(k), v if isinstance(v, str) else json.dumps(v)) for k, v in (meta_extra or {}).items()]
    db.executemany('INSERT OR REPLACE INTO meta VALUES (?,?)', meta_rows)
    db.executescript('CREATE INDEX idx_edges_u ON edges(u); CREATE INDEX idx_edges_v ON edges(v); CREATE INDEX idx_edges_cell ON edges(cell_lat, cell_lon); '
                     'CREATE INDEX idx_edges_way ON edges(way_id); CREATE INDEX idx_places_norm ON places(norm);')
    db.commit(); db.close()
    return {'directed_edges': n_dir, 'nodes': len(nodes), 'places': n_pl, 'rejected': dict(rejected)}


def _clean(x):
    return None if x is None or (isinstance(x, float) and math.isnan(x)) else x


NETWORK_TYPE = 'driving+service'
RAW_KEYS = ('junction', 'tunnel', 'bridge', 'layer', 'oneway', 'oneway:motorcar', 'oneway:motor_vehicle', 'access', 'motor_vehicle', 'motorcar', 'vehicle',
            'service', 'maxspeed', 'name', 'ref')


def raw_way_tags(pbf):
    """GROUND-TRUTH tags straight from the PBF, way by way, for routable highways only: {way_id(str): {tag: value}}. A way-only streaming pass (no nodes, no geometry), so it is
    fast and small even for a country. Needs pyosmium (`pip install osmium`); returns None if it is not installed. Used to supplement pyrosm, which silently drops tag
    columns it does not recognise or that are absent from its parse (roundabouts and tunnels are the ones that matter for routing and display)."""
    try:
        import osmium
    except ImportError:
        return None
    out = {}
    for o in osmium.FileProcessor(pbf, osmium.osm.WAY):
        hw = o.tags.get('highway')
        if hw is None or hw not in R.ROUTABLE:
            continue
        out[str(o.id)] = {k: o.tags[k] for k in RAW_KEYS if k in o.tags}
        out[str(o.id)]['highway'] = hw
    return out


def edges_from_pyrosm(osm, raw=None):
    """(way_id, u, v, coords, tags) tuples from pyrosm's node-to-node edges. Extra tag columns are requested only where the PBF has them; when `raw` (raw_way_tags) is given
    its tags are AUTHORITATIVE and override pyrosm's, so no tag depends on what pyrosm chose to expose."""
    extra = ['access', 'motor_vehicle', 'motorcar', 'vehicle', 'junction', 'maxspeed', 'oneway', 'ref', 'service', 'name', 'bridge', 'tunnel', 'layer']
    # 'driving+service', NOT 'driving': pyrosm's 'driving' type silently drops every highway=service way (found by auditing a real Romanian extract against the raw
    # PBF: 100% of 4,240 routable service ways, incl. service tunnels and roundabouts). Which service roads count is decided by OUR rules (osm_rules.access_ok).
    try:
        _nodes, edges = osm.get_network(network_type=NETWORK_TYPE, nodes=True, extra_attributes=extra)
    except Exception:
        _nodes, edges = osm.get_network(network_type=NETWORK_TYPE, nodes=True)
    cols = set(edges.columns)
    for row in edges.itertuples(index=False):
        d = row._asdict()
        tags = {k: _clean(d.get(k)) for k in TAG_KEYS if k in cols}
        rawj = d.get('tags')
        if isinstance(rawj, str) and rawj.startswith('{'):
            try:
                for k, v in json.loads(rawj).items():
                    if k in TAG_KEYS and tags.get(k) is None:
                        tags[k] = v
            except ValueError:
                pass
        wid = str(int(d['id']))
        if raw is not None and wid in raw:
            tags.update(raw[wid])
        geom = d['geometry']
        lines = [list(geom.coords)] if geom.geom_type == 'LineString' else [list(g.coords) for g in geom.geoms]
        for coords in lines:
            yield (wid, int(d['u']), int(d['v']), coords, tags)


def places_from_pyrosm(osm):
    pl = osm.get_data_by_custom_criteria(custom_filter={'place': True}, osm_keys_to_keep=['place', 'name'], filter_type='keep',
                                         keep_nodes=True, keep_ways=False, keep_relations=False)
    if pl is None:
        return
    for r in pl.itertuples(index=False):
        if getattr(r, 'name', None) and isinstance(r.name, str):
            yield (r.name, getattr(r, 'place', None), float(r.lat), float(r.lon))


def file_md5(path, chunk=1 << 20):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for blk in iter(lambda: f.read(chunk), b''):
            h.update(blk)
    return h.hexdigest()


def build(pbf, out_path, bbox=None, use_raw_tags=True):
    """Build routegraph.sqlite from a PBF, entirely locally. Records provenance (source file, size, md5 - compare with Geofabrik's .md5 - bbox, tag source, versions)."""
    from pyrosm import OSM
    osm = OSM(pbf, bounding_box=bbox) if bbox else OSM(pbf)
    raw = raw_way_tags(pbf) if use_raw_tags else None
    extra = {'source_file': os.path.basename(pbf), 'source_bytes': str(os.path.getsize(pbf)), 'source_md5': file_md5(pbf),
             'bbox': json.dumps(bbox) if bbox else 'none (whole file)', 'built_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
             'tag_source': 'pyrosm + raw PBF tags (pyosmium)' if raw is not None else 'pyrosm only (pyosmium not installed or disabled)',
             'raw_ways_read': str(len(raw)) if raw is not None else '0', 'pyrosm_network_type': NETWORK_TYPE}
    try:
        import pyrosm
        extra['pyrosm_version'] = getattr(pyrosm, '__version__', '?')
    except ImportError:
        pass
    return build_from_edges(edges_from_pyrosm(osm, raw), out_path, list(places_from_pyrosm(osm)), extra)


Edge = collections.namedtuple('Edge', 'eid way_id seg u v length_m cost_s highway name ref roundabout rev')
Snap = collections.namedtuple('Snap', 'edge dist_m along_m lat lon heading_err')
PLACE_RANK = {'city': 0, 'town': 1, 'village': 2, 'suburb': 3, 'hamlet': 4, 'neighbourhood': 5, 'quarter': 5, 'locality': 6}


class Graph(object):
    """Read-only access to routegraph.sqlite with small caches (lazy: nothing is loaded wholesale)."""
    COLS = 'eid, way_id, seg, u, v, length_m, cost_s, highway, name, ref, roundabout, rev'

    def __init__(self, path):
        self.path = path
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute('PRAGMA query_only=ON')
        meta = dict(self.db.execute('SELECT key, value FROM meta'))
        if meta.get('schema_version') not in READABLE_VERSIONS:
            raise RuntimeError('unsupported routegraph schema %r' % (meta.get('schema_version'),))
        self.meta = meta
        self.has_v2 = 'bridge' in {r[1] for r in self.db.execute('PRAGMA table_info(edges)')}
        self._out, self._in, self._edge, self._geom, self._node = {}, {}, {}, {}, {}

    def out_edges(self, node):
        r = self._out.get(node)
        if r is None:
            r = [Edge(*row) for row in self.db.execute('SELECT %s FROM edges WHERE u=? ORDER BY eid' % self.COLS, (node,))]
            if len(self._out) > 200000:
                self._out.clear()
            self._out[node] = r
        return r

    def edge_info(self, eid):
        """everything stored about one edge as a dict (v2 fields are None on a v1 database); the routing engine does not use these"""
        cols = ['eid', 'way_id', 'u', 'v', 'length_m', 'highway', 'name', 'ref', 'roundabout', 'rev', 'oneway', 'access']
        if self.has_v2:
            cols += ['bridge', 'tunnel', 'layer', 'maxspeed_kph']
        row = self.db.execute('SELECT %s FROM edges WHERE eid=?' % ', '.join(cols), (eid,)).fetchone()
        d = dict(zip(cols, row))
        for k in ('bridge', 'tunnel', 'layer', 'maxspeed_kph'):
            d.setdefault(k, None)
        return d

    def in_edges(self, node):
        r = self._in.get(node)
        if r is None:
            r = [Edge(*row) for row in self.db.execute('SELECT %s FROM edges WHERE v=? ORDER BY eid' % self.COLS, (node,))]
            if len(self._in) > 200000:
                self._in.clear()
            self._in[node] = r
        return r

    def _component(self, node, cap):
        seen, stack = {node}, [node]
        while stack and len(seen) < cap:
            x = stack.pop()
            for (y,) in self.db.execute('SELECT v FROM edges WHERE u=? UNION SELECT u FROM edges WHERE v=?', (x, x)):
                if y not in seen:
                    seen.add(y); stack.append(y)
        return seen

    def component_size(self, node, cap=200000):
        """Size (in nodes) of the road component containing `node`, ignoring direction (shared OSM node IDs only), counting up to
        `cap`. Used to tell 'the data is disconnected here' from 'a oneway/access restriction blocks every directed path'."""
        seen = self._component(node, cap)
        return len(seen), len(seen) >= cap

    def same_component(self, a, b, cap=200000):
        """True/False whether nodes a and b are connected ignoring direction; None if the search hit `cap` without finding b."""
        seen = self._component(a, cap)
        if b in seen:
            return True
        return None if len(seen) >= cap else False

    def edge(self, eid):
        e = self._edge.get(eid)
        if e is None:
            row = self.db.execute('SELECT %s FROM edges WHERE eid=?' % self.COLS, (eid,)).fetchone()
            e = self._edge[eid] = None if row is None else Edge(*row)
        return e

    def geom(self, eid):
        g = self._geom.get(eid)
        if g is None:
            t = self.db.execute('SELECT geom FROM edges WHERE eid=?', (eid,)).fetchone()[0]
            g = self._geom[eid] = [tuple(float(x) for x in p.split(',')) for p in t.split(';')]
        return g

    def raw_tags(self, eid):
        o, a = self.db.execute('SELECT oneway, access FROM edges WHERE eid=?', (eid,)).fetchone()
        t = json.loads(o); t.update(json.loads(a))
        return t

    def node_xy(self, node):
        p = self._node.get(node)
        if p is None:
            p = self._node[node] = self.db.execute('SELECT lat, lon FROM nodes WHERE id=?', (node,)).fetchone()
        return p

    def measure(self, eid, lat, lon, heading=None):
        """(dist_m, along_m, proj_lat, proj_lon, heading_err or None) of a fix against ONE directed edge."""
        g = self.geom(eid)
        best = None
        for i in range(len(g) - 1):
            d, al, plat, plon, _sl = geo.project(lat, lon, g[i][0], g[i][1], g[i + 1][0], g[i + 1][1])
            if best is None or d < best[0]:
                best = (d, i, al, plat, plon)
        d, i, al, plat, plon = best
        err = None
        if heading is not None and geo.is_finite_number(heading):
            err = abs(geo.ang_diff(geo.bearing(g[i][0], g[i][1], g[i + 1][0], g[i + 1][1]), heading))
        along = sum(geo.hav(g[j][0], g[j][1], g[j + 1][0], g[j + 1][1]) for j in range(i)) + al
        return d, along, plat, plon, err

    def snap(self, lat, lon, heading=None, max_m=150.0, limit=6):
        """Nearest directed edges to a position (within max_m), best first. With a heading, edges pointing more than 90 deg away
        from it are dropped (so a car is snapped to the carriageway it is driving along, not the opposite one)."""
        if not geo.valid_latlon(lat, lon):
            return []
        cl, co = int(math.floor(lat / CELL_DEG)), int(math.floor(lon / CELL_DEG))
        rows = self.db.execute('SELECT eid FROM edges WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?',
                               (cl - 2, cl + 2, co - 2, co + 2)).fetchall()
        out = []
        for (eid,) in rows:
            d, along, plat, plon, err = self.measure(eid, lat, lon, heading)
            if d > max_m or (err is not None and err > 90.0):
                continue
            out.append(Snap(self.edge(eid), d, along, plat, plon, err))
        out.sort(key=lambda s: s.dist_m + 0.3 * (s.heading_err or 0.0))
        return out[:limit]

    def find_places(self, name):
        """Places matching a name (diacritic/case-insensitive): exact matches first, then prefix, ranked by place type."""
        n = R.norm_name(name)
        rows = self.db.execute('SELECT name, place, lat, lon FROM places WHERE norm=?', (n,)).fetchall()
        kind = 'exact'
        if not rows:
            rows = self.db.execute("SELECT name, place, lat, lon FROM places WHERE norm LIKE ?", (n + '%',)).fetchall()
            kind = 'prefix'
        rows.sort(key=lambda r: PLACE_RANK.get(r[1], 9))
        return kind, rows
