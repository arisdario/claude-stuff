"""Build a route polyline from the national mapcurve v1 database (main roads only: motorway/trunk/primary + links) for a quick advisory test where no routing graph exists.
v1 has no node ids, so segments are joined where their end points have IDENTICAL coordinates (7-decimal), honouring the one-way code. Advisory test tool only - NOT the routing engine."""
import heapq
import math
import sqlite3
from collections import defaultdict

try:
    from . import geo
except ImportError:
    import geo


def load(db_path, bbox):
    db = sqlite3.connect('file:%s?mode=ro' % db_path, uri=True)
    la0, la1, lo0, lo1 = bbox
    rows = db.execute('SELECT way_id, segment_id, seq, lat, lon, direction, highway FROM pts WHERE lat BETWEEN ? AND ? AND lon BETWEEN ? AND ? ORDER BY way_id, segment_id, seq', (la0, la1, lo0, lo1)).fetchall()
    segs = {}
    for w, s, q, la, lo, d, hw in rows:
        e = segs.setdefault((w, s), {'pts': [], 'dir': d, 'hw': hw})
        e['pts'].append((la, lo))
    return segs


def build(segs):
    """vertex graph: every point is a node (coordinates rounded to 6 decimals ~ 0.1 m), consecutive points are edges honouring the one-way code. Two roads connect where they share a vertex."""
    key = lambda p: (round(p[0], 6), round(p[1], 6))
    adj = defaultdict(list)                  # node -> [(neighbour, (segment id, index), forward?, length)]
    for sid, e in segs.items():
        p = e['pts']
        for i in range(len(p) - 1):
            u, v = key(p[i]), key(p[i + 1])
            if u == v:
                continue
            ln = geo.hav(p[i][0], p[i][1], p[i + 1][0], p[i + 1][1])
            if e['dir'] in (0, 1):
                adj[u].append((v, (sid, i), True, ln))
            if e['dir'] in (0, -1):
                adj[v].append((u, (sid, i), False, ln))
    return adj


def nearest_node(adj, lat, lon):
    return min(adj, key=lambda n: geo.hav(lat, lon, n[0], n[1]))


def shortest(adj, src, dst):
    dist, prev, pq = {src: 0.0}, {}, [(0.0, src)]
    while pq:
        d, n = heapq.heappop(pq)
        if n == dst:
            break
        if d > dist.get(n, 1e18):
            continue
        for v, eid, fwd, ln in adj[n]:
            nd = d + ln
            if nd < dist.get(v, 1e18):
                dist[v], prev[v] = nd, (n, eid, fwd)
                heapq.heappush(pq, (nd, v))
    if dst not in prev and src != dst:
        return None
    path, n = [], dst
    while n != src:
        n0, eid, fwd = prev[n]
        path.append((eid, n0, n))
        n = n0
    return path[::-1]


def polyline(segs, path):
    pts = [path[0][1]]
    for _eid, _u, v in path:
        pts.append(v)
    return pts


class Poly(object):
    """what the curve code needs from a route: points, cum, distance_m"""
    def __init__(self, points):
        self.points = points
        self.cum = [0.0]
        for a, b in zip(points, points[1:]):
            self.cum.append(self.cum[-1] + geo.hav(a[0], a[1], b[0], b[1]))
        self.distance_m = self.cum[-1]


def route(db_path, a, b, margin=0.12):
    bbox = (min(a[0], b[0]) - margin, max(a[0], b[0]) + margin, min(a[1], b[1]) - margin, max(a[1], b[1]) + margin)
    segs = load(db_path, bbox)
    adj = build(segs)
    path = shortest(adj, nearest_node(adj, *a), nearest_node(adj, *b))
    if path is None:
        raise ValueError('no connected path between the two places in the national database')
    return Poly(polyline(segs, path)), sorted({segs[eid[0]]['hw'] for eid, _u, _v in path})
