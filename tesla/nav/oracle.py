"""Independent reachability oracle for validating the navigator. It deliberately shares NOTHING with the production routing path:
no router, no snapping, no A*, no osm_rules, no Graph class - only sqlite3 + json + math. It re-reads the edges table, re-derives
oneway/access legality for every directed edge from the RAW tags stored with it (its own small implementation of the OSM semantics),
and answers one question by plain breadth-first search over directed edge states:

    "A car is on directed edge E. Can it legally drive to a given target edge?"

State = directed edge. From edge E the car may take any legal edge leaving E's downstream node EXCEPT an immediate reversal onto E's
twin (a car cannot U-turn on the spot; a dead-end stub is therefore a trap). Start is E's downstream node - there is no coordinate and
no snapping anywhere in here, so a position sitting on a junction vertex cannot pick an exit it was not driving towards."""
import json
import sqlite3
from collections import deque

_DENY = {'no', 'private', 'customers', 'delivery', 'agricultural', 'forestry', 'military', 'emergency', 'permit', 'discouraged'}
_ROUTABLE = {'motorway', 'motorway_link', 'trunk', 'trunk_link', 'primary', 'primary_link', 'secondary', 'secondary_link', 'tertiary',
             'tertiary_link', 'unclassified', 'residential', 'living_street', 'service'}


def _v(tags, k):
    x = tags.get(k)
    return '' if x is None else str(x).strip().lower()


def _allowed(tags):
    """own re-implementation of 'may a car use this way at all'"""
    if _v(tags, 'highway') not in _ROUTABLE:
        return False
    if _v(tags, 'highway') == 'service' and _v(tags, 'service') in ('parking_aisle', 'driveway', 'drive-through', 'emergency_access'):
        return False
    for k in ('motorcar', 'motor_vehicle', 'vehicle', 'access'):          # most specific present tag decides
        val = _v(tags, k)
        if val:
            return val not in _DENY
    return True


def _drivable_forward(tags):
    """own re-implementation of oneway: is travelling ALONG the way's stored order legal? and AGAINST it? -> (fwd, rev)"""
    for k in ('oneway:motorcar', 'oneway:motor_vehicle', 'oneway'):
        val = _v(tags, k)
        if val in ('yes', '1', 'true'):
            return True, False
        if val in ('-1', 'reverse'):
            return False, True
        if val in ('no', '0', 'false'):
            return True, True
        if val in ('reversible', 'alternating'):
            return False, False
    if _v(tags, 'junction') in ('roundabout', 'circular') or _v(tags, 'highway') in ('motorway', 'motorway_link'):
        return True, False
    return True, True


class Oracle(object):
    def __init__(self, db_path):
        db = sqlite3.connect(db_path)
        self.edges = {}                 # eid -> (u, v, way_id, rev)
        self.length = {}                # eid -> length in metres
        self.out = {}                   # node -> [eid]  (only edges that are legal for a car in THEIR direction, per the raw tags)
        self.illegal = []               # directed edges present in the DB that the raw tags say a car may not drive
        for eid, way_id, u, v, rev, oneway, access, length in db.execute('SELECT eid, way_id, u, v, rev, oneway, access, length_m FROM edges'):
            tags = json.loads(oneway)
            tags.update(json.loads(access))
            fwd, bwd = _drivable_forward(tags)
            if not _allowed(tags) or not (bwd if rev else fwd):
                self.illegal.append(eid)
                continue
            self.edges[eid] = (u, v, way_id, rev)
            self.length[eid] = length
            self.out.setdefault(u, []).append(eid)
        db.close()

    def twin(self, eid):
        """the opposite-direction edge of the same OSM edge, if the DB has one"""
        u, v, way, rev = self.edges[eid]
        for e in self.out.get(v, []):
            u2, v2, w2, r2 = self.edges[e]
            if v2 == u and w2 == way and r2 != rev:
                return e
        return None

    def reachable(self, from_edge, targets, from_along=0.0):
        """(reachable, edges_on_shortest_path_or_None). `targets` = {directed edge id: position of the destination along that edge in metres}
        (None = anywhere on it; a plain set means that for every edge). Reaching a target means ENTERING it at its start, or - when the car
        is already on it - the destination still being AHEAD (from_along <= position): a car past the destination cannot turn back.
        Start is from_edge's downstream node with the no-immediate-reversal rule applied to from_edge."""
        tg = dict(targets) if isinstance(targets, dict) else {t: None for t in targets}
        if from_edge in tg and (tg[from_edge] is None or from_along <= tg[from_edge] + 1e-6):
            return True, [from_edge]
        if from_edge not in self.edges:
            return False, None
        parent = {from_edge: None}
        q = deque([from_edge])
        while q:
            e = q.popleft()
            u, v, _w, _r = self.edges[e]
            for n in self.out.get(v, []):
                nu, nv, _nw, _nr = self.edges[n]
                if nv == u and nu == v:
                    continue                                            # immediate reversal: not allowed
                if n in tg:                                             # entering a target edge at its start
                    path, c = [n], e
                    while c is not None:
                        path.append(c); c = parent[c]
                    return True, path[::-1]
                if n in parent:
                    continue
                parent[n] = e
                q.append(n)
        return False, None

    def targets_for(self, edge_id, along=None):
        """{edge: position} for the destination edge and its twin. For a destination `along` metres into edge_id, the twin runs the other
        way, so the same point is length - along into the twin."""
        t = {edge_id: along}
        if edge_id in self.edges:
            tw = self.twin(edge_id)
            if tw is not None:
                t[tw] = None if along is None else max(0.0, self.length[tw] - along)
        return t
