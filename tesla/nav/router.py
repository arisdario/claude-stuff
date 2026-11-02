"""A* routing over directed OSM edges (state = directed edge). Connectivity is exactly 'v of one edge == u of the next' - the
shared OSM node ID - so two roads that merely cross or pass close together (bridge, underpass, near-miss) are never joined.
Oneway/access are already baked into which directed edges exist (see routegraph/osm_rules). No immediate reversals; turns are
penalised by angle. Planning only: nothing here touches vehicle control."""
import heapq
import itertools

try:
    from . import geo
    from .routegraph import Snap
except ImportError:
    import geo
    from routegraph import Snap

V_MAX = 130.0 / 3.6                 # fastest assumed speed: keeps the straight-line heuristic admissible
TURN_COST_S = ((30.0, 0.0), (60.0, 2.0), (120.0, 6.0), (170.0, 12.0), (181.0, 25.0))   # (max |angle| deg, seconds)
MAX_EXPANSIONS = 3000000
SNAP_WINDOW_M = 10.0               # only edges essentially as near as the nearest one are start/goal candidates: a wider pool lets an
                                   # unreachable destination silently snap to a different road (or the adjacent leg of a switchback)


class NoRoute(Exception):
    pass


def out_bearing(graph, e, span=15.0):
    g = graph.geom(e.eid)
    b = g[-1]
    a = g[-2]
    acc, i = 0.0, len(g) - 1
    while i > 0 and acc < span:
        acc += geo.hav(g[i - 1][0], g[i - 1][1], g[i][0], g[i][1]); i -= 1
        a = g[i]
    return geo.bearing(a[0], a[1], b[0], b[1])


def in_bearing(graph, e, span=15.0):
    g = graph.geom(e.eid)
    a = g[0]
    b = g[1]
    acc, i = 0.0, 0
    while i < len(g) - 1 and acc < span:
        acc += geo.hav(g[i][0], g[i][1], g[i + 1][0], g[i + 1][1]); i += 1
        b = g[i]
    return geo.bearing(a[0], a[1], b[0], b[1])


def turn_angle(graph, e, n):
    """signed turn from edge e onto edge n in degrees (positive = right)"""
    return geo.ang_diff(in_bearing(graph, n), out_bearing(graph, e))


def turn_cost(graph, e, n):
    if e.roundabout and n.roundabout:
        return 0.0
    a = abs(turn_angle(graph, e, n))
    for lim, c in TURN_COST_S:
        if a < lim:
            return c
    return TURN_COST_S[-1][1]


def slice_geom(g, a_m, b_m):
    """points of polyline g between distances a_m and b_m along it (a_m <= b_m)"""
    out, cum = [], 0.0
    started = False
    for i in range(len(g) - 1):
        seg = geo.hav(g[i][0], g[i][1], g[i + 1][0], g[i + 1][1])
        if not started and cum + seg >= a_m:
            f = 0.0 if seg == 0 else (a_m - cum) / seg
            out.append((g[i][0] + f * (g[i + 1][0] - g[i][0]), g[i][1] + f * (g[i + 1][1] - g[i][1])))
            started = True
        if started:
            if cum + seg >= b_m:
                f = 0.0 if seg == 0 else (b_m - cum) / seg
                p = (g[i][0] + f * (g[i + 1][0] - g[i][0]), g[i][1] + f * (g[i + 1][1] - g[i][1]))
                if p != out[-1]:
                    out.append(p)
                return out
            out.append(g[i + 1])
        cum += seg
    return out if len(out) >= 1 else [g[-1]]


class Route(object):
    def __init__(self, graph, edges, start, goal, warnings):
        self.graph, self.edges, self.start, self.goal, self.warnings = graph, edges, start, goal, warnings
        self.points, self.edge_offsets = [], []
        running = 0.0
        n = len(edges)
        for i, e in enumerate(edges):
            g = graph.geom(e.eid)
            a = start.along_m if i == 0 else 0.0
            b = goal.along_m if i == n - 1 else e.length_m
            if n == 1:
                a, b = start.along_m, goal.along_m
            part = slice_geom(g, a, max(a, b))
            self.edge_offsets.append(running)
            if self.points and part and part[0] == self.points[-1]:
                part = part[1:]
            prev = self.points[-1] if self.points else None
            for p in part:
                if prev is not None:
                    running += geo.hav(prev[0], prev[1], p[0], p[1])
                prev = p
            self.points.extend(part)
        self.cum = [0.0]
        for i in range(len(self.points) - 1):
            self.cum.append(self.cum[-1] + geo.hav(self.points[i][0], self.points[i][1], self.points[i + 1][0], self.points[i + 1][1]))
        self.distance_m = self.cum[-1]
        # travel time (no turn / snap penalties): each edge's cost scaled by the fraction actually driven
        t = 0.0
        for i, e in enumerate(edges):
            a = start.along_m if i == 0 else 0.0
            b = goal.along_m if i == n - 1 else e.length_m
            if n == 1:
                a, b = start.along_m, goal.along_m
            t += e.cost_s * max(0.0, b - a) / e.length_m
        self.duration_s = t


def astar(graph, starts, goals):
    """starts/goals: lists of Snap. Returns the list of Edge from the start edge to the goal edge, or raises NoRoute."""
    glat = sum(s.lat for s in goals) / len(goals)
    glon = sum(s.lon for s in goals) / len(goals)
    goal_part = {}
    for s in goals:
        c = s.edge.cost_s * (s.along_m / s.edge.length_m) + s.dist_m / 8.0
        if s.edge.eid not in goal_part or c < goal_part[s.edge.eid][0]:
            goal_part[s.edge.eid] = (c, s)
    best_total, best_path_end, best_goal_edge = float('inf'), None, None
    g_end, parent, edges_by_id = {}, {}, {}
    heap, tick = [], itertools.count()

    def h(e):
        p = graph.geom(e.eid)[-1]
        return geo.hav(p[0], p[1], glat, glon) / V_MAX

    for s in starts:
        e = s.edge
        g0 = e.cost_s * (1.0 - s.along_m / e.length_m) + s.dist_m / 8.0
        if e.eid in goal_part and goal_part[e.eid][1].along_m >= s.along_m:      # start and goal on the same edge, in order
            tot = (goal_part[e.eid][1].along_m - s.along_m) / e.length_m * e.cost_s + s.dist_m / 8.0 + goal_part[e.eid][1].dist_m / 8.0
            if tot < best_total:
                best_total, best_path_end, best_goal_edge = tot, None, e
                parent[('direct', e.eid)] = None
        if g0 < g_end.get(e.eid, float('inf')):
            g_end[e.eid] = g0; parent[e.eid] = None; edges_by_id[e.eid] = e
            heapq.heappush(heap, (g0 + h(e), next(tick), e.eid))
    expansions = 0
    while heap:
        f, _, eid = heapq.heappop(heap)
        if f >= best_total:
            break
        e = edges_by_id[eid]
        if f - h(e) > g_end[eid] + 1e-9:
            continue                                                             # stale heap entry
        expansions += 1
        if expansions > MAX_EXPANSIONS:
            raise NoRoute('search limit reached')
        for n in graph.out_edges(e.v):
            if n.u == e.v and n.v == e.u:
                continue                                                         # no immediate reversal
            c = g_end[eid] + turn_cost(graph, e, n)
            if n.eid in goal_part:
                tot = c + goal_part[n.eid][0]
                if tot < best_total:
                    best_total, best_path_end, best_goal_edge = tot, eid, n
                    edges_by_id[n.eid] = n
            cn = c + n.cost_s
            if cn < g_end.get(n.eid, float('inf')):
                g_end[n.eid] = cn; parent[n.eid] = eid; edges_by_id[n.eid] = n
                heapq.heappush(heap, (cn + h(n), next(tick), n.eid))
    if best_goal_edge is None:
        raise NoRoute('no connected path between the snapped start and destination')
    path = [best_goal_edge]
    cur = best_path_end
    while cur is not None:
        path.append(edges_by_id[cur]); cur = parent[cur]
    path.reverse()
    if best_path_end is None and len(path) == 2 and path[0].eid == path[1].eid:
        path = path[:1]
    return path


def _goals(graph, goal_latlon, warnings):
    if not geo.valid_latlon(*goal_latlon):
        raise NoRoute('invalid coordinates %r' % (goal_latlon,))
    goals = graph.snap(goal_latlon[0], goal_latlon[1], None, 300.0) or graph.snap(goal_latlon[0], goal_latlon[1], None, 800.0)
    if not goals:
        raise NoRoute('destination is not within 800 m of a drivable road')
    best = min(x.dist_m for x in goals)
    goals = [x for x in goals if x.dist_m <= best + SNAP_WINDOW_M]
    if best > 100.0:
        warnings.append('destination is %.0f m from the nearest drivable road' % best)
    return goals


def find_route_from_edge(graph, edge, along_m, goal_latlon):
    """Plan from a car KNOWN to be on a specific directed edge (e.g. the edge tracker's): no start snapping at all, so the route cannot
    begin on a different, merely nearby, road. Raises NoRoute."""
    warnings = []
    goals = _goals(graph, goal_latlon, warnings)
    g = graph.geom(edge.eid)
    p = slice_geom(g, along_m, along_m)[0]
    start = Snap(edge, 0.0, max(0.0, min(along_m, edge.length_m)), p[0], p[1], None)
    path = astar(graph, [start], goals)
    g0 = next(s for s in goals if s.edge.eid == path[-1].eid)
    return Route(graph, path, start, g0, warnings)


def find_route(graph, start_latlon, goal_latlon, heading=None):
    """Plan a route. Snaps the start (with heading, if known, to the carriageway being driven) and the destination to the nearest
    legal directed edges. Raises NoRoute with a reason; never returns an unconnected route."""
    for pt in (start_latlon, goal_latlon):
        if not geo.valid_latlon(*pt):
            raise NoRoute('invalid coordinates %r' % (pt,))
    warnings = []
    starts = graph.snap(start_latlon[0], start_latlon[1], heading, 150.0) or graph.snap(start_latlon[0], start_latlon[1], heading, 400.0)
    if not starts:
        raise NoRoute('start is not within 400 m of a drivable road')
    goals = graph.snap(goal_latlon[0], goal_latlon[1], None, 300.0) or graph.snap(goal_latlon[0], goal_latlon[1], None, 800.0)
    if not goals:
        raise NoRoute('destination is not within 800 m of a drivable road')
    best = min(x.dist_m for x in starts)
    starts = [x for x in starts if x.dist_m <= best + SNAP_WINDOW_M]
    best = min(x.dist_m for x in goals)
    goals = [x for x in goals if x.dist_m <= best + SNAP_WINDOW_M]
    if min(x.dist_m for x in starts) > 100.0:
        warnings.append('start is %.0f m from the nearest drivable road' % min(x.dist_m for x in starts))
    if min(x.dist_m for x in goals) > 100.0:
        warnings.append('destination is %.0f m from the nearest drivable road' % min(x.dist_m for x in goals))
    path = astar(graph, starts, goals)
    s0 = next(s for s in starts if s.edge.eid == path[0].eid)
    g0 = next(s for s in goals if s.edge.eid == path[-1].eid)
    return Route(graph, path, s0, g0, warnings)
