"""Route validation for the offline navigation prototype: plans routes on a routegraph.sqlite, checks connectivity/oneway/access
independently from the stored raw tags, simulates GPS drives (noise, leaving the route -> recalculation), optionally compares with
a reference route you supply, and writes a Markdown report. Offline; no vehicle, no control.

  python3 -m tesla.nav.validate --pbf area.osm.pbf --build routegraph.sqlite            # build the graph (needs pyrosm; on a PC)
  python3 -m tesla.nav.validate --graph routegraph.sqlite --from "Predeal" --to "Timisu de Sus" \
        [--reference ref.gpx|ref.geojson|ref.csv] [--report report.md]                    # may be repeated: --route A B --route A C
Place names are matched case/diacritic-insensitively against place=* nodes in the data; 'lat,lon' is also accepted."""
import argparse
import json
import math
import random
import re
import sys
import xml.etree.ElementTree as ET

try:
    from . import geo, osm_rules as R
    from .routegraph import Graph, build
    from .router import find_route, find_route_from_edge, NoRoute
    from .oracle import Oracle
    from .instructions import build_steps, junctions, label
    from .matcher import Navigator
except ImportError:
    import geo
    import osm_rules as R
    from routegraph import Graph, build
    from router import find_route, find_route_from_edge, NoRoute
    from oracle import Oracle
    from instructions import build_steps, junctions, label
    from matcher import Navigator


# ------------------------------------------------------------------ connectivity ---------------------------------------
def check_connectivity(graph, route):
    """Problems found in a route (empty list = clean). Re-applies the oneway/access rules to the RAW tags stored per edge,
    independently of how the edge was selected."""
    bad = []
    es = route.edges
    for i, e in enumerate(es):
        tags = graph.raw_tags(e.eid)
        d = R.oneway_dir(tags)
        ok_dir = d == 'both' or (d == 'fwd' and e.rev == 0) or (d == 'rev' and e.rev == 1)
        if not ok_dir:
            bad.append('edge %d (way %s) is driven against its oneway (%s, rev=%d)' % (i, e.way_id, d, e.rev))
        acc, _f, why = R.access_ok(tags)
        if not acc:
            bad.append('edge %d (way %s) is not accessible to cars (%s)' % (i, e.way_id, why))
        if i:
            p = es[i - 1]
            if p.v != e.u:
                bad.append('edges %d->%d are NOT connected by a shared OSM node (%d != %d)' % (i - 1, i, p.v, e.u))
            if p.u == e.v and p.v == e.u:
                bad.append('immediate reversal at edge %d' % i)
            ge, gp = graph.geom(e.eid)[0], graph.geom(p.eid)[-1]
            gap = geo.hav(ge[0], ge[1], gp[0], gp[1])
            if gap > 0.5:
                bad.append('geometry gap of %.1f m between edges %d and %d' % (gap, i - 1, i))
    return bad


# ------------------------------------------------------------------ simulation ------------------------------------------
def _along(points, cum, s):
    import bisect
    s = max(0.0, min(cum[-1], s))
    i = min(len(points) - 2, max(0, bisect.bisect_right(cum, s) - 1))
    seg = cum[i + 1] - cum[i]
    f = 0.0 if seg == 0 else (s - cum[i]) / seg
    lat = points[i][0] + f * (points[i + 1][0] - points[i][0])
    lon = points[i][1] + f * (points[i + 1][1] - points[i][1])
    return lat, lon, geo.bearing(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])


NOISE_MODELS = ('gauss', 'ar1', 'heavy', 'uniform')


def _noise_sampler(model, sigma, rnd):
    """-> f() giving one (north, east) GPS position error in metres. All models have per-axis scale ~sigma. 'gauss' draws EXACTLY as the
    original simulator did (north then east), so existing seeded results are unchanged.
      ar1      temporally correlated error (rho 0.9, marginal sigma): a real receiver's error drifts, it is not white
      heavy    mixture: 90% N(0, 0.7 sigma), 10% N(0, 3 sigma) - occasional large excursions
      uniform  bounded, +-sqrt(3) sigma"""
    if model == 'gauss':
        return lambda: (rnd.gauss(0, sigma), rnd.gauss(0, sigma))
    if model == 'ar1':
        st = [rnd.gauss(0, sigma), rnd.gauss(0, sigma)]
        rho = 0.9
        k = math.sqrt(1.0 - rho * rho)
        def f():
            st[0] = rho * st[0] + k * rnd.gauss(0, sigma)
            st[1] = rho * st[1] + k * rnd.gauss(0, sigma)
            return st[0], st[1]
        return f
    if model == 'heavy':
        def f():
            return tuple(rnd.gauss(0, sigma * (3.0 if rnd.random() < 0.1 else 0.7)) for _ in range(2))
        return f
    if model == 'uniform':
        w = math.sqrt(3.0) * sigma
        return lambda: (rnd.uniform(-w, w), rnd.uniform(-w, w))
    raise ValueError('unknown noise model %r (choose from %s)' % (model, ', '.join(NOISE_MODELS)))


def fixes_along(points, speed=15.0, hz=1.0, noise_m=5.0, heading_noise=4.0, seed=1, t0=0.0, accuracy=None, noise_model='gauss', s0=0.0):
    """GPS fixes (t, lat, lon, heading, speed, accuracy) for driving a polyline at constant speed with GPS noise. The reported accuracy defaults to
    the noise actually injected (a receiver reports its own error), at least 3 m. s0 = distance along the polyline of the first fix (sampling phase);
    the true position of a fix is s0 + (t - t0) * speed. Identical arguments always give identical fixes."""
    if accuracy is None:
        accuracy = max(3.0, noise_m)
    cum = [0.0]
    for i in range(len(points) - 1):
        cum.append(cum[-1] + geo.hav(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]))
    rnd = random.Random(seed)
    sample = _noise_sampler(noise_model, noise_m, rnd)
    out, t, s = [], t0, s0
    m_lat = (math.pi / 180.0) * geo.R_EARTH
    while s <= cum[-1]:
        lat, lon, hdg = _along(points, cum, s)
        k = m_lat * math.cos(math.radians(lat))
        dn, de = sample()
        out.append((t, lat + dn / m_lat, lon + de / k, (hdg + rnd.gauss(0, heading_noise)) % 360.0, speed, accuracy))
        t += 1.0 / hz; s += speed / hz
    return out


def deviation_candidates(graph, route, at_m, leave_m=500.0, limit=12):
    """Wrong-turn scenarios: for each junction past at_m with a legal alternative exit, the route points up to the junction followed by
    the alternative road (continuing greedily straight for leave_m). Yields (points, junction_node, junction_index, wrong_road_edges)."""
    n = 0
    for i in range(1, len(route.edges)):
        if route.edge_offsets[i] < at_m:
            continue
        prev, cur = route.edges[i - 1], route.edges[i]
        alts = [x for x in graph.out_edges(prev.v) if x.eid != cur.eid and not (x.u == prev.v and x.v == prev.u)]
        cut = max(j for j in range(len(route.points)) if route.cum[j] <= route.edge_offsets[i])
        for alt in alts:
            path = list(route.points[:cut + 1])
            chain = []
            e, run = alt, 0.0
            while e is not None and run < leave_m:
                g = graph.geom(e.eid)
                chain.append(e)
                path.extend(g[1:] if path and path[-1] == g[0] else g)
                run += e.length_m
                nxt = [x for x in graph.out_edges(e.v) if not (x.u == e.v and x.v == e.u)]
                e = nxt[0] if nxt else None
            yield path, prev.v, cut, chain
            n += 1
            if n >= limit:
                return


def deviation_path(graph, route, at_m, leave_m=500.0):
    """First wrong-turn scenario (see deviation_candidates), or (None, None)."""
    for path, node, _cut, _chain in deviation_candidates(graph, route, at_m, leave_m):
        return path, node
    return None, None


PROBE_M = 40.0
AMBIGUOUS_FIX_M = 10.0            # parity: a fix with another edge this close is genuinely ambiguous (not a fair test); deliberately NOT the router's window


def _probe_edge(chain, probe_m=PROBE_M):
    """the wrong-road edge ~probe_m past the junction (a car on it has already committed to the wrong road)"""
    run, k = 0.0, 0
    while k < len(chain) - 1 and run < probe_m:
        run += chain[k].length_m
        k += 1
    return chain[k]


def describe_deviation(sd):
    """'wrong-road edge E (way W), junction node J, oracle reachable=R, navigator status=S' - the identifiers every failure must carry"""
    return 'wrong-road edge %s (way %s), junction node %s, oracle reachable=%s, navigator status=%s' % (
        sd['wrong_edge'], sd['wrong_way'], sd['junction_node'], sd['oracle_reachable'], sd['final_status'] or sd['last_status'])


def simulate_deviation(graph, route, at_frac=0.4, speed=15.0, seed=1, noise_m=5.0, oracle=None):
    """Take a wrong turn at a junction, judge the outcome against an INDEPENDENT reachability oracle (nav/oracle.py: its own SQL,
    oneway/access rules and BFS from the wrong road's directed edge - no router, no snapping), then let the navigator detect it and
    recalculate and drive the NEW route. Distinguishes:
      DETECTION_FAILURE    the navigator never noticed it left the route
      RECALC_FAILURE       it noticed, the oracle says the destination IS reachable, but no recalculation happened
      REROUTE_UNREACHABLE  it recalculated although the oracle says the destination is NOT reachable (a teleport/snap-back)
      SNAP_BACK            the new route does not start on the wrong road the car is on
      NO_ROUTE_CORRECT     unreachable, detected, no_route reported honestly
      REROUTED             detected, recalculated from the wrong road, new route legal and followed to the end
    Prefers a wrong turn from which the destination is still reachable. None only if the route has no junction with an alternative."""
    oracle = oracle or Oracle(graph.path)
    dest = (route.goal.lat, route.goal.lon)
    targets = oracle.targets_for(route.edges[-1].eid, route.goal.along_m)
    chosen = None
    for path, node, cut, chain in deviation_candidates(graph, route, route.distance_m * at_frac):
        probe = _probe_edge(chain)
        reach, _ = oracle.reachable(probe.eid, targets)
        if chosen is None or (reach and not chosen[2]):
            chosen = (path, node, reach, chain, probe)
        if reach:
            break
    if chosen is None:
        return None
    path, node, reach, chain, probe = chosen
    nav = Navigator(graph, dest)
    nav.plan(route.points[0][0], route.points[0][1], None)
    first_off = rerouted_at = last_t = None
    detected, last_status, statuses = False, None, {}
    for k, f in enumerate(fixes_along(path, speed=speed, noise_m=noise_m, seed=seed)):
        st = nav.update(*f)
        last_status = st.status
        statuses[st.status] = statuses.get(st.status, 0) + 1
        detected = detected or st.status in ('off_route', 'rerouted', 'no_route')
        if first_off is None and st.off_route_fixes:
            first_off = k
        if st.status == 'rerouted':
            rerouted_at, last_t = k, f[0]
            break
    out = {'junction_node': node, 'wrong_edge': probe.eid, 'wrong_way': probe.way_id, 'oracle_reachable': reach, 'recoverable': reach,
           'detected': detected, 'rerouted': rerouted_at is not None, 'last_status': last_status,
           'detect_fixes': None if rerouted_at is None or first_off is None else rerouted_at - first_off, 'reroutes': nav.reroutes,
           'new_route_km': None, 'new_route_problems': [], 'final_status': None, 'statuses': statuses, 'new_route_first_edge': None}
    if rerouted_at is None:
        out['outcome'] = 'DETECTION_FAILURE' if not detected else ('RECALC_FAILURE' if reach else 'NO_ROUTE_CORRECT')
        return out
    out['new_route_km'] = nav.route.distance_m / 1000.0
    out['new_route_first_edge'] = nav.route.edges[0].eid
    out['new_route_problems'] = check_connectivity(graph, nav.route)
    final = None
    for f in fixes_along(nav.route.points, speed=speed, noise_m=noise_m, seed=seed + 1, t0=last_t + 1.0):
        final = nav.update(*f)
    out['final_status'] = final.status if final else None
    out['reroutes'] = nav.reroutes
    if not reach:
        out['outcome'] = 'REROUTE_UNREACHABLE'
    elif nav.route.edges[0].eid not in {e.eid for e in chain} and out['new_route_first_edge'] not in {e.eid for e in chain}:
        out['outcome'] = 'SNAP_BACK'
    else:
        out['outcome'] = 'REROUTED'
    return out


def parity_check(graph, route, oracle=None, max_edges=300, step_m=150.0):
    """Planner vs oracle on every directed edge near the route, judged at MID-EDGE (not junction vertices):
       (1) from the edge itself  - find_route_from_edge(edge, mid) succeeds  <=>  the oracle says the destination is reachable;
       (2) from a GPS fix at the edge's midpoint with its heading - find_route(...) succeeds <=> reachable, checked only where no other
           edge lies within the snap window (otherwise the fix is genuinely ambiguous and the planner may legitimately pick another).
    Returns {'edges','fix_checks','disagreements'[...]} where each disagreement names the edge, way, junction (upstream) node, both verdicts."""
    oracle = oracle or Oracle(graph.path)
    dest = (route.goal.lat, route.goal.lon)
    targets = oracle.targets_for(route.edges[-1].eid, route.goal.along_m)
    seen, edges = set(), []
    n = len(route.points)
    for i in range(0, n, max(1, int(step_m / max(1.0, route.distance_m / max(1, n))))):
        for sn in graph.snap(route.points[i][0], route.points[i][1], None, 300.0, limit=40):
            if sn.edge.eid not in seen:
                seen.add(sn.edge.eid); edges.append(sn.edge)
        if len(edges) >= max_edges:
            break
    res = {'edges': 0, 'fix_checks': 0, 'disagreements': []}
    for e in edges[:max_edges]:
        reach, _ = oracle.reachable(e.eid, targets, e.length_m * 0.5)
        res['edges'] += 1
        try:
            find_route_from_edge(graph, e, e.length_m * 0.5, dest)
            ok = True
        except NoRoute:
            ok = False
        if ok != reach:
            res['disagreements'].append({'kind': 'from-edge', 'edge': e.eid, 'way': e.way_id, 'junction': e.u, 'oracle': reach, 'planner': ok})
        g = graph.geom(e.eid)
        mid = _along(g, [0.0] + [sum(geo.hav(g[j][0], g[j][1], g[j + 1][0], g[j + 1][1]) for j in range(k + 1)) for k in range(len(g) - 1)], e.length_m * 0.5)
        near = [s for s in graph.snap(mid[0], mid[1], mid[2], 150.0, limit=16) if s.edge.eid != e.eid]
        if any(s.dist_m <= AMBIGUOUS_FIX_M for s in near):
            continue                                                             # ambiguous fix: not a fair test of the planner
        res['fix_checks'] += 1
        try:
            find_route(graph, (mid[0], mid[1]), dest, mid[2])
            ok2 = True
        except NoRoute:
            ok2 = False
        if ok2 != reach:
            res['disagreements'].append({'kind': 'from-fix', 'edge': e.eid, 'way': e.way_id, 'junction': e.u, 'oracle': reach, 'planner': ok2})
    return res


# ------------------------------------------------------------------ reference comparison --------------------------------
def _dist_to_polyline(lat, lon, line):
    return min(geo.project(lat, lon, line[i][0], line[i][1], line[i + 1][0], line[i + 1][1])[0] for i in range(len(line) - 1))


def _sample(points, step_m=25.0):
    out, acc = [points[0]], 0.0
    for i in range(len(points) - 1):
        seg = geo.hav(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1])
        acc += seg
        while acc >= step_m:
            acc -= step_m
            f = 1.0 - (acc / seg if seg else 0.0)
            out.append((points[i][0] + f * (points[i + 1][0] - points[i][0]), points[i][1] + f * (points[i + 1][1] - points[i][1])))
    out.append(points[-1])
    return out


def compare_to_reference(route_pts, ref_pts, within_m=30.0):
    """Both directions (route->reference and reference->route) so a missing detour or an extra one both show up."""
    a = [_dist_to_polyline(p[0], p[1], ref_pts) for p in _sample(route_pts)]
    b = [_dist_to_polyline(p[0], p[1], route_pts) for p in _sample(ref_pts)]
    lr, lf = geo.polyline_length(route_pts), geo.polyline_length(ref_pts)
    return {'route_km': lr / 1000.0, 'reference_km': lf / 1000.0, 'length_diff_pct': 100.0 * (lr - lf) / lf if lf else None,
            'route_to_ref_mean_m': sum(a) / len(a), 'route_to_ref_max_m': max(a), 'ref_to_route_max_m': max(b),
            'pct_route_within': 100.0 * sum(1 for d in a if d <= within_m) / len(a), 'pct_ref_within': 100.0 * sum(1 for d in b if d <= within_m) / len(b)}


def load_reference(path):
    """Reference polyline as [(lat, lon)] from .gpx (trkpt/rtept), .geojson/.json (LineString/MultiLineString/Feature) or .csv (lat,lon)."""
    low = path.lower()
    if low.endswith('.gpx'):
        root = ET.parse(path).getroot()
        pts = [(float(e.get('lat')), float(e.get('lon'))) for e in root.iter() if e.tag.split('}')[-1] in ('trkpt', 'rtept')]
    elif low.endswith('.json') or low.endswith('.geojson'):
        gj = json.load(open(path))
        def lines(o):
            if o.get('type') == 'FeatureCollection':
                for f in o['features']:
                    for x in lines(f): yield x
            elif o.get('type') == 'Feature':
                for x in lines(o['geometry']): yield x
            elif o.get('type') == 'LineString':
                yield o['coordinates']
            elif o.get('type') == 'MultiLineString':
                for c in o['coordinates']: yield c
        pts = [(c[1], c[0]) for ln in lines(gj) for c in ln]
    else:
        pts = []
        for ln in open(path):
            m = re.match(r'\s*(-?\d+(?:\.\d+)?)\s*[,;\s]\s*(-?\d+(?:\.\d+)?)', ln)
            if m:
                pts.append((float(m.group(1)), float(m.group(2))))
    if len(pts) < 2:
        raise ValueError('reference route %s has fewer than 2 points' % path)
    return pts


# ------------------------------------------------------------------ place resolution ------------------------------------
def resolve(graph, text):
    """-> (lat, lon, description, candidates). 'lat,lon' or a place name from the data (all candidates are returned, never hidden)."""
    m = re.match(r'^\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*$', text)
    if m:
        return float(m.group(1)), float(m.group(2)), 'coordinates', []
    kind, rows = graph.find_places(text)
    if not rows:
        raise NoRoute('place %r not found in the data (try lat,lon)' % text)
    name, place, lat, lon = rows[0]
    return lat, lon, '%s (%s, %s match)' % (name, place, kind), rows


def diagnose_no_route(graph, a, b):
    """Why is there no route? Compares the road components of the snapped start and destination (direction ignored)."""
    sa = graph.snap(a[0], a[1], None, 300.0)
    sb = graph.snap(b[0], b[1], None, 300.0)
    if not sa or not sb:
        return 'the %s is not within 300 m of any drivable road in the data' % ('start' if not sa else 'destination')
    na, ca = graph.component_size(sa[0].edge.u)
    nb, cb = graph.component_size(sb[0].edge.u)
    ea, eb = sa[0].edge, sb[0].edge
    same = graph.same_component(ea.u, eb.u)
    if same is False:
        return ('start (%s, %d nodes%s) and destination (%s, %d nodes%s) are in DIFFERENT disconnected road components: the data has no '
                'connection between them (clipped extract or a missing road), this is not a routing failure' % (
                    ea.name or ea.highway, na, '+' if ca else '', eb.name or eb.highway, nb, '+' if cb else ''))
    if same is None:
        return 'no route found and the road components are too large to compare (search cap reached): check oneway/access around both ends'
    return ('start and destination are in the SAME connected road component but no legal directed path exists: oneway/access restrictions '
            'block every way (check the oneway tags around both ends)')


# ------------------------------------------------------------------ validation of one route ------------------------------
def validate_pair(graph, start, goal, reference=None, label_a='A', label_b='B', speed=15.0, seed=1):
    r = {'from': label_a, 'to': label_b, 'problems': [], 'warnings': []}
    a = resolve(graph, start) if isinstance(start, str) else (start[0], start[1], 'coordinates', [])
    b = resolve(graph, goal) if isinstance(goal, str) else (goal[0], goal[1], 'coordinates', [])
    r['start'], r['goal'] = a, b
    try:
        route = find_route(graph, (a[0], a[1]), (b[0], b[1]))
    except NoRoute as e:
        r['problems'].append('NO ROUTE: %s' % e)
        try:
            r['problems'].append('diagnosis: ' + diagnose_no_route(graph, (a[0], a[1]), (b[0], b[1])))
        except Exception as ex:
            r['warnings'].append('could not diagnose: %s' % ex)
        return r, None
    r['route'] = route
    r['warnings'] += route.warnings
    r['problems'] += check_connectivity(graph, route)
    steps = build_steps(graph, route)
    r['steps'] = steps
    r['junctions'] = junctions(graph, route)
    r['straight_km'] = geo.hav(a[0], a[1], b[0], b[1]) / 1000.0
    if r['straight_km'] > 0.2 and route.distance_m / 1000.0 / r['straight_km'] > 3.0:
        r['warnings'].append('route is %.1fx the straight-line distance (check connectivity/oneway around the endpoints)' % (route.distance_m / 1000.0 / r['straight_km']))
    # reverse direction (not expected to be identical: oneway streets, turn costs)
    try:
        rev = find_route(graph, (b[0], b[1]), (a[0], a[1]))
        r['reverse_km'] = rev.distance_m / 1000.0
        r['problems'] += ['reverse route: ' + p for p in check_connectivity(graph, rev)]
    except NoRoute as e:
        r['reverse_km'] = None
        r['warnings'].append('no reverse route: %s' % e)
    # GPS simulation 1: drive the whole route with noise -> must stay on route, no reroute, must arrive
    nav = Navigator(graph, (b[0], b[1]))
    nav.plan(route.points[0][0], route.points[0][1], None)
    fx = fixes_along(route.points, speed=speed, seed=seed)
    stat = {}
    for f in fx:
        s = nav.update(*f)
        stat[s.status] = stat.get(s.status, 0) + 1
    r['sim_on_route'] = {'fixes': len(fx), 'statuses': stat, 'reroutes': nav.reroutes}
    if nav.reroutes or stat.get('off_route'):
        r['problems'].append('simulated on-route drive triggered %d reroute(s) / %d off-route fixes (false positive)' % (nav.reroutes, stat.get('off_route', 0)))
    if 'arrived' not in stat:
        r['problems'].append('simulated drive never reached "arrived"')
    # GPS simulation 2: leave the route at a junction -> must detect, recalculate, follow the new route validly to the end
    sd = None
    for frac in (0.4, 0.2, 0.0):                  # prefer a junction well into the route, but take an earlier one rather than skip
        sd = simulate_deviation(graph, route, frac, speed, seed + 1)
        if sd is not None:
            sd['from_fraction'] = frac
            break
    r['sim_deviation'] = sd
    if sd is None:
        r['warnings'].append('no junction with a legal alternative exit anywhere on the route: deviation test skipped')
    else:
        tag = describe_deviation(sd)
        oc = sd['outcome']
        if oc == 'DETECTION_FAILURE':
            r['problems'].append('DETECTION FAILURE - left the route and the navigator never noticed: ' + tag)
        elif oc == 'RECALC_FAILURE':
            r['problems'].append('RECALCULATION FAILURE - detected, destination reachable per the oracle, but no recalculation: ' + tag)
        elif oc == 'REROUTE_UNREACHABLE':
            r['problems'].append('SNAP-BACK - recalculated although the oracle says the destination is unreachable from the wrong road: ' + tag)
        elif oc == 'SNAP_BACK':
            r['problems'].append('SNAP-BACK - the new route does not start on the wrong road the car is on: ' + tag)
        elif oc == 'NO_ROUTE_CORRECT':
            r['warnings'].append('every wrong turn after 40%% of the route leads somewhere the oracle says the destination cannot be reached from; detection verified, '
                                 'no_route correctly reported (%s)' % tag)
        else:
            r['problems'] += ['after reroute: ' + p + ' (%s)' % tag for p in sd['new_route_problems']]
            if sd['final_status'] not in ('arrived', 'on_route'):
                r['problems'].append('after recalculation the new route was not followed to the end: ' + tag)
            if sd['reroutes'] != 1:
                r['problems'].append('expected exactly 1 recalculation, got %d: %s' % (sd['reroutes'], tag))
    pc = parity_check(graph, route)
    r['parity'] = pc
    for d in pc['disagreements']:
        r['problems'].append('PARITY - %s edge %d (way %s), upstream node %d: oracle reachable=%s but planner %s' % (
            d['kind'], d['edge'], d['way'], d['junction'], d['oracle'], 'found a route' if d['planner'] else 'found none'))
    if reference:
        r['reference'] = compare_to_reference(route.points, reference)
    return r, route


# ------------------------------------------------------------------ report ----------------------------------------------
def _road_sequence(route):
    seq = []
    n = len(route.edges)
    for i, e in enumerate(route.edges):
        a = route.start.along_m if i == 0 else 0.0
        b = route.goal.along_m if i == n - 1 else e.length_m
        if n == 1:
            a, b = route.start.along_m, route.goal.along_m
        d = max(0.0, b - a)
        lab = label(e)
        if seq and seq[-1][0] == lab:
            seq[-1][1] += d
        else:
            seq.append([lab, d, e.highway])
    return seq


def render_report(results, graph):
    L = ['# Offline navigation - route validation report', '',
         'Graph: routegraph schema %s, turn restrictions: **%s**. Planning only - no vehicle control.' % (graph.meta.get('schema_version'), graph.meta.get('turn_restrictions')), '']
    rej = json.loads(graph.meta.get('rejected', '{}'))
    if rej:
        L.append('Edges excluded by access/oneway rules while building the graph: ' + ', '.join('%s x%d' % kv for kv in sorted(rej.items())) + '.')
        L.append('')
    for r, route in results:
        L += ['## %s -> %s' % (r['from'], r['to']), '']
        if route is None:
            L += ['**%s**' % p for p in r['problems']] + ['']
            continue
        L.append('- from: %s at %.5f, %.5f; to: %s at %.5f, %.5f' % (r['start'][2], r['start'][0], r['start'][1], r['goal'][2], r['goal'][0], r['goal'][1]))
        for who, key in (('from', 'start'), ('to', 'goal')):
            if len(r[key][3]) > 1:
                L.append('- %s: %d candidate places matched, using the highest-ranked: ' % (who, len(r[key][3])) + '; '.join('%s (%s)' % (c[0], c[1]) for c in r[key][3][:6]))
        L.append('- **distance %.2f km**, estimated drive time %.0f min (road-class speeds), straight line %.2f km (x%.2f)' % (
            route.distance_m / 1000.0, route.duration_s / 60.0, r['straight_km'], route.distance_m / 1000.0 / r['straight_km'] if r['straight_km'] else 0))
        if r.get('reverse_km') is not None:
            L.append('- reverse route %.2f km (differences are expected: oneway streets, turn costs)' % r['reverse_km'])
        L.append('- %d directed edges, %d instruction steps, %d junction nodes passed' % (len(route.edges), len(r['steps']), len(r['junctions'])))
        L += ['', '### Roads in order', '', '| # | road | class | km |', '|---|---|---|---|']
        for i, (lab, d, hw) in enumerate(_road_sequence(route), 1):
            L.append('| %d | %s | %s | %.2f |' % (i, lab, hw, d / 1000.0))
        L += ['', '### Instructions', '']
        for s in r['steps']:
            L.append('- %5.2f km  %s%s' % (s.start_m / 1000.0, s.text, '' if s.kind in ('depart', 'arrive') else '  (node %s, %s exits)' % (s.node, s.exits)))
        dec = [j for j in r['junctions'] if j['exits'] >= 2]
        L += ['', '### Junctions with alternatives (%d of %d nodes passed)' % (len(dec), len(r['junctions'])), '',
              '| at km | OSM node | legal exits | turn | from | to |', '|---|---|---|---|---|---|']
        for j in dec[:80]:
            L.append('| %.2f | %s | %d | %+.0f deg | %s | %s |' % (j['at_m'] / 1000.0, j['node'], j['exits'], j['angle'], j['from'], j['to']))
        if len(dec) > 80:
            L.append('| ... | %d more | | | | |' % (len(dec) - 80))
        L += ['', '### Checks', '']
        L.append('- connectivity (shared OSM node between every pair of edges, geometry continuity, no reversals, oneway + access re-checked from raw tags): **%s**' %
                 ('PASS' if not [p for p in r['problems'] if 'edge' in p or 'geometry' in p or 'reversal' in p] else 'FAIL'))
        so = r['sim_on_route']
        L.append('- simulated noisy GPS drive (%d fixes, 5 m noise): %s, reroutes %d -> **%s**' % (so['fixes'], so['statuses'], so['reroutes'], 'PASS' if not so['reroutes'] and 'arrived' in so['statuses'] else 'FAIL'))
        sd = r['sim_deviation']
        if sd is None:
            L.append('- leave-the-route test: skipped (no suitable junction)')
        else:
            good = sd['outcome'] in ('REROUTED', 'NO_ROUTE_CORRECT')
            L.append('- leave-the-route test: %s -> **%s**' % (describe_deviation(sd), 'PASS' if good else 'FAIL (%s)' % sd['outcome']))
            L.append('  - outcome %s; detected=%s, recalculated=%s%s, new route %s km, final status %s' % (
                sd['outcome'], sd['detected'], sd['rerouted'], '' if sd['detect_fixes'] is None else ' after %d fixes' % sd['detect_fixes'],
                '%.2f' % sd['new_route_km'] if sd['new_route_km'] else '-', sd['final_status']))
        pc = r.get('parity')
        if pc is not None:
            L.append('- independent oracle vs planner (%d directed edges near the route from-edge, %d mid-edge GPS fixes): **%s**' % (
                pc['edges'], pc['fix_checks'], 'PASS' if not pc['disagreements'] else 'FAIL (%d disagreements)' % len(pc['disagreements'])))
        if 'reference' in r:
            c = r['reference']
            L.append('- vs reference route: %.2f km vs %.2f km (%+.1f%%); route->reference mean %.0f m, max %.0f m; reference->route max %.0f m; %.0f%% of the route and %.0f%% of the reference within 30 m' % (
                c['route_km'], c['reference_km'], c['length_diff_pct'], c['route_to_ref_mean_m'], c['route_to_ref_max_m'], c['ref_to_route_max_m'], c['pct_route_within'], c['pct_ref_within']))
        else:
            L.append('- reference comparison: **not performed** (no trusted reference route supplied; pass --reference)')
        L += ['']
        if r['warnings']:
            L += ['Warnings:'] + ['- ' + w for w in r['warnings']] + ['']
        L += ['Connectivity problems: ' + ('none' if not r['problems'] else ''), ''] + ['- **%s**' % p for p in r['problems']] + ['']
    L += ['## Limitations', '',
          '- Turn-restriction relations (no_left_turn, only_straight_on, ...) are not in the graph: a route may contain a turn a sign forbids.',
          '- Time-conditional access, vehicle weight/height limits and tolls are not modelled; travel times use class/maxspeed speeds, not traffic.',
          '- Advisory only: nothing here is connected to steering or braking.']
    return '\n'.join(L) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pbf'); ap.add_argument('--build', metavar='GRAPH.sqlite'); ap.add_argument('--bbox')
    ap.add_argument('--graph'); ap.add_argument('--from', dest='frm'); ap.add_argument('--to')
    ap.add_argument('--route', nargs=2, action='append', metavar=('FROM', 'TO'))
    ap.add_argument('--reference'); ap.add_argument('--report', default='route_report.md')
    a = ap.parse_args(argv)
    if a.build:
        if not a.pbf:
            ap.error('--build needs --pbf')
        bbox = [float(x) for x in a.bbox.split(',')] if a.bbox else None
        print(build(a.pbf, a.build, bbox))
        if not a.graph:
            a.graph = a.build
    if not a.graph:
        ap.error('need --graph (or --build)')
    pairs = list(a.route or [])
    if a.frm and a.to:
        pairs.append((a.frm, a.to))
    if not pairs:
        return 0
    g = Graph(a.graph)
    ref = load_reference(a.reference) if a.reference else None
    results = [validate_pair(g, x, y, ref, x, y) for x, y in pairs]
    rep = render_report(results, g)
    open(a.report, 'w').write(rep)
    print(rep)
    return 1 if any(r['problems'] for r, _ in results) else 0


if __name__ == '__main__':
    sys.exit(main())
