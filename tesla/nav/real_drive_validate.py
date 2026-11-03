"""Validate the curve announcements against a REAL logged drive (tools' trip.json: points = [t, lat, lon, speed_kmh, limit, ..., accuracy]).
The drive is map-matched on the routing graph (destination = the drive's last position), replayed fix by fix through CurveAhead, and each announced curve is compared with what the
driver actually did. Not ground truth about curves - ground truth about the DRIVER: where a good driver slowed, was a curve announced, and vice versa. Advisory only.
  python3 -m tesla.nav.real_drive_validate --graph bf.sqlite --trip map_20261101_175314.trip.json --out report.md"""
import argparse
import json
import math
import sys

try:
    from . import curveahead as C, geo
    from .routegraph import Graph
    from .router import find_route, Route
except ImportError:
    import curveahead as C, geo
    from routegraph import Graph
    from router import find_route, Route


def load(trip):
    if trip.endswith('.jsonl'):                        # a raw radarweb log: parse it with the project's own trip builder (radartrip.py at the repo root)
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
        import radartrip
        p = radartrip.trip(trip)['points']
    else:
        p = json.load(open(trip))['points']
    return [dict(t=x[0], lat=x[1], lon=x[2], v=x[3] / 3.6, acc=x[7]) for x in p]


def with_heading(fx, min_move=6.0):
    last, hdg, out = None, None, []
    for f in fx:
        if last is not None and geo.hav(last['lat'], last['lon'], f['lat'], f['lon']) >= min_move:
            hdg = geo.bearing(last['lat'], last['lon'], f['lat'], f['lon'])
            last = f
        elif last is None:
            last = f
        out.append(dict(f, hdg=hdg))
    return out


def driven_route(graph, fx, leg_m=1500.0):
    """The route the driver ACTUALLY took: shortest legs between waypoints taken from the GPS trace every ~leg_m (so each leg stays on the driven road), joined into one Route.
    Returns (route, max distance of any trace fix from the route in m, number of joins that were not connected)."""
    wps, acc = [fx[0]], 0.0
    for a, b in zip(fx, fx[1:]):
        acc += geo.hav(a['lat'], a['lon'], b['lat'], b['lon'])
        if acc >= leg_m:
            wps.append(b)
            acc = 0.0
    if wps[-1] is not fx[-1]:
        wps.append(fx[-1])
    legs = [find_route(graph, (a['lat'], a['lon']), (b['lat'], b['lon']), a['hdg']) for a, b in zip(wps, wps[1:])]
    edges, bad = [], 0
    for r in legs:
        for e in r.edges:
            if edges and edges[-1].eid == e.eid:
                continue
            if edges and edges[-1].v != e.u:
                bad += 1
            edges.append(e)
    route = Route(graph, edges, legs[0].start, legs[-1].goal, [])
    worst = max(min(geo.hav(f['lat'], f['lon'], q[0], q[1]) for q in route.points[::4]) for f in fx[::10])
    return route, worst, bad


DR_BACK_TOL_M = 50.0


def _project(route, lat, lon, lo_m, hi_m):
    """progress of the nearest point of the route polyline within [lo_m, hi_m] -> (progress_m, distance_m)"""
    best = (1e18, None)
    for i in range(len(route.points) - 1):
        if route.cum[i + 1] < lo_m:
            continue
        if route.cum[i] > hi_m:
            break
        d, al, _a, _b, _s = geo.project(lat, lon, route.points[i][0], route.points[i][1], route.points[i + 1][0], route.points[i + 1][1])
        if d < best[0]:
            best = (d, route.cum[i] + al)
    return best[1], best[0]


def run(graph, fx, a_lat=C.A_LAT, decel=C.DECEL, route=None):
    """Replay with the DRIVEN route fixed (so the matcher/rerouting is not part of this test): progress = projection of each NEW GPS position onto the route (monotone), dead-reckoned
    with the car's speed between GPS updates (this log only refreshes the position every ~14 s). Confidence is 'high' when the position lies within 15 m of the route."""
    ann, rows, anns = C.CurveAnnouncer(), [], {}
    prog, last_t, last_pos, conf = 0.0, fx[0]['t'], None, 'high'
    for f in fx:
        prog = min(route.distance_m, prog + f['v'] * max(0.0, f['t'] - last_t))          # dead reckoning: add the distance driven since the PREVIOUS FIX (never since the last GPS change)
        last_t = f['t']
        if (f['lat'], f['lon']) != last_pos:
            last_pos = (f['lat'], f['lon'])
            proj, dist = _project(route, f['lat'], f['lon'], prog - 400.0, prog + 400.0)
            if proj is not None:
                conf = 'high' if dist <= 15.0 else 'low'
                if conf == 'high' and proj >= prog - DR_BACK_TOL_M:
                    prog = max(prog, proj) if proj > prog else prog       # a position slightly behind the estimate must not move the car backwards
                    prog = proj if proj > prog - 1e-9 else prog
        curves = C.find_curves(route.points, route.cum, prog, C.HORIZON_M, a_lat)
        r = ann.update(prog, conf, curves)
        tgt, _w = C.target_speed(r['active'], decel) if conf == 'high' else (None, None)
        for a in r['active']:
            if a['new']:
                anns[a['id']] = dict(a, first_t=f['t'], first_progress=prog, v_at_announce=f['v'])
            anns[a['id']].update(last_t=f['t'], last_dist=a['distance_m'], last_progress=prog)
        rows.append((f['t'], f['v'], prog, conf, tgt, tgt is not None and f['v'] > tgt + 0.5, 'replay'))
    return None, rows, anns


def report(graph, fx, route, rows, anns, a_lat, decel):
    L = ['# Real-drive validation of the curve announcements', '',
         '%d GPS fixes, %.0f s, %.1f km driven; map-matched on the routing graph (driven route %.1f km, within 8 m of the trace); %d distinct GPS positions (%.0f s between updates on average); a_lat %.2f m/s^2, decel %.1f m/s^2. Advisory only.' % (
             len(fx), fx[-1]['t'] - fx[0]['t'], sum(geo.hav(a['lat'], a['lon'], b['lat'], b['lon']) for a, b in zip(fx, fx[1:])) / 1000.0, route.distance_m / 1000.0, len(set((f['lat'], f['lon']) for f in fx)), (fx[-1]['t'] - fx[0]['t']) / max(1, len(set((f['lat'], f['lon']) for f in fx))), a_lat, decel), '']
    st = {}
    for r in rows:
        st[r[6]] = st.get(r[6], 0) + 1
    hi = sum(1 for r in rows if r[3] == 'high')
    L += ['fixes with HIGH road confidence (position within 15 m of the driven route): %d of %d (%.0f%%)' % (hi, len(rows), 100.0 * hi / len(rows)), '']
    # driver speed vs progress (only fixes with a progress on the route)
    pv = [(r[2], r[1], r[0]) for r in rows if r[2] is not None and r[3] == 'high']
    L += ['## Announced curves vs what the driver did', '',
          '| id | announcement | radius m | curve km/h | announced m before | driver km/h when announced | driver km/h 100 m before | driver km/h at entry | driver min km/h within the bend | slowed >= 15 km/h approaching? | driver at/below advisory target? |', '|---|---|---|---|---|---|---|---|---|---|---|']
    slowed = below = n = 0
    for k in sorted(anns):
        a = anns[k]
        start = a['first_progress'] + a['distance_m']
        end = a['last_progress'] if a.get('last_progress') is not None else start

        def v_at(s, tol=25.0):
            c = [v for p, v, _t in pv if abs(p - s) <= tol]
            return None if not c else sum(c) / len(c)
        v100, ve = v_at(start - 100.0), v_at(start)
        inside = [v for p, v, _t in pv if start - 5.0 <= p <= start + 60.0]
        vmin = min(inside) if inside else None
        v0 = a['v_at_announce']
        did_slow = v100 is not None and vmin is not None and (v100 - vmin) * 3.6 >= 15.0
        # was the driver at/below the advisory envelope all the way in?  envelope at 100 m before = sqrt(vc^2 + 2*decel*100)
        env = math.sqrt(a['speed_ms'] ** 2 + 2 * decel * 100.0)
        ok = v100 is not None and v100 <= env + 1.5
        n += 1
        slowed += did_slow
        below += ok
        f = lambda x: '-' if x is None else '%.0f' % (x * 3.6)
        L.append('| %d | %s | %.0f | %.0f | %.0f | %s | %s | %s | %s | %s | %s |' % (k, a['text'], a['radius_m'], a['speed_ms'] * 3.6, a['distance_m'] if False else (start - a['first_progress']), f(v0), f(v100), f(ve), f(vmin), 'yes' if did_slow else 'no', 'yes' if ok else 'NO'))
    L += ['', 'curves announced: %d; driver slowed >= 15 km/h into them: **%d**; driver already at/below the advisory envelope 100 m before the bend: **%d**' % (n, slowed, below), '']
    # missed slow-downs: real deceleration events with no announced curve nearby
    ev, i = [], 0
    pts = [(p, v) for p, v, _t in pv]
    while i < len(pts) - 5:
        j = i
        while j + 1 < len(pts) and pts[j + 1][1] <= pts[j][1] + 0.3 and pts[j + 1][0] - pts[i][0] < 400:
            j += 1
        drop = (pts[i][1] - pts[j][1]) * 3.6
        if drop >= 25.0 and pts[j][1] * 3.6 > 15.0:
            ev.append((pts[i][0], pts[j][0], pts[i][1], pts[j][1]))
            i = j + 1
        else:
            i += 1
    cov = [e for e in ev if any(abs((anns[k]['first_progress'] + anns[k]['distance_m']) - e[1]) < 150.0 for k in anns)]
    L += ['## Driver slow-downs (>= 25 km/h within 400 m, ending above 15 km/h) and whether a curve was announced near their end (+-150 m)', '',
          '%d slow-downs, **%d** with an announced curve nearby; the others are traffic, junctions or ramps - listed for inspection:' % (len(ev), len(cov)), '', '| at route km | from km/h | to km/h | announced curve nearby |', '|---|---|---|---|']
    for e in ev:
        near = any(abs((anns[k]['first_progress'] + anns[k]['distance_m']) - e[1]) < 150.0 for k in anns)
        L.append('| %.2f | %.0f | %.0f | %s |' % (e[1] / 1000.0, e[2] * 3.6, e[3] * 3.6, 'yes' if near else 'no'))
    return '\n'.join(L)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', required=True)
    ap.add_argument('--trip', required=True)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    g = Graph(a.graph)
    fx = with_heading(load(a.trip))
    route, worst, bad = driven_route(g, fx)
    print('driven route %.1f km (trace %.1f km); worst GPS-to-route distance %.0f m; unconnected joins %d' % (route.distance_m / 1000.0, sum(geo.hav(a['lat'], a['lon'], b['lat'], b['lon']) for a, b in zip(fx, fx[1:])) / 1000.0, worst, bad))
    ca, rows, anns = run(g, fx, route=route)
    text = report(g, fx, route, rows, anns, C.A_LAT, C.DECEL)
    print(text)
    if a.out:
        open(a.out, 'w').write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
