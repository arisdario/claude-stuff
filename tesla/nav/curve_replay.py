"""Replay a planned route through the curve-ahead predictor with noisy GPS and report, per REAL curve of the route (from its clean geometry):
distance at which it was first announced, radius, curve speed, approach speed, distance at which slowing should begin; plus false slowdowns.
  python3 -m tesla.nav.curve_replay --graph bf.sqlite --route "București" "Fundulea" --speed 25 --out report.md
Advisory only."""
import argparse
import sys

try:
    from . import curveahead as C, validate as V
    from .routegraph import Graph
    from .router import find_route
except ImportError:
    import curveahead as C, validate as V
    from routegraph import Graph
    from router import find_route

MATCH_M = 40.0


def replay(graph, route, dest, speed=25.0, noise=5.0, seed=1, decel=C.DECEL, a_lat=C.A_LAT):
    truth = C.find_curves(route.points, route.cum, 0.0, route.cum[-1], a_lat)
    for c in truth:
        c['s'] = c['dist_m']
        c['s0'], c['s1'] = c['start_m'], c['end_m']
    ca = C.CurveAhead(graph, dest, a_lat=a_lat, decel=decel)
    ca.start(route.points[0][0], route.points[0][1], None)
    first, false_fix, adv_fix, n, lowconf = {}, [], 0, 0, 0
    ann, dist_hist = {}, {}
    errs, bad_hi, hi, reroutes0 = [], 0, 0, 0
    fx = V.fixes_along(route.points, speed=speed, noise_m=noise, seed=seed)
    t0 = fx[0][0]
    for f in fx:
        msg = ca.update(f[0], f[1], f[2], f[3], f[4], f[5])
        n += 1
        if ca.nav.reroutes:
            reroutes0 = ca.nav.reroutes
        if msg['route_progress_m'] is not None and not reroutes0:       # progress error is only defined against the ORIGINAL route
            e = abs(msg['route_progress_m'] - (f[0] - t0) * speed)
            errs.append(e)
            if msg['confidence'] == 'high':
                hi += 1
                bad_hi += e > MATCH_M
        if msg['confidence'] != 'high':
            lowconf += 1
        if msg['route_progress_m'] is None:
            continue
        for a in msg['announce']['active']:
            if a['new']:
                ann[a['id']] = dict(a, at_progress=msg['route_progress_m'], abs=msg['route_progress_m'] + a['distance_m'] + 15.0)
            dist_hist.setdefault(a['id'], []).append(a['distance_m'])
        for r in msg['curves']:
            s_abs = msg['route_progress_m'] + r['dist_m']
            k = next((i for i, c in enumerate(truth) if abs(c['s'] - s_abs) < MATCH_M), None)
            if k is not None and k not in first and msg['confidence'] == 'high':
                first[k] = dict(r, at_fix_progress=msg['route_progress_m'], v_now=speed, announced_dist=r['dist_m'])
        if msg['advised_speed_ms'] is not None:
            adv_fix += 1
            s_abs = msg['route_progress_m'] + msg['curve_dist_m']
            if not any(c['s0'] - MATCH_M <= s_abs <= c['s1'] + MATCH_M for c in truth):       # a long bend's binding point can move as the window slides: judge by the bend's extent
                false_fix.append((round(msg['route_progress_m']), round(s_abs)))
    return truth, first, {'fixes': n, 'advisory_fixes': adv_fix, 'false_slowdown_fixes': len(false_fix), 'false_examples': false_fix[:5], 'low_confidence_fixes': lowconf,
                          'progress_err_median_m': sorted(errs)[len(errs) // 2] if errs else None, 'progress_err_p95_m': sorted(errs)[int(len(errs) * 0.95)] if errs else None, 'progress_err_max_m': max(errs) if errs else None,
                          'announcements': ann, 'dist_hist': dist_hist, 'high_conf_fixes': hi, 'high_conf_wrong_progress': bad_hi, 'reroutes': ca.nav.reroutes}


def report(a, b, route, truth, first, st, speed, decel):
    L = ['# Curve-ahead replay: %s -> %s' % (a, b), '',
         'route %.2f km, replay speed %.0f m/s (%.0f km/h), GPS noise 5 m, A_LAT %.2f m/s^2, DECEL %.2f m/s^2, horizon %.0f m. Advisory only.' % (route.distance_m / 1000.0, speed, speed * 3.6, C.A_LAT, decel, C.HORIZON_M), '',
         '%d real curves (curve speed < %.0f km/h, >= %d samples) in the route geometry; announced with high confidence: **%d**.' % (len(truth), C.CURVE_MAX_MS * 3.6, C.MIN_CURVE_SAMPLES, len(first)), '',
         '| # | at km | radius m | curve km/h | announced at (m before) | approach km/h then | slow-down must start (m before) | start slowing in (m) | early enough? |', '|---|---|---|---|---|---|---|---|---|']
    ok = 0
    for i, c in enumerate(truth):
        r = first.get(i)
        need = max(0.0, (speed * speed - c['v_curve'] ** 2) / (2 * decel)) if speed > c['v_curve'] else 0.0
        if r is None:
            L.append('| %d | %.2f | %.0f | %.0f | NOT ANNOUNCED | | %.0f | | no |' % (i + 1, c['s'] / 1000, c['radius_m'], c['v_curve'] * 3.6, need))
            continue
        early = r['announced_dist'] >= need
        ok += early
        L.append('| %d | %.2f | %.0f | %.0f | %.0f | %.0f | %.0f | %.0f | %s |' % (i + 1, c['s'] / 1000, c['radius_m'], c['v_curve'] * 3.6, r['announced_dist'], r['v_approach'] * 3.6, need, r['begin_slowing_in_m'] if r['begin_slowing_in_m'] is not None else 0, 'yes' if early else 'NO (announced %.0f m late)' % (need - r['announced_dist'])))
    ann = st['announcements']
    per_truth = {}
    for a in ann.values():
        k = next((i for i, c in enumerate(truth) if c['s0'] - MATCH_M <= a['abs'] <= c['s1'] + MATCH_M), None)
        per_truth.setdefault(k, []).append(a['id'])
    dup = sum(len(v) - 1 for k, v in per_truth.items() if k is not None and len(v) > 1)
    spurious = len(per_truth.get(None, []))
    missed = [i + 1 for i in range(len(truth)) if i not in per_truth]
    nonmono = [k for k, h in st['dist_hist'].items() if any(b > a + 25.0 for a, b in zip(h, h[1:]))]
    L += ['', '## Announcements (each curve announced once, distance then updated every fix)', '',
          '%d announcements for %d real curves: **duplicate announcements %d**, announcements not matching a real curve **%d**, real curves never announced **%s**, announced curves whose distance ever grew by > 25 m while approaching **%d**' % (len(ann), len(truth), dup, spurious, missed or 'none', len(nonmono)), '',
          '| id | first announced at route m | curve at route m | distance when announced | radius m | curve speed km/h | distance updates |', '|---|---|---|---|---|---|---|']
    for a in sorted(ann.values(), key=lambda q: q['id']):
        L.append('| %d | %.0f | %.0f | %.0f | %.0f | %.0f | %d |' % (a['id'], a['at_progress'], a['abs'], a['distance_m'], a['radius_m'], a['speed_ms'] * 3.6, len(st['dist_hist'][a['id']])))
    L += ['', 'announced early enough to slow at %.1f m/s^2: **%d / %d**' % (decel, ok, len(truth)),
          'false slowdowns (advisory pointing at a curve that is not in the route geometry): **%d of %d fixes** %s' % (st['false_slowdown_fixes'], st['fixes'], st['false_examples']),
          'road match: progress error median %s m, p95 %s m, max %s m; high-confidence fixes %d, of which progress wrong by > %.0f m (wrong forward geometry): **%d**; recalculations during replay: %d' % tuple(
              [None if st['progress_err_median_m'] is None else round(st[k], 1) for k in ('progress_err_median_m', 'progress_err_p95_m', 'progress_err_max_m')] + [st['high_conf_fixes'], MATCH_M, st['high_conf_wrong_progress'], st['reroutes']]),
          'fixes with advisory active: %d; fixes without high road confidence (no advised speed published): %d' % (st['advisory_fixes'], st['low_confidence_fixes']), '']
    return '\n'.join(L), ok


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--graph', required=True)
    ap.add_argument('--route', nargs=2, required=True)
    ap.add_argument('--speed', type=float, default=25.0)
    ap.add_argument('--decel', type=float, default=C.DECEL)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out')
    a = ap.parse_args(argv)
    g = Graph(a.graph)
    s, d = V.resolve(g, a.route[0]), V.resolve(g, a.route[1])
    route = find_route(g, (s[0], s[1]), (d[0], d[1]))
    truth, first, st = replay(g, route, (d[0], d[1]), a.speed, 5.0, a.seed, a.decel)
    text, ok = report(a.route[0], a.route[1], route, truth, first, st, a.speed, a.decel)
    print(text)
    if a.out:
        open(a.out, 'w').write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
