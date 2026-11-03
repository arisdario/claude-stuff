"""  python3 tesla/nav/detector_compare.py unused bf.sqlite trip.json [path to a clone of openpilotkr/openpilot]
Run three detectors on the same road geometry and score them (and their combinations) against what the driver did.
  OLD   = our own 30 m-baseline Menger detector (tesla/nav/own_detector.py), v = sqrt(1.85/k)
  CC    = CurveCue engine port (tesla/nav/curvecall_engine.py)
  MAPD  = openpilotkr mapd NodesData spline curvature (unmodified, scipy), a_lat 2.3
Driver evidence (fixed rule, set before looking): a curve counts as 'driver slowed' if the speed drops >= 12 km/h between 400..100 m before the bend and the slowest point
from 30 m before to 120 m into it, OR the slowest point is within 10 km/h of the curve speed."""
import sys, types, json, math, importlib.util
sys.path.insert(0, '.')
import numpy as np
import tesla.nav
from tesla.nav import curvecall_engine as E, geo, real_drive_validate as R
from tesla.nav.routegraph import Graph
base = (sys.argv[4] if len(sys.argv) > 4 else '/home/user/openpilotkr/openpilot') + '/selfdrive'
def pkg(n, p):
    m = types.ModuleType(n); m.__path__ = [p]; sys.modules[n] = m
pkg('selfdrive', base); pkg('selfdrive.mapd', base + '/mapd'); pkg('selfdrive.mapd.lib', base + '/mapd/lib')
hw = types.ModuleType('selfdrive.hardware'); hw.EON = False; sys.modules['selfdrive.hardware'] = hw
from selfdrive.mapd.lib import NodesData as N
from tesla.nav import own_detector as OWN

MIN_R = 200.0           # 'moderate or tighter' for every detector

def det_old(P, cum):
    return [dict(a=c['start_m'], b=c['end_m'], r=c['radius_m'], vc=c['v_curve']) for c in OWN.find_curves_own(P, cum, 0.0, cum[-1], 1.85) if c['radius_m'] < MIN_R]

def det_cc(P, cum):
    return [dict(a=c['distance_from_start'], b=c['end_distance'], r=c['min_radius'], vc=math.sqrt(1.85 * c['min_radius'])) for c in E.analyze_route(P) if c['min_radius'] < MIN_R]

def det_mapd(P, cum):
    pts = np.radians(np.array(P, dtype=float))
    v, dp, dn, dr, b = N.node_calculations(pts)
    curv, ds = N.spline_curvature_calculations(v, dp)
    out = []
    for s in N.speed_limits_for_curvatures_data(curv, ds):
        r = s[2] ** 2 / N._MAX_LAT_ACC
        if r < MIN_R and s[1] - s[0] >= 0:
            out.append(dict(a=s[0], b=s[1], r=r, vc=math.sqrt(1.85 * r)))
    return out

def clusters(dets, gap=40.0):
    items = sorted(((c['a'], c['b'], name, c) for name, cs in dets.items() for c in cs), key=lambda x: x[0])
    out = []
    for a, b, name, c in items:
        if out and a <= out[-1]['b'] + gap:
            out[-1]['b'] = max(out[-1]['b'], b); out[-1]['by'][name] = c
        else:
            out.append(dict(a=a, b=b, by={name: c}))
    return out

def evidence(pv, a, vc_kmh):
    before = [v for p, v in pv if a - 400 <= p <= a - 100]
    inside = [v for p, v in pv if a - 30 <= p <= a + 120]
    if len(before) < 2 or not inside:
        return None, None, None
    drop, vmin = max(before) - min(inside), min(inside)
    return (drop >= 12.0 or vmin <= vc_kmh + 10.0), max(before), min(inside)

if __name__ == '__main__':
    g = Graph(sys.argv[2]); fx = R.with_heading(R.load(sys.argv[3])); route, _, _ = R.driven_route(g, fx, 300.0)
    P = [(p[0], p[1]) for p in route.points]
    dets = {'OLD': det_old(P, route.cum), 'CC': det_cc(P, route.cum), 'MAPD': det_mapd(P, route.cum)}
    _, rows, _ = R.run(g, fx, route=route)
    pv = [(r[2], r[1] * 3.6) for r in rows if r[2] is not None]
    cl = clusters(dets)
    print('# Three detectors on the DRIVEN route (%.1f km): curves with radius < %.0f m\n' % (route.distance_m / 1000, MIN_R))
    print('| km | OLD | CC | MAPD | min R (m) | driver max before -> min in bend (km/h) | driver slowed? |\n|---|---|---|---|---|---|---|')
    lab = []
    for c in cl:
        rmin = min(x['r'] for x in c['by'].values()); vc = min(x['vc'] for x in c['by'].values()) * 3.6
        sl, vb, vm = evidence(pv, c['a'], vc)
        lab.append(sl)
        print('| %.2f | %s | %s | %s | %.0f | %s | %s |' % (c['a'] / 1000, 'x' if 'OLD' in c['by'] else '', 'x' if 'CC' in c['by'] else '', 'x' if 'MAPD' in c['by'] else '', rmin, '-' if vb is None else '%.0f -> %.0f' % (vb, vm), 'no data' if sl is None else ('YES' if sl else 'no')))
    print()
    known = [i for i, s in enumerate(lab) if s is not None]
    def score(name, pred):
        tp = sum(1 for i in known if pred(cl[i]) and lab[i]); fp = sum(1 for i in known if pred(cl[i]) and not lab[i])
        fn = sum(1 for i in known if not pred(cl[i]) and lab[i])
        print('| %s | %d | %d | %d | %s | %s |' % (name, tp, fp, fn, '-' if tp + fp == 0 else '%.0f%%' % (100.0 * tp / (tp + fp)), '-' if tp + fn == 0 else '%.0f%%' % (100.0 * tp / (tp + fn))))
    print('Scored on the %d curves with driver evidence (%d slowed):\n' % (len(known), sum(1 for i in known if lab[i])))
    print('| detector / combination | real slow-downs announced | announced but driver did not slow | real slow-downs missed | precision | recall |\n|---|---|---|---|---|---|')
    for n in ('OLD', 'CC', 'MAPD'):
        score(n, lambda c, n=n: n in c['by'])
    score('OLD or CC (union)', lambda c: 'OLD' in c['by'] or 'CC' in c['by'])
    score('any two agree', lambda c: len(c['by']) >= 2)
    score('all three', lambda c: len(c['by']) == 3)
    score('all of the above (union)', lambda c: True)

    # null baseline: the same 'driver slowed' rule applied every 250 m along the route, away (> 400 m) from every detected curve (any detector): how often does it fire for no curve at all?
    allc = [(c['a'], c['b']) for c in cl]
    base = [(s, evidence(pv, float(s), 40.0)) for s in range(500, int(route.distance_m) - 200, 250) if all(s < a - 400 or s > b + 400 for a, b in allc)]
    have = [(s, e) for s, e in base if e[0] is not None]
    fired = [s for s, e in have if e[0]]
    print('\nNull baseline (positions >= 400 m from any detected curve, with speed data): the rule fires at %d of %d (%.0f%%) -> chance that 5 of 5 curves look slowed by luck alone ~ %.4f' % (len(fired), len(have), 100.0 * len(fired) / max(1, len(have)), (len(fired) / max(1, len(have))) ** 5))
    print('positions where it fires (km):', [round(s / 1000, 1) for s in fired][:20])
