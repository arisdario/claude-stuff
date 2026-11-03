"""Simulated drive with the curve advisory active: a car cruises at a set speed and obeys target_speed() (braking at most DECEL). Reports where it slows and whether it slows into every curve and
NOT on straights. Advisory maths only - nothing here drives a car.
  python3 -m tesla.nav.advisory_drive_test --national national_v1.sqlite --from 44.8565,24.8692 --to 45.0997,24.3693 --speeds 22,28 --out report.md --png profile.png"""
import argparse
import sys

try:
    from . import curveahead as C, national_route as NR
except ImportError:
    import curveahead as C
    import national_route as NR


def run(route, v):
    sim = C.simulate_follow(route.points, route.cum, v)
    curves = [c for c in C.find_curves(route.points, route.cum, 0.0, route.cum[-1]) if C.SEVERITY_ORDER.index(c['severity']) >= 1]
    return sim, curves


def report(route, roads, name, speeds, results):
    L = ['# Simulated drive with the curve advisory: %s' % name, '',
         'route %.1f km (%d points, road classes: %s), built from the national map-curve database; detector: %s; a_lat %.2f m/s^2, decel %.1f m/s^2. Advisory only.' % (route.distance_m / 1000.0, len(route.points), ', '.join(roads), C.DEFAULT_DETECTOR, C.A_LAT, C.DECEL), '']
    for v, (sim, curves) in zip(speeds, results):
        tr = sim['trace']
        slow_len = sum(1 for t in tr if t[2] is not None and t[1] < v - 0.3)
        late = [e for e in sim['entries'] if e[2] > e[1] + 0.75]
        L += ['## Cruise %.0f km/h (%.0f m/s)' % (v * 3.6, v), '',
              '- curves of moderate severity or tighter on the route: **%d** (%d also confirmed by the CurveCue engine)' % (len(curves), sum(1 for c in curves if c.get('confirmed'))),
              '- curves entered: %d; entered above curve speed (+0.75 m/s): **%d**; max braking used **%.2f m/s^2**; lowest speed reached **%.0f km/h**' % (len(sim['entries']), len(late), sim['max_decel'], sim['min_speed'] * 3.6),
              '- fraction of the drive spent below cruise speed because of the advisory: **%.0f%%** (the rest is straight road at full speed)' % (100.0 * slow_len / max(1, len(tr))), '',
              '| # | at km | announcement | radius m | curve km/h | confirmed | speed at bend entry km/h | slowed? |', '|---|---|---|---|---|---|---|---|']
        ent = {round(e[0]): e for e in sim['entries']}
        for i, c in enumerate(curves):
            e = min(sim['entries'], key=lambda x: abs(x[0] - (c['start_m'])), default=None)
            ve = None if e is None or abs(e[0] - c['start_m']) > 60.0 else e[2]
            L.append('| %d | %.2f | %s | %.0f | %.0f | %s | %s | %s |' % (i + 1, c['start_m'] / 1000.0, c['text'], c['radius_m'], c['v_curve'] * 3.6, 'yes' if c.get('confirmed') else 'own only',
                                                                           '-' if ve is None else '%.0f' % (ve * 3.6), '-' if ve is None else ('yes (%.0f -> %.0f)' % (v * 3.6, ve * 3.6) if ve < v - 0.3 else 'no, was already slow enough')))
        L.append('')
    return '\n'.join(L)


def plot(route, results, speeds, png, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    v = speeds[-1]
    sim, curves = results[-1]
    tr = sim['trace']
    s = [t[0] / 1000.0 for t in tr]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(13, 6.2), sharex=True, gridspec_kw={'height_ratios': [3, 1.2]}, facecolor='#fcfcfb')
    a1.plot([0, route.distance_m / 1000.0], [v * 3.6] * 2, color='#2a78d6', lw=2, label='Cruise speed (%.0f km/h)' % (v * 3.6))
    a1.plot(s, [t[1] * 3.6 for t in tr], color='#eb6834', lw=2, label='Speed following the advisory')
    a1.fill_between(s, [t[1] * 3.6 for t in tr], v * 3.6, color='#eb6834', alpha=0.25, linewidth=0)
    for c in curves:
        a1.axvline(c['start_m'] / 1000.0, color='#52514e', lw=0.5, alpha=0.4)
    a1.set_ylabel('km/h'); a1.set_ylim(0, v * 3.6 * 1.15); a1.set_title(title, loc='left', fontsize=12); a1.legend(loc='lower right', fontsize=8, frameon=False)
    a2.fill_between(s, 0, [v * 3.6 - t[1] * 3.6 for t in tr], color='#eb6834', alpha=0.8, linewidth=0)
    a2.set_ylabel('slow down by (km/h)', fontsize=9); a2.set_xlabel('distance along route (km)')
    for a in (a1, a2):
        a.set_facecolor('#fcfcfb'); a.grid(axis='y', color='#e6e5e1', lw=0.8)
        for sp in ('top', 'right'):
            a.spines[sp].set_visible(False)
    fig.tight_layout(); fig.savefig(png, dpi=130)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--national', required=True)
    ap.add_argument('--from', dest='a', required=True)
    ap.add_argument('--to', dest='b', required=True)
    ap.add_argument('--speeds', default='22,28')
    ap.add_argument('--name', default=None)
    ap.add_argument('--out')
    ap.add_argument('--png')
    a = ap.parse_args(argv)
    A, B = [tuple(float(x) for x in t.split(',')) for t in (a.a, a.b)]
    route, roads = NR.route(a.national, A, B)
    speeds = [float(x) for x in a.speeds.split(',')]
    results = [run(route, v) for v in speeds]
    name = a.name or ('%s -> %s' % (a.a, a.b))
    text = report(route, roads, name, speeds, results)
    print(text)
    if a.out:
        open(a.out, 'w').write(text)
    if a.png:
        plot(route, results, speeds, a.png, name + ' - cruise %.0f km/h, speed when following the advisory' % (speeds[-1] * 3.6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
