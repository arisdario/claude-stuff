"""Advisory speed profile for a planned route: legal/class cruise speed vs the curve-limited recommended speed, with where and how much to slow.
Offline, planning only - nothing here touches vehicle control. Curve speed = sqrt(A_Y / |curvature|) (A_Y 1.85 m/s^2, same as mapcurve), braking
lookahead at A_DECEL 1.0 m/s^2 (same as mapcurve approach_speed). Curvature is Menger curvature over a +-SPAN_M baseline on a STEP_M resample of the
route geometry (a rougher estimate than the mapcurve builder, which uses raw OSM vertices)."""
import math
import sys

try:
    from . import geo, osm_rules as R
except ImportError:
    import geo, osm_rules as R

A_Y, A_DECEL, STEP_M, SPAN_M, V_MIN, FLOOR_KMH = 1.85, 1.0, 10.0, 30.0, 8.94, 20.0


def profile(graph, route):
    pts, cum = route.points, route.cum
    total = cum[-1]
    n = int(total // STEP_M) + 1
    xy, j = [], 0
    for i in range(n + 1):
        s = min(i * STEP_M, total)
        while j < len(cum) - 2 and cum[j + 1] < s:
            j += 1
        seg = max(cum[j + 1] - cum[j], 1e-9)
        f = min(max((s - cum[j]) / seg, 0.0), 1.0)
        xy.append((s, pts[j][0] + f * (pts[j + 1][0] - pts[j][0]), pts[j][1] + f * (pts[j + 1][1] - pts[j][1])))
    k = int(SPAN_M // STEP_M)
    cruise, curve = [], []
    eoff = list(route.edge_offsets) + [total + 1.0]
    ei = 0
    for i, (s, la, lo) in enumerate(xy):
        while ei < len(route.edges) - 1 and eoff[ei + 1] <= s:
            ei += 1
        e = graph.edge_info(route.edges[ei].eid)
        cruise.append(e['maxspeed_kph'] if e.get('maxspeed_kph') else R.DEFAULT_KPH.get(e['highway'], 30))
        if i < k or i + k >= len(xy):
            curve.append(255.0 * 3.6)
            continue
        a, b, c = xy[i - k], xy[i], xy[i + k]
        ab, bc, ca = geo.hav(a[1], a[2], b[1], b[2]), geo.hav(b[1], b[2], c[1], c[2]), geo.hav(c[1], c[2], a[1], a[2])
        m_lat = 111320.0
        m_lon = m_lat * math.cos(math.radians(b[1]))
        ax, ay = (a[2] - b[2]) * m_lon, (a[1] - b[1]) * m_lat
        cx, cy = (c[2] - b[2]) * m_lon, (c[1] - b[1]) * m_lat
        area2 = abs(ax * cy - ay * cx)
        kappa = 2.0 * area2 / max(ab * bc * ca, 1e-9)
        curve.append(3.6 * (max(V_MIN, min(255.0, math.sqrt(A_Y / kappa))) if kappa > 1e-6 else 255.0))
    cm = [c / 3.6 for c in curve]
    rec = [min(cr / 3.6, cv) for cr, cv in zip(cruise, cm)]
    for i in range(len(rec) - 2, -1, -1):                                    # braking lookahead, backwards pass
        rec[i] = min(rec[i], math.sqrt(rec[i + 1] ** 2 + 2.0 * A_DECEL * STEP_M))
    rec = [r * 3.6 for r in rec]
    return {'s_km': [x[0] / 1000.0 for x in xy], 'cruise': cruise, 'curve': curve, 'rec': rec,
            'cut': [max(0.0, c - r) for c, r in zip(cruise, rec)]}


def events(p, min_cut=FLOOR_KMH):
    """contiguous stretches where the advisory is >= min_cut km/h below cruise: (start_km, end_km, max_cut, min_rec)"""
    out, i, n = [], 0, len(p['cut'])
    while i < n:
        if p['cut'][i] >= min_cut:
            j = i
            while j + 1 < n and p['cut'][j + 1] >= min_cut:
                j += 1
            out.append((p['s_km'][i], p['s_km'][j], max(p['cut'][i:j + 1]), min(p['rec'][i:j + 1])))
            i = j + 1
        else:
            i += 1
    return out


def plot(p, title, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    BLUE, ORANGE, INK, MUTED, GRID = '#2a78d6', '#eb6834', '#0b0b0b', '#52514e', '#e6e5e1'
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(12, 6.2), sharex=True, gridspec_kw={'height_ratios': [3, 1.2]}, facecolor='#fcfcfb')
    s = p['s_km']
    a1.fill_between(s, p['rec'], p['cruise'], where=[c >= FLOOR_KMH for c in p['cut']], color=ORANGE, alpha=0.25, linewidth=0)
    a1.plot(s, p['cruise'], color=BLUE, lw=2, label='Cruise speed (maxspeed tag / road-class default)')
    a1.plot(s, p['rec'], color=ORANGE, lw=2, label='Recommended speed (curve-limited, 1.0 m/s² braking lookahead)')
    ev = events(p)
    for (s0, s1, cut, mn) in sorted(ev, key=lambda e: -e[2])[:4]:
        a1.annotate('-%d km/h\n(to %d)' % (round(cut), round(mn)), ((s0 + s1) / 2, mn), xytext=(0, -34), textcoords='offset points', ha='center', fontsize=8, color=INK,
                    arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.8))
    a1.set_ylabel('km/h', color=MUTED)
    a1.set_ylim(0, max(p['cruise']) * 1.12)
    a1.set_title(title, loc='left', color=INK, fontsize=12)
    a1.legend(loc='lower center', fontsize=8, frameon=False, ncol=1)
    a2.fill_between(s, 0, p['cut'], color=ORANGE, alpha=0.8, linewidth=0)
    a2.set_ylabel('slow down by (km/h)', color=MUTED, fontsize=9)
    a2.set_xlabel('distance along route (km)', color=MUTED)
    for a in (a1, a2):
        a.set_facecolor('#fcfcfb')
        a.grid(axis='y', color=GRID, lw=0.8)
        for sp in ('top', 'right'):
            a.spines[sp].set_visible(False)
        a.tick_params(colors=MUTED)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    return ev


def suspicious(p, min_cut=FLOOR_KMH):
    """Points to investigate later (NOT changed here): (a) 1-2 sample curvature spikes (<= 20 m wide, neighbours >= 2x faster): usually a single noisy/kinked OSM vertex
    or a junction node, not a real bend; (b) high-speed stretches (cruise >= 90) slowed by >= min_cut that are not within 150 m of a cruise-speed change (a bend on a
    fast road with no junction)."""
    n, out = len(p['curve']), []
    for i in range(2, n - 2):
        c = p['curve'][i]
        if p['cut'][i] >= min_cut and c < 0.5 * min(p['curve'][i - 2], p['curve'][i + 2]) and c < p['cruise'][i]:
            out.append(('spike', p['s_km'][i], p['cruise'][i], round(c), round(p['rec'][i])))
    chg = [p['s_km'][i] for i in range(1, n) if p['cruise'][i] != p['cruise'][i - 1]]
    for i in range(n):
        if p['cruise'][i] >= 90 and p['cut'][i] >= min_cut and not any(abs(p['s_km'][i] - c) < 0.15 for c in chg):
            out.append(('fast-road bend', p['s_km'][i], p['cruise'][i], round(p['curve'][i]), round(p['rec'][i])))
    merged, last = [], None
    for k, km, cr, cv, rc in out:                                    # collapse neighbours (< 50 m) of the same kind
        if last and last[0] == k and km - last[1] < 0.05:
            last[1] = km
            continue
        last = [k, km, cr, cv, rc]
        merged.append(last)
    return merged
