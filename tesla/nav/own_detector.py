"""Our own curve detector (restored from commit c98e2db; superseded by the CurveCue port in e356fa2 and restored as the PRIMARY detector after the three-way real-drive comparison, see HANDOVER Update 10).
Menger curvature over a +-30 m baseline on a 10 m resample anchored to the route, 3-point median against single-vertex noise, v = sqrt(a_lat / kappa). Geometry only; advisory only."""
import math

try:
    from . import geo
except ImportError:
    import geo

STEP_M = 10.0
SPAN_M = 30.0           # Menger curvature baseline (+-)
V_FLOOR = 8.94          # m/s, same floor as mapcurve
CURVE_MAX_MS = 30.0     # only bends whose curve speed is below this (108 km/h, R < ~490 m) count as curves
MIN_CURVE_SAMPLES = 2   # a curve must persist >= this many consecutive samples (20 m): a single-vertex kink is not a bend


def _sample(points, cum, s0, s1, step=STEP_M):
    out, j, s = [], 0, s0
    total = cum[-1]
    while s <= s1 + 1e-6:
        c = min(max(s, 0.0), total)
        while j < len(cum) - 2 and cum[j + 1] < c:
            j += 1
        seg = max(cum[j + 1] - cum[j], 1e-9)
        f = min(max((c - cum[j]) / seg, 0.0), 1.0)
        out.append((s, points[j][0] + f * (points[j + 1][0] - points[j][0]), points[j][1] + f * (points[j + 1][1] - points[j][1])))
        s += step
    return out


def curvatures(samples, span_m=SPAN_M):
    """(kappa 1/m per sample, signed cross product): Menger over +-span, 3-point median filter against single-vertex noise; 0 where the baseline does not fit"""
    k = max(1, int(round(span_m / STEP_M)))
    raw = [0.0] * len(samples)
    sgn = [0.0] * len(samples)
    for i in range(k, len(samples) - k):
        a, b, c = samples[i - k], samples[i], samples[i + k]
        ab, bc, ca = geo.hav(a[1], a[2], b[1], b[2]), geo.hav(b[1], b[2], c[1], c[2]), geo.hav(c[1], c[2], a[1], a[2])
        mlat = 111320.0
        mlon = mlat * math.cos(math.radians(b[1]))
        ax, ay, cx, cy = (a[2] - b[2]) * mlon, (a[1] - b[1]) * mlat, (c[2] - b[2]) * mlon, (c[1] - b[1]) * mlat
        cross = ax * cy - ay * cx                    # (a-b) x (c-b): > 0 means the road bends to the RIGHT travelling a -> b -> c (east/north axes)
        raw[i] = 2.0 * abs(cross) / max(ab * bc * ca, 1e-9)
        sgn[i] = cross
    filt = [sorted(raw[max(0, i - 1):i + 2])[len(raw[max(0, i - 1):i + 2]) // 2] for i in range(len(raw))]
    return filt, sgn


def find_curves_own(points, cum, s_now, horizon_m, a_lat):
    """curves in (s_now, s_now + horizon]: [{'dist_m' (tightest point), 'start_m', 'end_m', 'radius_m', 'v_curve', 'direction'}], nearest first. Geometry only."""
    s_from = math.floor((s_now - SPAN_M) / STEP_M) * STEP_M            # grid anchored at multiples of STEP_M along the route: the same curvature at the same place at every fix
    samples = _sample(points, cum, s_from, min(s_now + horizon_m + SPAN_M, cum[-1]))
    if len(samples) < 5:
        return []
    kap, sgn = curvatures(samples)
    vc = [max(V_FLOOR, min(255.0, math.sqrt(a_lat / x))) if x > 1e-6 else 255.0 for x in kap]
    out, i, n = [], 0, len(samples)
    while i < n:
        if vc[i] < CURVE_MAX_MS and samples[i][0] > s_now - 1e-6 and samples[i][0] <= s_now + horizon_m:
            j = i
            while j + 1 < n and vc[j + 1] < CURVE_MAX_MS:
                j += 1
            if j - i + 1 >= MIN_CURVE_SAMPLES:
                b = min(range(i, j + 1), key=lambda q: vc[q])
                out.append({'dist_m': samples[b][0] - s_now, 'radius_m': 1.0 / max(kap[b], 1e-9), 'v_curve': vc[b], 'start_m': samples[i][0] - s_now,
                            'end_m': samples[j][0] - s_now, 'direction': 'right' if sgn[b] > 0 else 'left'})
            i = j + 1
        else:
            i += 1
    return out
