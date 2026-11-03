"""Python port of the CurveCue / CurveCall curve-detection engine (https://github.com/mhathiyari/curve-call, web-demo/engine.js, MIT licence,
Copyright (c) 2025 Mustafa Hathiyari). Ported statement by statement so it can run on the EON (Python, no JVM/node); it is NOT a new algorithm.
Equivalence is checked against the original engine.js run under node (tesla/tests/test_curvecall_engine.py, golden files in tesla/tests/data/).
Ported: interpolation, Menger curvature (+spike rejection, rolling average), segmentation, classification (severity/direction/modifiers), speed advisory.
Not ported (not needed for detection/speed): compound/S-bend detection, lean angle, narration text.
Advisory only: nothing here touches vehicle control."""
import math

EARTH_RADIUS_M = 6371000.0
GRAVITY = 9.81
MS_TO_KMH = 3.6
SPIKE_RATIO = 0.2
SPIKE_NEIGHBOR_MIN_RADIUS = 100.0
MAX_LATERAL_DEVIATION = 15.0
RADIUS_CAP = 10000.0
DEFAULT_THRESHOLDS = {'gentle': 200.0, 'moderate': 100.0, 'firm': 50.0, 'sharp': 25.0}


def haversine(p1, p2):
    lat1, lat2 = math.radians(p1[0]), math.radians(p2[0])
    dlat, dlon = math.radians(p2[0] - p1[0]), math.radians(p2[1] - p1[1])
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return EARTH_RADIUS_M * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing(f, t):
    lat1, lat2, dlon = math.radians(f[0]), math.radians(t[0]), math.radians(t[1] - f[1])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def bearing_difference(b1, b2):
    d = b2 - b1
    while d > 180:
        d -= 360
    while d <= -180:
        d += 360
    return d


def _interp_point(p1, p2, fraction):
    if fraction <= 0:
        return (p1[0], p1[1])
    if fraction >= 1:
        return (p2[0], p2[1])
    lat1, lon1, lat2, lon2 = map(math.radians, (p1[0], p1[1], p2[0], p2[1]))
    d = haversine(p1, p2) / EARTH_RADIUS_M
    if d < 1e-12:
        return (p1[0], p1[1])
    a = math.sin((1 - fraction) * d) / math.sin(d)
    b = math.sin(fraction * d) / math.sin(d)
    x = a * math.cos(lat1) * math.cos(lon1) + b * math.cos(lat2) * math.cos(lon2)
    y = a * math.cos(lat1) * math.sin(lon1) + b * math.cos(lat2) * math.sin(lon2)
    z = a * math.sin(lat1) + b * math.sin(lat2)
    return (math.degrees(math.atan2(z, math.sqrt(x * x + y * y))), math.degrees(math.atan2(y, x)))


def _project_distance(point, s0, s1):
    d_total = haversine(s0, s1)
    if d_total < 1e-6:
        return haversine(point, s0)
    cos_lat = math.cos(math.radians(s0[0]))
    dx1, dy1 = (s1[1] - s0[1]) * cos_lat, s1[0] - s0[0]
    dx2, dy2 = (point[1] - s0[1]) * cos_lat, point[0] - s0[0]
    len_sq = dx1 * dx1 + dy1 * dy1
    t = 0.0 if len_sq < 1e-18 else max(0.0, min(1.0, (dx1 * dx2 + dy1 * dy2) / len_sq))
    return haversine(point, _interp_point(s0, s1, t))


def menger_radius(p1, p2, p3):
    a, b, c = haversine(p1, p2), haversine(p2, p3), haversine(p1, p3)
    s = (a + b + c) / 2
    area_sq = s * (s - a) * (s - b) * (s - c)
    if area_sq <= 0:
        return math.inf
    area = math.sqrt(area_sq)
    return math.inf if area < 1e-10 else (a * b * c) / (4 * area)


def menger_direction(p1, p2, p3):
    cos_lat = math.cos(math.radians(p2[0]))
    deg_to_m = EARTH_RADIUS_M * math.pi / 180
    v1x, v1y = (p2[1] - p1[1]) * cos_lat * deg_to_m, (p2[0] - p1[0]) * deg_to_m
    v2x, v2y = (p3[1] - p2[1]) * cos_lat * deg_to_m, (p3[0] - p2[0]) * deg_to_m
    cross = v1x * v2y - v1y * v2x
    if cross > 1e-6:
        return 'LEFT'
    if cross < -1e-6:
        return 'RIGHT'
    return None


def interpolate_route(points, spacing=5.0):
    if len(points) < 2:
        raise ValueError('Need at least 2 points to interpolate')
    result = [(points[0][0], points[0][1])]
    seg_i, dist_along, remaining = 0, 0.0, spacing
    while seg_i < len(points) - 1:
        s0, s1 = points[seg_i], points[seg_i + 1]
        seg_len = haversine(s0, s1)
        pos = dist_along
        while pos + remaining <= seg_len:
            pos += remaining
            result.append(_interp_point(s0, s1, max(0.0, min(1.0, pos / seg_len))))
            remaining = spacing
        remaining -= (seg_len - pos)
        dist_along = 0.0
        seg_i += 1
    if haversine(result[-1], points[-1]) > spacing * 0.5:
        result.append((points[-1][0], points[-1][1]))
    return result


def _rolling_average(values, w):
    if w <= 1:
        return list(values)
    half, n, out = w // 2, len(values), []
    for i in range(n):
        lo, hi = max(0, i - half), min(n - 1, i + half)
        out.append(sum(values[lo:hi + 1]) / (hi - lo + 1))
    return out


def _reject_outliers(raw, points):
    if len(raw) < 5:
        return
    for i in range(2, len(raw) - 2):
        nb = sorted([raw[i - 2], raw[i - 1], raw[i + 1], raw[i + 2]])
        med = (nb[1] + nb[2]) / 2
        isolated = raw[i - 1] > med * 0.5 and raw[i + 1] > med * 0.5
        radius_spike = raw[i] < med * SPIKE_RATIO and med > SPIKE_NEIGHBOR_MIN_RADIUS and isolated
        position_spike = _project_distance(points[i], points[i - 1], points[i + 1]) > MAX_LATERAL_DEVIATION
        if radius_spike or position_spike:
            raw[i] = med


def compute_curvature(points, window=5):
    n = len(points)
    raw, dirs = [0.0] * n, [None] * n
    for i in range(1, n - 1):
        raw[i] = menger_radius(points[i - 1], points[i], points[i + 1])
        dirs[i] = menger_direction(points[i - 1], points[i], points[i + 1])
    raw[0], raw[n - 1] = raw[1], raw[n - 2]
    dirs[0], dirs[n - 1] = dirs[1], dirs[n - 2]
    _reject_outliers(raw, points)
    smoothed = _rolling_average([min(r, RADIUS_CAP) for r in raw], window)
    return [{'radius': smoothed[i], 'raw': raw[i], 'direction': dirs[i]} for i in range(n)]


def _seg_len(points, a, b):
    return sum(haversine(points[i], points[i + 1]) for i in range(a, b))


def segment_route(cp, points, threshold=500.0, gap_merge=50.0):
    if not cp:
        return []
    is_curve = [c['radius'] < threshold for c in cp]
    segs, start, typ = [], 0, is_curve[0]
    for i in range(1, len(is_curve)):
        if is_curve[i] != typ:
            segs.append({'start': start, 'end': i - 1, 'curve': typ})
            start, typ = i, is_curve[i]
    segs.append({'start': start, 'end': len(is_curve) - 1, 'curve': typ})
    if len(segs) <= 1:
        return segs
    merged, idx = [], 0
    while idx < len(segs):
        cur = segs[idx]
        if not cur['curve']:
            prev = merged[-1] if merged else None
            nxt = segs[idx + 1] if idx + 1 < len(segs) else None
            if prev and prev['curve'] and nxt and nxt['curve'] and _seg_len(points, cur['start'], cur['end']) < gap_merge:
                merged.pop()
                merged.append({'start': prev['start'], 'end': nxt['end'], 'curve': True})
                idx += 2
                continue
        merged.append(cur)
        idx += 1
    return merged


def classify_severity(min_radius, th=DEFAULT_THRESHOLDS):
    if min_radius > th['gentle']:
        return 'GENTLE'
    if min_radius > th['moderate']:
        return 'MODERATE'
    if min_radius > th['firm']:
        return 'FIRM'
    if min_radius > th['sharp']:
        return 'SHARP'
    return 'HAIRPIN'


def _avg_radius(cp, a, b):
    vals = [min(cp[i]['radius'], RADIUS_CAP) for i in range(a, b + 1)]
    return sum(vals) / len(vals) if vals else math.inf


def _modifiers(cp, a, b, arc):
    mods, count = set(), b - a + 1
    if count < 3:
        return mods
    third = count // 3
    if third < 1:
        return mods
    first, last = _avg_radius(cp, a, a + third - 1), _avg_radius(cp, b - third + 1, b)
    if last < first * 0.8:
        mods.add('TIGHTENING')
    elif last > first * 1.2:
        mods.add('OPENING')
    if arc > 200:
        mods.add('LONG')
        if 'TIGHTENING' not in mods and 'OPENING' not in mods:
            mods.add('HOLDS')
    return mods


def advisory_speed_kmh(radius, lateral_g=0.35):
    """CurveCue's rounded advisory (floor to 5 km/h)"""
    if radius <= 0:
        return 0
    return math.floor(math.sqrt(radius * lateral_g * GRAVITY) * MS_TO_KMH / 5) * 5


def analyze_route(points, spacing=5.0, smoothing_window=5, curvature_threshold=500.0, gap_merge=50.0, lateral_g=0.35, thresholds=DEFAULT_THRESHOLDS):
    """points: [(lat, lon), ...] -> list of curve dicts (CurveCue field names, snake_case) along the interpolated route."""
    if not points or len(points) < 3:
        return []
    pts = interpolate_route(points, spacing)
    if len(pts) < 3:
        return []
    cp = compute_curvature(pts, smoothing_window)
    cum = [0.0]
    for i in range(1, len(pts)):
        cum.append(cum[-1] + haversine(pts[i - 1], pts[i]))
    curves = []
    for seg in segment_route(cp, pts, curvature_threshold, gap_merge):
        if not seg['curve']:
            continue
        a, b = seg['start'], seg['end']
        left = sum(1 for i in range(a, b + 1) if cp[i]['direction'] == 'LEFT')
        right = sum(1 for i in range(a, b + 1) if cp[i]['direction'] == 'RIGHT')
        min_r = min(cp[i]['radius'] for i in range(a, b + 1))
        arc = _seg_len(pts, a, b)
        sev = classify_severity(min_r, thresholds)
        adv = advisory_speed_kmh(min_r, lateral_g)
        needs = sev not in ('GENTLE',) and (adv < 70 if sev == 'MODERATE' else True)
        curves.append({'direction': 'LEFT' if left >= right else 'RIGHT', 'severity': sev, 'min_radius': min_r, 'arc_length': arc,
                       'modifiers': sorted(_modifiers(cp, a, b, arc)), 'advisory_speed_kmh': adv if needs else None,
                       'start_index': a, 'end_index': b, 'distance_from_start': cum[a], 'end_distance': cum[b]})
    return curves
