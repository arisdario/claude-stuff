"""Curve-ahead advisory predictor. Offline, ADVISORY ONLY: it publishes a recommended speed and a distance; nothing here touches steering, braking, acceleration, CAN,
UDS, CSA, mapcurve or PCC, and nothing imports them.

Two separate jobs:
  * which road is the car on / where along the route?  -> the existing Navigator (EdgeTracker + RouteMatcher). This module does not choose roads.
  * what bends are ahead on THAT road?                  -> this module: it reads only the confirmed route's own geometry (never a nearby road's) for HORIZON_M ahead.
If the road is not confidently identified (tracker contested / on another road / off route / GPS poor) the output has confidence 'low' or 'none' and NO advised
speed: it reports low confidence rather than guessing an unrelated road.

Maths (same constants as speedplot / mapcurve):  v_curve = sqrt(A_LAT / |kappa|);  v_approach(d) = sqrt(v_curve^2 + 2 * DECEL * d)  = the highest speed you may have
d metres before the curve and still reach v_curve at DECEL;  slowing must begin when  d <= (v^2 - v_curve^2) / (2 * DECEL)."""
import math

try:
    from . import geo
    from .matcher import Navigator
except ImportError:
    import geo
    from matcher import Navigator

A_LAT = 1.85            # m/s^2 lateral acceleration used for the curve speed (same as mapcurve A_Y)
DECEL = 1.0             # m/s^2 assumed comfortable deceleration (configurable)
HORIZON_M = 800.0       # look-ahead along the route
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


# severity bands by curve radius: the same bands CurveCue uses (github.com/mhathiyari/curve-call, MIT licence): gentle > 200 m, moderate 100-200, firm 50-100, sharp 25-50, hairpin < 25
SEVERITY_ORDER = ('gentle', 'moderate', 'firm', 'sharp', 'hairpin')
DEFAULT_MIN_SEVERITY = 'moderate'          # CurveCue's 'standard' verbosity: moderate and tighter are announced; 'gentle' only if asked for
SEVERITY_BANDS = ((25.0, 'hairpin'), (50.0, 'sharp'), (100.0, 'firm'), (200.0, 'moderate'), (1e9, 'gentle'))


def severity(radius_m):
    return next(name for limit, name in SEVERITY_BANDS if radius_m < limit)


def narration(sev, direction, speed_ms, unit='kmh'):
    """e.g. 'Sharp right ahead, slow to 40' (speed rounded to 5 km/h; moderate/gentle bends are announced without a speed)"""
    if sev in ('gentle', 'moderate'):
        return '%s %s ahead' % (sev.capitalize(), direction)
    return '%s %s ahead, slow to %d' % ('Hairpin' if sev == 'hairpin' else sev.capitalize(), direction, int(round(speed_ms * 3.6 / 5.0)) * 5)


def curvatures(samples, span_m=SPAN_M, with_sign=False):
    """kappa (1/m) per sample (Menger over +-span, 3-point median filter against single-vertex noise); 0 where the baseline does not fit"""
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
    return (filt, sgn) if with_sign else filt


def find_curves(points, cum, s_now, horizon_m=HORIZON_M, a_lat=A_LAT):
    """curves in (s_now, s_now + horizon]: [{'dist_m', 'radius_m', 'v_curve'}], nearest first. Geometry only - no position noise involved."""
    s_from = math.floor((s_now - SPAN_M) / STEP_M) * STEP_M            # sample grid anchored at multiples of STEP_M along the route: curvature at a given place is the same at every fix
    samples = _sample(points, cum, s_from, min(s_now + horizon_m + SPAN_M, cum[-1]))
    if len(samples) < 5:
        return []
    kap, sgn = curvatures(samples, with_sign=True)
    vc = [max(V_FLOOR, min(255.0, math.sqrt(a_lat / x))) if x > 1e-6 else 255.0 for x in kap]
    out, i, n = [], 0, len(samples)
    while i < n:
        if vc[i] < CURVE_MAX_MS and samples[i][0] > s_now - 1e-6 and samples[i][0] <= s_now + horizon_m:
            j = i
            while j + 1 < n and vc[j + 1] < CURVE_MAX_MS:
                j += 1
            if j - i + 1 >= MIN_CURVE_SAMPLES:
                b = min(range(i, j + 1), key=lambda q: vc[q])
                rad = 1.0 / max(kap[b], 1e-9)
                dirn = 'right' if sgn[b] > 0 else 'left'
                out.append({'dist_m': samples[b][0] - s_now, 'radius_m': rad, 'severity': severity(rad), 'direction': dirn, 'text': narration(severity(rad), dirn, vc[b]), 'v_curve': vc[b], 'start_m': samples[i][0] - s_now, 'end_m': samples[j][0] - s_now})
            i = j + 1
        else:
            i += 1
    return out


def advise(curves, v_now, decel=DECEL):
    """for each curve: approach speed now, and where slowing must begin. Returns (per_curve list, advised_speed_ms or None, binding curve or None)."""
    rows, best = [], None
    for c in curves:
        d, vc = max(c['dist_m'], 0.0), c['v_curve']
        v_app = math.sqrt(vc * vc + 2.0 * decel * d)
        need = max(0.0, (v_now * v_now - vc * vc) / (2.0 * decel)) if v_now > vc else 0.0
        r = dict(c, v_approach=v_app, slow_distance_m=need, begin_slowing_in_m=(d - need) if v_now > vc else None, must_slow_now=v_app < v_now)
        rows.append(r)
        if r['must_slow_now'] and (best is None or v_app < best['v_approach']):
            best = r
    return rows, (best['v_approach'] if best else None), best


class CurveAnnouncer(object):
    """Turns the per-fix list of curves ahead into ANNOUNCEMENTS: each real curve gets one id and is announced once ('new'); afterwards only its distance is updated each
    fix, until the car has passed it. A curve is the same curve if its position along the route (progress + distance) lies within the extent of the known one +- MERGE_M,
    so the binding point drifting as the window slides does not create a second announcement. New curves are only announced while the road is confidently identified."""
    MERGE_M = 5.0
    PASSED_M = 10.0

    def __init__(self, min_severity=DEFAULT_MIN_SEVERITY):
        self.curves, self.next_id = {}, 1
        self.min_rank = SEVERITY_ORDER.index(min_severity)

    def update(self, progress, confidence, curves):
        new, passed = [], []
        curves = [c for c in curves if SEVERITY_ORDER.index(c['severity']) >= self.min_rank]
        if progress is not None and confidence == 'high':
            for c in curves:
                a0, a1, ab = progress + c['start_m'], progress + c['end_m'], progress + c['dist_m']
                ov = [(min(a1, v['a1']) - max(a0, v['a0']), k) for k, v in self.curves.items() if a0 <= v['a1'] + self.MERGE_M and a1 >= v['a0'] - self.MERGE_M]
                hit = max(ov)[1] if ov else None
                if hit is None:
                    self.curves[self.next_id] = {'id': self.next_id, 'a0': a0, 'a1': a1, 'abs': ab, 'radius_m': c['radius_m'], 'speed_ms': c['v_curve'], 'severity': c['severity'], 'direction': c['direction'], 'text': c['text']}
                    new.append(self.next_id)
                    self.next_id += 1
                else:
                    v = self.curves[hit]
                    v['a0'], v['a1'] = min(v['a0'], a0), max(v['a1'], a1)
                    if c['v_curve'] <= v['speed_ms'] or ab == v['abs']:                  # radius/speed follow the tightest point seen
                        v['abs'], v['radius_m'], v['speed_ms'] = ab, c['radius_m'], c['v_curve']
                        v['severity'], v['direction'], v['text'] = c['severity'], c['direction'], c['text']
        out = []                                    # distance_m = distance to where the bend STARTS (0 = in it)
        for k in sorted(self.curves):
            v = self.curves[k]
            if progress is not None and progress > v['a1'] + self.PASSED_M:
                passed.append(k)
                continue
            out.append({'id': k, 'new': k in new, 'distance_m': None if progress is None else max(0.0, v['a0'] - progress), 'radius_m': v['radius_m'], 'speed_ms': v['speed_ms'], 'severity': v['severity'], 'direction': v['direction'], 'text': v['text']})
        for k in passed:
            del self.curves[k]
        return {'new': new, 'passed': passed, 'active': out}


class CurveAhead(object):
    """Wraps a Navigator only to know WHERE on the route the car is and HOW SURE that is. update() returns the advisory dict (see publish())."""
    def __init__(self, graph, destination, a_lat=A_LAT, decel=DECEL, horizon_m=HORIZON_M, navigator=None, min_severity=DEFAULT_MIN_SEVERITY):
        self.nav = navigator or Navigator(graph, destination)
        self.a_lat, self.decel, self.horizon = a_lat, decel, horizon_m
        self._last_prog, self._last_t = None, None
        self.announcer = CurveAnnouncer(min_severity)

    def start(self, lat, lon, heading=None):
        self.nav.plan(lat, lon, heading)

    def update(self, t, lat, lon, heading, speed, accuracy=None):
        st = self.nav.update(t, lat, lon, heading, speed, accuracy)
        route = self.nav.route
        tk = self.nav.tracker
        sure = st.status in ('on_route', 'arrived') and tk is not None and tk.known and not tk.contested and tk.edge.eid in self.nav.route_eids
        if st.progress_m is not None and st.status in ('on_route', 'arrived', 'rerouted', 'checking'):
            prog = st.progress_m if st.status != 'checking' else self.nav.matcher.progress
        else:
            prog = self.nav.matcher.progress
        if prog is None:
            return self.publish(t, 'none', None, [], None, None, st.status)
        conf = 'high' if sure else ('low' if st.status in ('checking', 'on_route', 'gps_poor') else 'none')
        curves = find_curves(route.points, route.cum, prog, self.horizon, self.a_lat)
        rows, adv, bind = advise(curves, speed or 0.0, self.decel)
        msg = self.publish(t, conf, prog, rows, adv if conf == 'high' else None, bind, st.status)
        msg['announce'] = self.announcer.update(prog, conf, curves)
        return msg

    @staticmethod
    def publish(t, conf, prog, rows, adv, bind, nav_status):
        """the advisory message (plain dict; the transport - e.g. a cereal/JSON publisher - is a later, separate step). advised_speed_ms is None unless confidence is 'high'."""
        return {'t': t, 'confidence': conf, 'route_progress_m': prog, 'nav_status': nav_status, 'advised_speed_ms': adv,
                'curve_dist_m': None if not bind else bind['dist_m'], 'curve_radius_m': None if not bind else bind['radius_m'],
                'curve_speed_ms': None if not bind else bind['v_curve'], 'begin_slowing_in_m': None if not bind else bind['begin_slowing_in_m'],
                'curves': rows}
