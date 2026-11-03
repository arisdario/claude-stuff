"""Curve-ahead advisory predictor. Offline, ADVISORY ONLY: it publishes a recommended speed and a distance; nothing here touches steering, braking, acceleration, CAN,
UDS, CSA, mapcurve or PCC, and nothing imports them.

Two separate jobs:
  * which road is the car on / where along the route?  -> the existing Navigator (EdgeTracker + RouteMatcher). This module does not choose roads.
  * what bends are ahead on THAT road?                  -> this module: it reads only the confirmed route's own geometry (never a nearby road's) for HORIZON_M ahead.
If the road is not confidently identified (tracker contested / on another road / off route / GPS poor) the output has confidence 'low' or 'none' and NO advised
speed: it reports low confidence rather than guessing an unrelated road.

Curve detection is CurveCue's engine (curvecall_engine.py, a port verified against the original). Maths used here (same constants as speedplot / mapcurve):  v_curve = sqrt(A_LAT / |kappa|);  v_approach(d) = sqrt(v_curve^2 + 2 * DECEL * d)  = the highest speed you may have
d metres before the curve and still reach v_curve at DECEL;  slowing must begin when  d <= (v^2 - v_curve^2) / (2 * DECEL)."""
import math

try:
    from . import geo, curvecall_engine as ENG, own_detector as OWN
    from .matcher import Navigator
    from .tracker import CONTEST_LLR
except ImportError:
    import geo
    import curvecall_engine as ENG
    import own_detector as OWN
    from matcher import Navigator
    from tracker import CONTEST_LLR

A_LAT = 1.85            # m/s^2 lateral acceleration used for the curve speed (same as mapcurve A_Y)
DECEL = 1.0             # m/s^2 assumed comfortable deceleration (configurable)
DR_MAX_AGE_S = 60.0     # progress is dead-reckoned from speed for at most this long without a NEW GPS position (then confidence drops to 'low')
DR_BACK_TOL_M = 50.0    # a new position slightly behind the dead-reckoned progress does not move the car backwards (it would re-announce a passed curve)
DEFAULT_DETECTOR = 'own'  # 'own' = our Menger detector (primary: caught 5 of 5 real slow-downs on the real drive vs 1 of 5 for CurveCue); 'curvecall' = CurveCue engine port
HORIZON_M = 800.0       # look-ahead along the route
V_FLOOR = 8.94          # m/s, same floor as mapcurve


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


_CACHE = {}


def _analysis(points, lateral_g):
    """CurveCue's engine run ONCE over the whole route polyline (cached per route); every fix then just reads the curves ahead of the car from it"""
    key = (id(points), len(points), round(lateral_g, 6))
    hit = _CACHE.get(key)
    if hit is None or hit[0] is not points:
        _CACHE.clear()
        hit = (points, ENG.analyze_route([(p[0], p[1]) for p in points], lateral_g=lateral_g))
        _CACHE[key] = hit
    return hit[1]


def find_curves(points, cum, s_now, horizon_m=HORIZON_M, a_lat=A_LAT, detector=None):
    """curves ahead on the route polyline in (s_now, s_now + horizon]: [{'dist_m', 'start_m', 'end_m', 'radius_m', 'v_curve', 'severity', 'direction', 'text', 'confirmed'}], nearest first.
    detector 'own' (default, our Menger detector) or 'curvecall' (CurveCue's engine). With 'own', each curve also carries confirmed=True when CurveCue's engine finds a curve at the same place
    (two independent detectors agree); this function only filters by distance and converts the speed with a_lat."""
    detector = detector or DEFAULT_DETECTOR
    cc = _analysis(points, a_lat / ENG.GRAVITY)
    out = []
    if detector == 'own':
        for c in OWN.find_curves_own(points, cum, s_now, horizon_m, a_lat):
            sev = severity(c['radius_m'])
            a0, a1 = s_now + c['start_m'], s_now + c['end_m']
            conf = any(k['distance_from_start'] - 30.0 <= a1 and k['end_distance'] + 30.0 >= a0 for k in cc)
            out.append(dict(c, severity=sev, text=narration(sev, c['direction'], c['v_curve']), confirmed=conf))
        return out
    for c in cc:
        d0, d1 = c['distance_from_start'] - s_now, c['end_distance'] - s_now
        if d1 < 0.0 or d0 > horizon_m:
            continue
        rad = c['min_radius']
        vc = max(V_FLOOR, min(255.0, math.sqrt(a_lat * rad)))
        sev, dirn = c['severity'].lower(), c['direction'].lower()
        out.append({'dist_m': d0, 'start_m': d0, 'end_m': d1, 'radius_m': rad, 'v_curve': vc, 'severity': sev, 'direction': dirn, 'text': narration(sev, dirn, vc), 'confirmed': True})
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
                    self.curves[self.next_id] = {'id': self.next_id, 'a0': a0, 'a1': a1, 'abs': ab, 'radius_m': c['radius_m'], 'speed_ms': c['v_curve'], 'severity': c['severity'], 'direction': c['direction'], 'text': c['text'], 'confirmed': c.get('confirmed', False)}
                    new.append(self.next_id)
                    self.next_id += 1
                else:
                    v = self.curves[hit]
                    v['a0'], v['a1'] = min(v['a0'], a0), max(v['a1'], a1)
                    if c['v_curve'] <= v['speed_ms'] or ab == v['abs']:                  # radius/speed follow the tightest point seen
                        v['abs'], v['radius_m'], v['speed_ms'] = ab, c['radius_m'], c['v_curve']
                        v['severity'], v['direction'], v['text'] = c['severity'], c['direction'], c['text']
                    v['confirmed'] = v.get('confirmed', False) or c.get('confirmed', False)
        out = []                                    # distance_m = distance to where the bend STARTS (0 = in it)
        for k in sorted(self.curves):
            v = self.curves[k]
            if progress is not None and progress > v['a1'] + self.PASSED_M:
                passed.append(k)
                continue
            out.append({'id': k, 'new': k in new, 'distance_m': None if progress is None else max(0.0, v['a0'] - progress), 'radius_m': v['radius_m'], 'speed_ms': v['speed_ms'], 'severity': v['severity'], 'direction': v['direction'], 'text': v['text'], 'confirmed': v.get('confirmed', False)})
        for k in passed:
            del self.curves[k]
        return {'new': new, 'passed': passed, 'active': out}


def target_speed(active, decel=DECEL):
    """THE advisory number: the speed the car should not exceed RIGHT NOW so that it can reach every announced curve's entry at that curve's speed using only `decel`.
    active = announcer 'active' list (distance_m = distance to where the bend starts, 0 = inside it; speed_ms = curve speed). v_target = min over curves of
    sqrt(v_curve^2 + 2 * decel * distance). It is an upper bound that falls smoothly as a curve approaches; far from any curve it is large (no constraint).
    Returns (target_ms or None, the limiting curve or None)."""
    best, who = None, None
    for a in active:
        if a['distance_m'] is None:
            continue
        v = math.sqrt(a['speed_ms'] ** 2 + 2.0 * decel * max(0.0, a['distance_m']))
        if best is None or v < best:
            best, who = v, a
    return best, who


def simulate_follow(points, cum, v_cruise, decel=DECEL, accel=1.0, min_severity=DEFAULT_MIN_SEVERITY, dt=0.5, a_lat=A_LAT):
    """Open-loop check on clean geometry (no GPS): a car that cruises at v_cruise and obeys target_speed() with at most `decel` of braking. Returns
    {'entries': [(bend start m, curve speed, speed at entry)], 'max_decel': ..., 'min_speed': ...}. Advisory maths only - nothing here drives a car."""
    ann, s, v = CurveAnnouncer(min_severity), 0.0, v_cruise
    entries, max_dec, vmin, pending = {}, 0.0, v, {}
    while s < cum[-1] - 1.0:
        r = ann.update(s, 'high', find_curves(points, cum, s, HORIZON_M, a_lat))
        tgt, _who = target_speed(r['active'], decel)
        want = v_cruise if tgt is None else min(v_cruise, tgt)
        nv = max(v - decel * dt, want) if want < v else min(v + accel * dt, want)
        max_dec = max(max_dec, (v - nv) / dt)
        for a in r['active']:
            pending.setdefault(a['id'], (s + a['distance_m'], a['speed_ms']))        # where the bend starts, recorded when first announced (distance_m is clipped to 0 once inside it)
            if a['distance_m'] <= 1.0 and a['id'] not in entries:
                entries[a['id']] = (s, a['speed_ms'], nv)
        v = nv
        vmin = min(vmin, v)
        s += v * dt
    return {'entries': [(pending[k][0], e[1], e[2]) for k, e in sorted(entries.items())], 'max_decel': max_dec, 'min_speed': vmin, 'announced': len(pending)}


class CurveAhead(object):
    """Wraps a Navigator only to know WHERE on the route the car is and HOW SURE that is. update() returns the advisory dict (see publish())."""
    def __init__(self, graph, destination, a_lat=A_LAT, decel=DECEL, horizon_m=HORIZON_M, navigator=None, min_severity=DEFAULT_MIN_SEVERITY, dead_reckon=True):
        self.nav = navigator or Navigator(graph, destination)
        self.a_lat, self.decel, self.horizon = a_lat, decel, horizon_m
        self.announcer = CurveAnnouncer(min_severity)
        self.dead_reckon = dead_reckon
        self._pos = self._pos_t = self._dr_prog = self._last_t = self._last_v = None
        self._conf, self._route = 'none', None

    def start(self, lat, lon, heading=None):
        self.nav.plan(lat, lon, heading)

    def _contested_off_route(self, tk):
        """the tracker's 'contested' flag also fires when the car simply moved several edges along the SAME road between two GPS positions (a long jump has many candidate edges
        ahead). That is not ambiguity about WHICH road. Only evidence for an edge that is NOT on the route counts against confidence here (a parallel road still does)."""
        if not tk.contested:
            return False
        eids = self.nav.route_eids
        return any(v >= CONTEST_LLR and k not in eids for d in (tk.llr, tk.llr_unconnected) for k, v in d.items())

    def update(self, t, lat, lon, heading, speed, accuracy=None):
        route = self.nav.route
        v = speed or 0.0
        stale = (self.dead_reckon and lat is not None and lon is not None and (lat, lon) == self._pos and self._dr_prog is not None and route is self._route
                 and self._conf in ('high', 'low'))
        if stale:
            # the GPS position has not changed since the last update: do NOT feed it to the matcher as a new fix; advance the progress by the car's own speed
            dt = max(0.0, t - self._last_t)
            prog = min(route.distance_m, self._dr_prog + 0.5 * (v + (self._last_v if self._last_v is not None else v)) * dt)
            conf, status = (self._conf if (t - self._pos_t) <= DR_MAX_AGE_S else 'low'), 'dead_reckoning'      # confidence is what the last REAL update gave; a position frozen for > DR_MAX_AGE_S is not trusted
        else:
            st = self.nav.update(t, lat, lon, heading, speed, accuracy)
            route = self.nav.route
            tk = self.nav.tracker
            sure = st.status in ('on_route', 'arrived') and tk is not None and tk.known and tk.edge.eid in self.nav.route_eids and not self._contested_off_route(tk)
            if st.progress_m is not None and st.status in ('on_route', 'arrived', 'rerouted', 'checking'):
                prog = st.progress_m if st.status != 'checking' else self.nav.matcher.progress
            else:
                prog = self.nav.matcher.progress
            conf = 'high' if sure else ('low' if st.status in ('checking', 'on_route', 'gps_poor') else 'none')
            status = st.status
            if prog is not None and route is self._route and self._dr_prog is not None and 0.0 < self._dr_prog - prog <= DR_BACK_TOL_M:
                prog = self._dr_prog                                                # monotone
            if lat is not None and lon is not None and status not in ('gps_lost', 'gps_poor'):
                self._pos, self._pos_t = (lat, lon), t
        self._route, self._last_t, self._last_v = route, t, v
        if prog is None:
            self._dr_prog, self._conf = None, 'none'
            return self.publish(t, 'none', None, [], None, None, status)
        self._dr_prog, self._conf = prog, conf
        curves = find_curves(route.points, route.cum, prog, self.horizon, self.a_lat)
        rows, adv, bind = advise(curves, v, self.decel)
        msg = self.publish(t, conf, prog, rows, adv if conf == 'high' else None, bind, status)
        msg['announce'] = self.announcer.update(prog, conf, curves)
        tgt, who = target_speed(msg['announce']['active'], self.decel) if conf == 'high' else (None, None)
        msg['advisory'] = {'confidence': conf, 'target_speed_ms': tgt, 'slow_down': tgt is not None and v > tgt + 0.5,
                           'curve_distance_m': None if who is None else who['distance_m'], 'curve_speed_ms': None if who is None else who['speed_ms'],
                           'curve_radius_m': None if who is None else who['radius_m'], 'curve_severity': None if who is None else who['severity'],
                           'dead_reckoned': bool(stale), 'position_age_s': None if self._pos_t is None else t - self._pos_t}
        return msg

    @staticmethod
    def publish(t, conf, prog, rows, adv, bind, nav_status):
        """the advisory message (plain dict; the transport - e.g. a cereal/JSON publisher - is a later, separate step). advised_speed_ms is None unless confidence is 'high'."""
        return {'t': t, 'confidence': conf, 'route_progress_m': prog, 'nav_status': nav_status, 'advised_speed_ms': adv,
                'curve_dist_m': None if not bind else bind['dist_m'], 'curve_radius_m': None if not bind else bind['radius_m'],
                'curve_speed_ms': None if not bind else bind['v_curve'], 'begin_slowing_in_m': None if not bind else bind['begin_slowing_in_m'],
                'curves': rows}
