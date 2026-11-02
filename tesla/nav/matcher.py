"""GPS-to-route matching, off-route detection and recalculation. ADVISORY: produces a NavState for display/logging only; it has
no connection to steering, braking or any actuator, and imports nothing from vehicle control.

Matching is monotonic: after the first fix only a window ahead of the last matched progress (and a short way back, for GPS
jitter) is searched, so on a switchback the car cannot snap to the adjacent leg of the same route. A candidate whose direction
is more than HEADING_MAX_DEG from the car's heading is rejected (driving the route backwards is not 'on route').
Off-route needs several consecutive bad fixes at speed (no flapping from one noisy fix); GPS loss/poor accuracy is never treated
as off-route and never triggers a recalculation."""
import bisect

try:
    from . import geo
    from .router import find_route, find_route_from_edge, NoRoute
    from .instructions import build_steps
    from .tracker import EdgeTracker
except ImportError:
    import geo
    from router import find_route, find_route_from_edge, NoRoute
    from instructions import build_steps
    from tracker import EdgeTracker

ON_ROUTE_M = 30.0            # max cross-track error to count as on route
HEADING_MAX_DEG = 100.0
BACK_M = 60.0                # how far behind the last progress is still searched (GPS jitter)
AHEAD_M = 300.0              # how far ahead of the last progress is searched per fix (plus speed * dt * 3)
OFF_FIXES = 3                # consecutive bad fixes before 'off_route'
OFF_FIXES_AMBIG = 8          # ...but when the road the car is on lies within AMBIG_M of the route (GPS noise cannot separate them) wait for this many
AMBIG_M = 15.0
MIN_SPEED_MS = 2.0           # below this a bad fix is just noise at a standstill
ACC_MAX_M = 30.0             # fixes less accurate than this are ignored
GPS_TIMEOUT_S = 5.0
REROUTE_COOLDOWN_S = 4.0
RETURN_FIXES = 3             # once off-route is established, this many consecutive good fixes are needed to clear it
AMBIG_GIVE_UP_FIXES = 20     # recalculating from a CONTESTED track is deferred (status stays off_route, ambiguous=True) for at most this many fixes; then it
                             # recalculates from the best estimate but flags confidence='low' - never silently as if the road were known
ARRIVE_M = 30.0


class Match(object):
    def __init__(self, ok, progress_m=None, cross_track_m=None, heading_err=None, seg=None, reason=''):
        self.ok, self.progress_m, self.cross_track_m, self.heading_err, self.seg, self.reason = ok, progress_m, cross_track_m, heading_err, seg, reason


class RouteMatcher(object):
    def __init__(self, route):
        self.route = route
        self.pts, self.cum = route.points, route.cum
        self.progress = None                                     # last matched distance along the route (m)

    def match(self, lat, lon, heading=None, max_ahead=AHEAD_M):
        if not geo.valid_latlon(lat, lon):
            return Match(False, reason='invalid position')
        pts, cum = self.pts, self.cum
        if len(pts) < 2:
            return Match(False, reason='empty route')
        if self.progress is None:
            lo, hi = 0, len(pts) - 2
        else:
            lo = max(0, bisect.bisect_right(cum, self.progress - BACK_M) - 1)
            hi = min(len(pts) - 2, bisect.bisect_left(cum, self.progress + max_ahead))
        use_h = heading is not None and geo.is_finite_number(heading)
        best, wrong_way = None, False
        for i in range(lo, hi + 1):
            d, al, _plat, _plon, sl = geo.project(lat, lon, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])
            err = None
            if use_h and sl > 0.5:
                err = abs(geo.ang_diff(geo.bearing(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1]), heading))
                if err > HEADING_MAX_DEG:
                    wrong_way = wrong_way or d <= ON_ROUTE_M
                    continue
            if best is None or d < best[0]:
                best = (d, cum[i] + al, err, i)
        if best is None:
            return Match(False, reason='opposite direction to the route' if wrong_way else 'no route segment in the search window')
        return Match(True, best[1], best[0], best[2], best[3])

    def commit(self, progress_m):
        self.progress = progress_m if self.progress is None else max(self.progress, progress_m)


class NavState(object):
    def __init__(self, **kw):
        self.status = kw.get('status', 'idle')
        self.progress_m = kw.get('progress_m')
        self.remaining_m = kw.get('remaining_m')
        self.cross_track_m = kw.get('cross_track_m')
        self.next_step = kw.get('next_step')
        self.distance_to_next_m = kw.get('distance_to_next_m')
        self.off_route_fixes = kw.get('off_route_fixes', 0)
        self.reroutes = kw.get('reroutes', 0)
        self.message = kw.get('message', '')
        self.ambiguous = kw.get('ambiguous', False)   # off route, but which road the car is on cannot yet be told: no recalculation is made
        self.confidence = kw.get('confidence', 'normal')

    def as_dict(self):
        d = dict(self.__dict__)
        for k in ('progress_m', 'remaining_m', 'cross_track_m', 'distance_to_next_m'):
            if d[k] is not None:
                d[k] = round(d[k], 1)
        return d


class Navigator(object):
    """Plans a route to a destination, follows it with GPS fixes, detects leaving it and recalculates. Planning is injected
    (`planner`) so tests can substitute it; the default is router.find_route."""
    def __init__(self, graph, destination, planner=find_route):
        self.graph, self.dest, self.planner = graph, destination, planner
        self.route = self.matcher = self.steps = self.tracker = None
        self.route_eids = set()
        self.reroutes = 0
        self.off_fixes = 0
        self.last_good_t = None
        self.last_reroute_t = -1e9
        self.last_t = None
        self.alarm = False           # off-route established (>= OFF_FIXES bad fixes); cleared by RETURN_FIXES good ones or a recalculation
        self.contested_fixes = 0
        self.good_run = 0
        self.state = NavState(status='idle')

    def _adopt(self, route):
        self.route = route
        self.matcher = RouteMatcher(route)
        self.steps = build_steps(self.graph, route)
        self.route_eids = {e.eid for e in route.edges}
        self.tracker = EdgeTracker(self.graph, route.edges[0], route.start.along_m)
        self.off_fixes = 0
        self.alarm = False
        self.good_run = 0
        self.contested_fixes = 0
        return route

    def plan(self, lat, lon, heading=None):
        return self._adopt(self.planner(self.graph, (lat, lon), self.dest, heading))

    def plan_from_edge(self, edge, along_m):
        """Recalculate for a car known to be on `edge` (no start snapping: cannot start on a merely nearby road)."""
        return self._adopt(find_route_from_edge(self.graph, edge, along_m, self.dest))

    def _next_step(self, progress):
        for s in self.steps:
            if s.kind != 'depart' and s.start_m > progress + 1.0:
                return s
        return None

    def update(self, t, lat=None, lon=None, heading=None, speed=None, accuracy=None):
        if self.route is None:
            raise RuntimeError('plan() first')
        prev = self.state
        # --- GPS health: never off-route, never reroute on missing/poor GPS -----------------------------------------
        bad = lat is None or lon is None or not geo.valid_latlon(lat, lon)
        poor = (not bad) and accuracy is not None and geo.is_finite_number(accuracy) and accuracy > ACC_MAX_M
        if bad or poor:
            lost = self.last_good_t is None or (t - self.last_good_t) > GPS_TIMEOUT_S
            self.state = NavState(status='gps_lost' if lost else 'gps_poor', progress_m=prev.progress_m, remaining_m=prev.remaining_m,
                                  next_step=prev.next_step, distance_to_next_m=prev.distance_to_next_m, reroutes=self.reroutes,
                                  message='GPS fix invalid' if bad else 'GPS accuracy %.0f m' % accuracy)
            return self.state
        dt = 1.0 if self.last_good_t is None else max(0.0, t - self.last_good_t)
        self.last_good_t = t
        v = speed if geo.is_finite_number(speed) else 0.0
        m = self.matcher.match(lat, lon, heading, AHEAD_M + 3.0 * v * dt)
        tk = self.tracker.update(lat, lon, heading if geo.is_finite_number(heading) else None, v * dt)
        # the car is on a connected edge that is NOT part of the route (a road beside it, a side street): never 'on route', however close
        other_road = tk.known and tk.edge.eid not in self.route_eids
        if m.ok and m.cross_track_m <= ON_ROUTE_M and not other_road:
            self.good_run += 1
            if self.alarm and self.good_run < RETURN_FIXES:      # still alarmed: one good-looking fix does not clear it
                self.state = NavState(status=prev.status if prev.status in ('off_route', 'no_route') else 'off_route',
                                      off_route_fixes=self.off_fixes, reroutes=self.reroutes,
                                      message='off route: %d/%d good fixes to clear' % (self.good_run, RETURN_FIXES))
                return self.state
            self.alarm = False
            self.off_fixes = 0
            self.matcher.commit(m.progress_m)
            prog = self.matcher.progress
            rem = self.route.distance_m - prog
            nxt = self._next_step(prog)
            status = 'arrived' if rem <= ARRIVE_M else 'on_route'
            self.state = NavState(status=status, progress_m=prog, remaining_m=rem, cross_track_m=m.cross_track_m,
                                  next_step=None if nxt is None else nxt.as_dict(),
                                  distance_to_next_m=None if nxt is None else nxt.start_m - prog, reroutes=self.reroutes)
            return self.state
        # --- bad match -----------------------------------------------------------------------------------------------
        self.good_run = 0
        if v >= MIN_SPEED_MS:
            self.off_fixes += 1
        # how much evidence is enough depends on how distinguishable the two roads are: a road 8 m from the route is inside one GPS error
        need = OFF_FIXES_AMBIG if (other_road and m.ok and m.cross_track_m is not None and m.cross_track_m < AMBIG_M) else OFF_FIXES
        reason = 'on another road (edge %d, %s)' % (tk.edge.eid, tk.edge.name or tk.edge.highway) if other_road else (
            m.reason or 'cross-track %.0f m' % (m.cross_track_m or 0.0))
        if self.off_fixes < need and not self.alarm:
            # first bad fix: keep showing the route (one outlier is noise); from the second on say we are checking
            keep = prev.status in ('on_route', 'rerouted') and self.off_fixes <= 1
            self.state = NavState(status='on_route' if keep else 'checking', progress_m=prev.progress_m,
                                  remaining_m=prev.remaining_m, cross_track_m=m.cross_track_m, next_step=prev.next_step,
                                  distance_to_next_m=prev.distance_to_next_m, off_route_fixes=self.off_fixes, reroutes=self.reroutes,
                                  message='checking: %s' % reason)
            return self.state
        # --- off route: recalculate from where the car is, heading the way it is going ----------------------------------
        self.alarm = True
        if t - self.last_reroute_t < REROUTE_COOLDOWN_S:
            self.state = NavState(status='off_route', off_route_fixes=self.off_fixes, reroutes=self.reroutes, message=reason)
            return self.state
        # never recalculate from a track that is contested: two roads that cannot be told apart at this GPS accuracy are reported as ambiguous
        if tk.known and tk.contested and self.contested_fixes < AMBIG_GIVE_UP_FIXES:
            self.contested_fixes += 1
            self.state = NavState(status='off_route', off_route_fixes=self.off_fixes, reroutes=self.reroutes, ambiguous=True,
                                  message='off route, but the road the car is on cannot yet be told (%d/%d)' % (self.contested_fixes, AMBIG_GIVE_UP_FIXES))
            return self.state
        low = tk.known and tk.contested
        self.last_reroute_t = t
        try:
            if tk.known:
                self.plan_from_edge(tk.edge, tk.along)              # the edge the car is really on
            else:
                self.plan(lat, lon, heading)                         # track lost (GPS gap/teleport): fall back to snapping
            self.reroutes += 1
            self.contested_fixes = 0
            self.matcher.commit(0.0)
            nxt = self._next_step(0.0)
            self.state = NavState(status='rerouted', progress_m=0.0, remaining_m=self.route.distance_m,
                                  next_step=None if nxt is None else nxt.as_dict(),
                                  distance_to_next_m=None if nxt is None else nxt.start_m, reroutes=self.reroutes,
                                  message='recalculated after leaving the route (%s)' % reason, confidence='low' if low else 'normal')
        except NoRoute as e:
            self.state = NavState(status='no_route', off_route_fixes=self.off_fixes, reroutes=self.reroutes, message=str(e))
        return self.state
