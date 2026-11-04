#!/usr/bin/env python3
"""
mapcurved.py - lightweight standalone map-based curvature speed limiter (road-aware).

Reads the car's position/heading, looks up the local mapcurve.sqlite (built by
tools/mapcurve_build.py) for the binding safe speed on the ROAD YOU ARE ON
within a lookahead window ahead of the car, and publishes liveMapData with
curvatureValid/curvature/distToTurn/mapValid so it drops straight into
PCC_module.py's existing max_v_in_mapped_curve_ms() consumer - NO changes
needed in PCC_module.py itself.

ROAD-AWARE LOOKUP (the fix for hairpins / switchbacks, e.g. DN73A):
  The old lookup took the lowest curve speed within a heading cone/corridor. On a
  switchback the road folds back on itself, so the geometrically-nearest grid
  point can belong to the adjacent hairpin leg or a parallel road, and the car
  would read that leg's speed instead of its own. The road-aware lookup instead:
    1. finds the nearest road sample within MATCH_RADIUS_M and locks to its
       (way_id, segment_id) - the road you are physically on;
    2. picks the travel direction along that segment's stored point order (seq)
       from your heading vs the segment tangent, honouring OSM oneway;
    3. walks ONLY forward along that segment (by seq, in the travel direction),
       accumulating along-road distance - never straight-line bearings to
       unrelated points;
    4. of the walked points within LOOKAHEAD_M, publishes the one that BINDS NOW
       (lowest approach_speed, same rule PCC_module/pcc_curves use), so a near
       moderate bend is not ignored in favour of a tighter curve much further on.
  It requires a road-aware DB (pts carry way_id/segment_id/seq/direction); an old
  nearest-point-only DB is refused with a clear message - rebuild it with
  tools/mapcurve_build.py.

curvature is back-derived from the stored speed using the SAME a_y_max=1.85
formula max_v_in_mapped_curve_ms() already uses, so that function's own
v = sqrt(a_y_max / curvature) reproduces the stored speed exactly:
    curvature = a_y_max / speed**2

POSITION: liveLocationKalman never had a valid fix on this car (posValid=False on
every line of mapcurve_check.log), so it falls back to gpsLocationExternal, which
carcontroller.py publishes from the car's own GPS on CAN (MCU_locationStatus +
UI_gpsVehicleHeading) when use_tesla_gps = True is set in /data/bb_openpilot.cfg.

This process only ever reads the sqlite file (PRAGMA query_only=ON enforces that
at the connection level) - it never writes to it.

Run standalone for testing (position from the car; -v prints each lookup):
    python3 mapcurved.py -v
Offline, without the car, use tools/mapcurve_check.py against the DB.
"""
import math
import os
import sqlite3
import sys
import time

# approach_speed is the ranking rule shared with pcc_curves; import it there so the
# runtime and PCC stay in lock-step, with a standalone fallback for offline use/tests.
try:
    from selfdrive.car.tesla.pcc_curves import approach_speed
except ImportError:
    try:
        from pcc_curves import approach_speed
    except ImportError:
        A_CURVE_DECEL = 1.0
        CURVE_REACTION_S = 2.0

        def approach_speed(v_curve, dist_m, v_ego):
            d = max(0., dist_m - max(v_ego, 0.) * CURVE_REACTION_S)
            return math.sqrt(max(v_curve, 0.) ** 2 + 2. * A_CURVE_DECEL * d)


DB_PATH = '/data/mapcurve.sqlite'
CELL_DEG = 0.01          # must match mapcurve_build.py's CELL_DEG
LOOKAHEAD_M = 400.0      # how far ahead (along the road) to search for the binding curve
A_Y_MAX = 1.85           # must match PCC_module.py's max_v_in_mapped_curve_ms
GPS_MAX_ACC_M = 20.0     # ignore gpsLocationExternal fixes worse than this (m)
GPS_MAX_AGE_S = 2.0      # ...or older than this
UPDATE_HZ = 4
MIN_SPEED_MS = 1.0       # guard against div-by-zero / garbage rows

DEBUG_FLAG_PATH = '/data/mapcurve_debug'


def debug_enabled():
    return ('-v' in sys.argv or '--verbose' in sys.argv) or os.path.exists(DEBUG_FLAG_PATH)


def meters_per_deg(lat_deg):
    lat_r = math.radians(lat_deg)
    m_per_deg_lat = 111320.0
    m_per_deg_lon = 111320.0 * math.cos(lat_r)
    return m_per_deg_lat, m_per_deg_lon


def bearing_deg_to(lat0, lon0, lat1, lon1):
    """Compass bearing (0=N, 90=E) from point0 to point1."""
    m_lat, m_lon = meters_per_deg(lat0)
    dx = (lon1 - lon0) * m_lon   # east component, metres
    dy = (lat1 - lat0) * m_lat   # north component, metres
    return math.degrees(math.atan2(dx, dy)) % 360.0


def angle_diff_deg(a, b):
    d = (a - b) % 360.0
    if d > 180.0:
        d -= 360.0
    return abs(d)


class MapCurveLookup:
    """Road-aware lookup for road-aware mapcurve DBs (pts carry way_id/segment_id/seq/direction).

    First match a nearby road segment using distance and its ordered geometry
    tangent. Then evaluate curve speeds only along that segment, in the
    direction of travel. This avoids taking the minimum from unrelated roads
    (the adjacent hairpin leg or a parallel road) that a nearest-point lookup hits.
    """
    MATCH_RADIUS_M = 35.0     # road identity must be established near the car, not by a far point
    MATCH_HEADING_DEG = 45.0  # tangent-vs-heading gate (widen if noisy GPS heading drops the match mid-bend)
    CANDIDATE_LIMIT = 16
    CACHE_SEGMENTS = 32
    # Route continuity (needs a v2 DB with OSM node IDs; see tools/mapcurve_build.py). Roads are linked ONLY through a
    # shared node ID - never because coordinates are close. Heading/geometry only choose among nodes that are really shared.
    MAX_HOPS = 8
    BEARING_SPAN_M = 20.0        # distance used to measure the approach / departure bearing at a junction
    MAX_TURN_DEG = 100.0         # a continuation turning more than this is not a continuation (U-turn / reversal)
    STRAIGHT_DEG = 30.0          # with several options, the best must be this close to straight ahead...
    AMBIGUITY_MARGIN_DEG = 25.0  # ...and beat the runner-up by this much, otherwise prediction STOPS (no guessing)

    def __init__(self, db_path=DB_PATH):
        self.db = sqlite3.connect(db_path, check_same_thread=False)
        self.db.execute('PRAGMA query_only=ON')  # this process never writes
        cols = {r[1] for r in self.db.execute('PRAGMA table_info(pts)')}
        required = {'lat', 'lon', 'speed', 'cell_lat', 'cell_lon',
                    'way_id', 'segment_id', 'seq', 'direction'}
        missing = required - cols
        if missing:
            raise RuntimeError('mapcurve DB is missing road-aware columns: %s; rebuild with '
                               'tools/mapcurve_build.py' % ', '.join(sorted(missing)))
        self.segment_cache = {}
        self.meta_cache = {}
        self._cell_key = None       # level 1: the rows of the current grid-cell window (the old code re-read them from SQLite on every query)
        self._cell_rows = []
        self._near = None           # level 2: (lat, lon, m_lat, m_lon, rows) - the cell rows that can matter near one anchor position
        seg_cols = {r[1] for r in self.db.execute('PRAGMA table_info(segments)')}
        # old (v1) DBs have no node IDs -> no cross-segment links: prediction stops at the segment end
        self.has_nodes = {'start_node_id', 'end_node_id', 'oneway'} <= seg_cols

    @staticmethod
    def _distance(lat, lon, plat, plon, m_lat, m_lon):
        return math.hypot((plat - lat) * m_lat, (plon - lon) * m_lon)

    @staticmethod
    def _tangent(rows, i):
        """Compass bearing along stored point order at index i."""
        a = max(0, i - 2)
        b = min(len(rows) - 1, i + 2)
        if a == b:
            return None
        return bearing_deg_to(rows[a][0], rows[a][1], rows[b][0], rows[b][1])

    def _segment(self, way_id, segment_id):
        key = (str(way_id), int(segment_id))
        if key in self.segment_cache:
            return self.segment_cache[key]
        rows = self.db.execute(
            'SELECT lat, lon, speed, seq, direction FROM pts '
            'WHERE way_id=? AND segment_id=? ORDER BY seq',
            key
        ).fetchall()
        if rows:
            # Small bounded cache; clear old entries rather than retaining the
            # entire national database in memory.
            if len(self.segment_cache) >= self.CACHE_SEGMENTS:
                self.segment_cache.pop(next(iter(self.segment_cache)))
            self.segment_cache[key] = rows
        return rows

    def _span_bearing(self, rows, e, step, m_lat, m_lon):
        """Compass bearing from rows[e] to the point ~BEARING_SPAN_M away stepping by `step` (+1/-1); None if the segment is
        degenerate (< 1 m)."""
        far = e
        walked = 0.0
        i = e
        while 0 <= i + step < len(rows) and walked < self.BEARING_SPAN_M:
            walked += self._distance(rows[i][0], rows[i][1], rows[i + step][0], rows[i + step][1], m_lat, m_lon)
            i += step
            far = i
        if walked < 1.0:
            return None
        return bearing_deg_to(rows[e][0], rows[e][1], rows[far][0], rows[far][1])

    def _continuation(self, cur_key, cur_rows, sign, visited, m_lat, m_lon):
        """Next segment after leaving cur_key at its end in direction `sign`, or (None, reason).
        Candidates are exactly the segments that share the exit node ID, legal for travel (oneway), not already visited.
        One legal candidate = take it. Several = take the best only if clearly straight-ahead and clearly better than the
        runner-up; otherwise stop (ambiguous)."""
        exit_i = len(cur_rows) - 1 if sign > 0 else 0
        meta = self._seg_meta(*cur_key)
        node = None if meta is None else (meta[2] if sign > 0 else meta[1])
        if node is None:
            return None, 'no_node_id'
        cands = self.db.execute('SELECT way_id, segment_id, oneway, start_node_id, end_node_id FROM segments '
                                'WHERE start_node_id=? OR end_node_id=?', (node, node)).fetchall()
        approach = self._span_bearing(cur_rows, exit_i, -sign, m_lat, m_lon)
        if approach is None:
            return None, 'no_bearing'
        approach = (approach + 180.0) % 360.0          # direction of travel into the junction
        options = []
        blocked = 0
        for w, sg, ow, sn, en in cands:
            key = (str(w), int(sg))
            if key in visited:
                continue
            ow = int(ow or 0)
            for enter_at_start, ok in ((True, sn == node and ow >= 0), (False, en == node and ow <= 0)):
                if sn == node and enter_at_start and ow < 0 or en == node and not enter_at_start and ow > 0:
                    blocked += 1                       # shares the node but travel in that direction is not allowed
                if not ok:
                    continue
                rows = self._segment(*key)
                if len(rows) < 2:
                    continue
                sgn = 1 if enter_at_start else -1
                e = 0 if enter_at_start else len(rows) - 1
                brg = self._span_bearing(rows, e, sgn, m_lat, m_lon)
                options.append((None if brg is None else angle_diff_deg(brg, approach), key, rows, e, sgn))
        if not options:
            return None, 'oneway' if blocked else 'dead_end'
        # The SAME OSM way continuing through the node (adjacent segment of one way_id) is established by node identity and
        # way identity, which is stronger evidence than any heading test: take it whatever the deflection (a hairpin apex
        # can turn ~170 degrees). Heading is only used below, to choose among DIFFERENT roads.
        same = [o for o in options if o[1][0] == cur_key[0] and abs(o[1][1] - cur_key[1]) == 1]
        if len(same) == 1:
            dev, key, rows, e, sgn = same[0]
            return {'key': key, 'rows': rows, 'idx': e, 'sign': sgn, 'node': node,
                    'deviation': 0.0 if dev is None else dev, 'candidates': len(options), 'same_way': True}, 'ok'
        if len(options) > 1 and any(o[0] is None for o in options):
            return None, 'ambiguous'
        options.sort(key=lambda o: (o[0] is None, o[0]))
        if options[0][0] is None:
            return None, 'no_bearing'
        legal = [o for o in options if o[0] <= self.MAX_TURN_DEG]
        if not legal:
            return None, 'sharp_turn'
        if len(legal) > 1:
            if legal[0][0] > self.STRAIGHT_DEG or legal[1][0] - legal[0][0] < self.AMBIGUITY_MARGIN_DEG:
                return None, 'ambiguous'
        dev, key, rows, e, sgn = legal[0]
        return {'key': key, 'rows': rows, 'idx': e, 'sign': sgn, 'node': node, 'deviation': dev,
                'candidates': len(options), 'same_way': False}, 'ok'

    def _seg_meta(self, way_id, segment_id):
        key = (str(way_id), int(segment_id))
        if key not in self.meta_cache:
            r = self.db.execute('SELECT oneway, start_node_id, end_node_id FROM segments WHERE way_id=? AND segment_id=?',
                                key).fetchone() if self.has_nodes else None
            self.meta_cache[key] = r
        return self.meta_cache[key]

    def _walk_segment(self, rows, i, sign, along, v_ego, best, m_lat, m_lon):
        """Walk rows from index i in direction sign, ranking each point by approach_speed. Returns
        (best, along, last_index, reached_end) - reached_end is False if the lookahead ran out first."""
        while True:
            speed = rows[i][2]
            if along <= LOOKAHEAD_M and speed is not None and speed >= MIN_SPEED_MS:
                v_now = approach_speed(speed, along, v_ego)
                if best is None or v_now < best[0]:
                    best = (v_now, speed, along)
            nxt = i + sign
            if nxt < 0 or nxt >= len(rows):
                return best, along, i, True
            along += self._distance(rows[i][0], rows[i][1], rows[nxt][0], rows[nxt][1], m_lat, m_lon)
            i = nxt
            if along > LOOKAHEAD_M:
                return best, along, i, False

    NEAR_MARGIN_M = 300.0    # level-2 subset keeps points within radius + this of the anchor...
    NEAR_MOVE_M = 200.0      # ...and is reused while the car stays within this of the anchor (100 m of slack for the changing lon scale)

    def _candidates(self, key, lat, lon, m_lat, m_lon, radius):
        """Candidate points (lat, lon, way_id, segment_id) for a query, in EXACTLY the order SQLite returned them before caching
        (equal-distance ties at shared junction nodes are decided by that order). A superset of the points within `radius`;
        the caller still applies the exact distance test."""
        if key != self._cell_key:
            self._cell_rows = self.db.execute(
                'SELECT lat, lon, way_id, segment_id FROM pts '
                'WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?', key).fetchall()
            self._cell_key = key
            self._near = None
        n = self._near
        if n is not None and math.hypot((lat - n[0]) * n[2], (lon - n[1]) * n[3]) <= self.NEAR_MOVE_M:
            return n[4]
        r2 = radius + self.NEAR_MARGIN_M
        keep = [r for r in self._cell_rows if abs((r[0] - lat) * m_lat) <= r2 and abs((r[1] - lon) * m_lon) <= r2]
        self._near = (lat, lon, m_lat, m_lon, keep)
        return keep

    def query_ahead(self, lat, lon, heading_deg, v_ego=0.0):
        """Return (binding_speed_ms, distance_m, match_info) or None.

        binding_speed_ms is the stored curve speed of the point that limits speed
        NOW (lowest approach_speed over the forward walk); distance_m is its
        along-road distance ahead; match_info names the matched road/direction.
        """
        # An unusable position or heading matches NOTHING (no road is guessed). NaN needs an explicit check: every comparison
        # with it is False, so the heading gate below would otherwise let it straight through.
        for x in (lat, lon, heading_deg):
            if not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x):
                return None
        if not (-90.0 < lat < 90.0 and -180.0 <= lon <= 180.0):
            return None
        if not isinstance(v_ego, (int, float)) or not math.isfinite(v_ego) or v_ego < 0.0:
            v_ego = 0.0
        m_lat, m_lon = meters_per_deg(lat)
        # Match radius is deliberately local: road identity must be established
        # near the car, not by a point hundreds of metres away.
        radius = self.MATCH_RADIUS_M
        span_lat = max(1, int(math.ceil(radius / (CELL_DEG * m_lat))) + 1)
        span_lon = max(1, int(math.ceil(radius / (CELL_DEG * m_lon))) + 1)
        cl = int(math.floor(lat / CELL_DEG))
        co = int(math.floor(lon / CELL_DEG))
        candidates = self._candidates((cl - span_lat, cl + span_lat, co - span_lon, co + span_lon), lat, lon, m_lat, m_lon, radius)

        # Retain the nearest local sample per segment as a candidate.
        nearest = {}
        for plat, plon, way_id, segment_id in candidates:
            d = self._distance(lat, lon, plat, plon, m_lat, m_lon)
            if d > radius:
                continue
            key = (str(way_id), int(segment_id))
            if key not in nearest or d < nearest[key][0]:
                nearest[key] = (d, plat, plon)

        if not nearest:
            return None

        ranked = sorted(nearest.items(), key=lambda item: item[1][0])[:self.CANDIDATE_LIMIT]
        best_match = None
        for (way_id, segment_id), (near_dist, _plat, _plon) in ranked:
            rows = self._segment(way_id, segment_id)
            if len(rows) < 2:
                continue

            # Locate the closest point in this segment and compare its local
            # tangent with vehicle heading. The database direction field is
            # OSM oneway: 1=forward only, -1=reverse only, 0=both directions.
            idx = min(range(len(rows)), key=lambda i:
                      self._distance(lat, lon, rows[i][0], rows[i][1], m_lat, m_lon))
            point_dist = self._distance(lat, lon, rows[idx][0], rows[idx][1], m_lat, m_lon)
            if point_dist > radius:
                continue
            tangent = self._tangent(rows, idx)
            if tangent is None:
                continue
            oneway = int(rows[idx][4] or 0)
            options = []
            if oneway >= 0:
                options.append((angle_diff_deg(tangent, heading_deg), 1))
            if oneway <= 0:
                options.append((angle_diff_deg((tangent + 180.0) % 360.0, heading_deg), -1))
            heading_diff, travel_sign = min(options, key=lambda x: x[0])
            if not (heading_diff <= self.MATCH_HEADING_DEG):      # written so a NaN can never pass
                continue
            # Distance dominates; heading breaks ties between nearby carriageways.
            score = point_dist + heading_diff * 0.35
            if best_match is None or score < best_match[0]:
                best_match = (score, point_dist, heading_diff, way_id,
                              segment_id, rows, idx, travel_sign)

        if best_match is None:
            return None

        (_score, match_dist, heading_diff, way_id, segment_id,
         rows, idx, travel_sign) = best_match

        # Walk forward along the matched segment by along-road distance, ranking each point by approach_speed (the speed
        # allowable NOW to reach it slowing at A_CURVE_DECEL) so the binding curve wins, near or far. When the segment ends
        # before the lookahead does, continue ONLY through a shared OSM node (see _continuation); any doubt stops here.
        cur_key = (str(way_id), int(segment_id))
        visited = {cur_key}
        connections = []
        best = None
        along = match_dist
        i = idx
        sign = travel_sign
        truncated = 'lookahead'
        while True:
            best, along, i, reached_end = self._walk_segment(rows, i, sign, along, v_ego, best, m_lat, m_lon)
            if not reached_end:
                break
            if len(connections) >= self.MAX_HOPS:
                truncated = 'hop_limit'
                break
            if not self.has_nodes:
                truncated = 'no_node_ids'
                break
            cont, why = self._continuation(cur_key, rows, sign, visited, m_lat, m_lon)
            if cont is None:
                truncated = why
                break
            connections.append({'way_id': cont['key'][0], 'segment_id': cont['key'][1], 'node': cont['node'],
                                'deviation': round(cont['deviation'], 1), 'candidates': cont['candidates'],
                                'same_way': cont['same_way'], 'at_m': round(along, 1)})
            visited.add(cont['key'])
            cur_key, rows, i, sign = cont['key'], cont['rows'], cont['idx'], cont['sign']

        if best is None:
            return None
        _v_now, best_speed, best_dist = best
        return best_speed, best_dist, {
            'way_id': way_id, 'segment_id': segment_id,
            'match_m': match_dist, 'heading_error': heading_diff,
            'direction': travel_sign,
            'connections': connections, 'truncated': truncated, 'node_ids': self.has_nodes
        }


def main():
    import cereal.messaging as messaging
    pm = messaging.PubMaster(['liveMapData'])
    sm = messaging.SubMaster(['liveLocationKalman', 'gpsLocationExternal'])
    lookup = MapCurveLookup()

    period = 1.0 / UPDATE_HZ
    last_gps_t = -1e9
    while True:
        t0 = time.monotonic()
        sm.update(0)

        msg = messaging.new_message('liveMapData')
        md = msg.liveMapData
        md.mapValid = False
        md.curvatureValid = False
        md.speedLimitValid = False   # this process only knows curves; carcontroller reads speedLimit only when valid

        ll = sm['liveLocationKalman']
        has_fix = (sm.updated['liveLocationKalman']
                   and ll.positionGeodetic.valid
                   and ll.orientationNED.valid)
        v_ego = 0.0
        lat = lon = heading_deg = None
        if has_fix:
            lat, lon, _alt = ll.positionGeodetic.value
            _roll, _pitch, yaw = ll.orientationNED.value
            heading_deg = math.degrees(yaw) % 360.0
            try:
                v_ego = math.sqrt(sum(x * x for x in ll.velocityECEF.value))
            except Exception:
                pass
        else:
            # the car's own GPS (carcontroller publishes it with use_tesla_gps = True)
            g = sm['gpsLocationExternal']
            if sm.updated['gpsLocationExternal']:
                last_gps_t = time.monotonic()
            if (time.monotonic() - last_gps_t <= GPS_MAX_AGE_S and 0 < g.accuracy <= GPS_MAX_ACC_M
                    and abs(g.latitude) > 0.1 and abs(g.longitude) > 0.1):
                has_fix = True
                lat, lon, heading_deg, v_ego = g.latitude, g.longitude, g.bearing % 360.0, g.speed

        if has_fix:
            result = lookup.query_ahead(lat, lon, heading_deg, v_ego)
            if result is not None:
                speed_ms, dist_m, match = result
                speed_ms = max(speed_ms, MIN_SPEED_MS)
                curvature = A_Y_MAX / (speed_ms ** 2)

                md.mapValid = True
                md.curvatureValid = True
                md.curvature = float(curvature)
                md.distToTurn = float(dist_m)

                if debug_enabled():
                    print('lat=%.5f lon=%.5f hdg=%.1f -> speed=%.1f m/s dist=%.0f m '
                          'way=%s seg=%s match=%.1fm hdgErr=%.1f dir=%+d'
                          % (lat, lon, heading_deg, speed_ms, dist_m,
                             match['way_id'], match['segment_id'], match['match_m'],
                             match['heading_error'], match['direction']))
            elif debug_enabled():
                print('lat=%.5f lon=%.5f hdg=%.1f -> no curve found within %.0fm (road-aware)'
                      % (lat, lon, heading_deg, LOOKAHEAD_M))
        elif debug_enabled():
            print('waiting for a position fix (liveLocationKalman or gpsLocationExternal)...')

        pm.send('liveMapData', msg)

        dt = time.monotonic() - t0
        if dt < period:
            time.sleep(period - dt)


if __name__ == '__main__':
    main()

# --- How this starts ----------------------------------------------------
# This is NOT registered in openpilot's process manager (this repo's deploy_car.sh
# only touches selfdrive/car/tesla, not selfdrive/manager.py). Instead
# mapcurve_launcher.ensure_running() - called once from carcontroller.__init__ -
# launches this script with the car stack when /data/mapcurve.sqlite exists.
# Disable with:  touch /data/mapcurve_off
# Run by hand:   cd /data/openpilot && PYTHONPATH=/data/openpilot python3 selfdrive/car/tesla/mapcurved.py -v
