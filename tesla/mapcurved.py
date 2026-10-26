#!/usr/bin/env python3
"""
mapcurved.py - lightweight standalone map-based curvature speed limiter.

Reads liveLocationKalman for current position/heading, looks up the local
mapcurve.sqlite (built by mapcurve_build.py) for the minimum safe speed
within a lookahead window ahead of the car, and publishes liveMapData with
curvatureValid/curvature/distToTurn/mapValid so it drops straight into
PCC_module.py's existing max_v_in_mapped_curve_ms() consumer - NO changes
needed in PCC_module.py itself.

curvature is back-derived from the stored speed using the SAME a_y_max=1.85
formula max_v_in_mapped_curve_ms() already uses, so that function's own
v = sqrt(a_y_max / curvature) reproduces the stored speed exactly:
    curvature = a_y_max / speed**2

This process only ever reads the sqlite file (PRAGMA query_only=ON enforces
that at the connection level) - it never writes to it.

Changes (see PCC_module.py / pcc_curves.py for the consumer side):
  - POSITION: liveLocationKalman never had a valid fix on this car (mapcurve_check.log: posValid=False on every
    line), so nothing was ever published. Falls back to gpsLocationExternal, which carcontroller.py publishes from
    the car's own GPS on CAN (MCU_locationStatus + UI_gpsVehicleHeading) when use_tesla_gps = True is set in
    /data/bb_openpilot.cfg.
  - WHICH CURVE: the slowest point within 400 m is not the one that matters - a 30 km/h corner 390 m away needs no
    slowing yet, a 50 km/h bend 60 m away does. Each point is ranked by the speed you may have NOW to still reach it
    at its speed slowing at pcc_curves.A_CURVE_DECEL (same rule PCC_module uses), and the binding one is published.
  - CORRIDOR: points are only taken from a corridor along the current heading (10 m + 30% of the distance to each
    side) instead of a 50 deg cone, which at 400 m spans ~650 m and picks up parallel and side streets.

Run standalone for testing:
    python3 mapcurved.py

Add to the process manager once confirmed working (see bottom of this file
for notes on wiring it in).
"""
import math
import sqlite3
import time
import sys
import os

import cereal.messaging as messaging
try:
    from selfdrive.car.tesla.pcc_curves import approach_speed
except ImportError:                       # running outside the openpilot tree (tests)
    from pcc_curves import approach_speed

DB_PATH = '/data/mapcurve.sqlite'
CELL_DEG = 0.01          # must match mapcurve_build.py's CELL_DEG
LOOKAHEAD_M = 400.0      # how far ahead to search for the tightest curve
HEADING_CONE_DEG = 50.0  # only consider points within this angle of current heading
CORRIDOR_M, CORRIDOR_FRAC = 10.0, 0.30   # ...and at most this far to the side: CORRIDOR_M + CORRIDOR_FRAC * distance ahead
GPS_MAX_ACC_M = 20.0     # ignore gpsLocationExternal fixes worse than this (m)
GPS_MAX_AGE_S = 2.0      # ...or older than this
A_Y_MAX = 1.85           # must match PCC_module.py's max_v_in_mapped_curve_ms
UPDATE_HZ = 4
MIN_SPEED_MS = 1.0       # guard against div-by-zero / garbage rows

DEBUG_FLAG_PATH = '/data/mapcurve_debug'


def debug_enabled():
    return os.path.exists(DEBUG_FLAG_PATH)


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
    def __init__(self, db_path=DB_PATH):
        self.db = sqlite3.connect(db_path, check_same_thread=False)
        self.db.execute('PRAGMA query_only=ON')  # this process never writes

    def query_ahead(self, lat, lon, heading_deg, v_ego=0.0):
        """Return (speed_ms, dist_m) of the curve point that limits the speed NOW (lowest approach speed, see the
        header) within LOOKAHEAD_M, HEADING_CONE_DEG and the corridor along the current heading, or None."""
        m_lat, m_lon = meters_per_deg(lat)
        cell_m_lat = CELL_DEG * m_lat
        cell_m_lon = CELL_DEG * m_lon
        span_lat = max(1, int(math.ceil(LOOKAHEAD_M / cell_m_lat)) + 1)
        span_lon = max(1, int(math.ceil(LOOKAHEAD_M / cell_m_lon)) + 1)

        cell_lat = int(math.floor(lat / CELL_DEG))
        cell_lon = int(math.floor(lon / CELL_DEG))

        rows = self.db.execute(
            'SELECT lat, lon, speed FROM pts '
            'WHERE cell_lat BETWEEN ? AND ? AND cell_lon BETWEEN ? AND ?',
            (cell_lat - span_lat, cell_lat + span_lat,
             cell_lon - span_lon, cell_lon + span_lon)
        ).fetchall()

        best = None                                   # (approach speed now, point speed, distance)
        h = math.radians(heading_deg)
        for plat, plon, speed in rows:
            dlat_m = (plat - lat) * m_lat
            dlon_m = (plon - lon) * m_lon
            dist = math.hypot(dlat_m, dlon_m)
            if dist < 1.0 or dist > LOOKAHEAD_M or speed is None or speed < MIN_SPEED_MS:
                continue
            brg = bearing_deg_to(lat, lon, plat, plon)
            if angle_diff_deg(brg, heading_deg) > HEADING_CONE_DEG:
                continue
            along = dlon_m * math.sin(h) + dlat_m * math.cos(h)
            side = abs(dlon_m * math.cos(h) - dlat_m * math.sin(h))
            if along <= 0 or side > CORRIDOR_M + CORRIDOR_FRAC * along:
                continue
            v_now = approach_speed(speed, dist, v_ego)
            if best is None or v_now < best[0]:
                best = (v_now, speed, dist)

        if best is None:
            return None
        return best[1], best[2]


def main():
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
        if has_fix:
            lat, lon, _alt = ll.positionGeodetic.value
            _roll, _pitch, yaw = ll.orientationNED.value
            heading_deg = math.degrees(yaw) % 360.0
            try: v_ego = math.sqrt(sum(x * x for x in ll.velocityECEF.value))
            except Exception: pass
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
                speed_ms, dist_m = result
                speed_ms = max(speed_ms, MIN_SPEED_MS)
                curvature = A_Y_MAX / (speed_ms ** 2)

                md.mapValid = True
                md.curvatureValid = True
                md.curvature = float(curvature)
                md.distToTurn = float(dist_m)

                if debug_enabled():
                    print('lat=%.5f lon=%.5f hdg=%.1f -> speed=%.1f m/s dist=%.0f m'
                          % (lat, lon, heading_deg, speed_ms, dist_m))
            elif debug_enabled():
                print('lat=%.5f lon=%.5f hdg=%.1f -> no curve found within %.0fm'
                      % (lat, lon, heading_deg, LOOKAHEAD_M))
        elif debug_enabled():
            print('waiting for liveLocationKalman fix...')

        pm.send('liveMapData', msg)

        dt = time.monotonic() - t0
        if dt < period:
            time.sleep(period - dt)


if __name__ == '__main__':
    main()

# --- Wiring into the process manager -----------------------------------
# This fork's process manager (likely selfdrive/manager.py in this version)
# needs an entry added for this process so it starts/stops with the rest of
# the stack. Need to see that file's existing entries (e.g. how 'locationd'
# or 'paramsd' are registered) to match the pattern before adding mapcurved.
