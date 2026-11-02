#!/usr/bin/env python3
"""
tesla_csa.py - turn Tesla's navigation curve-speed-adaptation broadcast (UI_csaRoadCurvature / UI_csaOfframpCurvature)
into a curve-speed constraint, ONLY when the values pass strict validation. Pure Python, no openpilot imports.

What Tesla actually sends (decoded in carstate.py): per message a coefficient pair C2, C3 and a Range (m), plus a
UsingTspline flag. It is NOT route coordinates. Interpretation used here (an ASSUMPTION - the DBC is in opendbc, not
this repo, and nothing else in the fork consumes these signals): the clothoid form that Tesla's other road-curvature
messages use,
        curvature(s) = C2 + C3 * s      (1/m, signed), for 0 <= s <= Range (m ahead)
Curvature is linear in s, so the tightest point is at s=0 or s=Range.

Because the interpretation and units cannot be verified offline, and a pre-AP car may broadcast zeros/SNA, the result is
used ONLY if it passes physical-plausibility gates; anything else returns None ("unavailable") and the caller falls back
to SQLite. A curve speed that fails the gates never reaches control. It is a SAFETY FLOOR only: it can lower the curve
limit, never raise it, and never overrides lead/speed-limit/driver inputs (the caller takes min()).

Gates (all must hold):
  - all inputs finite numbers
  - Range within [RANGE_MIN_M, RANGE_MAX_M]
  - not all-zero coefficients (zero/SNA frame)
  - |coefficient| not at a raw SNA saturation value
  - tightest curvature gives a radius in [R_MIN_M, R_MAX_M]  (a physically plausible road curve)
  - the resulting curve speed is above MIN_SPEED_MS
Kill switch: /data/tesla_csa_off.  Shadow mode (compute + log, never used): /data/tesla_csa_shadow.
"""
import math

A_Y_CSA = 1.85            # m/s^2 lateral - same as pcc_curves.A_Y_MAP so Tesla/SQLite/vision are compared like for like
RANGE_MIN_M = 20.0
RANGE_MAX_M = 400.0
R_MIN_M = 12.0            # tighter than this is not a drivable road curve -> implausible/garbage
R_MAX_M = 5000.0          # looser than this is effectively straight -> no constraint
MIN_SPEED_MS = 5.0
SNA_ABS = 1e6             # a raw SNA / saturated field decodes to a huge magnitude
OFF_FILE = '/data/tesla_csa_off'
SHADOW_FILE = '/data/tesla_csa_shadow'


class Csa(object):
    def __init__(self, speed, dist, conf, curvature, source, reason='ok'):
        self.speed = speed            # curve speed limit (m/s) - NOT yet an approach speed
        self.dist = dist              # distance (m) to the tightest point
        self.conf = conf              # 0..1
        self.curvature = curvature    # signed, 1/m at the tightest point
        self.source = source          # 'tesla_csa' | 'tesla_csa_offramp'
        self.reason = reason

    def as_dict(self):
        return {'source': self.source, 'speed': round(self.speed, 2), 'dist': round(self.dist, 1),
                'conf': round(self.conf, 2), 'curvature': round(self.curvature, 5)}


def _ok_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and abs(x) < SNA_ABS


def validate(c2, c3, rng):
    """None if the frame is usable, else a short reason string (for the log)."""
    for v in (c2, c3, rng):
        if not _ok_num(v):
            return 'non-finite/SNA'
    if not (RANGE_MIN_M <= rng <= RANGE_MAX_M):
        return 'range %.0f outside %.0f-%.0f' % (rng, RANGE_MIN_M, RANGE_MAX_M)
    if c2 == 0.0 and c3 == 0.0:
        return 'all-zero (no data / pre-AP)'
    return None


def csa_curve(c2, c3, rng, using_tspline=0, source='tesla_csa'):
    """Csa for a valid, plausible, constraining curve; None if unavailable, implausible, or effectively straight.
    Use csa_status() to learn WHY it was rejected (for logging)."""
    return csa_status(c2, c3, rng, using_tspline, source)[0]


def csa_status(c2, c3, rng, using_tspline=0, source='tesla_csa'):
    """(Csa or None, reason). reason explains a None."""
    bad = validate(c2, c3, rng)
    if bad:
        return None, bad
    k0 = c2
    k1 = c2 + c3 * rng
    if abs(k1) > abs(k0):          # ties -> the NEAR point (smaller distance = stricter approach speed)
        k, dist = k1, rng
    else:
        k, dist = k0, 0.0
    if abs(k) < 1.0 / R_MAX_M:
        return None, 'straight (R>%.0f m)' % R_MAX_M
    radius = 1.0 / abs(k)
    if radius < R_MIN_M:
        return None, 'implausible radius %.1f m' % radius
    speed = math.sqrt(A_Y_CSA * radius)
    if speed < MIN_SPEED_MS:
        return None, 'speed %.1f m/s below floor' % speed
    # confidence: validated-but-unverified-units source, so capped well below 1; a spline-backed curve is better
    conf = 0.75 if using_tspline else 0.6
    return Csa(speed, dist, conf, k, source), 'ok'


def best_csa(road, offramp):
    """road/offramp: (c2, c3, range, using_tspline) tuples or None. Returns (Csa or None, reason). The more restrictive of
    the two valid curves wins (a floor, not a vote). No double counting: they are alternatives, one result is returned."""
    cands = []
    reasons = []
    for tup, src in ((road, 'tesla_csa'), (offramp, 'tesla_csa_offramp')):
        if tup is None:
            continue
        c, why = csa_status(tup[0], tup[1], tup[2], tup[3] if len(tup) > 3 else 0, src)
        if c is not None:
            cands.append(c)
        else:
            reasons.append('%s:%s' % (src, why))
    if not cands:
        return None, '; '.join(reasons) or 'no input'
    return min(cands, key=lambda c: c.speed), 'ok'
