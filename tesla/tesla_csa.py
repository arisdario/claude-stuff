#!/usr/bin/env python3
"""
tesla_csa.py - validate Tesla's nav curve-speed-adaptation broadcast (UI_csaRoadCurvature / UI_csaOfframpCurvature) and
turn it into a curve-speed candidate. Pure Python, no openpilot imports. SHADOW (log-only) BY DEFAULT.

WHAT IS KNOWN (from this repo, tesla/carstate.py): per message four fields are decoded - UI_csaRoadCurvC2, UI_csaRoadCurvC3,
UI_csaRoadCurvRange, UI_csaRoadCurvUsingTspline (same four for the Offramp message). Nothing else in the fork consumes
them, and the DBC (tesla_can, in opendbc) is not in this repo.

WHAT IS NOT KNOWN (do not assume): the physical meaning, units and sign of C2/C3, and the reference point. The coefficients
are NOT assumed to be curvature. Two hypotheses are implemented, selectable, and BOTH are computed in shadow mode so a
drive log can show which one agrees with the OSM/SQLite curve at the same place:

  H 'clothoid' : C2 = curvature at s=0 (1/m), C3 = curvature rate (1/m^2):        k(s) = C2 + C3*s
  H 'offset'   : C2, C3 = coefficients of a lateral-offset polynomial in x (m):   y(x) = C2*x^2 + C3*x^3
                 (small-angle) -> k(x) = y'' = 2*C2 + 6*C3*x

Common to both (also hypotheses): s / x is distance AHEAD of the car along its path, measured from the vehicle (reference
point = ego position), valid on 0 <= s <= Range, Range in metres; curvature sign is not used (only |k|), so left/right
convention does not matter. k is linear in s, so the tightest point is at an end (s=0 or s=Range).

VERIFICATION PROCEDURE (required before enabling): drive known bends with shadow logging on (/data/csa_shadow.log), compare
each hypothesis's speed to the SQLite curve speed at the same place; only a hypothesis that tracks it (same order, rises
and falls with the bend) is plausible. Then enable it explicitly:  echo clothoid > /data/tesla_csa_enable  (or 'offset').

Because the interpretation cannot be verified offline, and a pre-AP car may broadcast zeros/SNA, every frame must pass
plausibility gates or it is "unavailable" (reason logged):
  finite numbers, not SNA-saturated, Range in [20, 400] m, not all-zero, radius in [12 m, 5 km], speed >= 5 m/s.

CONTROL: nothing here touches the car. PCC_module uses a result ONLY if /data/tesla_csa_enable names a model AND
/data/tesla_csa_off is absent; then it is a floor (min(), never raises a limit) and replaces the SQLite curve for that cycle
(one source per curve). Default (no enable file) = shadow: logged, never used.
"""
import json
import math
import os
import time

A_Y_CSA = 1.85            # m/s^2 lateral - same as pcc_curves.A_Y_MAP so sources compare like for like
RANGE_MIN_M = 20.0
RANGE_MAX_M = 400.0
R_MIN_M = 12.0            # tighter than this is not a drivable road curve -> implausible/garbage
R_MAX_M = 5000.0          # looser than this is effectively straight -> no constraint
MIN_SPEED_MS = 5.0
SNA_ABS = 1e6             # a raw SNA / saturated field decodes to a huge magnitude
MODELS = ('clothoid', 'offset')
OFF_FILE = '/data/tesla_csa_off'
ENABLE_FILE = '/data/tesla_csa_enable'   # content = model name; absent/blank/unknown = SHADOW only
SHADOW_LOG = '/data/csa_shadow.log'
SHADOW_LOG_MAX = 5 * 1024 * 1024


class Csa(object):
    def __init__(self, speed, dist, conf, curvature, source, model):
        self.speed = speed            # curve speed limit (m/s) - NOT yet an approach speed
        self.dist = dist              # distance (m) to the tightest point
        self.conf = conf              # 0..1, capped: units unverified
        self.curvature = curvature    # |k| signed as computed, 1/m at the tightest point
        self.source = source          # 'tesla_csa' | 'tesla_csa_offramp'
        self.model = model

    def as_dict(self):
        return {'source': self.source, 'model': self.model, 'speed': round(self.speed, 2), 'dist': round(self.dist, 1),
                'conf': round(self.conf, 2), 'curvature': round(self.curvature, 5)}


def _ok_num(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) and abs(x) < SNA_ABS


def validate(c2, c3, rng):
    """None if the frame is usable, else a short reason string (for the log)."""
    for v in (c2, c3, rng):
        if not _ok_num(v):
            return 'non-finite/SNA/missing'
    if not (RANGE_MIN_M <= rng <= RANGE_MAX_M):
        return 'range %.0f outside %.0f-%.0f' % (rng, RANGE_MIN_M, RANGE_MAX_M)
    if c2 == 0.0 and c3 == 0.0:
        return 'all-zero (no data / pre-AP)'
    return None


def _k_ends(c2, c3, rng, model):
    """curvature at the near (s=0) and far (s=Range) ends under the chosen hypothesis."""
    if model == 'clothoid':
        return c2, c2 + c3 * rng
    if model == 'offset':
        return 2.0 * c2, 2.0 * c2 + 6.0 * c3 * rng
    raise ValueError('unknown CSA model %r' % (model,))


def csa_status(c2, c3, rng, using_tspline=0, source='tesla_csa', model='clothoid'):
    """(Csa or None, reason). reason explains a None."""
    bad = validate(c2, c3, rng)
    if bad:
        return None, bad
    k0, k1 = _k_ends(c2, c3, rng, model)
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
    conf = 0.5 if using_tspline else 0.4          # capped low: interpretation unverified
    return Csa(speed, dist, conf, k, source, model), 'ok'


def csa_curve(c2, c3, rng, using_tspline=0, source='tesla_csa', model='clothoid'):
    return csa_status(c2, c3, rng, using_tspline, source, model)[0]


def best_csa(road, offramp, model='clothoid'):
    """road/offramp: (c2, c3, range, using_tspline) or None (signal missing). Returns (Csa or None, reason). The more
    restrictive valid curve wins - alternatives, never summed (no double counting)."""
    cands = []
    reasons = []
    for tup, src in ((road, 'tesla_csa'), (offramp, 'tesla_csa_offramp')):
        if tup is None:
            reasons.append('%s:missing' % src)
            continue
        c, why = csa_status(tup[0], tup[1], tup[2], tup[3] if len(tup) > 3 else 0, src, model)
        if c is not None:
            cands.append(c)
        else:
            reasons.append('%s:%s' % (src, why))
    if not cands:
        return None, '; '.join(reasons)
    return min(cands, key=lambda c: c.speed), 'ok'


def read_inputs(CS):
    """(road, offramp) tuples from a CarState-like object; an entry is None if any of its signals is missing."""
    def grab(prefix):
        try:
            return (getattr(CS, prefix + 'C2'), getattr(CS, prefix + 'C3'), getattr(CS, prefix + 'Range'),
                    getattr(CS, prefix + 'UsingTspline', 0))
        except AttributeError:
            return None
    return grab('csaRoadCurv'), grab('csaOfframpCurv')


def enabled_model(enable_file=ENABLE_FILE, off_file=OFF_FILE):
    """The model name that may influence control, or None = SHADOW (the default). Requires an explicit, known model name."""
    try:
        if os.path.exists(off_file):
            return None
        with open(enable_file) as f:
            m = f.read().strip().lower()
        return m if m in MODELS else None
    except Exception:
        return None


def shadow_record(road, offramp, v_sqlite=None, sqlite_conf=0.5, sqlite_dist=None, v_ego=None):
    """One side-by-side log record: both CSA hypotheses (value, confidence, rejection reason) next to SQLite."""
    rec = {'t': round(time.time(), 2), 'v_ego': None if v_ego is None else round(v_ego, 2),
           'raw': {'road': road, 'offramp': offramp},
           'sqlite': None if not v_sqlite else {'speed': round(v_sqlite, 2), 'dist': None if sqlite_dist is None else round(sqlite_dist, 1),
                                                'conf': sqlite_conf},
           'csa': {}}
    for m in MODELS:
        c, why = best_csa(road, offramp, m)
        rec['csa'][m] = c.as_dict() if c else {'rejected': why}
    return rec


def log_shadow(rec, path=SHADOW_LOG, max_bytes=SHADOW_LOG_MAX):
    """Append one JSON line; stop (not rotate) at max_bytes so it can never fill /data. Never raises."""
    try:
        if os.path.exists(path) and os.path.getsize(path) > max_bytes:
            return False
        with open(path, 'a') as f:
            f.write(json.dumps(rec, sort_keys=True) + '\n')
        return True
    except Exception:
        return False
