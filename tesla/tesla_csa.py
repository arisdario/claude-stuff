#!/usr/bin/env python3
"""
tesla_csa.py - validate Tesla's nav curve-speed-adaptation broadcast (UI_csaRoadCurvature / UI_csaOfframpCurvature) and
turn it into a curve-speed candidate. Pure Python, no openpilot imports. SHADOW (log-only) BY DEFAULT.

WHAT IS KNOWN (from this repo, tesla/carstate.py): per message four fields are decoded - UI_csaRoadCurvC2, UI_csaRoadCurvC3,
UI_csaRoadCurvRange, UI_csaRoadCurvUsingTspline (same four for the Offramp message). Nothing else in the fork consumes
them, and the DBC (tesla_can, in opendbc) is not in this repo.

WHAT IS NOT KNOWN (do not assume): the physical meaning, units and sign of C2/C3, and the reference point. The coefficients
are NOT assumed to be curvature. Two hypotheses are implemented, selectable, and BOTH are computed in shadow mode so the
broadcast is recorded under each reading for later analysis against the evidence listed under VERIFICATION below (how well
either reading matches SQLite is not that evidence):

  H 'clothoid' : C2 = curvature at s=0 (1/m), C3 = curvature rate (1/m^2):        k(s) = C2 + C3*s
  H 'offset'   : C2, C3 = coefficients of a lateral-offset polynomial in x (m):   y(x) = C2*x^2 + C3*x^3
                 (small-angle) -> k(x) = y'' = 2*C2 + 6*C3*x

Common to both (also hypotheses): s / x is distance AHEAD of the car along its path, measured from the vehicle (reference
point = ego position), valid on 0 <= s <= Range, Range in metres; curvature sign is not used (only |k|), so left/right
convention does not matter. k is linear in s, so the tightest point is at an end (s=0 or s=Range).

VERIFICATION (NOT DONE - neither hypothesis is verified): resemblance to the SQLite/OSM curve - or to anything else - is NOT
verification: both are estimates, and a wrong scale that happens to look plausible is exactly the failure to avoid.
Agreement with SQLite is NOT verification. Control may be considered only with documented evidence, one of:
  (a) the signal definition (scale, offset, unit, sign, reference point) from the correct Tesla DBC for this car's firmware,
      cited by file and commit; or
  (b) validated vehicle data: a CAN capture of the signals against an independent ground truth (e.g. GPS-derived path
      curvature and measured steering angle through a known-radius bend), analysed and written up.
The shadow log (/data/csa_shadow.log) only records what the car broadcasts so that such evidence can be gathered.

Because the interpretation cannot be verified offline, and a pre-AP car may broadcast zeros/SNA, every frame must pass
plausibility gates or it is "unavailable" (reason logged):
  finite numbers, not SNA-saturated, Range in [20, 400] m, not all-zero, radius in [12 m, 5 km], speed >= 5 m/s.

CONTROL SWITCHES (all files on the EON; the feature is fully implemented and integrated, and SHADOW-ONLY by default):
  default (no files)            decode + validate + log to /data/csa_shadow.log, even with PCC disengaged. No control effect.
  /data/tesla_csa_off           overrides every other setting: no control effect, ever. (Logging continues.)
  to enable control later, BOTH:
    /data/tesla_csa_enable      one line: the model name that the evidence supports ('clothoid' or 'offset')
    /data/tesla_csa_evidence    free text citing the DBC file+commit or the validated-vehicle-data write-up (>= 20 chars;
                                its content cannot be checked by code - it is a deliberate, recorded act, not verification)
  With only the enable file, CSA stays shadow-only. Even when enabled, PCC applies it as a floor only (min(), never raises a
  limit), with its braking authority capped at AUTHORITY_CAP (it cannot cause full curve braking), replacing the SQLite
  curve for that cycle only.
"""
import collections
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
EVIDENCE_MIN_CHARS = 20
AUTHORITY_CAP = 0.3       # max share of the curve-brake an unverified-source curve may use (never full braking)
OFF_FILE = '/data/tesla_csa_off'
ENABLE_FILE = '/data/tesla_csa_enable'   # content = model name; absent/blank/unknown = SHADOW only
EVIDENCE_FILE = '/data/tesla_csa_evidence'   # content = citation of the DBC / validated data; required alongside ENABLE_FILE
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


def enabled_model(enable_file=ENABLE_FILE, off_file=OFF_FILE, evidence_file=EVIDENCE_FILE):
    """The model name that may influence control, or None = SHADOW (the default). Needs a known model name in enable_file AND
    a non-trivial evidence citation in evidence_file, and no off_file. The off file overrides everything."""
    try:
        if os.path.exists(off_file):
            return None
        with open(evidence_file) as f:
            if len(f.read().strip()) < EVIDENCE_MIN_CHARS:
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


def limit_authority(authority, csa_binding):
    """Curve-brake authority (0..1). If CSA is the binding curve source it is capped at AUTHORITY_CAP, whatever the fusion
    mode or confidence, so an unverified source can never cause full curve braking."""
    return min(authority, AUTHORITY_CAP) if csa_binding else authority


def classify(tup):
    """What a (c2, c3, range, tspline) frame looks like: 'missing' | 'sna' | 'zero' | 'nonzero'. Used to establish whether
    the car broadcasts these signals at all (carstate initialises them to 0, so 'zero' cannot tell never-received from
    received-as-zero)."""
    if tup is None:
        return 'missing'
    vals = tup[:3]
    if any(not _ok_num(v) for v in vals):
        return 'sna'
    if vals[0] == 0.0 and vals[1] == 0.0:
        return 'zero'
    return 'nonzero'


class ShadowLogger(object):
    """Rate-limited shadow log that records EVERY kind of frame (missing, zero, SNA, valid, rejected) whether or not SQLite
    has a curve and whether or not PCC is engaged. Writes when the frame state changes (at most 1 Hz) or as a 10 s
    heartbeat, and each record carries cumulative per-class counts (per evaluation, i.e. ~1 Hz) so a long all-zero drive
    is visible as such."""
    MIN_INTERVAL_S = 1.0
    HEARTBEAT_S = 10.0

    def __init__(self, path=SHADOW_LOG, max_bytes=SHADOW_LOG_MAX):
        self.path, self.max_bytes = path, max_bytes
        self.last_t = None            # last WRITE
        self.last_eval = None         # last evaluation (the 20 Hz caller is throttled to 1 Hz before any work is done)
        self.last_sig = None
        self.counts = {'road': collections.Counter(), 'offramp': collections.Counter()}

    def maybe_log(self, road, offramp, v_sqlite=None, sqlite_dist=None, v_ego=None, engaged=None, now=None):
        now = time.time() if now is None else now
        if self.last_eval is not None and now - self.last_eval < self.MIN_INTERVAL_S:
            return False              # fast path: no allocation, no I/O between evaluations
        self.last_eval = now
        cr, co = classify(road), classify(offramp)
        self.counts['road'][cr] += 1
        self.counts['offramp'][co] += 1
        rec = shadow_record(road, offramp, v_sqlite, 0.5, sqlite_dist, v_ego)
        rec['t'] = round(now, 2)
        rec['class'] = {'road': cr, 'offramp': co}
        rec['sqlite_present'] = bool(v_sqlite)
        rec['pcc_engaged'] = engaged
        sig = json.dumps([rec['raw'], rec['csa'], rec['sqlite_present'], rec['pcc_engaged']], sort_keys=True, default=str)
        due = self.last_t is None or sig != self.last_sig or now - self.last_t >= self.HEARTBEAT_S
        if not due:
            return False
        self.last_t, self.last_sig = now, sig
        rec['seen'] = {k: dict(v) for k, v in self.counts.items()}
        return log_shadow(rec, self.path, self.max_bytes)


def resolve(CS, v_ego, v_map, approach_fn, enable_file=ENABLE_FILE, off_file=OFF_FILE, evidence_file=EVIDENCE_FILE):
    """The control-side use of CSA, as a pure function. Returns (v_csa, csa_src, csa_why, csa, v_map_out).
    SHADOW (the default - no enable+evidence files, or the off file): v_csa is None and v_map is returned
    UNCHANGED, so control is identical to a build without any CSA code. If a model is enabled and the frame is valid, v_csa
    is the approach speed and the SQLite curve is dropped for this cycle (one source per curve)."""
    csa_src, csa_why, v_csa, csa = 'none', 'shadow', None, None
    try:
        model = enabled_model(enable_file, off_file, evidence_file)
        if model is not None:
            road, off = read_inputs(CS)
            csa, csa_why = best_csa(road, off, model)
            if csa is not None:
                v_csa = approach_fn(csa.speed, csa.dist, v_ego)
                csa_src = csa.source
    except Exception:
        csa, csa_why, v_csa = None, 'error', None
    if v_csa is not None:
        v_map = None
    elif v_map:
        csa_src = 'sqlite'
    return v_csa, csa_src, csa_why, csa, v_map


def combine_nonfusion(v_curve, v_map, v_csa, vis_cs):
    """The curve-limit combination used when curve fusion is off: min() of the sources present. Returns
    (v_curve, csa_binding). With v_csa None this is exactly the legacy min(v_curve, v_map, vis_cs)."""
    if v_map:
        v_curve = min(v_curve, v_map)
    if v_csa is not None:
        v_curve = min(v_curve, v_csa)
    v_curve = min(v_curve, vis_cs)
    return v_curve, (v_csa is not None and v_csa <= vis_cs)


def shadow_tick(logger, CS, sqlite_state, v_ego, engaged, now=None):
    """Log-only. Reads the CSA signals, validates them (via the shadow record) and logs - nothing else. It takes the
    SQLite state as a plain (curvature, dist_to_turn, t) tuple or None and returns whether a line was written; it has no
    access to, and never returns, any control variable. Never raises."""
    try:
        now = time.time() if now is None else now
        road, off = read_inputs(CS)
        v_sql = dist = None
        if sqlite_state is not None and now - sqlite_state[2] < 2.0 and abs(sqlite_state[0] or 0.0) > 1e-5:
            v_sql = math.sqrt(A_Y_CSA / abs(sqlite_state[0]))          # raw SQLite curve speed, comparable with CSA's raw speed
            dist = max(0.0, (sqlite_state[1] or 0.0) - v_ego * (now - sqlite_state[2]))
        return logger.maybe_log(road, off, v_sql, dist, v_ego, bool(engaged), now=now)
    except Exception:
        return False
