#!/usr/bin/env python3
"""
pcc_curves.py - speed planning helpers for the pedal cruise control (PCC_module.py). No openpilot imports, so it can
be unit-tested on any machine (see tests/test_pcc_curves.py).

Why this exists (what the old PCC did wrong):
  - curve limits were applied as "the tightest curve anywhere ahead, right now" (vision) or with the time axis
    reversed (map: full set speed AT the curve, curve speed 8 s BEFORE it). A curve limit only matters as a braking
    problem: how fast may I go NOW so that, slowing at a comfortable rate, I reach the curve at its safe speed?
        v_allowed(d) = sqrt(v_curve^2 + 2 * A_CURVE_DECEL * max(0, d - v_ego * CURVE_REACTION_S))
    Both map and vision limits below are expressed that way, so the car starts slowing early and smoothly.
  - without a lead car the deceleration limit was ~0.5 m/s2, too weak to get down to a curve speed in time.
    CURVE_BRAKE is the deceleration the PCC may use while a curve limit is the binding constraint.
  - acceleration limits depended on the planner tables x 0.4 (no lead), weak and uneven. ACCEL_* give a simple,
    speed-dependent comfort profile -> close to linear speed build-up.
  - the pedal -> acceleration conversion was a fixed accel / 3. A Tesla's response to pedal drops with speed (power
    limited), so the PID under-drove at speed. PedalMap learns acceleration per unit of pedal per speed band.
All numbers are tunable at the top of this file.
"""
import json
import math

# --- curves ---------------------------------------------------------------------------------------------------
A_Y_MAP = 1.85             # m/s2 lateral, must match mapcurved.py (it stores speed, curvature = A_Y_MAP / speed^2)
A_CURVE_DECEL = 1.0        # m/s2 used to plan the approach to a curve (comfortable, reachable with regen alone)
CURVE_BRAKE = 1.6          # m/s2 the PCC may decelerate while a curve is the binding limit (regen max is ~2)
CURVE_REACTION_S = 2.0     # s of margin: the plan assumes slowing starts this much later (covers controller lag)
CURVE_WARN_OVER = 4.0      # m/s (~15 km/h) above the curve approach speed: regen alone can't make it -> tell the driver
VISION_RANGE_M = 192.      # use the whole camera path (old model: 192 points, 1 m apart): from 90 km/h a tight bend
                           # needs ~260 m to slow at 1 m/s2, so cutting the path short sees it too late
VISION_FIT_HALF = 10       # curvature from a least-squares quadratic over +-10 m of path (Savitzky-Golay): noise on the
                           # model path would otherwise turn into curvature spikes and random slowdowns
VISION_MIN_V = 20. * 0.44704  # never ask for less than 20 mph from vision alone (same floor as before)
VISION_AY_SCALE = 1.0      # 1.0 = the old formula (2.975 - 0.0375 v) m/s2 lateral; lower = slower in curves

# --- acceleration profile (no lead car) -------------------------------------------------------------------------
ACCEL_BP = [0., 15., 30.]  # m/s
ACCEL_V = [1.2, 1.0, 0.8]  # m/s2 max acceleration without a lead (was planner table x 0.4)
ACCEL_LAT_BLOCK = 1.5      # m/s2 lateral (from steering angle): above this, don't accelerate (was: steering > 10 deg)

# --- learned pedal response -------------------------------------------------------------------------------------
PEDAL_K_DEFAULT = 3.0      # m/s2 per full pedal: the old fixed accel / 3
PEDAL_K_MIN, PEDAL_K_MAX = 1.0, 6.0
PEDAL_BP = [0., 5., 10., 15., 20., 25., 30., 35., 40.]   # m/s speed bands
PEDAL_ALPHA = 0.01         # learning rate per accepted sample (20 Hz -> ~5 s of steady acceleration to move halfway)
PEDAL_DELAY_STEPS = 6      # acceleration answers the pedal ~0.3 s later (6 x 50 ms)


def interp(x, xp, fp):
  if x <= xp[0]: return fp[0]
  if x >= xp[-1]: return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = (x - xp[i - 1]) / (xp[i] - xp[i - 1]); return fp[i - 1] + t * (fp[i] - fp[i - 1])
  return fp[-1]


def approach_speed(v_curve, dist_m, v_ego):
  """max speed now so that slowing at A_CURVE_DECEL reaches v_curve at dist_m ahead"""
  d = max(0., dist_m - max(v_ego, 0.) * CURVE_REACTION_S)
  return math.sqrt(max(v_curve, 0.) ** 2 + 2. * A_CURVE_DECEL * d)


def map_curve_limit(curvature, dist_m, v_ego):
  """speed limit (m/s) from a mapped curve (liveMapData.curvature / distToTurn), or None"""
  k = abs(curvature or 0.)
  if k < 1e-5: return None
  return approach_speed(math.sqrt(A_Y_MAP / k), max(0., dist_m or 0.), v_ego)


def vision_ay_max(v_ego):
  return max(1.0, (2.975 - 0.0375 * v_ego)) * VISION_AY_SCALE


def _sg_derivs(np, y, half):
  """first and second derivative of y (1 m spacing) from a least-squares quadratic over +-half points (Savitzky-Golay);
  ends use the nearest full window"""
  i = np.arange(-half, half + 1, dtype=np.float64); m2 = np.mean(i ** 2)
  c1 = i / np.sum(i ** 2); c2 = 2. * (i ** 2 - m2) / np.sum((i ** 2 - m2) ** 2)
  n = len(y); w = 2 * half + 1
  if n < w: return None, None
  d1 = np.convolve(y, c1[::-1], mode='valid'); d2 = np.convolve(y, c2[::-1], mode='valid')
  return (np.concatenate([np.full(half, d1[0]), d1, np.full(half, d1[-1])]),
          np.concatenate([np.full(half, d2[0]), d2, np.full(half, d2[-1])]))


def vision_curve_limit(np, points, v_ego):
  """speed limit (m/s) from the camera model path (y offset per metre ahead), or None.
  Curvature from a local least-squares fit (noise-robust), then each point's curve speed is turned into an approach
  speed for its distance - a bend 100 m ahead no longer slows you as hard as one 20 m ahead."""
  if points is None or len(points) < 20: return None
  y = np.asarray(points, dtype=np.float64)[:int(VISION_RANGE_M)]
  yp, ypp = _sg_derivs(np, y, VISION_FIT_HALF)
  if yp is None: return None
  k = np.abs(ypp / (1. + yp ** 2) ** 1.5)
  x = np.arange(len(y), dtype=np.float64)
  v_c = np.sqrt(vision_ay_max(v_ego) / np.clip(k, 1e-4, None))
  d = np.maximum(0., x - max(v_ego, 0.) * CURVE_REACTION_S)
  v_allow = np.sqrt(v_c ** 2 + 2. * A_CURVE_DECEL * d)
  return max(VISION_MIN_V, float(np.min(v_allow)))


def accel_max_no_lead(v_ego):
  return interp(v_ego, ACCEL_BP, ACCEL_V)


def lateral_accel(v_ego, angle_steers_deg, steer_ratio, wheelbase):
  """lateral acceleration (m/s2) implied by speed and steering wheel angle (bicycle model)"""
  if not steer_ratio or not wheelbase: return 0.
  return v_ego ** 2 * abs(math.tan(math.radians(angle_steers_deg / steer_ratio))) / wheelbase


class PedalMap:
  """acceleration per unit of pedal (above the zero-torque point), learned per speed band while the PCC accelerates.
  Used as gb = accel / k(v) for positive accelerations; decelerations keep the old / 3. Starts at the old constant,
  so it behaves exactly like before until it has learned something. Kill switch: touch /data/pcc_pedalmap_off."""
  def __init__(self, path=None):
    self.path = path; self.k = [PEDAL_K_DEFAULT] * len(PEDAL_BP); self.hist = []; self.n = 0   # hist: (pedal, ok)
    if path:
      try:
        d = json.load(open(path))
        if d.get('bp') == PEDAL_BP and len(d.get('k', [])) == len(PEDAL_BP):
          self.k = [min(PEDAL_K_MAX, max(PEDAL_K_MIN, float(v))) for v in d['k']]; self.n = int(d.get('n', 0))
      except Exception:
        pass

  def gain(self, v_ego):
    return interp(v_ego, PEDAL_BP, self.k)

  def update(self, pedal_frac, a_ego, v_ego, ok):
    """pedal_frac: last commanded pedal above zero (0..1); ok: conditions allow learning (PCC accelerating, no brake,
    no close lead, no steep jerk). The pedal from PEDAL_DELAY_STEPS ago is matched with the acceleration now, and only
    if the conditions held for that whole window."""
    self.hist.append((pedal_frac, bool(ok)))
    if len(self.hist) > PEDAL_DELAY_STEPS: self.hist.pop(0)
    if len(self.hist) < PEDAL_DELAY_STEPS or not all(h[1] for h in self.hist) or v_ego < 3.: return
    u = self.hist[0][0]
    if u < 0.08 or a_ego < 0.15: return
    k_meas = min(PEDAL_K_MAX, max(PEDAL_K_MIN, a_ego / u))
    for i in range(len(PEDAL_BP)):                      # spread the sample over the two neighbouring speed bands
      lo = PEDAL_BP[i - 1] if i else PEDAL_BP[0] - 5.; hi = PEDAL_BP[i + 1] if i + 1 < len(PEDAL_BP) else PEDAL_BP[-1] + 5.
      if lo < v_ego < hi:
        wgt = 1. - abs(v_ego - PEDAL_BP[i]) / ((hi - lo) / 2.)
        if wgt > 0: self.k[i] += PEDAL_ALPHA * wgt * (k_meas - self.k[i])
    self.n += 1

  def save(self):
    if not self.path: return
    try:
      json.dump({'bp': PEDAL_BP, 'k': [round(v, 3) for v in self.k], 'n': self.n}, open(self.path, 'w'))
    except Exception:
      pass
