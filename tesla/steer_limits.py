"""Opt-in steering-angle limit layer (OFF by default).

Ported idea from the newer Tesla openpilot port (xnor-tech/openpilot, branch tesla-unity): a speed-dependent limit on how
fast the commanded wheel angle may change (separate windup / unwind tables) and a clip to +-20 deg around the MEASURED
angle ("to not fault the EPS"). Our 0.7.5.a carcontroller has its own rate clip commented out, so by default it applies
none of this. This layer is applied at a partial STRENGTH:

    limited = clip(request, last -/+ rate_limit(v), meas -/+ 20)
    out     = request + strength * (limited - request)           strength 0 = request unchanged, 1 = fully limited

Enable:  echo 0.25 > /data/steer_limits_on    (empty file = 0.25; value is clamped to 0..1)
Disable: rm /data/steer_limits_on
With no file the output is exactly the request. Pure functions, no openpilot imports.

Units: the tesla-unity tables are degrees per STEP of its 50 Hz steering message. Ours recomputes the command every control
cycle (100 Hz, DT_CTRL 0.01 s), so the per-step value is scaled by call_dt / 0.02.
"""
FLAG_FILE = '/data/steer_limits_on'
DEFAULT_STRENGTH = 0.25
SRC_STEP_S = 0.02                       # tesla-unity sends the steering request every 2 frames at 100 Hz
CLIP_TO_MEASURED_DEG = 20.              # tesla-unity: request stays within +-20 deg of the measured angle
RATE_BP = [0., 5., 15.]                 # m/s
RATE_UP = [5., 4., 0.75]                # deg per source step while winding up (|angle| growing, same side)
RATE_DOWN = [5., 5.5, 2.4]              # deg per source step otherwise


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      return fp[i - 1] + (fp[i] - fp[i - 1]) * (x - xp[i - 1]) / (xp[i] - xp[i - 1])
  return fp[-1]


def read_strength(path=FLAG_FILE):
  """0.0 when the flag file is absent (layer off); else its value (default 0.25), clamped to 0..1"""
  try:
    with open(path) as f:
      txt = f.read().strip()
  except Exception:
    return 0.0
  if not txt:
    return DEFAULT_STRENGTH
  try:
    return min(1.0, max(0.0, float(txt.split()[0])))
  except ValueError:
    return DEFAULT_STRENGTH


def limit_angle(request, last, measured, v_ego, strength, call_dt=0.01):
  """-> angle to send. strength <= 0 returns the request untouched."""
  if strength <= 0.:
    return request
  steer_up = last * request >= 0. and abs(request) > abs(last)
  tbl = RATE_UP if steer_up else RATE_DOWN
  lim = _interp(max(v_ego, 0.), RATE_BP, tbl) * (call_dt / SRC_STEP_S)
  limited = min(max(request, last - lim), last + lim)
  limited = min(max(limited, measured - CLIP_TO_MEASURED_DEG), measured + CLIP_TO_MEASURED_DEG)
  return request + min(1.0, strength) * (limited - request)
