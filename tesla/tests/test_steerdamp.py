"""LANEWIGGLE steering damping: the rate limit scales with lane confidence. Mirrors the formula in carcontroller.py
(interp(lane_prob, LANE_CONF_BP, [angle_rate_lim, LANE_DAMP_RL]) then clip to last_angle +- rl)."""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))


def interp(x, xp, fp):
  if x <= xp[0]: return fp[0]
  if x >= xp[-1]: return fp[-1]
  for i in range(1, len(xp)):
    if x <= xp[i]:
      t = (x - xp[i-1]) / (xp[i] - xp[i-1]); return fp[i-1] + t*(fp[i]-fp[i-1])
  return fp[-1]


def clip(v, lo, hi): return max(lo, min(hi, v))

LANE_CONF_BP = [0.2, 0.5]
LANE_DAMP_MULT = 8.0


def damp(desired, last_angle, lane_prob, angle_rate_lim):
  if lane_prob >= LANE_CONF_BP[1]:
    return desired                                  # lane visible: no limit
  rl = interp(lane_prob, LANE_CONF_BP, [angle_rate_lim, angle_rate_lim * LANE_DAMP_MULT])
  return clip(desired, last_angle - rl, last_angle + rl)


class T(unittest.TestCase):
  def test_visible_lane_no_limit(self):
    # lane prob high -> rl huge -> command passes unchanged even on a big jump
    self.assertAlmostEqual(damp(30.0, 0.0, 0.9, 0.25), 30.0)

  def test_blind_clamps_to_rate(self):
    # lane prob low -> rl = angle_rate_lim -> a jittery jump is clamped
    self.assertAlmostEqual(damp(30.0, 0.0, 0.1, 0.25), 0.25)
    self.assertAlmostEqual(damp(-30.0, 0.0, 0.1, 0.25), -0.25)

  def test_blind_small_move_passes(self):
    # a move within the rate limit is untouched (deliberate slow steering still works when blind)
    self.assertAlmostEqual(damp(0.2, 0.0, 0.1, 0.25), 0.2)

  def test_damping_ramps_with_confidence(self):
    tight = damp(30.0, 0.0, 0.2, 0.25)     # fully blind -> clamped to the windup rate
    mid = damp(30.0, 0.0, 0.35, 0.25)      # partial -> looser but still clamped
    self.assertAlmostEqual(tight, 0.25)
    self.assertGreater(mid, tight); self.assertLess(mid, 30.0)

  def test_matches_carcontroller_constants(self):
    import re
    src = open(os.path.join(os.path.dirname(__file__), '..', 'carcontroller.py')).read()
    self.assertIn('LANE_CONF_BP = [0.2, 0.5]', src)
    self.assertIn('interp(self.lane_prob, LANE_CONF_BP, [angle_rate_lim, angle_rate_lim * LANE_DAMP_MULT])', src)
    self.assertIn("STEERDAMP_OFF_FILE = '/data/steerdamp_off'", src)


if __name__ == '__main__':
  unittest.main()
