"""IC "raw lanes" view (display only, off by default).

The instrument cluster draws ONE lane pair: a centre curve (4 polynomial coefficients) plus a lane width and a 0..3
confidence per line (DAS_obj_lane, 0x557). Normally those come from pathPlan, i.e. after the 0.7.5 lane planner has
blended path and lanes and estimated the lane width. With /data/ic_raw_lanes present they come straight from the
model's own left/right lane polynomials, so what you see on the cluster is what the camera model sees, without the
planner in between. A swaying/flickering cluster lane then means the model output is unsteady; a steady one while the
car still wiggles points at the planner/controller.

Pure functions, no openpilot imports (tested off-car). Nothing here touches steering, braking or acceleration.
Polynomials are the 0.7.5 order [c3, c2, c1, c0], y positive to the left, as in ModelData.leftLane.poly.
"""
RAW_LANES_FILE = '/data/ic_raw_lanes'
MAX_MODEL_AGE_S = 0.5      # older than this = stale: fall back to the normal pathPlan view
MIN_PROB = 0.25            # same lowest level as the existing 0..3 buckets (below it the line is 'not seen')


def level(prob):
  """0..3 confidence bucket, same thresholds as carcontroller.handlePathPlanSocketForCurvatureOnIC"""
  if prob > 0.75: return 3
  if prob > 0.5: return 2
  if prob > 0.25: return 1
  return 0


def raw_lane_ic(left_poly, right_poly, left_prob, right_prob, fallback_width):
  """-> (lLine, rLine, c3, c2, c1, c0, width) of the model's raw lanes, or None when neither lane is seen.
  Centre curve = mean of the two lane polynomials (a single seen lane is shifted by half the fallback width).
  Width = left y0 - right y0 when both are seen, else fallback_width. Values are NOT sign-flipped or clipped here:
  the caller applies the same IC conversion it uses for pathPlan."""
  if left_poly is None or right_poly is None or len(left_poly) < 4 or len(right_poly) < 4:
    return None
  l_ok = left_prob > MIN_PROB
  r_ok = right_prob > MIN_PROB
  if not l_ok and not r_ok:
    return None
  if l_ok and r_ok:
    centre = [(l + r) / 2. for l, r in zip(left_poly[:4], right_poly[:4])]
    width = left_poly[3] - right_poly[3]
  elif l_ok:
    centre = list(left_poly[:4]); centre[3] -= fallback_width / 2.
    width = fallback_width
  else:
    centre = list(right_poly[:4]); centre[3] += fallback_width / 2.
    width = fallback_width
  return (level(left_prob), level(right_prob), centre[0], centre[1], centre[2], centre[3], width)
