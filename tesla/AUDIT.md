# Code audit — driving-path changes

Line-by-line reviews of the changes that actually affect how the car drives, done against the stock
BogGyver/openpilot 0.7.5a tree. Reviews only; any fixes are noted with their commit.

---

## PCC longitudinal control — `PCC_module.py`, `pcc_curves.py`

Reviewed the full diff of `PCC_module.py` vs stock 0.7.5a and all of `pcc_curves.py`.

**Verdict: no correctness bugs in the braking / curve / follow path. No fix needed.** The acceleration behaviour was
left unchanged (deliberately — it is the tuned, liked behaviour already on the car).

### Changes that are themselves bug fixes (old code was worse)

- **`_sec_til_collision`: `min(0.1, …)` → `max(0.1, …)`** — the old form capped "seconds to collision" at 0.1 for
  *any* closing lead, so the decel table saw near-maximum braking even for a far lead closing slowly. `max` makes
  braking proportional: gentle for a distant/slow-closing car, still full strength for a close/fast one (the 0.1
  floor plus the close-lead branches above handle the dangerous case). Safe direction, and part of why it is smoother.
- **`calc_follow_speed_ms` no-lead case (NOLEADFIX)** — stock returned `(None, None, None)` before the first
  `radarState`, which then crashed in `min()`. The fix sets `lead_dist_m = 0`, which routes into the explicit
  "no lead → accelerate to set speed" branch (`lead_dist_m == 0`). Correct; only relevant at startup.
- **`self.Loc.pid.d` → `self.LoC.pid.d`** — a real typo that would raise `AttributeError` if that config branch ran.

### New curve-slowing — correct and fail-safe

- Approach-speed math is correct: `v_allowed = sqrt(v_curve² + 2·A_CURVE_DECEL·d)` with a reaction-time margin.
  Map and vision limits both use it (`pcc_curves.approach_speed`).
- The map limit fixes the stock reversed time axis (full set speed *at* the turn, curve speed 8 s *before*).
- Vision curvature uses a local least-squares (Savitzky-Golay) fit over ±10 m — derivative coefficients verified
  correct — instead of the single global cubic, so a straight-then-sharp bend is not smeared out.
- `CURVE_BRAKE` only *enables* stronger regen when above the curve speed; it permits, it does not force.
- `_curve_speed` returns "no limit" (255) on any error, missing points, or the kill-switch file
  (`/data/curve_limit_off`), so a bad/empty model path can **never cause a spurious brake** — worst case it just
  does not slow for the curve.

### Notes (not bugs)

- **Minor:** `pcc_curves.PedalMap.update` matches acceleration with the pedal from 5 steps ago, not the 6 the comment
  claims (0.25 s vs 0.3 s). Harmless; left as-is since the acceleration behaviour is intentionally unchanged.
- **Important real-world point:** the **vision** curve-slowing only does anything once the **0.8.3 model** is deployed
  — the stock 0.7.5 model does not publish the path points it reads (only with `DEBUG`), so the feature is currently
  dormant (returns "no limit"). The **map** curve-slowing only works where there is valid `liveMapData`. Both are
  fail-safe, so nothing brakes falsely while dormant.

### Containment of the learned pedal map

Only affects positive acceleration (`gb = accel / k(v)`); deceleration keeps the fixed `/3`. `k` is clamped to
[1.0, 6.0], learned slowly (EMA α = 0.01), and has a kill switch (`/data/pcc_pedalmap_off`). It cannot affect braking
authority.

---

## 0.8.3 model port — `model083/`

Reviewed `model083_adapter.h`, `driving.cc`, `driving.h` in full against the real 0.8.3 `driving.cc` and the 0.7.5a
consumer (`driving.cc`, `radard.py`, `radar_helpers.py`).

**Verdict: sound. One minor bug found and fixed.**

- Output-layout constants (`*_IDX`, group sizes, `OUTPUT_SIZE`, `TEMPORAL_SIZE = 512`) match 0.8.3 exactly.
- Lead **relative** velocity confirmed correct in both versions (`vLead = v_ego + v`); the `y` sign flip and the
  dropped ×10 distance scale line up with the 0.7.5 consumer.
- Path/plan, lane lines, longitudinal, road edges, pose: indices and conversions verified; rotation correctly left in
  radians (0.8.3 is radians; the old posenet converted degrees).
- **Fixed (commit 0672ca2):** `desirePrediction` was not shifted to 0.7.5's `[desire-1]` indexing while `desireState`
  was — an off-by-one. It only feeds the lane-departure-warning HUD flags when **disengaged** (no steering effect),
  but it was wrong. Now both desire fields are shifted the same way, with regression tests in `test_adapter.cc`.

### Steering path is unchanged by the port

The model only produces perception. The lateral planner, lateral controller, `carcontroller` steering, the steering
CAN message, and the **panda** are all untouched. This fork steers by **angle** (not torque); the angle command,
its `carcontroller` clip/rate limits and the panda's `TESLA_MAX_ANGLE` clamp are identical with either model. A new
model can only change the requested angle *value* (within the same limits), never how it is sent or checked.
