# Code audit — driving-path changes

Line-by-line reviews of the changes that actually affect how the car drives, done against the stock
BogGyver/openpilot 0.7.5a tree. Reviews only; any fixes are noted with their commit.

---

## PCC longitudinal control — `PCC_module.py`, `pcc_curves.py`

Reviewed the full diff of `PCC_module.py` vs stock 0.7.5a and all of `pcc_curves.py`, then re-audited from scratch:
every name the new code reads checked against `carstate.py`/`interface.py`, the call order checked, and the curve
limiter run end to end on the real 0.8.3 adapter output.

**Verdict (re-audit): one real bug, fixed — phantom curve slowing with the 0.8.3 model.** The first pass of this audit
said "no bugs, no fix needed"; that was incomplete because it never ran the curve limiter on what the 0.8.3 adapter
actually publishes. The acceleration behaviour was left unchanged (deliberately — it is the tuned, liked behaviour
already on the car).

### Bug fixed (PATHEND): phantom curve slowing with the 0.8.3 model

- **Cause:** the vision curve limiter used all 192 path points. The 0.8.3 plan only reaches ~10 s ahead, so below
  ~69 km/h it stops short of 192 m and the adapter holds the path flat past that point (marked with
  `NO_DATA_STD = 1e3`). In a curve that flat tail leaves a kink, which the limiter read as a sharp bend.
- **Effect, measured on 153 speed × radius cases through the real adapter + `pcc_curves`:** acceleration capped far too
  low (43 km/h on a 400 m curve capped at 61 km/h; its safe speed is 114 km/h), and **7 spurious regen-braking cases**
  between 58 and 68 km/h on 150–400 m curves (68 km/h on a 200 m curve braked toward 60 km/h; safe speed 77 km/h).
  No false "Sharp curve - brake!" alert. Dormant with the stock model (it publishes no path points).
- **Fix:** `vision_curve_limit` takes the path stds and stops at the first "no data" point (`vision_valid_len`,
  threshold `VISION_NO_DATA_STD = 100`); `PCC_module._curve_speed` passes `model.path.stds`. Without stds (stock model)
  behaviour is unchanged. Result on the same 153 cases: 0 spurious brakes, never above a curve's real safe speed, and
  real tight bends inside the prediction still slow (65 km/h into an 80 m curve → 48 km/h limit). Nothing real is lost:
  past the prediction the adapter's flat path carried no curve information.
- **Tests:** `tesla/tests/test_pcc_curves.py` — the adapter-shaped case (brakes without stds, not with them), a real
  bend inside the prediction still slows, stds optional.

### Re-verified in the re-audit

- Every attribute the new code reads exists at runtime (`a_ego`, `brake_pressed`, `angle_steers`, `CP.steerRatio`,
  `CP.wheelbase`, …), and the 4-argument `custom_alert_message(status, message, duration, sound)` matches its
  signature — no hidden `AttributeError`/`TypeError` in the control loop.
- `calc_cruise_accel_limits(v_ego, following, is_tesla)` accepts the new third argument.
- The curve-brake limits are applied *before* `speed_smoother` consumes `accel_limits`/`jerk_limits` — effective, not a
  no-op.
- The pedal map is live: `LongControl(CS.CP, tesla_compute_gb)`, and 0.7.5's PI controller passes its whole output
  through that conversion.

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
- **Regen onset:** `PEDAL_MAX_DOWN` (SMOOTH) now takes 0.7 s for a full pedal release instead of 0.4 s — part of the
  smooth feel, but it adds roughly 0.1 s before full regen when lifting off from cruise.
- **Top acceleration:** `interface.py` `PIDTUNE` caps gas at a flat 30 % of pedal (`gasMaxV = 0.3`), so the maximum
  acceleration is 0.3 × the learned pedal gain: about 0.9 m/s² at the default gain, lower at speed if the map learns a
  smaller gain.
- **`APGATE` (`carstate.py`):** `apEnabled` now stays `True`, so the "AP disabled in MCU" engagement check in
  `interface.py` can never block. Needed on pre-AP (no such MCU setting), but that check is gone.

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
- **Desire input** (re-audit, checked in source rather than from the adapter's comment): 0.7.5's `modeld.cc` sets
  `vec_desire[pathPlan.desire - 1]`, 0.8.3's sets `vec_desire[desire]`, so `desire_to_083`'s +1 shift is correct.
  This matters most of the desire handling: it shapes the path the model steers on during a lane change.
- `modeld.cc` only passes the output struct through to `model_publish`/`posenet_publish`, so the slimmed
  `ModelDataRaw { raw }` is safe. Pose matches 0.8.3's own `posenet_publish` exactly; `leads[1]` (2 s) → `leadFuture`.
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
