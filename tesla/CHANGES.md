# Pre-AP Tesla pedal cruise (PCC): curve slowing and smooth acceleration

The first commit of this folder is the code exactly as it was on the car; the next commit holds the changes below,
so `git diff` shows every change.

**This is longitudinal control of a car.** The changes were tested with unit tests and an offline simulation of
`PCC_module.py` (simplified stand-ins for openpilot and the car, see the bottom), not on the road. Try them first on an
empty road you know, foot over the brake. Every new behaviour has a kill switch.

## What was wrong

| # | Problem | Effect |
|---|---|---|
| 1 | `mapcurved.py` needs `liveLocationKalman`, which never had a valid fix on this car (`mapcurve_check.log`: `posValid=False` on all 4,146 lines) | the map database never slowed the car |
| 2 | `max_v_in_mapped_curve_ms` interpolated time-to-turn the wrong way: full set speed **at** the turn, curve speed 8 s **before** it | even with GPS: slow early, speed up into the curve |
| 3 | `liveMapData` (4 Hz) read with `recv_one_or_none` in a 20 Hz loop | map limit seen 1 cycle in 5 - flickered, smoother ignored it |
| 4 | Without a lead the deceleration limit was ~0.5 m/s2 | 90 -> 50 km/h takes ~22 s / ~400 m: too late for most curves |
| 5 | Vision curve limit = tightest point anywhere in the 192 m path, applied immediately, from raw second differences | noise spikes slow you randomly; a close bend can't be slowed for in time |
| 6 | Speed frozen whenever steering > 10 deg | no acceleration *and* no slowing in every gentle bend |
| 7 | No lead -> acceleration limit = planner table x 0.4 (and speed dependent) | weak, uneven acceleration |
| 8 | Fixed pedal conversion `accel / 3` | Tesla's pedal response drops with speed (power limit): under-driven at speed |
| 9 | `calc_follow_speed_ms` returned `(None, None, None)` before the first `radarState` | `TypeError` in `min()` (reproduced in the simulation) |

## What changed

**`pcc_curves.py` (new, no openpilot imports, unit-tested)**
- Every curve limit (map and camera) is now a braking problem: the fastest speed *now* from which slowing at
  `A_CURVE_DECEL` (1.0 m/s2) still reaches the curve's speed at the curve:
  `v = sqrt(v_curve^2 + 2 * A_CURVE_DECEL * (distance - v_ego * CURVE_REACTION_S))`, `CURVE_REACTION_S` = 2 s margin.
- Camera curvature from a least-squares quadratic over +-10 m of path (Savitzky-Golay) over the whole 192 m path:
  5 cm of path noise on a straight road no longer causes slowing; real bends still do.
- Acceleration without a lead: 1.2 / 1.0 / 0.8 m/s2 at 0 / 54 / 108 km/h (`ACCEL_BP`, `ACCEL_V`).
- `PedalMap`: learns acceleration per unit of pedal for each speed band while the PCC accelerates on its own (no brake,
  no human pedal, no close lead, steering < 15 deg; pedal matched with the acceleration 0.3 s later, conditions must
  hold for that whole window). Starts at the old `/ 3`, so it behaves like before until it has learned. Saved to
  `/data/params/pccPedalMap.json` when PCC disengages. Only used for accelerating; slowing keeps `/ 3`.

**`PCC_module.py`**
- Map limit (#2) and caching of the last `liveMapData` for 2 s with the distance reduced by what you drove since (#3).
- `CURVEBRAKE`: while a curve is the binding limit and you are above its approach speed, deceleration up to
  `CURVE_BRAKE` (1.6 m/s2) is allowed (#4).
- `CURVEWARN`: the pedal can only use regen. When you are more than `CURVE_WARN_OVER` (~15 km/h) above a curve's approach
  speed, the cluster shows **"Sharp curve - brake!"** (at most every 3 s) - only the driver's brake can do more.
- Vision limit via `pcc_curves.vision_curve_limit` (#5).
- Instead of freezing at > 10 deg steering: no acceleration when the steering means more than 1.5 m/s2 lateral
  acceleration (`ACCEL_LAT_BLOCK`), turn signal or ALCA as before (#6).
- No-lead acceleration profile (#7), learned pedal map in `tesla_compute_gb` (#8), no-radar crash (#9).
- With a lead car, the following logic and its limits are unchanged.

**`mapcurved.py`**
- Falls back to `gpsLocationExternal`, which `carcontroller.py` publishes from the car's own GPS on CAN when
  **`use_tesla_gps = True`** (#1). Fixes worse than 20 m or older than 2 s are ignored.
- Publishes the curve point that limits the speed *now* (same braking rule) instead of the slowest point within 400 m.
- Points only from a corridor along the heading (10 m + 30% of the distance to each side) instead of a 50 deg cone,
  which at 400 m spans ~650 m and reaches parallel and side streets.

## Install

```
cd /data/openpilot/selfdrive/car/tesla
cp PCC_module.py PCC_module.py.before_curves         # keep the old one
# copy from this folder: PCC_module.py pcc_curves.py mapcurved.py
```

In `/data/bb_openpilot.cfg` set `use_tesla_gps = True` (then reboot), and keep `mapcurved.py` running (it isn't in the
process manager yet - see the note at the bottom of `mapcurved.py`; until then start it by hand:
`cd /data/openpilot && PYTHONPATH=/data/openpilot python3 selfdrive/car/tesla/mapcurved.py -v` shows what it finds).
`/data/mapcurve.sqlite` is used as before (built by your `mapcurve_build.py`).

## Kill switches and debug

| File | Effect |
|---|---|
| `touch /data/curve_limit_off` | no camera curve limit (as before) |
| `touch /data/pcc_pedalmap_off` | fixed `accel / 3` again |
| `touch /data/mapcurve_debug` | prints map-curve decisions (as before) |
| `rm /data/params/pccPedalMap.json` | forget the learned pedal response |

Tuning constants are at the top of `pcc_curves.py`.

## Tests

```
python3 -m unittest tesla/tests/test_pcc_curves.py tesla/tests/test_mapcurved.py     # 18 tests, no openpilot needed
```

Offline simulation of the old and new `PCC_module.py` (openpilot pieces replaced by simplified stand-ins, a simple
power-limited Model S with regen; not committed):

| Scenario | Old | New |
|---|---|---|
| 50 -> 95 km/h, no lead | ~0.3 m/s2, not at 95 km/h after 40 s | steady 0.9-1.0 m/s2, 15 s |
| mapped 40 km/h curve from 90 km/h | reaches it at 90 km/h | starts slowing ~250 m before, reaches it at 43 km/h |
| 50 m radius bend seen only by the camera, from 90 km/h (safe ~36 km/h) | 84 km/h | 40 km/h + "Sharp curve - brake!" |
| before the first radarState | `TypeError` crash | runs |
| following a slower car | - | identical to old |

The simulation's speed smoother is simpler than openpilot's, so it trails a falling target by a few km/h; on the car
the curve entry speed may be closer to the target. The real numbers will differ: car, road grade and openpilot's own
controller are not modelled.

## Not changed / not possible

- The car's own navigation sends curve data (`UI_csaRoadCurvature`) but its values showed no relation to the curvature
  actually driven in the 25 Oct log, so it is not used.
- Vision: the camera model's path is used (improved above). Detecting pedestrians/cyclists from vision would need a
  newer openpilot driving model; the old one only gives path, lanes and a lead car.

# Auto high beam (AHB) on a pre-AP car with MCU2

**Why it never worked:** `AHB_module.py` only acts when the MCU sends `MCU_chassisControl.MCU_ahlbEnable = 1`, the
setting from the Autopilot menu. Your MCU2 has no Autopilot menu and sends 0 (decoded from your car: `60 02 10 3c c1
cf`), so AHB stopped at its first check. The decision was also only sent with `has_tesla_ic_integration = True`, which
on this car would take lane change (`MCU_pedalSafetyEnable`) and the speed limit from MCU settings it never sends.

**How it works (Tesla's design, unchanged):** the system can only **dip** the beams. "On" is *undecided*, meaning your
stalk decides - so for auto high beam you push the stalk to high beam and leave it; it dips for traffic.

| File | Effect |
|---|---|
| `touch /data/ahb_auto` | AHB without the MCU setting and without IC integration; high beam allowed from **30 km/h**, dipped again below **20 km/h** (the gap stops flicker) |
| `touch /data/ahb_test` | Parked, lights on, stalk in high beam: dips and releases every 2 s - shows whether your light controller obeys the (fake) Autopilot computer at all. `rm` it after |

Unchanged on purpose: dipping for oncoming/preceding traffic (radar car within 100 m, camera lead, bright camera image
= headlights/street lights, 5 s hold). Without them it would dazzle other drivers.

**Test first, parked:** `touch /data/ahb_test`, restart openpilot (or reboot), lights on, stalk in high beam. If the beams
dip every 2 s, the car obeys and `/data/ahb_auto` will work. If nothing happens, the light controller ignores the fake
Autopilot computer on a no-AP config (like the cluster) - tell me. Tests: `python3 tesla/tests/test_ahb.py` (11).
