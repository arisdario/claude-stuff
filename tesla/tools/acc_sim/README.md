# ACC closed-loop simulator (no car needed)

Runs the REAL `tesla/ACC_module.py` (`update_acc`, unchanged) against:

* openpilot 0.7.5's longitudinal MPC built from its own generated code (`libmpc1.so`) and the planner logic (`replay.py`,
  `speed_smoother.py`), with the lead distance passed through the real `_gap_distance()` of `tools/patch_planner_gap.py`;
* a radar model with the errors measured on this car (radarState vRel ~3 km/h low, +-2 km/h scatter, +25 km/h blips,
  distance noise, far-track jumps, drop-outs);
* a Tesla cruise model from the logs: a = 0.9 x set-speed error, +-0.6 m/s2, jerk 0.8 m/s3, set change visible 0.2 s
  after the press, RES = previous set speed, cancel = regen -1.5 m/s2. The plan arrives one cycle (50 ms) late.
* nobody brakes (worst case).

13 scenarios x 60 / 90 / 120 km/h x 3 random seeds; every failure of every run is listed.

    ./build_libmpc.sh /path/to/openpilot-v0.7.5                 # once
    git show d15539e:tesla/ACC_module.py > ACC_before.py        # the version on the car, for comparison
    python3 harness.py new      # -> failures per run, results_new.json
    python3 harness.py before
    python3 plots.py            # -> plots/<scenario>.png (speed, accel, jerk, distance; both versions)

This is a model. It shows logic errors and relative behaviour; it does not prove anything about the real car.
