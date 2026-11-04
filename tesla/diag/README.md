# Steering diagnostics (read-only)

Finds where a left-right steering oscillation starts: **camera -> model -> pose/path -> localization -> controller -> CAN command -> physical response.**
It only *observes*: cereal subscriptions, `/proc`, `/sys`, `/dev/c920_live.jpg`. It never publishes, never writes a Param (except its own
`DiagEnabled` switch file), never touches CAN transmit, never imports control code (a test enforces this). It runs as one extra process at
`nice +10`; all disk I/O happens in its own writer thread; when it cannot keep up it drops *its own* records (counted) and never waits.

## Use (on the EON, openpilot running; **bench or private road only - not on public roads while the wiggle is unresolved**)

    cd /data/openpilot/tesla/diag        # (or wherever this folder was copied)
    ./start_diagnostics.sh check         # 6 s: which cereal fields exist in THIS openpilot build (run once, send me the output)
    ./start_diagnostics.sh               # creates /data/steering_diagnostics/drive_YYYYMMDD_HHMMSS/ and prints RECORDING ACTIVE
    ./start_diagnostics.sh status        # is it still recording, counts, free disk
    echo 0 > /data/params/d/DiagEnabled  # pause (1 = resume)
    ./stop_diagnostics.sh                # flush, analyse, plot, build drive_....zip  -> send me the zip
    python3 overhead_check.py            # does the logger disturb camerad/modeld/controlsd?  (run parked, once)

If the shell cannot execute the files directly use `sh ./start_diagnostics.sh`. If power is lost mid-drive, run `./stop_diagnostics.sh`
afterwards: it finalizes whatever was written (line-flushed segments) and builds the zip.

## What is recorded

| folder | content | rate |
|---|---|---|
| `camera/` | camerad `frame` (frameId, timestampEof, frameLength, integLines, gain, lens), image quality (brightness, contrast, sharpness, saturation, over/under-exposed %, directional-blur ratio), USB (C920 present, speed Mbit/s, bus address, hubs) | frames <=10 Hz, quality 1 Hz, USB 0.5 Hz |
| `model/` | path poly, lanes + probabilities, road edges (if the port publishes them), frameId, timestampEof, execution time, frameDropPerc | <=10 Hz (full in ring/events) |
| `pose/` | cameraOdometry trans/rot (+std) | same |
| `calibration/` | start snapshot (intrinsics, CalibrationParams, vanishing point/pitch/yaw, mount) + `changes.jsonl` (rpy/vp changes, status, resets) | on change |
| `can/` | panda health/error counters; raw + decoded frames of 0x370 EPAS (angle, torsion torque, status), 0x0E steering sensor, 0x488 angle request, 0x108/0x118 DI torque/speed, 0x155 ESP, 0x2B9 DAS, 0x368 DI state | decoded with the project DBC layout; full rate for 0x370/0x488/0x0E |
| `localization/` | liveLocationKalman (yaw rate, orientation, velocity, acceleration, flags), GPS, IMU gyro/accel, liveParameters (+`param_changes.jsonl`), copies of the original maplogs | <=10 Hz |
| `steering/` | controlsState (desired/actual angle, `lateralControlState` P/I/feedforward/saturation, curvature, cumLagMs), carState, carControl, pathPlan, copies of intervention events | controlsd rate in ring/events, <=10 Hz continuous |
| `performance/` | 1 Hz: CPU total/per core/per process, memory, thermal zones, CPU MHz (throttling), GPU busy, per-process run-queue delay + context switches + threads, logger self-stats (CPU, queue depth, drops, disk write p95, fsync probe, free disk), per-second min/mean/max/std intervals of frame/model/controlsd/pose, model end-to-end latency, IPC delay (rt-lt), CAN rate and longest gap per address | 1 Hz |
| `events/` | `anomalies.jsonl` + one folder per anomaly: `summary.json` (why), `telemetry_full.jsonl.gz` (>=30 s before, >=30 s after, full rate, every service incl. raw CAN), `frames/*.jpg` | on trigger |
| `ring/` | the rolling full-rate buffer (last ~100 s telemetry + ~3 fps camera frames); bounded by size and age | rolling |

Time: every record has `rt` = `CLOCK_BOOTTIME` seconds (same reference as cereal `logMonoTime` and camerad `timestampEof`) and, if the message
has one, `lt`. `rt-lt` = inter-process delay; `lt - eof` of a model message = capture-to-output latency. Wall time = `rt + wall_minus_boot` (`metadata/clock.json`).

## Anomaly triggers (all thresholds in `metadata/config.json`, overridable via `/data/steering_diagnostics/config.json`)

`steering_swing` (>=8 deg peak-to-peak, >=2 sign changes in 4 s, engaged), `steering_oscillation` (>=4 deg, >=4 sign changes in 8 s, period 0.8-6 s),
`disengagement`, `steer_error_large` (|desired-actual|>5 deg for 0.5 s), `controller_saturation`, `model_path_jump` (c0/c1/path@20 m jump between
frames), `pose_discontinuity`, `camera_frame_drop` / `camera_frame_late` / `camera_duplicate_frame` / `camera_frame_delayed`, `model_latency_high`,
`model_exec_slow`, `model_frame_drop`, `can_gap` / `can_timeout` (per watched address, relative to its own median period), `can_errors`,
`ipc_delay_high`, `control_loop_gap`, `localization_invalid` (inputs/sensors/gps, liveParameters.valid, paramsValid), `calibration_reset`,
`usb_camera_missing/reconnected/changed`, `camera_image_underexposed/overexposed/blurry`, `disk_soft/disk_hard`.
Triggers inside an open event extend it (max 150 s); the same kind re-opens a new event only after 12 s. Max 40 events per session.

## Storage safety
Ring: bounded by age (100 s) and size (250 MB); continuous traces capped (400 MB); events max 40. Disk guard: below 1500 MB free stops
frames/traces, below 600 MB stops everything except the anomaly log. `start_diagnostics.sh` refuses to start with <2000 MB free.

## Analysis (also what `stop_diagnostics.sh` runs; works on a PC, stdlib only)
`python3 analyze.py all SESSION_DIR` -> `summary.txt`, `plots/index.html` (+ per-event 9-panel SVG: desired vs actual steering, torque
command vs measured, model path curvature, yaw rate/lateral accel, pose, camera intervals, CAN gaps, CPU/mem/temp, latencies, with the trigger and
anomaly timeline), `root_cause_report.txt`: per oscillation event the **onset order** (which signal left its baseline first), the oscillation
frequency, the SNR of every signal at that frequency, lead/lag cross-correlations (path -> desired -> actual -> yaw), and a rule-based reading:

* steering oscillates, model path **and** pose flat at that frequency -> *downstream* (controller gain/parameters or physical response);
* path/pose oscillate **before** the controller -> *perception/pose candidate*;
* controller moves first, path follows -> *closed loop* (the path moves because the car moved).

It prints evidence; it never proposes parameter changes.

## Tools
* `event_forensics.py ev_*.json` - checks far-path (40 m) values and steering angles of radarweb intervention events (parsing, extrapolation, glitches, lead/lag).
* `replay_events.py OUT ev_*.json` - replays recorded intervention events through the same detectors/event code.
* `overhead_check.py` - baseline vs with-logger comparison of camerad/modeld/controlsd CPU, model rate/worst interval, controlsd worst interval, temperature.

## Validation (what was and was not tested)
Tested off-car (`tests/test_diag.py`, `tests/test_diag_workflow.py`, ~70 s): detectors on a simulated 100 Hz car with injected frame drop, CAN timeout, oscillation,
path jump, localization loss; 30 s pre/post windows; ring boundedness; non-blocking writer; timestamp consistency; read-only source check; the real
shell scripts + collector process against a time-driven fake cereal (start, status, double-start refusal, anomalies, frames preserved, stop, analysis,
manifest listing every file, ZIP integrity, crash-then-finalize). Replayed the five recorded intervention events. Measured on a desktop: ~17 us per
record, ~50 MB RSS, ~8 % of one core for ~1100 rec/s (fake load is heavier than the real CAN rates).
**Not tested here:** the real cereal field names/units of this openpilot build (hence `./start_diagnostics.sh check`), camera quality metrics with cv2 on the EON
(falls back to PIL, or is skipped and reported), and CPU/thermal impact on the Snapdragon 821 (hence `overhead_check.py`). Run those two once, parked, before relying on a recording.

## Image orientation: who rotates the picture? (checked)
Exactly one place rotates it: `tesla/camerad/cameras/camera_ext_c920.cc` (`g_flip`, from `/data/camera_flip`, a 180 degree turn of the decoded frame
*before* it is packed for the model). `camera_qcom.c` keeps `rear.transform` = identity ("assume the device is upside-down (not anymore)"), `bayer_flip`
is not used by the external camera, and nothing in `tesla/model083/` (driving.cc, adapter, replay tool) flips or rotates. The stock 0.7.5 `modeld`
is not in this repo, so confirm on the EON: `grep -n -i "rotat\|flip" /data/openpilot/selfdrive/modeld/*.cc /data/openpilot/selfdrive/modeld/transforms/*`
(expect no matches in the model path).
`/dev/c920_live.jpg` (and so `/video`) is the RAW frame; the page rotates it for display only when `camera_flip=1`, so **what `/video` shows with
"upside down" ticked is what the model receives** (sky at the top = correct). The logger records `camera_flip` in `metadata/system.json` and every
image-quality sample has `model_view_top/bottom` brightness; `camera_orientation_suspect` fires in daylight if the top of the model's view is darker than the bottom.
