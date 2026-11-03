# Traffic awareness (advisory only) - Phase 0: measure first

Nothing here controls the car, and nothing here may import vehicle-control code (a test enforces it). No neural network exists yet. This phase only
builds the tools to **measure** what the EON can spare, and the **governor** that will switch any experimental detector off.

## Facts the design starts from
- Camera: the C920 only. camerad reads it as **1280x720 MJPEG at 20 fps** (not 30: openpilot's pipeline and the driving model run at 20 Hz, and `SRC_FPS` is 20 in
  `tesla/camerad/cameras/camera_ext_c920.cc`). 5 fps detection = every 4th camera frame.
- The driving model sees only a 512x256 window (about +-16 deg across, 3 deg above to 13 deg below the horizon). It publishes path, lanes, road edges, two leads and a
  few probabilities. It has **no** lights, signs, pedestrians or crossings.
- Newer comma models stop for lights and stop signs *inside the network* (0.9.0, Experimental mode), with both road cameras of the comma three. That is a different model;
  there is no light/sign output to read. Not usable on this EON/camera as far as we know (unverified).
- Frame source for a prototype: camerad already writes the full 1280x720 JPEG to `/dev/c920_live.jpg` (about 10 per second); the vision stream (YUV, 1164x874) is the production source.

## Tools
| file | role |
|---|---|
| `sysstats.py` | reads /proc and /sys: CPU total/per core/rest-of-system, per-process CPU, free RAM, all thermal zones, GPU load/clock, CPU clocks, battery temp |
| `benchmark.py` | `hw` (what accelerators/libraries exist), `run` (CSV every second, plus the driving model's and camerad's real message rates, frame-id gaps and thermald status when cereal is importable), `summary` (percentiles, per-process table, suggested limits) |
| `governor.py` | says "allowed" or not. Limits file is mandatory; missing sensor = off; thermald not green = off at once; model fps < 19 = off; rest of system busier than ever seen in the baseline = off; own CPU over its budget = off; hold, margins, warm-up, latch after repeated trips |

## Measurement protocol (run on the EON; the detector does not exist yet, so this is the baseline)
```
python3 /data/claude-stuff/tesla/traffic/benchmark.py hw > /data/traffic_hw.json
mkdir -p /data/traffic_bench
# 1. bench, onroad (bench_started + replay), 15 min
python3 /data/claude-stuff/tesla/traffic/benchmark.py run --out /data/traffic_bench/bench.csv --label bench --duration 900
# 2. real drive, city + faster road, at least 30 min, ideally on a warm day (start it before you leave, Ctrl+C when parked)
python3 /data/claude-stuff/tesla/traffic/benchmark.py run --out /data/traffic_bench/drive1.csv --label drive
python3 /data/claude-stuff/tesla/traffic/benchmark.py summary /data/traffic_bench/drive1.csv
```
Run it in `tmux` so it survives the SSH session. Do not run anything else heavy meanwhile (the benchmark itself uses about 1 percent of one core).
Send back the `hw` JSON and the `summary` text. Only then does a **limits file** get written (`summary --suggest /data/traffic_limits.json`); the governor cannot start without it.
Limits come from the baseline: rest-of-system CPU p99 + 5 points, temperature p99 + 3 C, free RAM min - 100 MB; the detector's own budget defaults to 40 percent of one core.

## Next, in this order (each only after the previous one is validated)
1. Hardware probe (`hw`): which runtimes exist (SNPE CPU/GPU/DSP, OpenCV, NCNN/TFLite), what the GPU/DSP already do for modeld. **Not yet known.**
2. Headroom while driving (above): the number that decides whether anything can run at all.
3. Prototype at 5 fps on the camera stream with the *smallest* pretrained detector that fits; log-only; compare CPU/temperature/model-fps with and without it.
   Candidates to evaluate (sizes/speeds are published figures for other hardware, NOT measured here): SSDLite-MobileNetV2, NanoDet, YOLOX-nano, YOLOv8n at about 320 px.
4. Tracking between inference frames (simple tracker, 20 fps), so 5 fps inference is enough for slow objects.
5. Phases 2-5: lights, stop signs, pedestrians/cyclists, crosswalks. OSM is context only (a prior that raises or lowers confidence, never a precondition).

Acceptance numbers (recall by distance and light, false alarms per hour, CPU/temperature deltas, model fps unchanged) are agreed before each phase and measured on your own logs.
