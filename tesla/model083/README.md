# Running the openpilot 0.8.3 driving model on Tinkla 0.7.5.a

This runs comma's **0.8.3 driving model** (`supercombo.dlc`) inside your existing **0.7.5** install. Nothing else is
upgraded. You replace the model file and two source files, add one header, and rebuild one program (`modeld`). The
planners, `radard`, the Tesla code, the panda and radarweb all keep running with the exact same messages.

> **This changes what the car sees and therefore how it drives.** It has only been tested offline (see
> [What is and isn't tested](#what-is-and-isnt-tested)). Treat the first drives as a test: watch it **disengaged**
> first, then engage only on a straight, empty road with your hands on the wheel and your foot over the brake.
> You can roll back in two minutes ([Roll back](#roll-back)).

---

## 2D vs 3D — what you get

The 0.8.3 model is a **3D** model: for every point ahead it predicts forward distance **x**, sideways offset **y**
**and height z**, plus full 3D orientation. 0.7.5 is a 2D stack: its message format and planners only use sideways
position vs forward distance on a flat ground plane.

The adapter feeds 0.7.5 the **2D slice** (x and y); the height **z** is computed by the model but dropped, because
0.7.5 has nowhere to put it. Lane-keeping and following are 2D on this stack, so **nothing is lost for normal
flat-road driving**. You just don't gain the 3D model's advantage on hills, crests and banked curves. It works.

---

## Why it drops in at all

| | 0.7.5 expects | 0.8.3 model gives | |
|---|---|---|---|
| Inputs | 2 frames 512×256 YUV, desire (8), traffic convention (2), recurrent | `input_imgs` 12×128×256, `desire` (8), `traffic_convention` (2), `initial_state` (512) | same shapes (checked against the real 0.8.3 ONNX) |
| Input slots in the SNPE runner | desire, traffic, recurrent | same order | same |
| Camera warp | medmodel frame, EON focal 910, centre 582/437 | identical constants | same |
| Output message | path, 2 lanes, lead, longitudinal, meta, pose | 11327 floats: plan, 4 lane lines, road edges, leads, meta, pose | **converted** by `model083_adapter.h` |

The conversion (all in `model083_adapter.h`, unit-tested) turns the 0.8.3 output into the 0.7.5 `model` message:
path from the most likely plan (one point per metre, 0–191 m), the inner two of the 4 lane lines at their real
positions, left/right sign flipped (0.8.3 is y-right, 0.7.5 is y-left), lead now / in 2 s in metres from the camera,
desire shifted to 0.7.5's indexing, pose in radians, and road edges tucked into the unused `freePath` list for
radarweb. Path/lane points are always published (stock 0.7.5 only did that with `DEBUG` set), which is what finally
feeds `PCC_module`'s camera curve limit.

---

## Before you start — prerequisites

- [ ] You are on **Tinkla 0.7.5.a** (BogGyver/openpilot). If your `selfdrive/modeld/modeld.cc` was changed from that
      version, stop and tell me first — the drop-in assumes the stock `modeld.cc`.
- [ ] You can SSH into the EON and it has internet (the EON clock must be right for https — the date fix from before).
- [ ] This repo is on the EON at `/data/claude-stuff` (`git clone` it there once if not).
- [ ] You have ~150 MB free on `/data` (`df -h /data`).
- [ ] You know how to engage/disengage and you'll do the first drive somewhere empty.

---

## Install (on the EON, parked)

```bash
cd /data/claude-stuff && git pull
sh tesla/model083/install.sh
reboot
```

`install.sh` does, in order, and stops with a clear message if any step fails:

1. **Backups** of everything it replaces, kept as `*.075` next to each file (first run only, so a re-run never
   overwrites the originals): the model, `driving.h`, `driving.cc`, `PCC_module.py`, `pcc_curves.py`.
2. **The 0.8.3 model** from comma's v0.8.3 release, rejected unless it is the real file: 63,447,299 bytes,
   sha256 `059efa4e4c5f1430271362147e4b2eda650dc7ca9bad6a63725cc616afdf7f77` (both verified against the download).
3. **The adapter**: `driving.h`, `driving.cc`, `model083_adapter.h` into `selfdrive/modeld/models/`.
4. **PCC with the curve-limit fix** (`PCC_module.py`, `pcc_curves.py` into `selfdrive/car/tesla/`). Required: the new
   model publishes path points, which switches PCC's camera curve limit on, and without the fix it braked for
   curves that aren't there at 58–68 km/h (see `tesla/AUDIT.md`). If your car's PCC files had local changes, they are
   in the `*.075` backups.
5. **Rebuild `modeld`** (`scons -j4 selfdrive/modeld/_modeld`). If `/data/openpilot/prebuilt` exists openpilot never
   rebuilds on its own, so this is needed either way.

---

## Step 6 — check it works, with the car ON but openpilot NOT engaged

After the reboot, car on, openpilot running but **not engaged**:

```bash
cd /data/openpilot
PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/check_model.py 60
```

Driving centred in a clearly marked lane, you should see roughly:

- **left lane ≈ +1.5 … +2.0 m**, **right lane ≈ −1.5 … −2.0 m** (left is positive)
- **path ≈ 0** (you're centred)
- a **lead only when there's a car ahead**, at a distance close to what radarweb's radar shows

`/dash` and the road view in radarweb should show the same lanes and path moving correctly as the road bends. When
you steer toward a line, that line's number should head toward 0.

If that all looks right: engage on a **straight, empty road**, hands on the wheel, and watch how it tracks before
trusting it in a curve.

---

## Bench test with the camera on screen (no car, no panda)

The EON's normal UI only shows the camera with the model's path and lanes when the car is on. On the bench,
`bench_started.py` pretends the ignition is on, so openpilot goes onroad by itself: point the road camera at a screen
playing a front-facing dashcam video and watch the path and lanes on the EON's screen.

```bash
# stop openpilot, and anything that talks to a panda
tmux kill-session -t comma; sleep 2; pkill -f boardd; pkill -f pandad
# start openpilot without boardd (so only the bench script publishes the panda's health)
cd /data/openpilot && tmux new -d -s bench 'NOBOARD=1 ./launch_openpilot.sh'
# pretend ignition on - the EON switches to the camera view within ~10 s
PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_started.py
# in a second SSH session, the numbers:
cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/check_model.py 30
```

Ctrl+C in the bench script switches ignition off again; then `reboot` to go back to normal. The script refuses to
run if a panda is connected or boardd is running. Nothing reaches a car: without boardd there is no CAN at all.
**Speed on the bench (`bench_can_replay.py`):** replays a logged CAN capture (the old `tesla_can_bridge.log`, a real drive: vehicle speed 0 to about 66 mph) into the local `can` channel, so `controlsd` and the UI get speed
instead of waiting for CAN. It is a rewrite (the original bridge code was lost) and has NOT been run on an EON yet. Check the log first with `--dry`; the pace is a setting (`--fps`, the log has no timestamps). It never opens a CAN
interface and refuses to run with a panda on USB or boardd running. Copy the log to the EON (e.g. `/data/tesla_can_bridge.log`; do not commit it to git).
On the bench `controlsd` waits for CAN that never comes and the UI may show "unavailable"-type alerts: expected.

**Calibration on the bench:** by default `bench_started.py` switches calibration to *straight ahead* for the test
(right for a video recorded by an EON camera) and puts your car's calibration back on Ctrl+C. Your calibration is kept
in `/data/bench_calibration_backup.json` meanwhile; if a run is cut off, the next start restores it first. Use
`--car-calib` to keep the car's calibration instead.

**Getting the model to see the screen as a road** (it was trained on the EON camera looking at real roads):

1. Play a drive **recorded by an EON camera** (the demo video `bench_demo_drive.mp4`: comma's public daytime sample,
   lanes 1.00/0.98 and lead p1.00 when fed straight to the model) — not a phone or fisheye dashcam video.
2. Full screen. **Lens-to-screen distance = 1.04 × the height of the picture on the screen** (the video then covers
   the EON camera's ~65° view exactly). A 4:3 video full-height on a 16:9 monitor: 24" → ~31 cm, 27" → ~35 cm,
   32" → ~41 cm, 15.6" laptop → ~20 cm.
3. The video opens with 5 s of an alignment frame: put the white border on the edges of the camera view and the
   cross in the centre; lens at the picture's centre, phone parallel to the screen.
4. Dim room, screen brightness up, no reflections on the screen.

Working: lane probabilities above 0.8, lane width about 3.5 m, a lead box (`lead p` > 0.5) behind the car ahead.

### Bench quick start (copy-paste, EON in the room, no car, no panda)

Each part runs in its own `tmux` session, so nothing needs a second SSH window and it all survives the SSH connection.

```
# 0. which road camera? (it is chosen when camerad starts; the C920 must be plugged in)
sh /data/claude-stuff/tesla/camerad/camera_switch.sh status          # external = C920, internal = the phone camera
sh /data/claude-stuff/tesla/camerad/camera_switch.sh external        # or: internal

# 1. stop everything, then start openpilot WITHOUT boardd (nothing can reach a car)
tmux kill-session -t replay 2>/dev/null; tmux kill-session -t ign 2>/dev/null
tmux kill-session -t bench 2>/dev/null; tmux kill-session -t comma 2>/dev/null
pkill -f boardd; pkill -f pandad; pkill -f manager.py; sleep 3
cd /data/openpilot && tmux new -d -s bench 'NOBOARD=1 ./launch_openpilot.sh'
sleep 20

# 2. pretend the ignition is on (the EON switches to the camera view within ~10 s).
#    --car-calib keeps your car's calibration; without it the bench uses a "straight ahead" one and restores yours on Ctrl+C
tmux new -d -s ign 'PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_started.py --car-calib'

# 3. replay the logged CAN capture (speed, steering, GPS) into the local cereal socket
tmux new -d -s replay 'PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/bench_can_replay.py /data/tesla_can_bridge.log'
```

Check it is running (about 30 s later):
```
tmux ls                                                    # bench, ign, replay
ps | grep -E "camerad|_modeld|controlsd" | grep -v grep
tmux capture-pane -p -t bench -S -5000 | grep -i "ROAD CAMERA\|external camera"    # which camera camerad really used
cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/check_model.py 30   # model numbers
```
Look at a session: `tmux a -t ign` (leave it with `Ctrl+B` then `D`, never Ctrl+C unless you mean to stop it).

Stop and go back to normal:
```
tmux kill-session -t replay; tmux kill-session -t ign; tmux kill-session -t bench
reboot
```

Notes
- Start order matters: openpilot first, then the ignition script, then the replay. The ignition script refuses to run with a panda or `boardd` present.
- **Calibration:** without `--car-calib` the script swaps in a neutral calibration and puts yours back on Ctrl+C (`/data/bench_calibration_backup.json`). With it, openpilot keeps whatever is stored; delete `/data/params/d/CalibrationParams` first if you want to watch it learn from zero. Do not trust a calibration learned from a screen.
- **Learned vehicle parameters (important):** the replay feeds real steering angles and speeds while the camera sees a still scene, so
  `paramsd` learns nonsense (a steering ratio near double the car's, a tiny tyre stiffness) and SAVES it in `/data/params/d/LiveParameters`.
  The next real drive then shows "Vehicle Parameter Identification Failed" and steers with a doubled gain: it oscillates. `bench_started.py`
  now saves the car's values at start and puts them back on Ctrl+C (`/data/bench_liveparameters_backup.json`). If a bench run ended any other
  way (reboot, power cut) run `python3 .../bench_started.py --restore` BEFORE driving. If the file already holds bad values (steerRatio far from
  the car's `interface.py` value, stiffnessFactor at 0.5), delete it first: `rm /data/params/d/LiveParameters`, then start the bench.
- **Switch files** read by `launch_chffrplus.sh` (only if that edit was made): `/data/no_loggerd` and `/data/no_mapcurved` keep those processes from starting. `rm` them to turn them back on, then restart.
- `"waiting for CarParams"` and the uploader's `KeyError: 'dongle_id'` lines in the `bench` pane are normal on the bench.
- Measuring the EON's CPU and heat on the bench: `tesla/traffic/README.md`.

---

## Step 7 — tuning notes after it's driving

- Lateral tuning (steer ratio, PID, camera offset) was set for the **old** model. The 0.8.3 path is smoother and
  centres slightly differently; small camera-offset / tuning tweaks may help. Change these gently, one at a time.
- Because path points are now published, **`PCC_module`'s camera curve limit becomes active** (it was effectively
  off before, with no points to read); `install.sh` puts in the fixed version. To switch it off while you settle in:
  `touch /data/curve_limit_off`. radarweb's `/dash` shows a `CURVE` chip whenever it is holding your speed.

---

## If `modeld` does not start

`check_model.py` printing **"no model messages"** means `modeld` crashed. See why:

```bash
tmux a        # attach to manager output
# or, with openpilot stopped and camerad running:
cd /data/openpilot/selfdrive/modeld && ./modeld
```

Most likely cause and fix:

- **SNPE / DLC load error** (a Qualcomm runtime message about the model or DLC version) — **the most likely
  blocker**. The 0.8.3 model was converted with SNPE **1.43** (`converter-version=1.43.0.2307` inside the file);
  0.7.5's SNPE is older (openpilot only moved to 1.41 in 0.7.10). 0.8.3's own SNPE 1.41 libraries are available and
  verified (all 8 that modeld uses match openpilot's recorded checksums; `libc++_shared.so` is identical to yours).
  The fix would give **modeld alone** its own SNPE folder: driver monitoring runs on the same SNPE on the DSP, and
  swapping SNPE system-wide could break it (and with it, engaging). Send me the error and I'll write that step.
- **Wrong output size / garbage lanes**: the `.dlc` you downloaded isn't the 0.8.3 supercombo, or is the LFS pointer.
  Re-check step 3.
- Anything else: send me the output of the command above.

---

## Roll back (≈2 minutes)

```bash
sh /data/claude-stuff/tesla/model083/uninstall.sh && reboot
```

Puts back exactly what `install.sh` replaced (the `*.075` backups) and rebuilds `modeld`. Add `KEEP_PCC=1` in front
to keep the fixed PCC files (harmless with the old model).

---

## What is and isn't tested

**Tested (off the car):**

- `test_adapter.cc` — 38 checks on a synthetic 0.8.3 output built with 0.8.3's own read formulas; all pass
  (`g++ -std=c++11 -O2 -o /tmp/t test_adapter.cc && /tmp/t`).
- `driving.cc` plus 0.7.5's unchanged `modeld.cc` with the new `driving.h` compile against Tinkla 0.7.5.a's headers
  and message schema (cereal `0b329dc`).
- The **real 0.8.3 model** (its ONNX twin, on a PC) run on a rendered road and decoded through the adapter: lane
  lines land on the correct sides at about ±1.6 m, path centred, no false lead; moving the car 0.8 m to either side
  moves both lines the right way (`tools/`).

**Not tested:**

- That the EON's SNPE runtime actually loads the 0.8.3 `.dlc` (see [the troubleshooting section](#if-modeld-does-not-start)).
- Model frame rate on the EON.
- Anything on the road.

Built against Tinkla `0.7.5.a` (BogGyver/openpilot).

## Steering-wiggle investigation: `pose_watch`

Read-only logger (subscribes only; no control interaction) to separate
"model path/pose oscillates" (camera geometry, model inputs) from "path is stable
but steering oscillates" (downstream control gain):

    # EON, openpilot running; ideally `rm /data/no_loggerd` first
    python3 /data/openpilot/tesla/tools/pose_watch.py log        # -> /data/posewatch/pose_*.jsonl
    python3 /data/openpilot/tesla/tools/pose_watch.py summary /data/posewatch/pose_XXXX.jsonl

Records model path/lanes + frameId/timestampEof/frameDropPerc, cameraOdometry
rot/trans (+std), gyro, liveLocationKalman yaw rate, GPS bearing, controlsState
desired vs actual angle, liveParameters, liveCalibration, carState.
