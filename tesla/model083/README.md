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

## Step by step (on the EON)

Everything below is run over SSH on the EON. Copy one block at a time and read the comment before running it.

### 1. Get the latest adapter files

```bash
cd /data/claude-stuff && git pull
```

### 2. Back up everything you're about to replace

So rollback is just a copy-back. **Do not skip this.**

```bash
cd /data/openpilot
cp models/supercombo.dlc                 models/supercombo.dlc.075
cp selfdrive/modeld/models/driving.h     selfdrive/modeld/models/driving.h.075
cp selfdrive/modeld/models/driving.cc    selfdrive/modeld/models/driving.cc.075
```

### 3. Download the 0.8.3 model file

The model is a Git-LFS file, so a plain download can give you a tiny text "pointer" instead of the real model.
The size check below catches that.

```bash
cd /data/openpilot
curl -L -o models/supercombo.dlc.083 \
  https://media.githubusercontent.com/media/commaai/openpilot/v0.8.3/models/supercombo.dlc

ls -l models/supercombo.dlc.083      # must be tens of MB (~60 MB). If it's a few hundred BYTES, the download
                                     # gave you an LFS pointer, not the model - stop and tell me.
```

Optional integrity check (convenience, not authoritative — if comma's published hash differs, trust theirs):

```bash
python3 -c "import hashlib;print(hashlib.sha256(open('models/supercombo.dlc.083','rb').read()).hexdigest())"
# recorded here: 059efa4e4c5f1430271362147e4b2eda650dc7ca9bad6a63725cc616afdf7f77
```

Put it in place:

```bash
cp models/supercombo.dlc.083 models/supercombo.dlc
```

### 4. Install the adapter and the two drop-in source files

```bash
cp /data/claude-stuff/tesla/model083/driving.h \
   /data/claude-stuff/tesla/model083/driving.cc \
   /data/claude-stuff/tesla/model083/model083_adapter.h \
   /data/openpilot/selfdrive/modeld/models/
```

### 5. Rebuild `modeld` and reboot

Only `modeld` changed, so only it needs building. If `/data/openpilot/prebuilt` exists, openpilot never rebuilds on
its own, so this step is required either way.

```bash
cd /data/openpilot
scons -j4 selfdrive/modeld/_modeld     # must end with no error
reboot
```

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

## Step 7 — tuning notes after it's driving

- Lateral tuning (steer ratio, PID, camera offset) was set for the **old** model. The 0.8.3 path is smoother and
  centres slightly differently; small camera-offset / tuning tweaks may help. Change these gently, one at a time.
- Because path points are now published, **`PCC_module`'s camera curve limit becomes active** (it was effectively
  off before, with no points to read). If you want it off while you settle in: `touch /data/curve_limit_off`.

---

## If `modeld` does not start

`check_model.py` printing **"no model messages"** means `modeld` crashed. See why:

```bash
tmux a        # attach to manager output
# or, with openpilot stopped and camerad running:
cd /data/openpilot/selfdrive/modeld && ./modeld
```

Most likely cause and fix:

- **SNPE / DLC load error** (a Qualcomm runtime message about the model version): the SNPE libraries in 0.7.5 are
  older than the ones 0.8.3 shipped, and can't load the newer model. The fix is to also bring in 0.8.3's SNPE
  libraries (`phonelibs/snpe`) — **this is the most likely blocker; tell me and I'll write that step.**
- **Wrong output size / garbage lanes**: the `.dlc` you downloaded isn't the 0.8.3 supercombo, or is the LFS pointer.
  Re-check step 3.
- Anything else: send me the output of the command above.

---

## Roll back (≈2 minutes)

Puts the original 0.7.5 model and sources back exactly.

```bash
cd /data/openpilot
cp models/supercombo.dlc.075            models/supercombo.dlc
cp selfdrive/modeld/models/driving.h.075  selfdrive/modeld/models/driving.h
cp selfdrive/modeld/models/driving.cc.075 selfdrive/modeld/models/driving.cc
rm -f selfdrive/modeld/models/model083_adapter.h
scons -j4 selfdrive/modeld/_modeld && reboot
```

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
