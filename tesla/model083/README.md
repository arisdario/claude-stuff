# The openpilot 0.8.3 driving model on Tinkla 0.7.5.a

Runs comma's 0.8.3 driving model (`supercombo.dlc`) inside your existing 0.7.5 install. Nothing else is upgraded:
you replace the model file and two source files, add one, and rebuild `modeld`. Planners, radard, the Tesla code,
the panda and radarweb keep working with the same messages.

## Why it fits

| | 0.7.5 | 0.8.3 model | |
|---|---|---|---|
| Inputs | 2 frames 512x256 YUV, desire 8, traffic convention 2, recurrent | `input_imgs` 12x128x256, `desire` 8, `traffic_convention` 2, `initial_state` 512 | same (checked in the real 0.8.3 ONNX) |
| Input order in the SNPE runner | desire = 1, traffic = 2, recurrent = 3 | same positions in 0.8.3's runner | same |
| Camera warp | medmodel frame, EON focal 910, centre 582/437 | identical constants | same |
| Output | path, 2 lanes, lead MDN, ... | 11327 floats: plan, 4 lane lines, road edges, leads, meta, pose | **converted** by `model083_adapter.h` |

What the adapter converts, so 0.7.5 sees what it expects:
- **path**: from the most likely 0.8.3 plan, interpolated to one point per metre (0-191 m) and fitted like 0.7.5
- **lanes**: the inner two of the 4 lane lines; real positions (no more +-1.8 m offset)
- **left/right**: 0.8.3 is y-positive-right, 0.7.5 left-positive -> flipped (lanes, path, lead `relY`)
- **lead** / **leadFuture**: now / 2 s, metres from the camera (radard subtracts the camera offset as before)
- **desire**: 0.7.5 feeds and reads it shifted by one; shifted for the model and back for the planners
- **pose** (`cameraOdometry`): already radians in 0.8.3
- path/lane **points are always published** (stock 0.7.5 only with `DEBUG` set)
- **road edges** (curbs, walls; no 0.7.5 field) go into the unused `freePath` list: radarweb's intervention recorder uses them

## Tested here (not on the car)

- `test_adapter.cc`: 37 checks on a synthetic 0.8.3 output written with 0.8.3's own read formulas - all pass
  (`g++ -std=c++11 -O2 -o /tmp/t test_adapter.cc && /tmp/t`)
- `driving.cc`, and 0.7.5's unchanged `modeld.cc` with the new `driving.h`, compile against Tinkla 0.7.5.a's headers
  and its message schema (cereal 0b329dc)
- the **real 0.8.3 model** (its ONNX twin, on a PC) on a rendered road, decoded by the adapter: lanes on the right
  sides at about +-1.6 m, path centred, no lead; with the car moved 0.8 m to either side both lines move the right way
  (`tools/`)

**Not tested**: on the EON (that SNPE there loads the 0.8.3 file, and speed) and on the road. Treat the first drives
as a test: watch it disengaged first (below).

## Install (on the EON)

The EON clock must be right for https (the date fix from before). Then:

```
cd /data/claude-stuff && git pull
cd /data/openpilot

# 1. backups
cp models/supercombo.dlc models/supercombo.dlc.075
cp selfdrive/modeld/models/driving.h  selfdrive/modeld/models/driving.h.075
cp selfdrive/modeld/models/driving.cc selfdrive/modeld/models/driving.cc.075

# 2. the 0.8.3 model (63,447,299 bytes)
curl -L -o models/supercombo.dlc.083 https://media.githubusercontent.com/media/commaai/openpilot/v0.8.3/models/supercombo.dlc
python3 -c "import hashlib;print(hashlib.sha256(open('models/supercombo.dlc.083','rb').read()).hexdigest())"
#    must print 059efa4e4c5f1430271362147e4b2eda650dc7ca9bad6a63725cc616afdf7f77 - if not, stop here
cp models/supercombo.dlc.083 models/supercombo.dlc

# 3. the adapter
cp /data/claude-stuff/tesla/model083/driving.h /data/claude-stuff/tesla/model083/driving.cc \
   /data/claude-stuff/tesla/model083/model083_adapter.h selfdrive/modeld/models/

# 4. rebuild modeld, then reboot
scons -j4 selfdrive/modeld/_modeld && reboot
```

If `/data/openpilot/prebuilt` exists, openpilot never rebuilds on its own, so step 4 is needed either way.

## Check before engaging

Car on, openpilot running but **not engaged**:

```
cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/model083/check_model.py 60
```

Driving centred in a marked lane you should see the left lane around +1.5..+2.0 m, the right around -1.5..-2.0 m,
path near 0, and the lead only with a car ahead (distance close to the radar's in radarweb). `/dash` shows the same
lanes and path. Then engage on a straight, empty road with your hands on the wheel.

Lateral tuning (steer ratio, PID, camera offset) was set with the 0.7.5 model; the 0.8.3 path is smoother and may
need small changes. Because path points are now published, the camera curve limit in `PCC_module.py` gets data and
becomes active (it was effectively off before): `touch /data/curve_limit_off` turns it off.

## If modeld does not start

`check_model.py` printing "no model messages" means modeld crashed. See why:

```
tmux a        # manager output; or with openpilot stopped and camerad running:  cd selfdrive/modeld && ./modeld
```

- the model file failed to load (SNPE error about the DLC/version): the Qualcomm runtime in 0.7.5 is older than the
  one 0.8.3 shipped. The fix is 0.8.3's SNPE libraries (`phonelibs/snpe`) - tell me and I'll write that step.
- anything else: send me the output.

## Roll back

```
cd /data/openpilot
cp models/supercombo.dlc.075 models/supercombo.dlc
cp selfdrive/modeld/models/driving.h.075  selfdrive/modeld/models/driving.h
cp selfdrive/modeld/models/driving.cc.075 selfdrive/modeld/models/driving.cc
rm -f selfdrive/modeld/models/model083_adapter.h
scons -j4 selfdrive/modeld/_modeld && reboot
```

Built against Tinkla `0.7.5.a` (BogGyver/openpilot). If your `selfdrive/modeld/modeld.cc` was changed from that
version, tell me before installing.
