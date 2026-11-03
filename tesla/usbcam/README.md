# USB camera test for the EON (`uvctest`)

Step 2 of checking whether a USB road camera (e.g. a Logitech C920) can replace the EON's own camera. The EON's
kernel has **no webcam driver** (`uvcvideo` is not in `/sys/bus/usb/drivers/`), so the camera has to be read from user
space over raw USB — the same way `boardd` talks to the panda (libusb). `uvctest` does exactly that with
[libuvc](https://github.com/libuvc/libuvc) on top, to prove video streams on this phone before any openpilot work.

It is a **standalone program**: it does not touch openpilot, camerad or the panda.

- `libuvc/` — libuvc v0.0.7 (commit 68d07a0), BSD licence (`libuvc/LICENSE.txt`), built without libjpeg. ONE LOCAL PATCH in `src/stream.c` (`_uvc_stream_params_negotiated`): the payload-size equality check is dropped, because it made a C920 fail every mode with "Invalid mode" (-51)
- `uvctest.c` — the test
- `build.sh` — builds it on the EON with the phone's own compiler (termux clang + libusb, as boardd uses)

## Run it (parked)

Camera plugged **straight into the EON** with a USB-C OTG adapter (no hub). Check it is listed first:

```bash
for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && echo "$(cat $d/idVendor):$(cat $d/idProduct) $(cat $d/product 2>/dev/null) speed=$(cat $d/speed 2>/dev/null)"; done
```

Then:

```bash
cd /data/claude-stuff && git pull
sh tesla/usbcam/build.sh
cd tesla/usbcam && ./uvctest 0c45:6422          # your camera's vendor:product from the list
```

If it says the mode isn't available, pick one from the list it prints: `./uvctest 0c45:6422 WIDTH HEIGHT FPS`.

## What it reports

- the camera's formats and modes
- whether it has the two controls a road camera needs: **exposure priority off** (keeps the frame rate at night)
  and **autofocus off** (no focus hunting) — a laptop camera may have neither; the C920 has both
- the frame rate actually received over 10 s, incomplete frames, and a saved frame (`frame.jpg` or `frame.ppm`)
- a one-line verdict: `OK`, `STREAMS BUT SLOW/INCOMPLETE`, or `NO FRAMES`

What it does **not** answer: whether a hub can carry the panda *and* a camera while the phone charges — that is a
separate test, and the deciding one for a C920 road camera.

## Result on the owner's EON (3 Nov 2026)
Logitech C920 (046d:082d, label V-U0028), plugged straight into the EON, car parked: `./uvctest 046d:082d 1280 720 30` -> **MJPEG 1280x720 @ 30 fps, 297 frames in 9.9 s = 30.0 fps, average frame 130,721 bytes (~31 Mbit/s), VERDICT OK**,
`frame.jpg` saved. Exposure priority off and autofocus off are both available. Required: the local libuvc patch in `libuvc/src/stream.c` (the payload-size equality check made every mode fail with "Invalid mode" -51 on this camera).
Not yet tested: a long run (10 min: `./uvctest 046d:082d 1280 720 30 600`), panda + camera + power through a hub on the step-down supply, the picture itself (focus/exposure/mount), and anything on the openpilot side.

**10-minute run (EON on the 12 V step-down, no panda, camera straight into the EON):** `UVC_FOCUS=0 ./uvctest 046d:082d 1280 720 30 600` -> 18,001 frames in 600.0 s = **30.0 fps**, average frame 185,180 bytes (~44 Mbit/s), VERDICT OK. No restart or dropout.

## Recording a clip for the model (offline, on a PC - nothing touches openpilot)
`UVC_FOCUS=0 UVC_SAVE=clip.mjpg UVC_SAVE_SECS=25 ./uvctest 046d:082d 1280 720 30 25` writes the camera's raw MJPEG frames (about 5 MB/s: keep the seconds short, disk is limited). Copy `clip.mjpg` to a PC and run
`sh clip_to_eon.sh clip.mjpg fcamera_c920.hevc`, then `tesla/model083/tools/replay083.py` (needs the model file and ffmpeg) to see the path and lanes the driving model draws on the C920's picture. The geometry conversion is explained at the top of `clip_to_eon.sh`.
