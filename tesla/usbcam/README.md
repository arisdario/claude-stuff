# USB camera test for the EON (`uvctest`)

Step 2 of checking whether a USB road camera (e.g. a Logitech C920) can replace the EON's own camera. The EON's
kernel has **no webcam driver** (`uvcvideo` is not in `/sys/bus/usb/drivers/`), so the camera has to be read from user
space over raw USB — the same way `boardd` talks to the panda (libusb). `uvctest` does exactly that with
[libuvc](https://github.com/libuvc/libuvc) on top, to prove video streams on this phone before any openpilot work.

It is a **standalone program**: it does not touch openpilot, camerad or the panda.

- `libuvc/` — libuvc v0.0.7 (commit 68d07a0), BSD licence (`libuvc/LICENSE.txt`), built without libjpeg
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
