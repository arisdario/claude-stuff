# External road camera (Logitech C920) with a switch back to the phone's camera

`camerad` can use a C920 on USB as the road camera instead of the phone's rear camera. The phone's FRONT camera (driver monitoring) is not touched. Default = the phone's camera, exactly as before.

## Install (once, on the EON; openpilot stopped)
```
cd /data/claude-stuff && git fetch --depth 1 origin offline-nav && git checkout -B offline-nav FETCH_HEAD
tmux kill-session -t comma
sh tesla/camerad/install_external_camera.sh            # shows what it would do
sh tesla/camerad/install_external_camera.sh --yes      # backs up the stock camerad (/data/camerad_backup_stock), copies the sources, builds camerad
reboot                                                 # still the phone's camera
```
Undo any time: `sh tesla/camerad/install_external_camera.sh --restore` (puts back the stock binary and sources; no build needed), then reboot.

## Switch
```
sh tesla/camerad/camera_switch.sh external     # C920 (plugged straight into the EON), then reboot
sh tesla/camerad/camera_switch.sh internal     # the phone's camera again, then reboot
sh tesla/camerad/camera_switch.sh status
sh tesla/camerad/camera_switch.sh focus 0      # C920 focus: 0 = infinity (the road)
```
At camerad start, "external" is used only if the C920 is plugged in AND delivers frames within 4 s; otherwise camerad logs `external road camera requested but ... using the phone's rear camera` and uses the phone's camera.
Look for `ROAD CAMERA: external C920` in the camerad output.

## What it does
MJPEG 1280x720 @ 20 fps -> decoded (libjpeg) -> 58 columns cropped on each side -> centred on a black 1164x874 BGR canvas (no scaling: the C920's focal length at 1280x720 is ~907 px against the EON's 910 px) -> into the road camera's buffers.
The picture geometry was checked against ffmpeg on a real C920 frame (black bars exactly empty, BGR order, mean difference 0.4/255). Exposure priority off (steady 20 fps in the dark), autofocus off, focus from /data/camera_focus (default 0).

## Limits - read before driving
- NOT built or run on an EON yet by the author: camera_qcom.c cannot be compiled off the phone. The new file was syntax-checked against stock openpilot 0.7.5 headers; the install/restore script was tested on a fake tree.
- If the C920 stops mid-drive there is NO live fallback to the phone camera (its sensor is not powered in external mode): openpilot loses the road picture until you restart. The thread tries to reopen the C920 every second.
- The C920 sees 43 deg vertically (the EON camera 51 deg): the model gets a narrower view with black bars above and below. After switching, openpilot's calibration must re-learn the vanishing point (drive straight for a few minutes; if it is wildly off, remove /data/params/d/CalibrationParams and reboot).
- Not tested yet: panda + C920 on the EON's USB at the same time in the car; day/night exposure; heat. Test parked first, then a short drive with a passenger watching the screen.
- Nothing here changes steering, braking or acceleration code.

## Status LED

The C920's blue LED is switched off by camerad at every start (the camera forgets the setting at power loss), through the Logitech peripheral
extension unit (selector 9, "LED1 Mode" = 0). It was verified on a real C920 with `tesla/usbcam/ledtool` (`set 0` printed VERIFIED). camerad uses the same
strict checks (control must be readable, settable and 5 bytes long; only the 2 mode bits change; read back) and logs
`external camera: LED off verified`. To leave the LED on auto, create `/data/camera_led_auto`. Nothing else about the camera is changed.

## Startup patience (cold boots)

After a cold boot the C920 can show up on USB, and deliver its first frames, later than camerad starts. Camerad used to try once and fall
back to the phone's camera for the whole session. It now waits up to 12 s for the camera to appear and tries the self-test up to 3 times (4 s each),
logging `C920 appeared after N ms` / `attempt N of 3 failed, retrying`. If the C920 is selected but really absent, it still falls back to the phone
camera after the wait (so a boot without the camera takes up to 12 s longer). Look for the result in the log:
`tmux capture-pane -p -t bench -S -5000 | grep -i "ROAD CAMERA\|external camera"`.


## LED button (radarweb /video)
`LED off now` writes `/data/camera_led_now`; camerad (checked once a second) switches the LED off through the Logitech XU, verifies it by reading it back and
deletes the file. While streaming it also re-checks the LED every 5 s and switches it off again (quietly) if the camera turned it back on. `keep off`
unticked creates `/data/camera_led_auto` (camerad then leaves the LED to the camera from the next camera start). Limit: when the car's USB power comes up the
camera lights its LED by itself, and software can only switch it off once camerad is running (a few seconds after boot or openpilot start). Needs camerad rebuilt:
`install_external_camera.sh --yes` with openpilot stopped.
