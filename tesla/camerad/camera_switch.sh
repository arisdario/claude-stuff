#!/bin/sh
# Road camera selector (takes effect when openpilot restarts, e.g. reboot):
#   sh camera_switch.sh external      use the Logitech C920 on USB as the road camera (needs the camerad built by install_external_camera.sh)
#   sh camera_switch.sh internal      use the phone's own rear camera again (the default; also what you get if this file is removed)
#   sh camera_switch.sh status        what is selected, and whether the C920 is plugged in
#   sh camera_switch.sh focus N       C920 lens focus, 0 = infinity (the road; default), up to ~250 = closest. Only matters for the external camera.
#   sh camera_switch.sh format h264   C920 stream format: h264 = camera sends H.264, camerad decodes it (opt-in, to test); mjpeg = the default (also what you get if the file is removed).
#                                     Applies at the next camerad start (reboot). If H.264 cannot be used for any reason, camerad falls back to MJPEG by itself.
# If "external" is selected but the C920 is missing or gives no frames when camerad starts, camerad uses the phone's camera by itself (see the log line "ROAD CAMERA").
F=/data/camera_source
case "$1" in
  external) echo external > $F; echo "road camera: EXTERNAL (C920). Restart openpilot (reboot) to apply. After the first start with the new camera, let openpilot re-learn its calibration (see tesla/camerad/README_external_camera.md).";;
  internal) rm -f $F; echo "road camera: INTERNAL (the phone's camera). Restart openpilot (reboot) to apply.";;
  format)   case "$2" in h264) echo h264 > /data/camera_format; echo "C920 format: H.264 (opt-in). Reboot to apply. Look for the log line ROAD CAMERA ... H.264; if it says MJPEG, the fallback was used (the line before it says why).";;
              mjpeg) rm -f /data/camera_format; echo "C920 format: MJPEG (the default). Reboot to apply.";;
              *) echo "usage: sh $0 format h264 | mjpeg"; exit 1;; esac;;
  focus)    [ -n "$2" ] || { echo "usage: sh $0 focus N   (0..250)"; exit 1; }; echo "$2" > /data/camera_focus; echo "C920 focus set to $2 (applied at the next camerad start)";;
  status|"")
    if [ -f $F ] && grep -q '^external' $F; then echo "selected: EXTERNAL (C920)"; else echo "selected: INTERNAL (the phone's camera)"; fi
    found=no
    for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && [ "$(cat $d/idVendor)" = "046d" ] && [ "$(cat $d/idProduct)" = "082d" ] && found=yes; done
    echo "C920 plugged in: $found"
    [ -f /data/camera_focus ] && echo "C920 focus: $(cat /data/camera_focus)" || echo "C920 focus: 0 (default, infinity)"
    if [ -f /data/camera_format ] && grep -qi "^h264" /data/camera_format; then echo "C920 format: H.264 asked for (MJPEG if camerad cannot use it)"; else echo "C920 format: MJPEG (default)"; fi
    [ -f /data/camera_status.json ] && echo "running format: $(sed -n 's/.*"format":"\([a-z0-9]*\)".*/\1/p' /data/camera_status.json)";;
  *) echo "usage: sh $0 external | internal | status | focus N | format h264|mjpeg"; exit 1;;
esac
