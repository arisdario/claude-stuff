#!/bin/sh
# Road camera selector (takes effect when openpilot restarts, e.g. reboot):
#   sh camera_switch.sh external      use the Logitech C920 on USB as the road camera (needs the camerad built by install_external_camera.sh)
#   sh camera_switch.sh internal      use the phone's own rear camera again (the default; also what you get if this file is removed)
#   sh camera_switch.sh status        what is selected, and whether the C920 is plugged in
#   sh camera_switch.sh focus N       C920 lens focus, 0 = infinity (the road; default), up to ~250 = closest. Only matters for the external camera.
# If "external" is selected but the C920 is missing or gives no frames when camerad starts, camerad uses the phone's camera by itself (see the log line "ROAD CAMERA").
F=/data/camera_source
case "$1" in
  external) echo external > $F; echo "road camera: EXTERNAL (C920). Restart openpilot (reboot) to apply. After the first start with the new camera, let openpilot re-learn its calibration (see tesla/camerad/README_external_camera.md).";;
  internal) rm -f $F; echo "road camera: INTERNAL (the phone's camera). Restart openpilot (reboot) to apply.";;
  focus)    [ -n "$2" ] || { echo "usage: sh $0 focus N   (0..250)"; exit 1; }; echo "$2" > /data/camera_focus; echo "C920 focus set to $2 (applied at the next camerad start)";;
  status|"")
    if [ -f $F ] && grep -q '^external' $F; then echo "selected: EXTERNAL (C920)"; else echo "selected: INTERNAL (the phone's camera)"; fi
    found=no
    for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && [ "$(cat $d/idVendor)" = "046d" ] && [ "$(cat $d/idProduct)" = "082d" ] && found=yes; done
    echo "C920 plugged in: $found"
    [ -f /data/camera_focus ] && echo "C920 focus: $(cat /data/camera_focus)" || echo "C920 focus: 0 (default, infinity)";;
  *) echo "usage: sh $0 external | internal | status | focus N"; exit 1;;
esac
