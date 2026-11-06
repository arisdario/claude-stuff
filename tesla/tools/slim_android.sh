#!/bin/sh
# slim_android.sh - stop Android services the EON does not use, AT RUNTIME ONLY. Nothing is written to disk and /system is untouched: a reboot brings every service back.
#   sh slim_android.sh                  dry run: shows what is running, what it would stop, and the memory now
#   sh slim_android.sh --yes            stops the SMALL set (fingerprint daemon, perf profiler, IFAA daemon, DRM server)
#   sh slim_android.sh --yes --telephony   also stops the phone/IMS daemons (rild x2, imsdatadaemon, imsqmidaemon). Park the car; see the warning printed.
# Undo: reboot, or  start <service>.  Chosen from this EON's own `getprop init.svc.*` and procrank listing; none of these is used by openpilot, the C920, the panda, WiFi, ssh or the screen.
SMALL="qfp-daemon perfprofd ifaadaemon drm"
TELEPHONY="ril-daemon ril-daemon2 imsdatadaemon imsqmidaemon"
YES=0; TEL=0
for a in "$@"; do case "$a" in --yes) YES=1;; --telephony) TEL=1;; *) echo "unknown option $a"; exit 2;; esac; done
LIST="$SMALL"; [ $TEL = 1 ] && LIST="$SMALL $TELEPHONY"
echo "memory now:"; free -m 2>/dev/null | sed -n 1,2p
echo
for s in $LIST; do
  st=$(getprop init.svc.$s 2>/dev/null)
  if [ "$st" != "running" ]; then echo "  $s: not running ($st), skipped"; continue; fi
  if [ $YES = 1 ]; then stop $s && echo "  $s: STOPPED" || echo "  $s: stop FAILED"; else echo "  $s: running, would stop"; fi
done
if [ $YES = 0 ]; then echo; echo "dry run only. Add --yes to apply. A reboot undoes everything."; exit 0; fi
[ $TEL = 1 ] && echo "WARNING: the phone app may keep retrying the modem and log noise; if CPU or the log gets busy, reboot to restore it."
sleep 3; echo; echo "memory after:"; free -m 2>/dev/null | sed -n 1,2p
echo "check: ssh/WiFi still up, camera and model running, panda connected, screen alive. Undo = reboot."
