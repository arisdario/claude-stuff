#!/bin/sh
# Road-camera focus lock. openpilot's autofocus on this EON walks the lens away from focus (OnePlus 3T: starts at the
# far guess 300 and climbs ~3 steps/s towards the close-focus limit 456, so far objects go soft). While
# /data/focus_lock exists, camerad holds the lens at the number written in it instead; delete it for normal autofocus.
#   sh focus_lock.sh install     patch camera_qcom.c (backup .075), rebuild camerad, restart it (lock NOT set yet)
#   sh focus_lock.sh set 300     hold the lens at 300 (OnePlus 3T: 224..456, higher = closer focus); live, ~1 s
#   sh focus_lock.sh off         remove the lock: normal autofocus again
#   sh focus_lock.sh uninstall   restore camera_qcom.c.075, rebuild, restart camerad, remove the lock
set -e
OP=${OP:-/data/openpilot}
CAM=$OP/selfdrive/camerad/cameras/camera_qcom.c
FLAG=/data/focus_lock

restart_cam() { P=$(pidof camerad || true); [ -n "$P" ] && kill $P && echo "camerad restarted (openpilot starts it again, ~30 s on the bench)"; true; }

case "$1" in
  install)
    [ -f "$CAM" ] || { echo "STOP: $CAM not found"; exit 1; }
    [ -f "$CAM.075" ] || cp "$CAM" "$CAM.075"
    python3 - "$CAM" <<'PYEOF'
import sys
p = sys.argv[1]; s = open(p).read()
if 'FOCUS_LOCK' in s:
  print('camera_qcom.c already patched'); sys.exit(0)
anchor = 'static void do_autofocus(CameraState *s) {\n'
if s.count(anchor) != 1:
  sys.exit('STOP: camera_qcom.c is not the expected 0.7.5.a version - not patching')
block = '''  // FOCUS_LOCK: while /data/focus_lock exists, hold the lens at the position written in it (re-read about once a
  // second). Removing the file resumes autofocus from that position.
  {
    static int fl_check = 0;
    static int fl_pos = -1;
    if (--fl_check <= 0) {
      fl_check = 10;
      fl_pos = -1;
      FILE *ff = fopen("/data/focus_lock", "r");
      if (ff != NULL) {
        int v;
        if (fscanf(ff, "%d", &v) == 1) fl_pos = v;
        fclose(ff);
      }
    }
    if (fl_pos >= 0) {
      const int lo = s->device == DEVICE_LP3 ? 366 : 224;
      const int hi = s->device == DEVICE_LP3 ? 634 : 456;
      const int target = fl_pos < lo ? lo : (fl_pos > hi ? hi : fl_pos);
      s->lens_true_pos = target;
      actuator_move(s, target);
      return;
    }
  }
'''
open(p, 'w').write(s.replace(anchor, anchor + block))
print('patched camera_qcom.c')
PYEOF
    cd "$OP" && scons -j4 selfdrive/camerad/camerad || { echo "STOP: camerad build failed - run: sh $0 uninstall"; exit 1; }
    restart_cam; echo "installed. Lock the focus with: sh $0 set 300" ;;
  set)
    [ -n "$2" ] || { echo "usage: sh $0 set NUMBER   (OnePlus 3T: 224..456, higher = closer)"; exit 1; }
    echo "$2" > "$FLAG"; echo "lens held at $2 (applies within ~1 s)" ;;
  off) rm -f "$FLAG"; echo "focus lock off: normal autofocus" ;;
  uninstall)
    [ -f "$CAM.075" ] && cp "$CAM.075" "$CAM" && echo "restored camera_qcom.c"
    rm -f "$FLAG"; cd "$OP" && scons -j4 selfdrive/camerad/camerad && restart_cam ;;
  *) echo "usage: sh $0 install | set NUMBER | off | uninstall"; exit 1 ;;
esac
