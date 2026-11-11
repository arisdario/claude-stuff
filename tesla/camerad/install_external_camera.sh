#!/bin/sh
# Installs the external-camera build of camerad on the EON (the C920 option). Nothing changes until you run it with --yes. Stop openpilot first (tmux kill-session -t comma), build, then reboot.
#   sh install_external_camera.sh             shows what would happen (copies and builds nothing)
#   sh install_external_camera.sh --yes       backs up the stock camerad ONCE (/data/camerad_backup_stock), copies the new sources, builds camerad
#   sh install_external_camera.sh --restore   puts the stock camerad (binary and sources) back - no build needed
# The new camerad behaves exactly like the old one until /data/camera_source says "external" (sh camera_switch.sh external).
set -e
OP=${OP:-/data/openpilot}
HERE=$(cd "$(dirname "$0")" && pwd)
DIR=$OP/selfdrive/camerad
BK=${BK:-/data/camerad_backup_stock}
SCONS=${SCONS:-scons}
[ -d "$DIR" ] || { echo "STOP: $DIR not found (set OP=...)"; exit 1; }
FILES="SConscript cameras/camera_qcom.c cameras/camera_ext_c920.cc cameras/camera_ext_c920.h cameras/camera_profile.h cameras/camera_h264.h"
# sign/light detection was REMOVED: these are deleted from camerad on install, so the build gate in
# SConscript (needs cameras/sign_detect.cc) leaves it out and camerad is built without it.
REMOVED="cameras/sign_detect.cc cameras/sign_detect.h cameras/sign_detect_io.cc cameras/sign_detect_io.h"

if [ "$1" = "--restore" ]; then
  [ -d "$BK" ] || { echo "STOP: no backup at $BK"; exit 1; }
  for f in $FILES; do [ -f "$BK/$f" ] && cp "$BK/$f" "$DIR/$f" || rm -f "$DIR/$f"; done   # the two new files did not exist before: they are removed
  rm -rf "$DIR/libuvc"
  [ -f "$BK/camerad" ] && cp "$BK/camerad" "$DIR/camerad" && chmod +x "$DIR/camerad"
  echo "stock camerad restored (binary and sources). Reboot."; exit 0
fi

echo "will copy into $DIR:"; for f in $FILES libuvc; do echo "  $f"; done
if [ "$1" != "--yes" ]; then echo; echo "nothing done. To install:  sh $0 --yes   (stop openpilot first: tmux kill-session -t comma)"; exit 0; fi

if [ ! -d "$BK" ]; then
  mkdir -p "$BK/cameras"
  for f in $FILES; do [ -f "$DIR/$f" ] && cp "$DIR/$f" "$BK/$f"; done
  [ -f "$DIR/camerad" ] && cp "$DIR/camerad" "$BK/camerad"
  echo "stock camerad backed up in $BK"
else
  echo "backup already exists in $BK (kept)"
fi
for f in $FILES; do cp "$HERE/$f" "$DIR/$f"; done
rm -rf "$DIR/libuvc" && cp -r "$HERE/libuvc" "$DIR/libuvc"
for f in $REMOVED; do rm -f "$DIR/$f"; done
cd "$OP"
echo "building camerad ..."
if $SCONS -j2 selfdrive/camerad/camerad; then
  echo "built. Reboot to use it. It still uses the phone's camera until you run:  sh $HERE/camera_switch.sh external"
else
  echo "BUILD FAILED. Put the stock camerad back before rebooting:  sh $0 --restore"; exit 1
fi
