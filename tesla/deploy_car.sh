#!/bin/sh
# Copies this repo's Tesla car code into openpilot (selfdrive/car/tesla) as ONE consistent set - e.g. carcontroller.py
# needs uds_relay.py and the new AHB_module.py, so copying it alone would crash openpilot at start.
#   sh /data/claude-stuff/tesla/deploy_car.sh         lists what would change (copies nothing)
#   sh /data/claude-stuff/tesla/deploy_car.sh --yes   backs up (*.075, first time only), copies, checks they compile
# Python only: no build needed, just restart openpilot (reboot). The model is separate: model083/install.sh.
set -e
OP=${OP:-/data/openpilot}
HERE=$(cd "$(dirname "$0")" && pwd)
CAR=$OP/selfdrive/car/tesla
[ -d "$CAR" ] || { echo "STOP: $CAR not found (set OP=...)"; exit 1; }

CHANGED=""
for f in $(cd "$HERE" && ls *.py speed_utils/*.py 2>/dev/null); do
  if [ ! -f "$CAR/$f" ]; then CHANGED="$CHANGED $f"; echo "new      $f"
  elif ! cmp -s "$HERE/$f" "$CAR/$f"; then CHANGED="$CHANGED $f"; echo "changed  $f"; fi
done
[ -n "$CHANGED" ] || { echo "the car code already matches the repo - nothing to do"; exit 0; }
if [ "$1" != "--yes" ]; then
  echo; echo "nothing copied. Check the list (diff a file: diff $CAR/FILE $HERE/FILE), then:  sh $0 --yes"; exit 0
fi

for f in $CHANGED; do
  [ -f "$CAR/$f" ] && [ ! -f "$CAR/$f.075" ] && cp "$CAR/$f" "$CAR/$f.075"
  cp "$HERE/$f" "$CAR/$f"; echo "copied $f"
done
if (cd "$OP" && for f in $CHANGED; do python3 -m py_compile "selfdrive/car/tesla/$f" || exit 1; done); then
  echo "all copied files compile - reboot to use them"
else
  echo "STOP: a file does not compile. Put the originals back before rebooting:"
  for f in $CHANGED; do [ -f "$CAR/$f.075" ] && echo "  cp $CAR/$f.075 $CAR/$f" || echo "  rm $CAR/$f   (was new)"; done
  exit 1
fi
echo "undo later: cp FILE.075 FILE for each file listed above (new files: delete them)"
