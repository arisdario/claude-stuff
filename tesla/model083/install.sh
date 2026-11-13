#!/bin/sh
# Installs the openpilot 0.8.3 driving model into Tinkla 0.7.5.a, on the EON, parked. See README.md in this folder.
#   sh /data/claude-stuff/tesla/model083/install.sh
# Undo with uninstall.sh (same folder). Nothing is deleted: originals are kept as *.075 next to each file.
set -e
OP=${OP:-/data/openpilot}
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
MD=$OP/selfdrive/modeld/models
CAR=$OP/selfdrive/car/tesla
SHA=059efa4e4c5f1430271362147e4b2eda650dc7ca9bad6a63725cc616afdf7f77
URL=https://media.githubusercontent.com/media/commaai/openpilot/v0.8.3/models/supercombo.dlc

say() { echo; echo "=== $* ==="; }
die() { echo "STOP: $*"; exit 1; }
sha() { python3 -c "import hashlib,sys;print(hashlib.sha256(open(sys.argv[1],'rb').read()).hexdigest())" "$1"; }
keep() { [ -f "$1" ] && [ ! -f "$1.075" ] && cp "$1" "$1.075" && echo "backed up $1"; true; }   # first run only

[ -f "$MD/driving.cc" ] && [ -f "$OP/models/supercombo.dlc" ] || die "$OP does not look like openpilot (set OP=...)"
grep -q "0.7.5" "$OP/RELEASES.md" 2>/dev/null || echo "note: could not confirm 0.7.5 in $OP/RELEASES.md - continuing"

say "1. backups (kept as *.075)"
keep "$OP/models/supercombo.dlc"; keep "$MD/driving.h"; keep "$MD/driving.cc"
keep "$CAR/PCC_module.py"; keep "$CAR/pcc_curves.py"

say "2. the 0.8.3 model"
if [ -f "$OP/models/supercombo.dlc.083" ] && [ "$(sha "$OP/models/supercombo.dlc.083")" = "$SHA" ]; then
  echo "already downloaded and verified"
else
  curl -L --fail -o "$OP/models/supercombo.dlc.083" "$URL" || die "download failed (internet? clock right for https?)"
  SZ=$(wc -c < "$OP/models/supercombo.dlc.083")
  [ "$SZ" -gt 1000000 ] || die "got $SZ bytes - a Git LFS pointer, not the model"
  [ "$(sha "$OP/models/supercombo.dlc.083")" = "$SHA" ] || die "checksum mismatch - not the 0.8.3 model, not installing"
  echo "downloaded, checksum OK"
fi

say "3. keep the sources modeld is built from now, to put back if the build fails"
PREV=$MD/.prev_install; rm -rf "$PREV"; mkdir -p "$PREV"
for f in driving.h driving.cc model083_adapter.h model0812_adapter.h; do if [ -f "$MD/$f" ]; then cp "$MD/$f" "$PREV/"; fi; done
echo "saved in $PREV"

say "4. the adapter (0.8.3 output -> 0.7.5 messages)"
cp "$HERE/driving.h" "$HERE/driving.cc" "$HERE/model083_adapter.h" "$MD/"
rm -f "$MD/model0812_adapter.h"     # driving.cc would pick the 0.8.12 adapter if it were there (install0812.sh)
echo "copied driving.h driving.cc model083_adapter.h"

say "5. PCC with the curve-limit fix (the new model turns PCC's camera curve limit on)"
for f in PCC_module.py pcc_curves.py; do
  if [ -f "$CAR/$f" ] && cmp -s "$REPO/tesla/$f" "$CAR/$f"; then echo "$f already current"; else cp "$REPO/tesla/$f" "$CAR/$f"; echo "updated $f"; fi
done

say "6. rebuild modeld (the model file is swapped only AFTER this succeeds)"
if ! (cd "$OP" && scons -j4 selfdrive/modeld/_modeld); then
  echo "build FAILED - putting the previous sources back; the model file was NOT changed"
  rm -f "$MD/model083_adapter.h" "$MD/model0812_adapter.h"; cp "$PREV"/* "$MD/" 2>/dev/null || true
  (cd "$OP" && scons -j4 selfdrive/modeld/_modeld) >/dev/null 2>&1 || true
  die "build failed - nothing switched. Send me the error above"
fi

say "7. switch the model file (build OK)"
cp "$OP/models/supercombo.dlc.083" "$OP/models/supercombo.dlc"
echo "models/supercombo.dlc = the 0.8.3 model"

say "done"
echo "Reboot, then with the car on and openpilot NOT engaged:"
echo "  cd $OP && PYTHONPATH=$OP python3 $HERE/check_model.py 60"
echo "If 'no model messages': cd $OP/selfdrive/modeld && ./modeld   (with openpilot stopped) and send me the output."
