#!/bin/sh
# THNEED for Tinkla 0.7.5.a's modeld (port of openpilot 0.8.3's thneed) - see THNEED.md. Stop openpilot first:
#   tmux kill-session -t comma
#   sh install_thneed.sh             shows what would happen (changes nothing)
#   sh install_thneed.sh --yes       checks, backs up once, copies, builds _modeld (+ the bench), runs the bench both ways;
#                                    the new _modeld is KEPT ONLY IF the bench says it works, otherwise restored at once
#   sh install_thneed.sh --bench     runs only the bench again
#   sh install_thneed.sh --restore   puts the previous files and _modeld back (no build needed), then reboot
# Switch thneed off without uninstalling: touch /data/thneed_off (modeld then runs plain SNPE, as before).
set -e
OP=${OP:-/data/openpilot}
M=$OP/selfdrive/modeld
HERE=$(cd "$(dirname "$0")" && pwd)
BK=${BK:-/data/thneed_backup}
SNPE_LIB=/data/pythonpath/phonelibs/snpe/aarch64
GATE_S=${THNEED_GATE_S:-20}     # how long the real modeld must survive its start
FILES="SConscript runners/snpemodel.cc runners/snpemodel.h"
NEW="thneed/thneed.cc thneed/thneed.h thneed/input_map.h thneed/bench.cc thneed/include/msm_kgsl.h"
# the car's files this port was made from (Tinkla 0.7.5.a, /data/openpilot as uploaded on 15 Nov 2026)
WANT="SConscript:1a889cc27436bd023d3104283131354a runners/snpemodel.cc:eeedef0912cbbaa343b2295bd61e34a3 runners/snpemodel.h:464e4477b48c9592d23ffa4b4623a269"
# the first version of this port (73a76e3, 16 Nov 2026 - crashed modeld at start: class layout differed between files)
OLDPORT="SConscript:6534d7877e5b19ac589bc01c2628f6e5 runners/snpemodel.cc:f5afcb88d2a92a81173ae32649409ac4 runners/snpemodel.h:bae049c4cb271b8b0717a3d4722f80a3"

[ -d "$M" ] || { echo "STOP: $M not found (set OP=...)"; exit 1; }
md5() { md5sum "$1" 2>/dev/null | cut -d' ' -f1; }

out_floats() {
  if [ -f "$M/models/model0812_adapter.h" ]; then echo 6472
  elif [ -f "$M/models/model083_adapter.h" ]; then echo 11327
  else echo 0; fi
}

bench() {
  N=$(out_floats)
  [ "$N" != 0 ] || { echo "STOP: no 0.8.3 / 0.8.12 model adapter in $M/models - the bench needs to know the output size"; exit 1; }
  [ -x "$M/thneed/thneed_bench" ] || { echo "no thneed/thneed_bench: thneed was not built in, nothing to bench"; return 0; }
  if pgrep -f _modeld >/dev/null 2>&1; then echo "STOP: modeld is running - stop openpilot first (tmux kill-session -t comma)"; exit 1; fi
  cd "$M"
  echo "bench: plain SNPE (as now), 50 runs ..."
  LD_LIBRARY_PATH=$SNPE_LIB:$LD_LIBRARY_PATH THNEED_OFF=1 ./thneed/thneed_bench ../../models/supercombo.dlc "$N" 50 > /tmp/thneed_plain.log 2>&1 || true
  echo "bench: thneed, 50 runs ..."
  LD_LIBRARY_PATH=$SNPE_LIB:$LD_LIBRARY_PATH ./thneed/thneed_bench ../../models/supercombo.dlc "$N" 50 > /tmp/thneed_on.log 2>&1 || true
  P=$(grep "per run" /tmp/thneed_plain.log | tail -1); T=$(grep "per run" /tmp/thneed_on.log | tail -1)
  if [ -z "$P" ] || [ -z "$T" ]; then
    tail -5 /tmp/thneed_plain.log /tmp/thneed_on.log | sed 's/^/  /'
    echo "RESULT: the bench did not finish (crash?) - this build must not run in the car"; BENCH=crashed; return 0
  fi
  echo "  $P"; echo "  $T"
  grep -E "thneed selftest|thneed:|Thneed::stop" /tmp/thneed_on.log | sed 's/^/  /' | head -8
  CP=$(echo "$P" | sed -n 's/.*checksum \([0-9a-f]*\).*/\1/p'); CT=$(echo "$T" | sed -n 's/.*checksum \([0-9a-f]*\).*/\1/p')
  if grep -q "thneed selftest passed" /tmp/thneed_on.log && [ -n "$CP" ] && [ "$CP" = "$CT" ]; then
    echo "RESULT: thneed works on this EON - same output as plain SNPE after 50 runs."; BENCH=works
  elif grep -q "thneed selftest passed" /tmp/thneed_on.log; then
    echo "RESULT: self-test passed but the 50-run output DIFFERS from plain SNPE - not usable"; BENCH=differs
  else
    echo "RESULT: thneed cannot be used on this EON (see the lines above)"; BENCH=unusable
  fi
  echo "full logs: /tmp/thneed_plain.log /tmp/thneed_on.log"
}

if [ "$1" = "--restore" ]; then
  [ -d "$BK" ] || { echo "STOP: no backup at $BK"; exit 1; }
  for f in $FILES; do cp "$BK/$f" "$M/$f"; done
  cp "$BK/_modeld" "$M/_modeld" && chmod +x "$M/_modeld"
  echo "previous modeld restored (sources and binary). Reboot."; exit 0
fi
if [ "$1" = "--bench" ]; then bench; exit 0; fi

echo "will copy into $M:"; for f in $FILES $NEW; do echo "  $f"; done
for pair in $WANT; do
  f=${pair%%:*}; want=${pair#*:}; have=$(md5 "$M/$f"); port=$(md5 "$HERE/$f")
  old=$(for o in $OLDPORT; do [ "${o%%:*}" = "$f" ] && echo "${o#*:}"; done; true)   # (true: set -e would end the script here)
  if [ "$have" != "$want" ] && [ "$have" != "$port" ] && [ "$have" != "$old" ]; then
    echo "STOP: $M/$f is not the file this port was made from (md5 $have, expected $want) - nothing changed. Send me the file."; exit 1
  fi
done
[ "$(out_floats)" != 0 ] || { echo "STOP: no 0.8.3 / 0.8.12 model adapter installed in $M/models"; exit 1; }
if [ "$1" != "--yes" ]; then echo; echo "checks passed, nothing done. To install: tmux kill-session -t comma; sh $0 --yes"; exit 0; fi
if pgrep -f _modeld >/dev/null 2>&1; then echo "STOP: modeld is running - stop openpilot first (tmux kill-session -t comma)"; exit 1; fi

if [ ! -d "$BK" ]; then
  mkdir -p "$BK/runners"
  for f in $FILES; do cp "$M/$f" "$BK/$f"; done
  cp "$M/_modeld" "$BK/_modeld"
  echo "backed up in $BK"
else
  echo "backup already exists in $BK (kept)"
fi
mkdir -p "$M/thneed/include"
for f in $FILES $NEW; do cp "$HERE/$f" "$M/$f"; done

cd "$OP"
echo "building _modeld ..."
if ! scons -j2 selfdrive/modeld/_modeld 2>&1 | tee /tmp/thneed_build.log; then
  echo "BUILD FAILED - put the previous modeld back before rebooting: sh $0 --restore"; exit 1
fi
if grep -q "BUILD FAILED\|scons: building terminated because of errors" /tmp/thneed_build.log; then
  echo "BUILD FAILED - put the previous modeld back before rebooting: sh $0 --restore"; exit 1
fi
if grep -q "modeld: thneed is built in" /tmp/thneed_build.log; then
  grep "modeld: thneed is built in" /tmp/thneed_build.log | sed 's/^/  /'
  echo "  (comma's own NEOS 14 thneed, openpilot 0.7.6, used dlsym offset -724)"
  scons -j2 selfdrive/modeld/thneed/thneed_bench 2>&1 | tail -3
  BENCH=none
  bench
  if [ "$BENCH" = works ]; then
    # GATE 2 (added after 16 Nov 2026): the REAL _modeld must start and load the model (the bench alone missed a crash
    # in modeld's own start). With openpilot stopped it then waits for camera frames - alive after 20 s = it started.
    echo "real modeld: starting it for $GATE_S s ..."
    cd "$M"
    ./modeld > /tmp/thneed_modeld.log 2>&1 &
    MP=$!
    sleep $GATE_S
    if kill -0 $MP 2>/dev/null && grep -q "loaded model" /tmp/thneed_modeld.log; then
      kill -INT $MP 2>/dev/null; sleep 2; kill -9 $MP 2>/dev/null || true
      echo "  real modeld started and loaded the model"
    else
      kill -9 $MP 2>/dev/null || true
      tail -5 /tmp/thneed_modeld.log | sed 's/^/  /'
      echo "RESULT: the real modeld did not survive its start"; BENCH=modeld_died
    fi
  fi
  if [ "$BENCH" = works ]; then
    echo "KEPT: the new modeld uses thneed. Reboot to use it."
  else
    for f in $FILES; do cp "$BK/$f" "$M/$f"; done
    cp "$BK/_modeld" "$M/_modeld" && chmod +x "$M/_modeld"
    echo "RESTORED: the previous modeld is back (nothing changes in the car). Send me /tmp/thneed_build.log /tmp/thneed_plain.log /tmp/thneed_on.log /tmp/thneed_modeld.log"
  fi
else
  grep "modeld: thneed" /tmp/thneed_build.log || true
  echo "thneed was NOT built in: _modeld was rebuilt exactly as before (plain SNPE). Send me /tmp/thneed_build.log."
fi
