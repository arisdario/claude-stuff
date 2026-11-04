#!/usr/bin/env sh
# start_diagnostics.sh - start the READ-ONLY steering diagnostics. Plain POSIX sh: works with Android mksh/toybox and bash.
# Usage:  ./start_diagnostics.sh            start a new session
#         ./start_diagnostics.sh status     show whether recording is active
#         ./start_diagnostics.sh check      6 s read-only self-check of every signal the logger needs
# It never touches openpilot processes: it only starts one extra low-priority python process.
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=${DIAG_BASE:-/data/steering_diagnostics}
PIDF=$BASE/.collector.pid
PARAMS=${DIAG_PARAMS:-/data/params/d}
PYTHONBIN=${DIAG_PYTHON:-}
[ -z "$PYTHONBIN" ] && for p in python3 python /usr/local/pyenv/shims/python; do command -v "$p" >/dev/null 2>&1 && PYTHONBIN=$p && break; done
[ -z "$PYTHONBIN" ] && { echo "no python found (set DIAG_PYTHON=/path/to/python)"; exit 1; }
mkdir -p "$BASE" || exit 1

alive() { # $1=pid ; true only if it is OUR collector
  [ -n "$1" ] && [ -r "/proc/$1/cmdline" ] && tr '\0' ' ' < "/proc/$1/cmdline" | grep -q "collector.py"
}
status() {
  PID=$(cat "$PIDF" 2>/dev/null)
  if alive "$PID"; then
    S=$(cat "$(cat "$BASE/.current" 2>/dev/null)/status.json" 2>/dev/null)
    echo "RECORDING ACTIVE  pid=$PID  session=$(cat "$BASE/.current")"
    echo "  $S"
    return 0
  fi
  echo "not recording"; return 1
}
if [ "$1" = "status" ]; then status; exit $?; fi
if [ "$1" = "check" ]; then # 6 s read-only look at which cereal fields exist in this openpilot build
  PYTHONPATH=/data/openpilot:$HERE:$PYTHONPATH "$PYTHONBIN" "$HERE/collector.py" --selfcheck; exit $?
fi

if status >/dev/null 2>&1; then echo "already recording:"; status; echo "run ./stop_diagnostics.sh first"; exit 1; fi

# pre-flight (python only: no df/awk/ps/pgrep needed): free disk on /data and which openpilot processes run
PF=$("$PYTHONBIN" "$HERE/collector.py" --preflight "$BASE" 2>/dev/null)
FREE=$(echo "$PF" | grep FREE_MB | cut -d= -f2)
PROCS=$(echo "$PF" | grep PROCS | cut -d= -f2)
[ -z "$FREE" ] && FREE=0
if [ "$FREE" -lt 2000 ]; then echo "only ${FREE} MB free on /data - need >= 2000 MB. Free space first (old drives under $BASE)."; exit 1; fi
case "$PROCS" in *controlsd*) ;; *) echo "WARNING: controlsd is not running (running: ${PROCS:-none}) - the logger will wait for data.";; esac

TS=$(date +%Y%m%d_%H%M%S)
SESSION=$BASE/drive_$TS
mkdir -p "$SESSION" || exit 1
echo "$SESSION" > "$BASE/.current"
CFG=""
[ -f "$BASE/config.json" ] && CFG="--cfg $BASE/config.json"
{ echo 1 > "$PARAMS/DiagEnabled"; } 2>/dev/null     # 0 pauses recording at any time

# detach: survives this shell closing; collector handles SIGTERM/SIGINT/SIGHUP itself
PYTHONPATH=/data/openpilot:$HERE:$PYTHONPATH nohup nice -n 10 "$PYTHONBIN" "$HERE/collector.py" --root "$SESSION" --params-dir "$PARAMS" $CFG > "$SESSION/collector.out" 2>&1 &
echo $! > "$PIDF"

# wait for the first status heartbeat
i=0
while [ $i -lt 30 ]; do
  sleep 1; i=$((i+1))
  PID=$(cat "$PIDF")
  if ! alive "$PID"; then echo "collector exited early:"; tail -20 "$SESSION/collector.out"; exit 2; fi
  [ -f "$SESSION/status.json" ] && break
done
[ -f "$SESSION/status.json" ] || { echo "no heartbeat after 30 s:"; tail -20 "$SESSION/collector.out"; exit 2; }
echo "=================================================="
echo " RECORDING ACTIVE   (vehicle control is NOT affected)"
echo " session : $SESSION"
echo " check   : ./start_diagnostics.sh status"
echo " pause   : echo 0 > $PARAMS/DiagEnabled   (1 resumes)"
echo " finish  : ./stop_diagnostics.sh   -> $SESSION.zip"
echo "=================================================="
sleep 3
status
