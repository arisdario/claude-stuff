#!/usr/bin/env sh
# stop_diagnostics.sh - stop the collector cleanly, flush, analyse, plot and build ONE zip. Plain POSIX sh.
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=${DIAG_BASE:-/data/steering_diagnostics}
PIDF=$BASE/.collector.pid
PARAMS=${DIAG_PARAMS:-/data/params/d}
PYTHONBIN=${DIAG_PYTHON:-}
[ -z "$PYTHONBIN" ] && for p in python3 python /usr/local/pyenv/shims/python; do command -v "$p" >/dev/null 2>&1 && PYTHONBIN=$p && break; done
[ -z "$PYTHONBIN" ] && { echo "no python found"; exit 1; }
SESSION=$(cat "$BASE/.current" 2>/dev/null)
[ -z "$SESSION" ] || [ ! -d "$SESSION" ] && { echo "no session to finish"; exit 1; }
PID=$(cat "$PIDF" 2>/dev/null)
if [ -n "$PID" ] && [ -r "/proc/$PID/cmdline" ] && tr '\0' ' ' < "/proc/$PID/cmdline" | grep -q "collector.py"; then
  echo "stopping collector pid $PID (flushing buffers)..."
  kill -TERM "$PID"
  i=0
  while [ $i -lt 45 ] && [ -r "/proc/$PID/cmdline" ]; do sleep 1; i=$((i+1)); done
  if [ -r "/proc/$PID/cmdline" ]; then echo "collector did not exit in 45 s - forcing (event copies in progress may be incomplete)"; kill -KILL "$PID"; sleep 1; fi
else
  echo "collector is not running (stopped earlier or the EON lost power) - finalizing what was recorded"
fi
rm -f "$PIDF"
{ echo 0 > "$PARAMS/DiagEnabled"; } 2>/dev/null
echo "analysing, plotting, zipping (a few minutes on the EON for a long drive)..."
nice -n 15 "$PYTHONBIN" "$HERE/finalize_session.py" "$SESSION"
RC=$?
echo "----------------------------------------"
head -40 "$SESSION/summary.txt" 2>/dev/null
echo "----------------------------------------"
[ -f "$SESSION/root_cause_report.txt" ] && head -30 "$SESSION/root_cause_report.txt"
[ $RC -eq 0 ] && echo "SEND ME THIS FILE: $SESSION.zip"
exit $RC
