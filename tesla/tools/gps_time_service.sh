#!/system/bin/sh
# gps_time_service.sh - background service wrapper around gps_boot_time_sync.sh. Launch it once,
# backgrounded, at boot (see WIRING below). It keeps retrying until the clock is set from GPS or the
# clock becomes sane another way (e.g. NTP once WiFi is up), then EXITS - it is not a permanent daemon.
#
# Why a wrapper and not just the one-shot: a cold GPS chip can take several minutes to get a satellite
# time lock, and at boot gpsd itself (started by openpilot's manager) may not be publishing yet. So we
# retry: each attempt fails fast (~8s) while gpsd is silent, then waits up to ATTEMPT_TIMEOUT_S once
# gpsd is up, sleeping between attempts, for up to TOTAL_WINDOW_S overall.
#
# All output goes to a log file (LOG), since this runs in the background with no terminal.
#   tail -f /data/gps_time_service.log      # watch it live
#
# WIRING (auto-start): add this one line to /data/openpilot/launch_chffrplus.sh, anywhere AFTER the
# PYTHONPATH export and BEFORE './manager.py' (so the openpilot env is set; manager then starts gpsd
# alongside it):
#   (sh /data/claude-stuff/tesla/tools/gps_time_service.sh &)
# Back up launch_chffrplus.sh first: cp launch_chffrplus.sh launch_chffrplus.sh.bak
#
# Env overrides:
#   TOTAL_WINDOW_S     - overall give-up window in seconds (default 1200 = 20 min)
#   ATTEMPT_TIMEOUT_S  - max wait per attempt once gpsd is publishing (default 60)
#   SLEEP_BETWEEN_S    - sleep between attempts (default 20)
#   GPS_TIME_LOG       - log file path (default /data/gps_time_service.log)
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
SYNC="$HERE/gps_boot_time_sync.sh"
LOG="${GPS_TIME_LOG:-/data/gps_time_service.log}"
TOTAL_WINDOW_S="${TOTAL_WINDOW_S:-1200}"
ATTEMPT_TIMEOUT_S="${ATTEMPT_TIMEOUT_S:-60}"
SLEEP_BETWEEN_S="${SLEEP_BETWEEN_S:-20}"

log() { echo "$(date -u '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null) gps_time_service: $*" >> "$LOG" 2>/dev/null; }

if [ ! -r "$SYNC" ]; then
  log "cannot find $SYNC - aborting"
  exit 2
fi

# how many attempts fit in the window (each attempt ~ ATTEMPT_TIMEOUT_S + SLEEP_BETWEEN_S of wall time)
per_attempt=$(( ATTEMPT_TIMEOUT_S + SLEEP_BETWEEN_S ))
[ "$per_attempt" -lt 1 ] && per_attempt=1
max_attempts=$(( TOTAL_WINDOW_S / per_attempt + 1 ))

log "started: up to $max_attempts attempts over ~${TOTAL_WINDOW_S}s"

i=0
while [ "$i" -lt "$max_attempts" ]; do
  i=$(( i + 1 ))
  # gps_boot_time_sync.sh returns 0 when the clock is already sane OR was just set from GPS - either
  # way we are done. It returns nonzero only when the clock is reset AND no GPS time was obtained.
  if TIMEOUT_S="$ATTEMPT_TIMEOUT_S" sh "$SYNC" >> "$LOG" 2>&1; then
    log "clock is sane (set from GPS or already correct) after $i attempt(s) - done"
    exit 0
  fi
  log "attempt $i/$max_attempts: no GPS time yet, retrying in ${SLEEP_BETWEEN_S}s"
  sleep "$SLEEP_BETWEEN_S"
done

log "gave up after $max_attempts attempts (~${TOTAL_WINDOW_S}s) - clock left as-is for NTP"
exit 1
