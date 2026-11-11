#!/system/bin/sh
# autoclock_service.sh - background boot service that sets the EON system clock when it has no RTC
# backup battery (loses the time every full power-off). Source priority:
#   1. CAN  - the car's own gateway clock (GTW_carState 0x318), via can_time_sync.py. Reliable, needs
#             no GPS sky view and no network: works on a bench whenever the car is powered and boardd
#             is passing CAN. This is the primary source.
#   2. GPS  - gps_boot_time_sync.sh, as a fallback (only useful with a real satellite fix / sky view).
# It retries (boardd/gpsd may not be up yet right at boot), and EXITS as soon as the clock is sane -
# it is not a permanent daemon. All output goes to a log file since it runs with no terminal.
#   tail -f /data/autoclock.log
#
# WIRING (auto-start): add this one line to /data/openpilot/launch_chffrplus.sh, AFTER the PYTHONPATH
# export and BEFORE './manager.py' (so the openpilot env is set and manager then brings up boardd/gpsd):
#   (sh /data/claude-stuff/tesla/tools/autoclock_service.sh &)
# Back it up first:  cp launch_chffrplus.sh launch_chffrplus.sh.bak
#
# Env overrides: FLOOR_EPOCH_S, TOTAL_WINDOW_S (default 900), SLEEP_BETWEEN_S (default 15),
#                AUTOCLOCK_LOG, PY_CMD.
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
CAN="$HERE/can_time_sync.py"
GPS="$HERE/gps_boot_time_sync.sh"
LOG="${AUTOCLOCK_LOG:-/data/autoclock.log}"
FLOOR_EPOCH_S="${FLOOR_EPOCH_S:-1735689600}"   # 2025-01-01; a clock >= this is treated as already-sane
TOTAL_WINDOW_S="${TOTAL_WINDOW_S:-900}"         # keep trying up to 15 min
SLEEP_BETWEEN_S="${SLEEP_BETWEEN_S:-15}"
PY_CMD="${PY_CMD:-python3}"

log() { echo "$(date -u '+%Y-%m-%dT%H:%M:%SZ' 2>/dev/null) autoclock: $*" >> "$LOG" 2>/dev/null; }

clock_sane() {
  now="$(date +%s 2>/dev/null)"
  [ -n "$now" ] && [ "$now" -ge "$FLOOR_EPOCH_S" ] 2>/dev/null
}

if clock_sane; then
  log "clock already sane - nothing to do"
  exit 0
fi

log "clock looks reset - trying CAN (primary) then GPS, up to ${TOTAL_WINDOW_S}s"
per=$(( SLEEP_BETWEEN_S + 15 )); [ "$per" -lt 1 ] && per=1
max=$(( TOTAL_WINDOW_S / per + 1 ))

i=0
while [ "$i" -lt "$max" ]; do
  i=$(( i + 1 ))

  # 1. CAN - the car's gateway clock. Succeeds (exit 0) only when it actually set the clock.
  if [ -r "$CAN" ] && TIMEOUT_S=10 "$PY_CMD" "$CAN" >> "$LOG" 2>&1; then
    log "clock set from CAN on attempt $i - done"
    exit 0
  fi

  # 2. GPS fallback (returns 0 if it set the clock or it was already sane; guard with clock_sane).
  if [ -r "$GPS" ] && TIMEOUT_S=15 sh "$GPS" >> "$LOG" 2>&1 && clock_sane; then
    log "clock set from GPS on attempt $i - done"
    exit 0
  fi

  # NTP (once WiFi is up) could also fix it while we loop.
  if clock_sane; then
    log "clock became sane on attempt $i (NTP?) - done"
    exit 0
  fi

  log "attempt $i/$max: no time source yet, retry in ${SLEEP_BETWEEN_S}s"
  sleep "$SLEEP_BETWEEN_S"
done

log "gave up after $max attempts - clock left as-is"
exit 1
