#!/system/bin/sh
# gps_boot_time_sync.sh - sets the EON's system clock from GPS time on boot, for the case where the
# phone has no working RTC backup battery and loses the date/time every time it's fully powered off
# (NTP then can't run either, since without a sane clock TLS cert validation for the WiFi/network
# stack can itself fail - the classic chicken-and-egg problem this script exists to break).
#
# WHY THIS NEEDS ONE PYTHON CALL: the EON's own internal GPS chip is NOT exposed as a plain NMEA
# serial device bash could `cat` (unlike an external USB GPS dongle). The only thing that already
# talks to it is openpilot's own selfdrive/sensord/gpsd, which bridges it through Android's GPS HAL
# and republishes fixes on the cereal topic 'gpsNMEA' (see cereal/log.capnp's GPSNMEAData). That's a
# capnp/msgq pub-sub socket, not a file - there is no bash-only way to read it. So this script shells
# out to one small inline python3 snippet to pull a single timestamp off that socket; every other
# step (deciding whether to bother, validating the value, setting the clock, remembering it) is
# plain shell, no python needed.
#
# Safety: never sets the clock backward past FLOOR_EPOCH_S (a hardcoded "no sane boot is before this
# date" floor, bumped forward over time - bump it periodically) or past a saved "last known good"
# time on disk (LASTGOOD_FILE), and never trusts a GPS timestamp more than MAX_SKEW_YEARS past that
# floor either (catches a garbled/corrupt NMEA parse jumping decades ahead). If the clock already
# looks sane (>= floor) on boot - e.g. the phone kept power this time, or NTP already ran - this is a
# fast no-op; it only waits on GPS when the clock is actually in the obviously-reset state.
# Read-only otherwise: never touches CAN, never touches any openpilot/car process.
#
#   sh gps_boot_time_sync.sh            # normal use, called from boot (see below for wiring it in)
#
# Env overrides (for testing, or if your NEOS layout differs):
#   FLOOR_EPOCH_S   - hardcoded sanity floor, seconds since epoch (default below, ~2025-01-01 UTC)
#   LASTGOOD_FILE   - persisted last-known-good time, survives reboot even with no RTC (default /data/gps_time_sync_last_good)
#   TIMEOUT_S       - max seconds to wait for a usable GPS fix before giving up (default 90)
#   MAX_SKEW_YEARS  - reject a GPS timestamp more than this many years past FLOOR_EPOCH_S (default 5)
#   DATE_CMD        - the date binary to use (default: date)
#   PY_CMD          - the python3 binary to use (default: python3)
#   NOW_CMD         - how to read the current time as epoch seconds (default: "$DATE_CMD +%s")
#
# After confirming this works on your device (see tests/test_gps_boot_time_sync.py and the
# check_gps_nmea_time.py diagnostic sent earlier), wire it into boot by adding it as an early,
# non-blocking step wherever your fork launches sensord (manager.py's process list) - NOT before
# sensord/gpsd itself is running, since this script depends on gpsd already publishing gpsNMEA.
set -u

FLOOR_EPOCH_S="${FLOOR_EPOCH_S:-1735689600}"   # 2025-01-01T00:00:00Z - bump forward periodically
LASTGOOD_FILE="${LASTGOOD_FILE:-/data/gps_time_sync_last_good}"
TIMEOUT_S="${TIMEOUT_S:-90}"
MAX_SKEW_YEARS="${MAX_SKEW_YEARS:-5}"
DATE_CMD="${DATE_CMD:-date}"
PY_CMD="${PY_CMD:-python3}"
NOW_CMD="${NOW_CMD:-$DATE_CMD +%s}"

log() { echo "gps_boot_time_sync: $*"; }

# --- floor: never go below FLOOR_EPOCH_S, and never below whatever we last knew to be good ---
effective_floor() {
  floor="$FLOOR_EPOCH_S"
  if [ -r "$LASTGOOD_FILE" ]; then
    saved="$(cat "$LASTGOOD_FILE" 2>/dev/null)"
    case "$saved" in
      ''|*[!0-9]*) ;;  # not a plain non-negative integer - ignore a corrupt file
      *) [ "$saved" -gt "$floor" ] && floor="$saved" ;;
    esac
  fi
  echo "$floor"
}

write_lastgood() {
  tmp="$LASTGOOD_FILE.tmp.$$"
  echo "$1" > "$tmp" && mv -f "$tmp" "$LASTGOOD_FILE"
}

# --- is a plain non-negative integer in [floor, floor + MAX_SKEW_YEARS] ? ---
is_sane_epoch() {
  val="$1"; floor="$2"
  case "$val" in
    ''|*[!0-9]*) return 1 ;;
  esac
  ceiling=$((floor + MAX_SKEW_YEARS * 365 * 86400))
  [ "$val" -ge "$floor" ] && [ "$val" -le "$ceiling" ]
}

set_clock() {
  epoch="$1"
  # %Y-%m-%d %H:%M:%S in UTC is the one `date -s` spec both toybox and busybox agree on;
  # avoids relying on '@epoch' support, which older Android date implementations may lack.
  spec="$($DATE_CMD -u -d "@$epoch" +'%Y-%m-%d %H:%M:%S' 2>/dev/null)"
  if [ -z "$spec" ]; then
    # BusyBox/toybox date has no -d on some builds; fall back to python3 for the format only
    # (still not touching the clock from python - just formatting the string for `date -s`).
    spec="$($PY_CMD -c "import datetime,sys; print(datetime.datetime.utcfromtimestamp(int(sys.argv[1])).strftime('%Y-%m-%d %H:%M:%S'))" "$epoch" 2>/dev/null)"
  fi
  if [ -z "$spec" ]; then
    log "could not format epoch $epoch into a date -s spec - giving up"
    return 1
  fi
  if ! "$DATE_CMD" -u -s "$spec" >/dev/null 2>&1; then
    log "date -u -s '$spec' failed (are we root?) - giving up"
    return 1
  fi
  log "system clock set to $spec UTC (from GPS epoch $epoch)"
  return 0
}

# --- the one non-bash step: wait up to TIMEOUT_S for a usable gpsNMEA timestamp. The chosen epoch is
# the ONLY thing printed to stdout (so the shell can capture it with $(...)); all progress and
# diagnostics go to stderr so you can see it is alive and WHY it is or isn't getting a fix. It fails
# FAST (does not burn the whole timeout) when gpsd is not publishing at all. Exit codes:
#   0 found a usable time   2 cereal not importable   3 gpsd silent (no msgs)   1 msgs but no time lock
GPS_SILENT_GRACE_S="${GPS_SILENT_GRACE_S:-8}"   # if not one gpsNMEA msg arrives in this long, give up early
poll_gps_epoch() {
  floor="$1"
  GPS_SILENT_GRACE_S="$GPS_SILENT_GRACE_S" "$PY_CMD" - "$floor" "$TIMEOUT_S" "$MAX_SKEW_YEARS" <<'PYEOF'
import os, sys, time
floor = int(sys.argv[1])
timeout_s = float(sys.argv[2])
max_skew_years = float(sys.argv[3])
grace_s = float(os.environ.get("GPS_SILENT_GRACE_S", "8"))
ceiling = floor + max_skew_years * 365 * 86400

def err(m):
    sys.stderr.write(m + "\n"); sys.stderr.flush()

try:
    import cereal.messaging as messaging
except Exception as e:
    err("gpsNMEA: cannot import cereal (%s)" % e)
    err("         -> run this from /data/openpilot with PYTHONPATH set, with openpilot built.")
    sys.exit(2)

sm = messaging.SubMaster(['gpsNMEA'])
start = time.monotonic()
deadline = start + timeout_s
seen = 0
last_report = -2.0
while time.monotonic() < deadline:
    sm.update(1000)
    el = time.monotonic() - start
    if sm.updated['gpsNMEA']:
        seen += 1
        epoch_s = sm['gpsNMEA'].timestamp // 1000
        if floor <= epoch_s <= ceiling:
            err("gpsNMEA: usable fix after %ds (epoch %d)" % (int(el), epoch_s))
            print(int(epoch_s)); sys.exit(0)
    # heartbeat every ~2s so it never looks frozen
    if el - last_report >= 2.0:
        last_report = el
        err("  ...%ds/%ds  gpsNMEA msgs: %d  (none with satellite time lock yet)" % (int(el), int(timeout_s), seen))
    # fail fast: if gpsd never emits a single message, there's nothing to wait for
    if seen == 0 and el >= grace_s:
        err("gpsNMEA: 0 messages in %ds -> gpsd is NOT publishing." % int(grace_s))
        err("         Is openpilot running? gpsd is started by manager. (ps | grep gpsd)")
        sys.exit(3)
err("gpsNMEA: saw %d msgs but none had a usable time in %ds -> GPS chip has no satellite time lock." % (seen, int(timeout_s)))
err("         Needs a clear sky view / antenna. Indoors on a bench it will never lock.")
sys.exit(1)
PYEOF
}

main() {
  floor="$(effective_floor)"
  now="$($NOW_CMD 2>/dev/null)"

  if is_sane_epoch "$now" "$floor"; then
    log "system time already plausible ($now >= floor $floor) - nothing to do"
    write_lastgood "$now"
    return 0
  fi

  log "system time looks reset ($now < floor $floor) - waiting up to ${TIMEOUT_S}s for a GPS fix..."
  picked="$(poll_gps_epoch "$floor")"
  if ! is_sane_epoch "$picked" "$floor"; then
    log "no usable GPS time within ${TIMEOUT_S}s - leaving clock as-is (NTP will fix it once online)"
    return 1
  fi

  if set_clock "$picked"; then
    write_lastgood "$picked"
    return 0
  fi
  return 1
}

main
