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

# --- the one non-bash step: wait up to TIMEOUT_S for a usable gpsNMEA timestamp, print it, exit ---
# Prints nothing (and exits nonzero) if no sane fix arrives in time. Validation happens twice - once
# here (so a clearly-bad reading doesn't even get printed) and again in is_sane_epoch after we read
# it back, since env-var floor/ceiling overrides in tests must be honored on the bash side too.
poll_gps_epoch() {
  floor="$1"
  "$PY_CMD" - "$floor" "$TIMEOUT_S" "$MAX_SKEW_YEARS" <<'PYEOF'
import sys, time
floor = int(sys.argv[1])
timeout_s = float(sys.argv[2])
max_skew_years = float(sys.argv[3])
ceiling = floor + max_skew_years * 365 * 86400

try:
    import cereal.messaging as messaging
except ImportError:
    sys.exit(1)

sm = messaging.SubMaster(['gpsNMEA'])
deadline = time.monotonic() + timeout_s
while time.monotonic() < deadline:
    sm.update(1000)
    if sm.updated['gpsNMEA']:
        ts_ms = sm['gpsNMEA'].timestamp
        epoch_s = ts_ms // 1000
        if floor <= epoch_s <= ceiling:
            print(int(epoch_s))
            sys.exit(0)
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
