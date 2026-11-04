#!/usr/bin/env sh
# eon_snapshot.sh - read-only health snapshot of the EON for remote troubleshooting. One text file, no secrets:
#   * no dongle id / tokens / keys / SSH keys, no GPS positions, no full config files (only the enable_* switches), no logs of drives
#   * it changes nothing and starts nothing
# Usage:  sh eon_snapshot.sh            -> /data/eon_snapshot_YYYYmmdd_HHMMSS.txt  (and prints the path)
OUT=/data/eon_snapshot_$(date +%Y%m%d_%H%M%S).txt
S() { echo; echo "=== $1"; }
{
echo "eon_snapshot $(date)  uptime: $(cat /proc/uptime | cut -d' ' -f1) s"
S "kernel / python"; uname -a; python3 --version 2>&1
S "repo (tesla fork clone)"
for R in /data/claude-stuff /data/openpilot; do
  [ -d "$R/.git" ] && { echo "$R: $(git -C $R log -1 --format='%h %cd %s' 2>&1 | cut -c1-120)  branch: $(git -C $R rev-parse --abbrev-ref HEAD 2>&1)"; git -C $R status --short 2>/dev/null | head -8; }
done
S "openpilot processes"
for p in manager controlsd plannerd radard modeld camerad boardd loggerd uploader deleter paramsd locationd calibrationd sensord thermald ui mapcurved radarweb collector; do
  n=$(for d in /proc/[0-9]*; do tr '\0' ' ' < $d/cmdline 2>/dev/null | grep -q "$p" && echo x; done | wc -l); echo "$p: $n"
done
S "CPU load / memory"; head -4 /proc/loadavg; grep -E "MemTotal|MemAvailable|MemFree" /proc/meminfo
S "temperatures (C)"
for z in /sys/class/thermal/thermal_zone*; do t=$(cat $z/type 2>/dev/null); v=$(cat $z/temp 2>/dev/null); echo "$t $v"; done | head -30
S "CPU freq (kHz)"; for c in /sys/devices/system/cpu/cpu[0-9]*; do echo "$(basename $c) $(cat $c/cpufreq/scaling_cur_freq 2>/dev/null) online=$(cat $c/online 2>/dev/null)"; done
S "learned parameters"; cat /data/params/d/LiveParameters 2>&1 | sed 's/"carVin": "[^"]*"/"carVin": "..."/'; echo; cat /data/params/d/CalibrationParams 2>&1
S "camera"
echo "source: $(cat /data/camera_source 2>&1) focus: $(cat /data/camera_focus 2>&1) flip: $(cat /data/camera_flip 2>&1) led_auto_file: $([ -f /data/camera_led_auto ] && echo yes || echo no)"
echo "profile mode: $(cat /data/camera_profile 2>&1)"; echo "profile conf:"; cat /data/camera_profile.conf 2>&1 | head -40
echo "camera status:"; cat /data/camera_status.json 2>&1
echo "last profile log:"; tail -8 /data/camera_profile.log 2>&1
ls -l --time-style=full-iso /dev/c920_live.jpg 2>&1; date
for d in /sys/bus/usb/devices/*; do [ -f $d/idVendor ] && echo "usb $(basename $d) $(cat $d/idVendor):$(cat $d/idProduct) speed=$(cat $d/speed) $(cat $d/product 2>/dev/null)"; done
echo "camerad binary: $(ls -l --time-style=full-iso /data/openpilot/selfdrive/camerad/camerad 2>&1 | cut -c1-120)"
echo "camerad log (external camera lines):"; grep -a "external camera\|ROAD CAMERA" /tmp/camerad.log 2>/dev/null | tail -15
S "block files / switches"; ls /data/no_* 2>&1 | head; grep -E "^(enable_|use_)[a-z_]* *=" /data/bb_openpilot.cfg 2>/dev/null | head -30
S "diagnostics logger"
sh /data/claude-stuff/tesla/diag/start_diagnostics.sh status 2>&1 | cut -c1-300
ls -d /data/steering_diagnostics/drive_* 2>/dev/null | tail -4; df -h /data 2>/dev/null | tail -1
S "last error lines of openpilot logs"
for f in /tmp/controlsd.log /tmp/modeld.log /tmp/plannerd.log /tmp/boardd.log; do echo "-- $f"; grep -a -i -E "error|exception|traceback|fail" $f 2>/dev/null | tail -4 | cut -c1-200; done
S "radarweb"; grep -a -i -E "error|traceback" /data/radarweb.log 2>/dev/null | tail -4 | cut -c1-200
} > "$OUT" 2>&1
echo "$OUT   ($(wc -c < "$OUT") bytes)"
