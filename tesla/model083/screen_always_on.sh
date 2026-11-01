#!/bin/sh
# BENCH: keep the EON screen on while /data/screen_always_on exists (Tinkla's UI turns it off 3 s after the last touch
# with the display button on OFF, 30 s on OP). For lining the camera up with a monitor.
#   sh screen_always_on.sh install     patch selfdrive/ui/ui.cc (backup ui.cc.075), rebuild the UI, restart it, flag on
#   sh screen_always_on.sh on | off    create / remove the flag (no rebuild; takes effect within ~2 s)
#   sh screen_always_on.sh uninstall   restore ui.cc.075, rebuild, restart the UI, remove the flag
# With the flag removed the UI behaves exactly as before (the added code only checks for the file).
set -e
OP=${OP:-/data/openpilot}
UI=$OP/selfdrive/ui/ui.cc
FLAG=/data/screen_always_on

restart_ui() { P=$(pidof _ui || true); [ -n "$P" ] && kill $P && echo "UI restarted (openpilot's manager starts it again)"; true; }

case "$1" in
  install)
    [ -f "$UI" ] || { echo "STOP: $UI not found"; exit 1; }
    [ -f "$UI.075" ] || cp "$UI" "$UI.075"
    python3 - "$UI" <<'PYEOF'
import sys
p = sys.argv[1]; s = open(p).read()
if 'BENCH_SCREEN_ON' in s:
  print('ui.cc already patched'); sys.exit(0)
anchor = '    // manage wakefulness\n'
if s.count(anchor) != 1:
  sys.exit('STOP: ui.cc is not the expected 0.7.5.a version (anchor not found once) - not patching')
block = '''    // BENCH_SCREEN_ON: while /data/screen_always_on exists the screen stays on (checked every ~2 s)
    {
      static int aon_check = 0;
      static bool aon = false;
      if (--aon_check <= 0) { aon_check = 60; aon = access("/data/screen_always_on", F_OK) == 0; }
      if (aon) { set_awake(s, true); s->awake_timeout = 30*30; }
    }
'''
open(p, 'w').write(s.replace(anchor, block + anchor))
print('patched ui.cc')
PYEOF
    cd "$OP" && scons -j4 selfdrive/ui/_ui || { echo "STOP: UI build failed - run: sh $0 uninstall"; exit 1; }
    touch "$FLAG"; restart_ui; echo "screen stays on. 'sh $0 off' to let it sleep again." ;;
  on)  touch "$FLAG"; echo "screen stays on (needs 'install' once)" ;;
  off) rm -f "$FLAG"; echo "normal screen timeout" ;;
  uninstall)
    [ -f "$UI.075" ] && cp "$UI.075" "$UI" && echo "restored ui.cc"
    rm -f "$FLAG"; cd "$OP" && scons -j4 selfdrive/ui/_ui && restart_ui ;;
  *) echo "usage: sh $0 install | on | off | uninstall"; exit 1 ;;
esac
