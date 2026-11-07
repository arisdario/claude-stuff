#!/usr/bin/env python3
"""add_sign_detect_display.py - adds a stop-sign distance badge + traffic-light color dot near the top
centre of the EON's own on-screen UI (bbui.h), so it's visible without lightdash/WiFi. Reads the same
/data/sign_detect.json camerad's optional detector writes (see camerad/cameras/SIGN_DETECT.md) - same
tiny fixed-format parse lightdash.cc already uses, duplicated here because bbui.h is a separate
translation unit (compiled into the `ui` binary, not `lightdash`) with no shared code between them.

Read-only / display-only: only reads a file, never touches CAN, never calls anything control-related.
Off unless /data/sign_detect.json exists and is fresh (same staleness rule as lightdash's own reader).

  python3 add_sign_detect_display.py /data/openpilot/selfdrive/ui/bbui.h            # dry run
  python3 add_sign_detect_display.py /data/openpilot/selfdrive/ui/bbui.h --apply    # backup (.bak_signdetect), patch
  python3 add_sign_detect_display.py FILE --restore                                 # put the backup back

After --apply, rebuild/restart the UI the way you normally do for a bbui.h change (same as
swap_cpu_bat_temp_display.py). Idempotent: --apply a second time is a no-op.
"""
import os
import re
import shutil
import sys

INCLUDES_ANCHOR = '#include "cereal/gen/c/ui.capnp.h"'
INCLUDES_ADD = '\r\n#include <sys/stat.h>\r\n#include <time.h>   // both added for bb_ui_draw_sign_detect() below'

FUNC_MARKER_START = '// SIGN_DETECT-BEGIN'
FUNC_MARKER_END = '// SIGN_DETECT-END'

FUNC_BODY = '''%s
// Optional stop-sign distance badge / traffic-light color dot, top-centre of the EON's own screen.
// Reads camerad's detector output (see camerad/cameras/SIGN_DETECT.md); off unless that file exists
// and is fresh. Read-only: no CAN, no control. See tesla/sign_detect_reader.py (same data, the
// Python side that feeds the car's actual cluster) and lightdash.cc (same parse, the web view).
void bb_ui_draw_sign_detect(UIState *s) {
\tstruct stat st;
\tif (stat("/data/sign_detect.json", &st) != 0) return;
\tif (time(NULL) - st.st_mtime > 2) return;   // stale - camerad stopped writing
\tFILE *f = fopen("/data/sign_detect.json", "r");
\tif (!f) return;
\tchar buf[512] = {0};
\tsize_t n = fread(buf, 1, sizeof(buf) - 1, f);
\tfclose(f);
\tif (n == 0) return;
\tchar *p;
\tbool stop = (p = strstr(buf, "\\"stop_sign\\":")) && !strncmp(p + 12, "true", 4);
\tbool light = (p = strstr(buf, "\\"light_present\\":")) && !strncmp(p + 17, "true", 4);
\tif (!stop && !light) return;
\tdouble dist = (p = strstr(buf, "\\"stop_sign_dist_m\\":")) ? atof(p + 20) : -1;
\tint color = (p = strstr(buf, "\\"light_color\\":")) ? atoi(p + 15) : -1;

\tconst int cx = vwp_w / 2, cy = 90;
\tif (stop) {
\t\tchar txt[32];
\t\tif (dist >= 0) snprintf(txt, sizeof(txt), "STOP %%dm", (int)(dist + 0.5));
\t\telse snprintf(txt, sizeof(txt), "STOP");
\t\tnvgBeginPath(s->vg);
\t\tnvgRoundedRect(s->vg, cx - 90, cy - 26, 180, 52, 12);
\t\tnvgFillColor(s->vg, nvgRGBA(190, 20, 20, 220));
\t\tnvgFill(s->vg);
\t\tnvgFontFace(s->vg, "sans-semibold");
\t\tnvgFontSize(s->vg, 46);
\t\tnvgFillColor(s->vg, nvgRGBA(255, 255, 255, 255));
\t\tnvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
\t\tnvgText(s->vg, cx, cy, txt, NULL);
\t}
\tif (light) {
\t\tNVGcolor lc = color == 0 ? nvgRGBA(220, 40, 40, 230) : color == 1 ? nvgRGBA(230, 180, 20, 230)
\t\t             : color == 2 ? nvgRGBA(40, 180, 70, 230) : nvgRGBA(140, 140, 140, 230);
\t\tint lx = cx + (stop ? 120 : 0), ly = cy;
\t\tnvgBeginPath(s->vg);
\t\tnvgCircle(s->vg, lx, ly, 20);
\t\tnvgFillColor(s->vg, lc);
\t\tnvgFill(s->vg);
\t\tnvgStrokeColor(s->vg, nvgRGBA(255, 255, 255, 180));
\t\tnvgStrokeWidth(s->vg, 2);
\t\tnvgStroke(s->vg);
\t}
}
%s
''' % (FUNC_MARKER_START, FUNC_MARKER_END)

CALL_LINE = '    bb_ui_draw_sign_detect(s);\r\n'


def patch(text):
  if FUNC_MARKER_START in text:
    return text, 'already applied - nothing to do'

  if INCLUDES_ANCHOR not in text:
    raise SystemExit('REFUSED: include anchor not found; nothing changed')
  if text.count(INCLUDES_ANCHOR) != 1:
    raise SystemExit('REFUSED: include anchor appears more than once; nothing changed')
  t = text.replace(INCLUDES_ANCHOR, INCLUDES_ANCHOR + INCLUDES_ADD, 1)

  func_anchor = 'void bb_ui_draw_UI( UIState *s) {'
  if func_anchor not in t:
    raise SystemExit('REFUSED: bb_ui_draw_UI anchor not found; nothing changed')
  if t.count(func_anchor) != 1:
    raise SystemExit('REFUSED: bb_ui_draw_UI anchor appears more than once; nothing changed')
  t = t.replace(func_anchor, FUNC_BODY.replace('\n', '\r\n') + func_anchor, 1)

  # call it once per draw, right alongside bb_draw_buttons(s); in the tri_state_switch==1 and ==3
  # branches (matches where other info overlays are drawn; ==2 is the deliberately minimal layout).
  call_anchor = '    bb_draw_buttons(s);\r\n'
  n_calls = t.count(call_anchor)
  if n_calls < 2:
    raise SystemExit('REFUSED: expected at least 2 bb_draw_buttons(s); call sites, found %d; nothing changed' % n_calls)
  t = t.replace(call_anchor, call_anchor + CALL_LINE)

  return t, 'applied (new function + %d call site%s)' % (n_calls, '' if n_calls == 1 else 's')


def main(argv):
  if len(argv) < 1:
    print(__doc__); return 2
  path = argv[0]
  with open(path, newline='') as f:
    text = f.read()
  if len(argv) > 1 and argv[1] == '--restore':
    bak = path + '.bak_signdetect'
    if not os.path.exists(bak):
      raise SystemExit('REFUSED: no backup at %s' % bak)
    shutil.copy(bak, path)
    print('restored %s from %s' % (path, bak))
    return 0
  new_text, msg = patch(text)
  apply_ = len(argv) > 1 and argv[1] == '--apply'
  print(('APPLY: ' if apply_ else 'DRY RUN: ') + msg)
  if not apply_:
    print('rerun with --apply to write it (backup: %s.bak_signdetect)' % path)
    return 0
  if new_text == text:
    return 0
  bak = path + '.bak_signdetect'
  if not os.path.exists(bak):
    shutil.copy(path, bak)
    print('backed up original to %s' % bak)
  with open(path, 'w', newline='') as f:
    f.write(new_text)
  print('wrote %s - rebuild/restart the UI to see it' % path)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
