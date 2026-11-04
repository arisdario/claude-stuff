#!/usr/bin/env python3
"""add_web_button.py - turn the EMPTY slot 3 of the on-screen button row ("WEB") into a radarweb on/off button. No C++ / UI rebuild.

The driving screen already draws up to six buttons defined in carstate.py (btns_init). Slot 3 is empty (it was the old 'visiond' switch). This patch:
  1. defines slot 3 as  ["web", "WEB", ["OFF","ON"]]   -> a round button labelled WEB showing OFF / ON, toggled by touch like ALC / ACC / DSP
  2. replaces the old slot-3 handler in CarState.update_ui_buttons (it ran ch_visiond.sh) with: write '1'/'0' to /data/web_switch
param_switch.py (small watcher) then starts/stops radarweb from that file. carstate.py itself starts nothing and spawns no process.

  python3 add_web_button.py /data/openpilot/selfdrive/car/tesla/carstate.py            # dry run: shows what it would change
  python3 add_web_button.py /data/openpilot/selfdrive/car/tesla/carstate.py --apply    # backup (.bak_webbtn), patch, py_compile; restores itself if compiling fails
  python3 add_web_button.py FILE --restore                                             # put the backup back
Idempotent. Refuses (changes nothing) if the file does not contain exactly the expected text.
"""
import os
import py_compile
import re
import shutil
import sys

SLOT_RE = re.compile(r'(\["dsp",\s*"DSP",\s*\["OP","MIN","OFF","GYRO"\]\],\s*\n\s*)\[""\s*,\s*""\s*,\s*\[""\]\]')
SLOT_NEW = r'\1["web",               "WEB",                      ["OFF","ON"]]'
HANDLER_OLD_RE = re.compile(r'(  def update_ui_buttons\(self,btn_id,btn_status\):\n)')
HANDLER_NEW = r'''\1    # WEB button (slot 3): radarweb on/off. Only writes a file; tesla/tools/param_switch.py starts/stops radarweb from it.
    if btn_id == 3:
      try:
        on = self.cstm_btns.btns[3].btn_label2 == 'ON'
        with open('/data/web_switch.tmp', 'w') as f_:
          f_.write('1\\n' if on else '0\\n')
        os.replace('/data/web_switch.tmp', '/data/web_switch')
      except Exception:
        pass
      return
'''


def patch(text):
  if '"web"' in text and 'web_switch' in text:
    return text, 'already patched'
  if len(SLOT_RE.findall(text)) != 1:
    raise SystemExit('REFUSED: the btns_init block does not look as expected (slot 3 after "dsp"); nothing changed')
  if len(HANDLER_OLD_RE.findall(text)) != 1:
    raise SystemExit('REFUSED: def update_ui_buttons(self,btn_id,btn_status) not found exactly once; nothing changed')
  if not re.search(r'^import os|^import .*\bos\b|^from os', text, re.M):
    raise SystemExit('REFUSED: carstate.py does not import os; nothing changed')
  t = SLOT_RE.sub(SLOT_NEW, text, count=1)
  t = HANDLER_OLD_RE.sub(HANDLER_NEW, t, count=1)
  return t, 'patched'


def main(a):
  if len(a) < 1:
    print(__doc__); return 2
  path = a[0]
  bak = path + '.bak_webbtn'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak); return 0
  text = open(path).read()
  new, msg = patch(text)
  if msg == 'already patched':
    print(msg); return 0
  if '--apply' not in a:
    import difflib
    print(''.join(difflib.unified_diff(text.splitlines(True), new.splitlines(True), 'before', 'after', n=1)))
    print('dry run only; add --apply to write it'); return 0
  shutil.copy2(path, bak)
  open(path, 'w').write(new)
  try:
    py_compile.compile(path, doraise=True)
  except py_compile.PyCompileError as e:
    shutil.copy2(bak, path)
    print('compile failed, original restored:', e); return 1
  print('patched; backup at', bak, '- reboot to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
