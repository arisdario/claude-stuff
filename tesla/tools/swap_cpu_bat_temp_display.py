#!/usr/bin/env python3
"""swap_cpu_bat_temp_display.py - swap the on-screen ORDER of the CPU TEMP / BAT TEMP rows in the UI's left
measures panel (bbui.h, bb_ui_draw_measures_left / _left2), so CPU TEMP is the one drawn in the position you
actually look at (currently BAT TEMP is first/top, CPU TEMP second/bottom; this swaps which one is first).

Pure reordering - no computation, scaling, threshold or label text is touched, so there is no way to
introduce a math/formatting bug: the two complete blocks (each "if (true) { ... }") are moved as-is,
just in the opposite order. Does not touch maxCpuTemp/maxBatTemp sourcing (ui.cc) or anything else.

Display-only: this is the Qt/NVG-less C++ UI drawing code (selfdrive/ui on this 0.7.5-era build). It
does not touch carcontroller/carstate/safety code and cannot change how the car drives.

  python3 swap_cpu_bat_temp_display.py /data/openpilot/selfdrive/ui/bbui.h            # dry run
  python3 swap_cpu_bat_temp_display.py /data/openpilot/selfdrive/ui/bbui.h --apply    # backup (.bak_tempswap), patch
  python3 swap_cpu_bat_temp_display.py FILE --restore                                 # put the backup back

After --apply, rebuild and restart the UI the way you normally do for a bbui.h change (this repo doesn't
script that - it's specific to your build setup), then check the dashboard. Idempotent: running --apply
twice is a no-op the second time (it detects the already-swapped order and does nothing).
Refuses (changes nothing) if the file doesn't contain the expected CPU-then-BAT block pair, since that
means the file doesn't match what this script expects to find (or has already been swapped the other way).
"""
import os
import re
import shutil
import sys

# Captured by structure, not hand-typed: "//add CPU temperature" (or "...battery...") up to the first
# closing "\t}" line that follows it. Robust to the file's actual whitespace instead of guessing it.
# \r?\n (not a bare \n) because this file uses CRLF line endings - matching only \n would both fail to
# match and, if it somehow did, risk the file's line endings - this script reads/writes with newline=''
# (see main()) specifically so no line ending anywhere in the file is touched, swapped region or not.
CPU_RE = r'[ \t]*//add CPU temperature\b.*?\r?\n\t\}\r?\n'
BAT_RE = r'[ \t]*//add battery temperature\b.*?\r?\n\t\}\r?\n'
PAIR_CPU_THEN_BAT = re.compile(r'(' + CPU_RE + r')(\r?\n)(' + BAT_RE + r')', re.S)
PAIR_BAT_THEN_CPU = re.compile(r'(' + BAT_RE + r')(\r?\n)(' + CPU_RE + r')', re.S)


def patch(text):
  n_old = len(PAIR_CPU_THEN_BAT.findall(text))
  n_new = len(PAIR_BAT_THEN_CPU.findall(text))
  if n_old == 0 and n_new > 0:
    return text, 'already swapped (%d place%s) - nothing to do' % (n_new, '' if n_new == 1 else 's')
  if n_old == 0:
    raise SystemExit('REFUSED: the expected CPU-then-BAT block pair was not found (0 matches); '
                      'this file may differ from what this script expects - nothing changed')
  t = PAIR_CPU_THEN_BAT.sub(lambda m: m.group(3) + m.group(2) + m.group(1), text)
  return t, 'swapped %d place%s (now BAT TEMP first, CPU TEMP second)' % (n_old, '' if n_old == 1 else 's')


def main(argv):
  if len(argv) < 1:
    print(__doc__); return 2
  path = argv[0]
  # newline='': disables universal-newline translation on both read and write, so this file's actual
  # line endings (CRLF) are preserved exactly, everywhere except the bytes this script deliberately moves.
  with open(path, newline='') as f:
    text = f.read()
  if len(argv) > 1 and argv[1] == '--restore':
    bak = path + '.bak_tempswap'
    if not os.path.exists(bak):
      raise SystemExit('REFUSED: no backup at %s' % bak)
    shutil.copy(bak, path)
    print('restored %s from %s' % (path, bak))
    return 0
  new_text, msg = patch(text)
  apply_ = len(argv) > 1 and argv[1] == '--apply'
  print(('APPLY: ' if apply_ else 'DRY RUN: ') + msg)
  if not apply_:
    print('rerun with --apply to write it (backup: %s.bak_tempswap)' % path)
    return 0
  if new_text == text:
    return 0  # already swapped; nothing to back up or write
  bak = path + '.bak_tempswap'
  if not os.path.exists(bak):
    shutil.copy(path, bak)
    print('backed up original to %s' % bak)
  with open(path, 'w', newline='') as f:
    f.write(new_text)
  print('wrote %s - rebuild/restart the UI to see it' % path)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
