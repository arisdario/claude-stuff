#!/usr/bin/env python3
"""bus2_listen_only.py - make panda CAN bus 2 RECEIVE-ONLY, so it can be tapped onto another car bus (e.g. powertrain CAN)
without the panda ever putting a bit on it. Two independent layers, both in board/drivers/can.h:

  1. hardware: bus 2's CAN controller is always initialised in SILENT mode (bxCAN SILM). In silent mode the chip
     cannot drive the bus at all - no frames, no ACK bits, no error frames - but still receives everything.
  2. software: can_send() drops every frame addressed to bus 2 before it is queued: openpilot's own sends over USB,
     the safety code's sends, and frames forwarded from another bus (tesla_fwd_hook can forward 0 -> 2).

  python3 bus2_listen_only.py                  dry run: shows what would change in /data/openpilot/panda/board/drivers/can.h
  python3 bus2_listen_only.py --apply          backup (can.h.bak_bus2, first time only), patch
  python3 bus2_listen_only.py --restore        put the backup back
  python3 bus2_listen_only.py --check          exit 0 if patched, 1 if not
  FILE=/path/to/can.h python3 bus2_listen_only.py ...   another panda checkout

After --apply the firmware must be REBUILT and FLASHED (see the chat / README). Refuses to touch the file if the two
places it edits do not look exactly as expected (a different panda version): nothing is changed in that case.
"""
import os
import re
import shutil
import sys

FILE = os.environ.get('FILE', '/data/openpilot/panda/board/drivers/can.h')
MARK = 'BUS2_LISTEN_ONLY'

DEFINE_ANCHOR = '#define ALL_CAN_LIVE 0\n'
DEFINE_ADD = ('\n// ' + MARK + ' (tesla/panda/bus2_listen_only.py): bus 2 is receive-only - silent controller + no sends.\n'
              '#define LISTEN_ONLY_BUS 2U\n')

SILENT_RE = re.compile(r'(llcan_set_speed\(CAN, can_speed\[bus_number\], can_loopback, )\(unsigned int\)\(can_silent\) & \(1U << can_number\)\)')
SILENT_NEW = r'\1((unsigned int)(can_silent) & (1U << can_number)) | ((bus_number == LISTEN_ONLY_BUS) ? 1U : 0U))'

SEND_RE = re.compile(r'(void can_send\(CAN_FIFOMailBox_TypeDef \*to_push, uint8_t bus_number, bool skip_tx_hook\) \{\n)')
SEND_ADD = r'\1  if (bus_number == LISTEN_ONLY_BUS) {  // ' + MARK + ': never queue anything for bus 2\n    return;\n  }\n'


def patch(text):
  """returns (new_text, message). Raises SystemExit('REFUSED ...') if the code is not as expected."""
  if MARK in text:
    return text, 'already applied - nothing to do'
  if text.count(DEFINE_ANCHOR) != 1:
    raise SystemExit('REFUSED: "#define ALL_CAN_LIVE 0" not found exactly once; nothing changed')
  if len(SILENT_RE.findall(text)) != 1:
    raise SystemExit('REFUSED: the llcan_set_speed(... can_silent ...) call was not found exactly once; nothing changed')
  if len(SEND_RE.findall(text)) != 1:
    raise SystemExit('REFUSED: the can_send() definition was not found exactly once; nothing changed')
  t = text.replace(DEFINE_ANCHOR, DEFINE_ANCHOR + DEFINE_ADD, 1)
  t = SILENT_RE.sub(SILENT_NEW, t, count=1)
  t = SEND_RE.sub(SEND_ADD, t, count=1)
  return t, 'applied: bus 2 silent controller + can_send() drops bus 2'


def main(argv):
  bak = FILE + '.bak_bus2'
  if '--restore' in argv:
    if not os.path.exists(bak):
      print('no backup at %s' % bak); return 1
    shutil.copyfile(bak, FILE); print('restored %s from %s - rebuild and flash the firmware' % (FILE, bak)); return 0
  text = open(FILE).read()
  if '--check' in argv:
    ok = MARK in text
    print('patched' if ok else 'NOT patched'); return 0 if ok else 1
  new, msg = patch(text)
  if '--apply' not in argv:
    print('DRY RUN: %s' % msg)
    if new != text:
      import difflib
      sys.stdout.writelines(difflib.unified_diff(text.splitlines(True), new.splitlines(True), FILE, FILE + ' (patched)'))
    print('\nnothing written. To apply: python3 %s --apply' % sys.argv[0]); return 0
  if new == text:
    print(msg); return 0
  if not os.path.exists(bak):
    shutil.copyfile(FILE, bak)
  open(FILE, 'w').write(new)
  print('%s\nwrote %s (backup %s). Now rebuild + flash the panda firmware.' % (msg, FILE, bak))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
