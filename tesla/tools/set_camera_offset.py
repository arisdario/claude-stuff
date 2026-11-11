#!/usr/bin/env python3
"""set_camera_offset.py - patches CAMERA_OFFSET in selfdrive/controls/lib/lane_planner.py (core
openpilot, not in this repo) to match your car's actual camera mounting instead of comma's reference
phone-mount assumption.

WHAT THIS CHANGES AND WHY IT MATTERS: lane_planner.py applies this constant to both lPoly[3] and
rPoly[3] (the lane-line lateral-position terms) before publishing pathPlan - the SAME pathPlan that
feeds controlsd's lateral path-following AND this fork's own ic_raw_lanes.py/dashboard. Working out
the geometry: car_frame_y = raw_y + CAMERA_OFFSET, so CAMERA_OFFSET is "how many meters left of the
car's true centerline the camera is assumed to be mounted." Comma's default 0.06 assumes their
reference phone mount sits 6cm left of center; it does NOT auto-detect your actual camera position -
nothing in calibrationd.py touches it (that only self-calibrates pitch/yaw via the vanishing point).

This IS a vehicle-control-relevant change (not just a display number) - it shifts where pathPlan
reports lane center to be, which controlsd's lateral control follows. Test it (e.g. capture
pathPlan.lPoly/rPoly via driving_logger.py before/after) before trusting it on the road.

  python3 set_camera_offset.py FILE 0.0                 # dry run: show what would change
  python3 set_camera_offset.py FILE 0.0 --apply          # backup (.bak_camoffset), patch
  python3 set_camera_offset.py FILE --restore            # put the backup back

Idempotent: re-running --apply with the same value is a no-op (still refreshes the backup check).
Refuses if the CAMERA_OFFSET anchor line isn't found, or appears more than once.
"""
import os
import re
import shutil
import sys

ANCHOR_RE = re.compile(r'^(CAMERA_OFFSET\s*=\s*)([-+]?[0-9]*\.?[0-9]+)(\s*(?:#.*)?)$', re.MULTILINE)


def patch(text, new_value):
  matches = list(ANCHOR_RE.finditer(text))
  if not matches:
    raise SystemExit('REFUSED: CAMERA_OFFSET anchor not found; nothing changed')
  if len(matches) != 1:
    raise SystemExit('REFUSED: CAMERA_OFFSET anchor appears more than once (%d); nothing changed' % len(matches))

  m = matches[0]
  old_value = float(m.group(2))
  new_value_str = ('%g' % new_value)
  if '.' not in new_value_str and 'e' not in new_value_str:
    new_value_str += '.0'

  if old_value == new_value:
    return text, 'already %s - nothing to do' % new_value_str

  new_line = m.group(1) + new_value_str + m.group(3)
  t = text[:m.start()] + new_line + text[m.end():]
  return t, 'changed CAMERA_OFFSET from %g to %s' % (old_value, new_value_str)


def main(argv):
  if len(argv) < 1:
    print(__doc__)
    return 2
  path = argv[0]
  with open(path) as f:
    text = f.read()

  if len(argv) > 1 and argv[1] == '--restore':
    bak = path + '.bak_camoffset'
    if not os.path.exists(bak):
      raise SystemExit('REFUSED: no backup at %s' % bak)
    shutil.copy(bak, path)
    print('restored %s from %s' % (path, bak))
    return 0

  if len(argv) < 2:
    print(__doc__)
    return 2
  try:
    new_value = float(argv[1])
  except ValueError:
    raise SystemExit('REFUSED: %r is not a number' % argv[1])

  new_text, msg = patch(text, new_value)
  apply_ = len(argv) > 2 and argv[2] == '--apply'
  print(('APPLY: ' if apply_ else 'DRY RUN: ') + msg)
  if not apply_:
    print('rerun with --apply to write it (backup: %s.bak_camoffset)' % path)
    return 0
  if new_text == text:
    return 0
  bak = path + '.bak_camoffset'
  if not os.path.exists(bak):
    shutil.copy(path, bak)
    print('backed up original to %s' % bak)
  with open(path, 'w') as f:
    f.write(new_text)
  print('wrote %s - restart controlsd/the stack to pick it up' % path)
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
