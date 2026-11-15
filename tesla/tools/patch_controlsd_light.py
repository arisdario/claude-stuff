#!/usr/bin/env python3
"""patch_controlsd_light.py - make controlsd's obstacle-warning read cheaper (found by the CPU profile, 15 Nov 2026).

get_obstacle_level() read radard's ObstacleWarning parameter 10x a second through a NEW Params() object each time
(1.5 % of controlsd's CPU). Now: 5x a second (an advisory warning: at most 0.2 s later) through one reused Params().

  python3 patch_controlsd_light.py /data/openpilot/selfdrive/controls/controlsd.py            # dry run
  python3 patch_controlsd_light.py FILE --apply | --check | --restore                          # backup FILE.bak_light
Loaded at the next REBOOT (openpilot pre-loads controlsd when the EON starts).
"""
import difflib
import py_compile
import shutil
import sys

MARK = '# CPULIGHT (tesla/tools/patch_controlsd_light.py)'
ANCHORS = [
  ("_obs_cache = [0, 0]  # last level, frame of the last read\n",
   "_obs_cache = [0, 0]  # last level, frame of the last read\n"
   "_obs_params = []     " + MARK + ": one Params() reused instead of a new one per read\n"),
  ("  if frame - _obs_cache[1] >= 10 or frame < _obs_cache[1]:\n",
   "  if frame - _obs_cache[1] >= 20 or frame < _obs_cache[1]:   # CPULIGHT: 5x a second (was 10)\n"),
  ("      raw = Params().get(\"ObstacleWarning\", encoding='utf8')\n",
   "      if not _obs_params:\n"
   "        _obs_params.append(Params())\n"
   "      raw = _obs_params[0].get(\"ObstacleWarning\", encoding='utf8')\n"),
]


def patch(text):
  if MARK in text:
    return text, 'already patched'
  for old, _ in ANCHORS:
    if text.count(old) != 1:
      raise SystemExit('REFUSED: %r not found exactly once - not the expected controlsd.py; nothing changed' % old.strip()[:80])
  for old, new in ANCHORS:
    text = text.replace(old, new, 1)
  return text, 'patched'


def main(a):
  if not a:
    print(__doc__); return 2
  path, bak = a[0], a[0] + '.bak_light'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak, '- reboot to load it'); return 0
  text = open(path).read()
  if '--check' in a:
    print('patched' if MARK in text else 'not patched'); return 0
  new, msg = patch(text)
  if msg == 'already patched':
    print(msg); return 0
  if '--apply' not in a:
    print(''.join(difflib.unified_diff(text.splitlines(True), new.splitlines(True), 'before', 'after', n=1)))
    print('dry run only; add --apply'); return 0
  shutil.copy2(path, bak)
  open(path, 'w').write(new)
  try:
    py_compile.compile(path, doraise=True)
  except py_compile.PyCompileError as e:
    shutil.copy2(bak, path); print('compile failed, original restored:', e); return 1
  print('patched; backup at', bak, '- reboot to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
