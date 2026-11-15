#!/usr/bin/env python3
"""patch_controlsd_profile.py - a switchable CPU profiler hook in controlsd (tools/pyprof.py). Does NOTHING unless the
file /data/profile_controlsd exists when controlsd starts; then it samples controlsd for 60 s (about 1 % overhead),
writes /data/controlsd_profile.txt and renames the trigger to /data/profile_controlsd.done.

  python3 patch_controlsd_profile.py /data/openpilot/selfdrive/controls/controlsd.py            # dry run
  python3 patch_controlsd_profile.py FILE --apply | --check | --restore                          # backup FILE.bak_profile
Use:  touch /data/profile_controlsd ; turn the car off and on ; drive ~1 min (ACC on) ; cat /data/controlsd_profile.txt
"""
import difflib
import py_compile
import shutil
import sys

MARK = '# PROFILE (tesla/tools/patch_controlsd_profile.py)'
OLD = "def main(sm=None, pm=None, logcan=None):\n  controlsd_thread(sm, pm, logcan)\n"
NEW = ("def main(sm=None, pm=None, logcan=None):\n"
       "  " + MARK + ": only when /data/profile_controlsd exists - 60 s sampling, report in /data/controlsd_profile.txt\n"
       "  if os.path.exists('/data/profile_controlsd'):\n"
       "    try:\n"
       "      import importlib.util as _pu\n"
       "      _ps = _pu.spec_from_file_location('pyprof', '/data/claude-stuff/tesla/tools/pyprof.py')\n"
       "      _pm = _pu.module_from_spec(_ps); _ps.loader.exec_module(_pm)\n"
       "      _pm.start('/data/controlsd_profile.txt', 60., trigger='/data/profile_controlsd')\n"
       "    except Exception:\n"
       "      pass\n"
       "  controlsd_thread(sm, pm, logcan)\n")


def patch(text):
  if MARK in text:
    return text, 'already patched'
  if text.count(OLD) != 1 or '\nimport os\n' not in text:
    raise SystemExit('REFUSED: controlsd main() not found exactly as expected; nothing changed')
  return text.replace(OLD, NEW, 1), 'patched'


def main(a):
  if not a:
    print(__doc__); return 2
  path, bak = a[0], a[0] + '.bak_profile'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak, '- turn the car off and on to load it'); return 0
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
  print('patched; backup at', bak, '- turn the car off and on to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
