#!/usr/bin/env python3
"""patch_controlsd_sound.py - make controlsd find the EON sound card whatever its ALSA number is.

Problem: the Logitech C920 is also a USB microphone. If it enumerates before the EON's own sound card (msm8996-tasha), ALSA gives the C920 card0 and the EON
card becomes card1. controlsd (tinkla 0.7.5.a) only looks at /proc/asound/card0/state, which a USB audio card does not have -> FileNotFoundError -> controlsd dies
-> no engagement. Which card is card0 depends on boot timing, so it works on some boots and not on others.

The patch changes ONE line (same indentation) so that ANY card whose state file says ONLINE counts as an available sound card. The safety intent is unchanged: with no
online sound card, 'soundsUnavailable' still blocks engagement.

  python3 patch_controlsd_sound.py /data/openpilot/selfdrive/controls/controlsd.py            # dry run
  python3 patch_controlsd_sound.py /data/openpilot/selfdrive/controls/controlsd.py --apply    # backup .bak_sound, patch, py_compile (restores itself on failure)
  python3 patch_controlsd_sound.py FILE --restore
"""
import difflib
import py_compile
import re
import shutil
import sys

OLD = re.compile(r"^(?P<ind>[ \t]*)sounds_available = not os\.path\.isfile\('/EON'\) or \(os\.path\.isdir\('/proc/asound/card0'\) and open\('/proc/asound/card0/state'\)\.read\(\)\.strip\(\) == 'ONLINE'\)[ \t]*$", re.M)
NEW = (r"\g<ind>sounds_available = not os.path.isfile('/EON') or any(os.path.isfile('/proc/asound/%s/state' % c_) and "
       r"open('/proc/asound/%s/state' % c_).read().strip() == 'ONLINE' for c_ in os.listdir('/proc/asound') if c_.startswith('card'))")


def patch(text):
  if "for c_ in os.listdir('/proc/asound')" in text:
    return text, 'already patched'
  if len(OLD.findall(text)) != 1:
    raise SystemExit('REFUSED: the sounds_available line was not found exactly once in the expected form; nothing changed')
  return OLD.sub(NEW, text, count=1), 'patched'


def main(a):
  if not a:
    print(__doc__); return 2
  path, bak = a[0], a[0] + '.bak_sound'
  if '--restore' in a:
    shutil.copy2(bak, path); print('restored from', bak); return 0
  text = open(path).read()
  new, msg = patch(text)
  if msg == 'already patched':
    print(msg); return 0
  if '--apply' not in a:
    print(''.join(difflib.unified_diff(text.splitlines(True), new.splitlines(True), 'before', 'after', n=0)))
    print('dry run only; add --apply'); return 0
  shutil.copy2(path, bak)
  open(path, 'w').write(new)
  try:
    py_compile.compile(path, doraise=True)
  except py_compile.PyCompileError as e:
    shutil.copy2(bak, path); print('compile failed, original restored:', e); return 1
  print('patched; backup at', bak, '- restart openpilot (reboot) to load it'); return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
