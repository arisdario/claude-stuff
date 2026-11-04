import os, sys, shutil, tempfile, py_compile, textwrap
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import patch_controlsd_sound as p

SRC = textwrap.dedent('''
    import os
    def f():
      while True:
        sounds_available = not os.path.isfile('/EON') or (os.path.isdir('/proc/asound/card0') and open('/proc/asound/card0/state').read().strip() == 'ONLINE')
        return sounds_available
''')


def test_patch_restore_and_semantics():
  d = tempfile.mkdtemp(); f = os.path.join(d, 'controlsd.py'); open(f, 'w').write(SRC)
  assert p.main([f, '--apply']) == 0
  new = open(f).read()
  assert "for c_ in os.listdir('/proc/asound')" in new and new.count('sounds_available =') == 1
  py_compile.compile(f, doraise=True)
  assert p.main([f, '--apply']) == 0 and open(f).read() == new                      # idempotent
  # semantics on a fake /proc/asound: card0 = USB audio without state, card1 = EON card ONLINE -> True; none online -> False
  ns = {}
  code = new.replace('/proc/asound', os.path.join(d, 'asound')).replace("os.path.isfile('/EON')", "True")
  for case, expect in ((('card0', None, 'card1', 'ONLINE'), True), (('card0', None, 'card1', 'OFFLINE'), False)):
    root = os.path.join(d, 'asound'); shutil.rmtree(root, ignore_errors=True)
    for name, st in ((case[0], case[1]), (case[2], case[3])):
      os.makedirs(os.path.join(root, name))
      if st: open(os.path.join(root, name, 'state'), 'w').write(st + '\n')
    ns = {}; exec(code.replace('not True or', 'False or'), ns)
    assert ns['f']() is expect
  assert p.main([f, '--restore']) == 0 and open(f).read() == SRC
  q = os.path.join(d, 'other.py'); open(q, 'w').write('x = 1\n')
  try:
    p.main([q, '--apply']); assert False
  except SystemExit as e:
    assert 'REFUSED' in str(e) and open(q).read() == 'x = 1\n'
