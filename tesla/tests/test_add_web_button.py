import os, sys, shutil, tempfile, py_compile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import add_web_button as w

SRC = os.path.join(os.path.dirname(__file__), '..', 'carstate.py')


def test_patch_apply_idempotent_restore_and_refusal():
  d = tempfile.mkdtemp(); p = os.path.join(d, 'carstate.py'); shutil.copy(SRC, p)
  orig = open(p).read()
  assert w.main([p, '--apply']) == 0
  new = open(p).read()
  assert '["web",' in new and "/data/web_switch" in new and new.count('btn_id == 3:') >= 2
  py_compile.compile(p, doraise=True)
  assert w.main([p, '--apply']) == 0 and open(p).read() == new            # idempotent
  assert w.main([p, '--restore']) == 0 and open(p).read() == orig
  # a different file layout is refused and left untouched
  q = os.path.join(d, 'other.py'); open(q, 'w').write('x = 1\n')
  try:
    w.main([q, '--apply']); assert False
  except SystemExit as e:
    assert 'REFUSED' in str(e) and open(q).read() == 'x = 1\n'
