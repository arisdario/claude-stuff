import os, subprocess, tempfile, unittest
SH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools', 'block_athenad.sh')
ORIG = '#!/bin/bash\n  export PYTHONPATH="$PWD"\n  for p in loggerd uploader deleter mapcurved; do\n    if [ -f /data/no_$p ]; then export BLOCK="${BLOCK:+$BLOCK,}$p"; fi\n  done\n'


def run(args, text=ORIG):
  d = tempfile.mkdtemp(prefix='blk_')
  env = dict(os.environ, LAUNCH=os.path.join(d, 'launch.sh'), BAK=os.path.join(d, 'bak'), MARK=os.path.join(d, 'no_manage_athenad'))
  if text is not None:
    open(env['LAUNCH'], 'w').write(text)
  def go(a):
    r = subprocess.run(['sh', SH] + a, env=env, stdout=subprocess.PIPE, universal_newlines=True)
    return r.returncode, r.stdout
  return d, env, go


class T(unittest.TestCase):
  def test_dry_run_changes_nothing(self):
    d, env, go = run([])
    rc, out = go([]); self.assertEqual(rc, 0); self.assertIn('dry run only', out)
    self.assertEqual(open(env['LAUNCH']).read(), ORIG); self.assertFalse(os.path.exists(env['MARK'])); self.assertFalse(os.path.exists(env['BAK']))

  def test_yes_patches_backs_up_and_creates_the_marker(self):
    d, env, go = run([])
    rc, out = go(['--yes']); self.assertEqual(rc, 0, out)
    new = open(env['LAUNCH']).read()
    self.assertIn('for p in loggerd uploader deleter mapcurved manage_athenad; do', new)
    self.assertEqual(new.replace(' manage_athenad', ''), ORIG)                      # nothing else changed
    self.assertEqual(open(env['BAK']).read(), ORIG); self.assertTrue(os.path.exists(env['MARK']))

  def test_yes_twice_is_harmless_and_keeps_the_first_backup(self):
    d, env, go = run([])
    go(['--yes']); first = open(env['LAUNCH']).read()
    rc, out = go(['--yes']); self.assertEqual(rc, 0); self.assertIn('already patched', out)
    self.assertEqual(open(env['LAUNCH']).read(), first); self.assertEqual(open(env['BAK']).read(), ORIG)

  def test_undo_restores_the_original_and_removes_the_marker(self):
    d, env, go = run([])
    go(['--yes']); rc, out = go(['--undo']); self.assertEqual(rc, 0)
    self.assertEqual(open(env['LAUNCH']).read(), ORIG); self.assertFalse(os.path.exists(env['MARK']))

  def test_unexpected_launch_script_is_left_alone(self):
    other = ORIG.replace('mapcurved', 'mapcurved extra')
    d, env, go = run([], text=other)
    rc, out = go(['--yes']); self.assertEqual(rc, 1); self.assertIn('not in the expected form', out)
    self.assertEqual(open(env['LAUNCH']).read(), other); self.assertFalse(os.path.exists(env['MARK'])); self.assertFalse(os.path.exists(env['BAK']))

  def test_missing_launch_script(self):
    d, env, go = run([], text=None)
    rc, out = go(['--yes']); self.assertEqual(rc, 1); self.assertIn('not found', out)


if __name__ == '__main__':
  unittest.main()
