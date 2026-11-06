#!/usr/bin/env python3
"""Tests for lightdash_launcher.ensure_running()'s guard logic (no real process is spawned):
    python3 -m unittest tesla/tests/test_lightdash_launcher.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import lightdash_launcher as LL  # noqa: E402


class TestEnsureRunning(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.bin = os.path.join(self.d, 'lightdash')
    self.pid = os.path.join(self.d, 'lightdash.pid')
    self.off = os.path.join(self.d, 'lightdash_off')
    self.log = os.path.join(self.d, 'lightdash.log')
    self.spawned = []
    self._real_popen = LL.subprocess.Popen
    LL.subprocess.Popen = self._fake_popen

  def tearDown(self):
    LL.subprocess.Popen = self._real_popen

  def _fake_popen(self, *a, **k):
    self.spawned.append((a, k))
    class P:
      pid = 4242
    return P()

  def _run(self):
    return LL.ensure_running(bin_path=self.bin, pid_path=self.pid, off_flag=self.off, log_path=self.log,
                              lib_dir=os.path.join(self.d, 'no-such-lib-dir'))

  def test_not_built_no_launch(self):
    self.assertFalse(self._run())
    self.assertEqual(self.spawned, [])

  def test_built_launches_once(self):
    open(self.bin, 'w').close()
    self.assertTrue(self._run())
    self.assertEqual(len(self.spawned), 1)
    self.assertEqual(open(self.pid).read().strip(), '4242')
    self.assertEqual(self.spawned[0][0][0], [self.bin])        # argv: just the binary, no args

  def test_off_flag_blocks_launch(self):
    open(self.bin, 'w').close()
    open(self.off, 'w').close()
    self.assertFalse(self._run())
    self.assertEqual(self.spawned, [])

  def test_already_running_not_respawned(self):
    open(self.bin, 'w').close()
    # our own pid IS alive, and /proc/<pid>/cmdline contains 'python' - but the guard needs the lightdash tag.
    with open(self.pid, 'w') as f:
      f.write(str(os.getpid()))
    LL.PROC_TAG = 'python'            # make the liveness check match this test process
    try:
      self.assertFalse(self._run())
      self.assertEqual(self.spawned, [])
    finally:
      LL.PROC_TAG = 'lightdash'

  def test_stale_pid_respawns(self):
    open(self.bin, 'w').close()
    with open(self.pid, 'w') as f:
      f.write('999999')              # a pid that isn't running
    self.assertTrue(self._run())
    self.assertEqual(len(self.spawned), 1)

  def test_lib_dir_added_when_present(self):
    open(self.bin, 'w').close()
    libdir = os.path.join(self.d, 'lib')
    os.makedirs(libdir)
    self.assertTrue(LL.ensure_running(bin_path=self.bin, pid_path=self.pid, off_flag=self.off, log_path=self.log,
                                       lib_dir=libdir))
    self.assertIn(libdir, self.spawned[0][1]['env']['LD_LIBRARY_PATH'])


if __name__ == '__main__':
  unittest.main()
