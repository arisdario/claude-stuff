#!/usr/bin/env python3
"""Tests for mapcurve_launcher.ensure_running()'s guard logic (no real process is spawned):
    python3 -m unittest tesla/tests/test_mapcurve_launcher.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import mapcurve_launcher as ML  # noqa: E402


class TestEnsureRunning(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.db = os.path.join(self.d, 'mapcurve.sqlite')
    self.pid = os.path.join(self.d, 'mapcurved.pid')
    self.off = os.path.join(self.d, 'mapcurve_off')
    self.op = os.path.join(self.d, 'op')
    os.makedirs(os.path.join(self.op, 'selfdrive', 'car', 'tesla'))
    open(os.path.join(self.op, ML.SCRIPT), 'w').close()   # pretend mapcurved.py is deployed
    self.spawned = []
    self._real_popen = ML.subprocess.Popen
    ML.subprocess.Popen = self._fake_popen

  def tearDown(self):
    ML.subprocess.Popen = self._real_popen

  def _fake_popen(self, *a, **k):
    self.spawned.append((a, k))
    class P:
      pid = 4242
    return P()

  def _run(self):
    return ML.ensure_running(db_path=self.db, op_dir=self.op, pid_path=self.pid,
                             off_flag=self.off, log_path=os.path.join(self.d, 'log'))

  def test_no_db_no_launch(self):
    self.assertFalse(self._run())
    self.assertEqual(self.spawned, [])

  def test_db_present_launches_once(self):
    open(self.db, 'w').close()
    self.assertTrue(self._run())
    self.assertEqual(len(self.spawned), 1)
    # pid recorded; cwd + PYTHONPATH set so imports resolve
    self.assertEqual(open(self.pid).read().strip(), '4242')
    self.assertEqual(self.spawned[0][1]['cwd'], self.op)
    self.assertIn(self.op, self.spawned[0][1]['env']['PYTHONPATH'])

  def test_off_flag_blocks_launch(self):
    open(self.db, 'w').close()
    open(self.off, 'w').close()
    self.assertFalse(self._run())
    self.assertEqual(self.spawned, [])

  def test_not_deployed_no_launch(self):
    open(self.db, 'w').close()
    os.remove(os.path.join(self.op, ML.SCRIPT))
    self.assertFalse(self._run())
    self.assertEqual(self.spawned, [])

  def test_already_running_not_respawned(self):
    open(self.db, 'w').close()
    # our own pid IS alive, and /proc/<pid>/cmdline contains 'python' - but the guard needs the mapcurved tag.
    with open(self.pid, 'w') as f:
      f.write(str(os.getpid()))
    ML.PROC_TAG = 'python'            # make the liveness check match this test process
    try:
      self.assertFalse(self._run())
      self.assertEqual(self.spawned, [])
    finally:
      ML.PROC_TAG = 'mapcurved.py'

  def test_stale_pid_respawns(self):
    open(self.db, 'w').close()
    with open(self.pid, 'w') as f:
      f.write('999999')              # a pid that isn't running
    self.assertTrue(self._run())
    self.assertEqual(len(self.spawned), 1)


if __name__ == '__main__':
  unittest.main()
