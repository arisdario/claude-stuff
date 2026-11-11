#!/usr/bin/env python3
"""Tests for set_camera_offset.py, against a synthetic fixture matching lane_planner.py's real
CAMERA_OFFSET line (the real file isn't in this repo - it's core openpilot):
    python3 -m unittest tesla/tests/test_set_camera_offset.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import set_camera_offset as SC  # noqa: E402

FIXTURE = (
  "import numpy as np\n"
  "from common.numpy_fast import interp\n"
  "\n"
  "CAMERA_OFFSET = 0.06  # m from center car to camera\n"
  "\n"
  "class LanePlanner():\n"
  "  def __init__(self):\n"
  "    self.l_poly = [0., 0., 0., 0.]\n"
)


class TestPatch(unittest.TestCase):
  def test_dry_run_reports_the_change(self):
    new_text, msg = SC.patch(FIXTURE, 0.0)
    self.assertIn('changed CAMERA_OFFSET from 0.06 to 0.0', msg)
    self.assertNotEqual(new_text, FIXTURE)

  def test_sets_the_new_value_and_preserves_the_comment(self):
    new_text, _ = SC.patch(FIXTURE, 0.0)
    self.assertIn('CAMERA_OFFSET = 0.0  # m from center car to camera', new_text)

  def test_preserves_everything_else(self):
    new_text, _ = SC.patch(FIXTURE, 0.0)
    for line in FIXTURE.split('\n'):
      if line.strip() and 'CAMERA_OFFSET' not in line:
        self.assertIn(line, new_text)

  def test_already_at_target_value_is_a_noop(self):
    once, _ = SC.patch(FIXTURE, 0.06)
    self.assertEqual(once, FIXTURE)

  def test_idempotent_second_patch_is_a_noop(self):
    once, _ = SC.patch(FIXTURE, 0.0)
    twice, msg = SC.patch(once, 0.0)
    self.assertEqual(once, twice)
    self.assertIn('already', msg)

  def test_negative_offset_works(self):
    new_text, msg = SC.patch(FIXTURE, -0.05)
    self.assertIn('CAMERA_OFFSET = -0.05', new_text)
    self.assertIn('from 0.06 to -0.05', msg)

  def test_refuses_on_missing_anchor(self):
    with self.assertRaises(SystemExit):
      SC.patch('nothing matching here\n', 0.0)

  def test_refuses_if_anchor_appears_more_than_once(self):
    bad = FIXTURE + 'CAMERA_OFFSET = 0.06  # duplicate\n'
    with self.assertRaises(SystemExit):
      SC.patch(bad, 0.0)


class TestMainEndToEnd(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.path = os.path.join(self.d, 'lane_planner.py')
    with open(self.path, 'w') as f:
      f.write(FIXTURE)

  def test_dry_run_does_not_touch_the_file(self):
    before = open(self.path, 'rb').read()
    SC.main([self.path, '0.0'])
    after = open(self.path, 'rb').read()
    self.assertEqual(before, after)

  def test_apply_then_restore_is_byte_identical(self):
    original = open(self.path, 'rb').read()
    SC.main([self.path, '0.0', '--apply'])
    changed = open(self.path, 'rb').read()
    self.assertNotEqual(original, changed)
    self.assertTrue(os.path.exists(self.path + '.bak_camoffset'))
    SC.main([self.path, '--restore'])
    restored = open(self.path, 'rb').read()
    self.assertEqual(original, restored)

  def test_apply_twice_same_value_is_idempotent_on_disk(self):
    SC.main([self.path, '0.0', '--apply'])
    once = open(self.path, 'rb').read()
    SC.main([self.path, '0.0', '--apply'])
    twice = open(self.path, 'rb').read()
    self.assertEqual(once, twice)

  def test_apply_actually_writes_the_requested_value(self):
    SC.main([self.path, '0.0', '--apply'])
    content = open(self.path).read()
    self.assertIn('CAMERA_OFFSET = 0.0', content)


if __name__ == '__main__':
  unittest.main()
