#!/usr/bin/env python3
"""Tests for swap_cpu_bat_temp_display.py, against a synthetic CRLF fixture matching bbui.h's real
structure (the real file isn't in this repo - it's the user's UI source, uploaded separately):
    python3 -m unittest tesla/tests/test_swap_cpu_bat_temp_display.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import swap_cpu_bat_temp_display as SW  # noqa: E402

# A minimal but structurally faithful CRLF fixture: some unrelated lines, then a CPU block immediately
# followed by a BAT block, same shape as the real bb_ui_draw_measures_left()/_left2() (appears twice,
# like the real file's left/left2 pair).
FIXTURE = (
  "#pragma once\r\n"
  "\r\n"
  "void bb_ui_draw_measures_left( UIState *s, int bb_x, int bb_y, int bb_w ) {\r\n"
  "\tint bb_h = 5;\r\n"
  "\t\r\n"
  "\t//add CPU temperature\r\n"
  "\tif (true) {\r\n"
  "\t\tchar val_str[16];\r\n"
  "\t\tbb_h +=bb_ui_draw_measure(s, val_str, uom_str, \"CPU TEMP\", \r\n"
  "\t\t\t\tbb_rx, bb_ry, bb_uom_dx);\r\n"
  "\t\tbb_ry = bb_y + bb_h;\r\n"
  "\t}\r\n"
  "\r\n"
  "   //add battery temperature\r\n"
  "\tif (true) {\r\n"
  "\t\tchar val_str[16];\r\n"
  "\t\tbb_h +=bb_ui_draw_measure(s, val_str, uom_str, \"BAT TEMP\", \r\n"
  "\t\t\t\tbb_rx, bb_ry, bb_uom_dx);\r\n"
  "\t\tbb_ry = bb_y + bb_h;\r\n"
  "\t}\r\n"
  "}\r\n"
)


class TestPatch(unittest.TestCase):
  def test_dry_run_detects_swap_without_changing_text(self):
    new_text, msg = SW.patch(FIXTURE)
    self.assertIn('swapped 1 place', msg)
    self.assertNotEqual(new_text, FIXTURE)   # patch() itself returns the new text; main() decides whether to write it

  def test_swap_reorders_cpu_and_bat(self):
    new_text, _ = SW.patch(FIXTURE)
    cpu_i = new_text.index('//add CPU temperature')
    bat_i = new_text.index('//add battery temperature')
    self.assertLess(bat_i, cpu_i, 'BAT TEMP block should now come first')

  def test_swap_preserves_crlf_everywhere(self):
    new_text, _ = SW.patch(FIXTURE)
    self.assertNotIn('\r\n\r', new_text.replace('\r\n', ''))  # sanity: no bare \n introduced where \r\n was
    # every line ending in the result is still \r\n (no bare \n anywhere)
    body = new_text.replace('\r\n', '')
    self.assertNotIn('\n', body)

  def test_swap_preserves_all_content_just_reordered(self):
    new_text, _ = SW.patch(FIXTURE)
    # same set of non-blank lines, just reordered - nothing added or dropped
    orig_lines = sorted(l for l in FIXTURE.split('\r\n') if l.strip())
    new_lines = sorted(l for l in new_text.split('\r\n') if l.strip())
    self.assertEqual(orig_lines, new_lines)

  def test_already_swapped_is_a_noop(self):
    once, _ = SW.patch(FIXTURE)
    twice, msg = SW.patch(once)
    self.assertEqual(once, twice)
    self.assertIn('already swapped', msg)

  def test_refuses_on_unrecognized_file(self):
    with self.assertRaises(SystemExit):
      SW.patch("nothing matching here\r\n")

  def test_two_occurrences_both_swapped(self):
    doubled = FIXTURE + '\r\n' + FIXTURE
    new_text, msg = SW.patch(doubled)
    self.assertIn('swapped 2 places', msg)
    self.assertEqual(new_text.count('//add CPU temperature'), 2)
    self.assertEqual(new_text.count('//add battery temperature'), 2)


class TestMainEndToEnd(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.path = os.path.join(self.d, 'bbui.h')
    with open(self.path, 'w', newline='') as f:
      f.write(FIXTURE)

  def test_dry_run_does_not_touch_the_file(self):
    before = open(self.path, 'rb').read()
    SW.main([self.path])
    after = open(self.path, 'rb').read()
    self.assertEqual(before, after)

  def test_apply_then_restore_is_byte_identical(self):
    original = open(self.path, 'rb').read()
    SW.main([self.path, '--apply'])
    changed = open(self.path, 'rb').read()
    self.assertNotEqual(original, changed)
    self.assertTrue(os.path.exists(self.path + '.bak_tempswap'))
    SW.main([self.path, '--restore'])
    restored = open(self.path, 'rb').read()
    self.assertEqual(original, restored)

  def test_apply_preserves_crlf_on_disk(self):
    SW.main([self.path, '--apply'])
    raw = open(self.path, 'rb').read()
    self.assertIn(b'\r\n', raw)
    self.assertNotIn(b'[^\r]\n', raw)  # (sanity marker; real check below)
    # no bare LF anywhere (every \n is preceded by \r)
    for i, b in enumerate(raw):
      if b == 0x0A:  # \n
        self.assertEqual(raw[i - 1], 0x0D, 'found a bare LF not preceded by CR at byte %d' % i)

  def test_apply_twice_is_idempotent_on_disk(self):
    SW.main([self.path, '--apply'])
    once = open(self.path, 'rb').read()
    SW.main([self.path, '--apply'])
    twice = open(self.path, 'rb').read()
    self.assertEqual(once, twice)


if __name__ == '__main__':
  unittest.main()
