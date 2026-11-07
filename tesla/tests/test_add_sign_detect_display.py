#!/usr/bin/env python3
"""Tests for add_sign_detect_display.py, against a synthetic CRLF fixture matching bbui.h's real
structure (the real file isn't in this repo - it's the user's UI source, uploaded separately):
    python3 -m unittest tesla/tests/test_add_sign_detect_display.py
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import add_sign_detect_display as AD  # noqa: E402

FIXTURE = (
  "#pragma once\r\n"
  "\r\n"
  "#include \"cereal/gen/c/ui.capnp.h\"\r\n"
  "\r\n"
  "void bb_ui_draw_measures_left( UIState *s, int bb_x, int bb_y, int bb_w ) {\r\n"
  "\tint bb_h = 5;\r\n"
  "}\r\n"
  "\r\n"
  "void bb_ui_draw_UI( UIState *s) {\r\n"
  "  if (s->b.tri_state_switch == 1) {\r\n"
  "    bb_draw_buttons(s);\r\n"
  "    bb_ui_draw_custom_alert(s);\r\n"
  "  }\r\n"
  "  if (s->b.tri_state_switch ==2) {\r\n"
  "    bb_draw_buttons(s);\r\n"
  "  }\r\n"
  "}\r\n"
)


class TestPatch(unittest.TestCase):
  def test_dry_run_reports_what_it_would_do(self):
    new_text, msg = AD.patch(FIXTURE)
    self.assertIn('applied', msg)
    self.assertNotEqual(new_text, FIXTURE)

  def test_adds_the_includes_once(self):
    new_text, _ = AD.patch(FIXTURE)
    self.assertIn('#include <sys/stat.h>', new_text)
    self.assertIn('#include <time.h>', new_text)
    self.assertEqual(new_text.count('#include <sys/stat.h>'), 1)

  def test_adds_the_function_once(self):
    new_text, _ = AD.patch(FIXTURE)
    self.assertEqual(new_text.count('void bb_ui_draw_sign_detect(UIState *s)'), 1)
    self.assertEqual(new_text.count(AD.FUNC_MARKER_START), 1)
    self.assertEqual(new_text.count(AD.FUNC_MARKER_END), 1)

  def test_calls_it_at_every_bb_draw_buttons_site(self):
    new_text, _ = AD.patch(FIXTURE)
    n_buttons = FIXTURE.count('bb_draw_buttons(s);')
    self.assertEqual(new_text.count('bb_ui_draw_sign_detect(s);'), n_buttons)

  def test_preserves_crlf_everywhere(self):
    new_text, _ = AD.patch(FIXTURE)
    body = new_text.replace('\r\n', '')
    self.assertNotIn('\n', body)

  def test_preserves_all_original_content(self):
    new_text, _ = AD.patch(FIXTURE)
    for line in FIXTURE.split('\r\n'):
      if line.strip():
        self.assertIn(line, new_text)

  def test_braces_are_balanced_in_the_new_function(self):
    new_text, _ = AD.patch(FIXTURE)
    a = new_text.index(AD.FUNC_MARKER_START)
    b = new_text.index(AD.FUNC_MARKER_END)
    chunk = new_text[a:b]
    self.assertEqual(chunk.count('{'), chunk.count('}'))

  def test_already_applied_is_a_noop(self):
    once, _ = AD.patch(FIXTURE)
    twice, msg = AD.patch(once)
    self.assertEqual(once, twice)
    self.assertIn('already applied', msg)

  def test_refuses_on_unrecognized_file(self):
    with self.assertRaises(SystemExit):
      AD.patch("nothing matching here\r\n")

  def test_refuses_if_too_few_button_call_sites(self):
    bad = FIXTURE.replace('  if (s->b.tri_state_switch ==2) {\r\n    bb_draw_buttons(s);\r\n  }\r\n', '')
    with self.assertRaises(SystemExit):
      AD.patch(bad)


class TestMainEndToEnd(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.path = os.path.join(self.d, 'bbui.h')
    with open(self.path, 'w', newline='') as f:
      f.write(FIXTURE)

  def test_dry_run_does_not_touch_the_file(self):
    before = open(self.path, 'rb').read()
    AD.main([self.path])
    after = open(self.path, 'rb').read()
    self.assertEqual(before, after)

  def test_apply_then_restore_is_byte_identical(self):
    original = open(self.path, 'rb').read()
    AD.main([self.path, '--apply'])
    changed = open(self.path, 'rb').read()
    self.assertNotEqual(original, changed)
    self.assertTrue(os.path.exists(self.path + '.bak_signdetect'))
    AD.main([self.path, '--restore'])
    restored = open(self.path, 'rb').read()
    self.assertEqual(original, restored)

  def test_apply_twice_is_idempotent_on_disk(self):
    AD.main([self.path, '--apply'])
    once = open(self.path, 'rb').read()
    AD.main([self.path, '--apply'])
    twice = open(self.path, 'rb').read()
    self.assertEqual(once, twice)

  def test_apply_preserves_crlf_on_disk(self):
    AD.main([self.path, '--apply'])
    raw = open(self.path, 'rb').read()
    for i, b in enumerate(raw):
      if b == 0x0A:
        self.assertEqual(raw[i - 1], 0x0D, 'found a bare LF not preceded by CR at byte %d' % i)


if __name__ == '__main__':
  unittest.main()
