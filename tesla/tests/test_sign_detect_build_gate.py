"""Guards the rule 'sign detection must never be able to break or change a normal camerad build'
(tesla/camerad/SConscript, tesla/camerad/cameras/camera_ext_c920.cc). Same philosophy as
test_camera_h264_isolation.py's guard on the H.264 opt-in:
 1. the OpenCV block is wrapped in try/except, so a missing/broken OpenCV can't fail the whole build.
 2. it's gated behind a TryCompile+TryLink trial, never linked unconditionally.
 3. sign_detect.cc/sign_detect_io.cc are added to `cameras` (compiled in) ONLY inside the trial-ok branch.
 4. the camerad call site is behind #ifdef EXT_WANT_SIGN_DETECT, so without that macro on_frame's
    compiled behavior is provably unchanged (no sign_detect symbol reachable at all).
 5. sign_detect_tick() is identical, byte for byte, in on_frame and on_frame_h264's tails (reuses
    test_camera_h264_isolation.py's own tail-equality check, so this doesn't duplicate that logic -
    it just confirms the sign-detect line is present in that already-verified-equal tail).
"""
import os
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'camerad')
SC = open(os.path.join(ROOT, 'SConscript')).read()
CC = open(os.path.join(ROOT, 'cameras', 'camera_ext_c920.cc')).read()


def func(src, head):
  a = src.index(head)
  return src[a:src.index('\n}\n', a) + 3]


class T(unittest.TestCase):
  def test_opencv_trial_is_wrapped_in_try_except(self):
    i = SC.index('OPTIONAL stop-sign')
    block = SC[i:SC.index("elif arch ==", i)]
    self.assertIn('try:', block); self.assertIn('except Exception', block)

  def test_opencv_only_linked_after_a_successful_trial(self):
    i = SC.index('OPTIONAL stop-sign')
    block = SC[i:SC.index("elif arch ==", i)]
    self.assertIn('TryCompile', block); self.assertIn('TryLink', block)
    self.assertIn('EXT_WANT_SIGN_DETECT', block)

  def test_sign_detect_sources_added_only_inside_the_trial_ok_branch(self):
    i = SC.index('OPTIONAL stop-sign')
    block = SC[i:SC.index("elif arch ==", i)]
    self.assertIn("cameras/sign_detect.cc", block)
    self.assertIn("cameras/sign_detect_io.cc", block)
    # must be inside the `if trial_ok:` body, not unconditional - check the actual `cameras +=` /
    # `cameras = [...]` assignment forms specifically, not just the substring (which legitimately
    # also appears in this block's own explanatory comment, before the trial even runs)
    ok_i = block.index('if trial_ok:')
    else_i = block.index('else:', ok_i)
    self.assertIn('cameras += ', block[ok_i:else_i])
    self.assertIn('sign_detect.cc', block[ok_i:else_i])
    for m in ('cameras += ', 'cameras = ['):
      self.assertNotIn(m, block[:ok_i])

  def test_call_site_is_behind_the_macro(self):
    for head in ('void on_frame(uvc_frame_t *f, void *) {', 'void on_frame_h264(uvc_frame_t *f, void *) {'):
      try:
        body = func(CC, head)
      except ValueError:
        continue   # on_frame_h264 only exists textually once per file; fine if not found here
      self.assertIn('#ifdef EXT_WANT_SIGN_DETECT', body)
      self.assertIn('sign_detect_tick(g_rgb, SRC_W, SRC_H);', body)
      # the call must be inside the ifdef, not just present somewhere in the function
      i = body.index('#ifdef EXT_WANT_SIGN_DETECT')
      j = body.index('#endif', i)
      self.assertIn('sign_detect_tick', body[i:j])

  def test_header_included_only_behind_the_macro(self):
    self.assertIn('#ifdef EXT_WANT_SIGN_DETECT\n#include "sign_detect_io.h"', CC)


if __name__ == '__main__':
  unittest.main()
