"""Shadow auto-high-beam vision (tesla/camerad/cameras/ahb_vision.cc):
 1. like sign detection, it can never break or change a normal camerad build: try/except, trial compile + link,
    sources added only inside the trial-ok branch, call sites behind #ifdef EXT_WANT_AHB_VISION, identical in both tails.
 2. the installer ships both files.
 3. the synthetic-scene C++ tests pass (needs g++)."""
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'camerad')
SC = open(os.path.join(ROOT, 'SConscript')).read()
CC = open(os.path.join(ROOT, 'cameras', 'camera_ext_c920.cc')).read()
INST = open(os.path.join(ROOT, 'install_external_camera.sh')).read()


def func(src, head):
  a = src.index(head)
  return src[a:src.index('\n}\n', a) + 3]


class T(unittest.TestCase):
  def block(self):
    i = SC.index('OPTIONAL shadow auto-high-beam')
    return SC[i:SC.index('elif arch ==', i)]

  def test_build_gate(self):
    b = self.block()
    self.assertIn('try:', b); self.assertIn('except Exception', b)
    self.assertIn('TryCompile', b); self.assertIn('TryLink', b)
    ok = b.index('if trial_ok:')
    self.assertGreater(b.index("cameras += ['cameras/ahb_vision.cc']"), ok)
    self.assertGreater(b.index('-DEXT_WANT_AHB_VISION'), ok)
    self.assertLess(b.index("cameras += ['cameras/ahb_vision.cc']"), b.index('else:', ok))

  def test_call_sites_behind_the_macro_in_both_tails(self):
    for head in ('void on_frame(uvc_frame_t *f, void *) {', 'void on_frame_h264(uvc_frame_t *f, void *) {'):
      body = func(CC, head)
      self.assertEqual(body.count('ahb_vision_tick(g_rgb, SRC_W, SRC_H);'), 1, head)
      i = body.index('ahb_vision_tick(')
      self.assertGreater(body[:i].rfind('#ifdef EXT_WANT_AHB_VISION'), body[:i].rfind('#endif'))
      self.assertGreater(i, body.index('if (g_flip)'))          # after the flip: same orientation as the model sees
    inc = CC.index('#include "ahb_vision.h"')
    self.assertGreater(CC[:inc].rfind('#ifdef EXT_WANT_AHB_VISION'), CC[:inc].rfind('#endif'))

  def test_installer_ships_it(self):
    self.assertIn('cameras/ahb_vision.cc', INST); self.assertIn('cameras/ahb_vision.h', INST)

  @unittest.skipUnless(shutil.which('g++'), 'needs g++')
  def test_synthetic_scenes(self):
    d = tempfile.mkdtemp()
    try:
      exe = os.path.join(d, 'av')
      t = os.path.join(ROOT, 'test', 'ext_c920')
      subprocess.check_call(['g++', '-std=c++11', '-O2', '-I' + os.path.join(ROOT, 'cameras'),
                             os.path.join(t, 'ahb_vision_test.cc'), os.path.join(ROOT, 'cameras', 'ahb_vision.cc'), '-o', exe])
      r = subprocess.run([exe], stderr=subprocess.PIPE, universal_newlines=True)
      self.assertEqual(r.returncode, 0, r.stderr)
    finally:
      shutil.rmtree(d)


if __name__ == '__main__':
  unittest.main()
