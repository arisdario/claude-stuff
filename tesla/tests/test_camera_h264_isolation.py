"""Guards the rule 'the MJPEG camera path must not be compromised by the optional H.264 input' (tesla/camerad/cameras/camera_ext_c920.cc).
 1. on_frame (the MJPEG callback) is frozen: its text must equal the version from before H.264 existed. Changing it on purpose means updating FROZEN_SHA here.
 2. on_frame_h264 ends with the very same tail (flip, luma sample, dispatch into openpilot's buffer) as on_frame - compared text for text, so they cannot drift apart.
 3. every H.264-only piece is inside #ifdef EXT_HAVE_AVCODEC, and the header only defines that macro when the build asks for it (EXT_WANT_AVCODEC).
 4. the build script sets that flag only after a trial compile, and the install script ships the new header."""
import hashlib, os, re, unittest
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'camerad')
CC = open(os.path.join(ROOT, 'cameras', 'camera_ext_c920.cc')).read()
HDR = open(os.path.join(ROOT, 'cameras', 'camera_h264.h')).read()
SC = open(os.path.join(ROOT, 'SConscript')).read()
INST = open(os.path.join(ROOT, 'install_external_camera.sh')).read()
FROZEN_SHA = '546fdc9b84a9bff847471454146f2b172f703e054f53a6bfab3d06ee35e855b6'


def func(src, head):
  a = src.index(head)
  return src[a:src.index('\n}\n', a) + 3]


class T(unittest.TestCase):
  def test_mjpeg_callback_is_frozen(self):
    self.assertEqual(hashlib.sha256(func(CC, 'void on_frame(uvc_frame_t *f, void *) {').encode()).hexdigest(), FROZEN_SHA)

  def test_tails_are_identical(self):
    on = func(CC, 'void on_frame(uvc_frame_t *f, void *) {')
    tail_mjpeg = on[on.index('  if (g_flip) {'):on.rindex('}\n')]
    h = func(CC, 'void on_frame_h264(uvc_frame_t *f, void *) {')
    tail_h264 = h[h.index('  //TAIL-BEGIN\n') + len('  //TAIL-BEGIN\n'):h.index('  //TAIL-END\n')]
    self.assertEqual(tail_mjpeg, tail_h264)

  def test_h264_code_is_behind_the_macro(self):
    self.assertIn('defined(EXT_WANT_AVCODEC)', HDR)
    ifdef = CC.index('#ifdef EXT_HAVE_AVCODEC\n// ---- H.264 input')
    end = CC.index('#endif  // EXT_HAVE_AVCODEC', ifdef)
    for name in ('on_frame_h264', 'wants_h264', 'publish_live_rgb', 'g_dec;'):
      for m in re.finditer(re.escape(name), CC):
        if ifdef <= m.start() <= end:
          continue
        # outside the block it may only appear in an #ifdef EXT_HAVE_AVCODEC region of ext_open / capture_thread / selftest
        before = CC[:m.start()]
        self.assertGreater(before.rfind('#ifdef EXT_HAVE_AVCODEC'), before.rfind('#endif'), 'H.264 symbol %s outside the macro at char %d' % (name, m.start()))
    self.assertNotIn('g_h264', func(CC, 'void on_frame(uvc_frame_t *f, void *) {'))

  def test_build_and_install(self):
    self.assertIn('TryCompile', SC); self.assertIn('EXT_WANT_AVCODEC', SC)
    self.assertIn('cameras/camera_h264.h', INST)
    self.assertNotIn("'avcodec'", SC)          # never linked: loaded at run time


if __name__ == '__main__':
  unittest.main()
