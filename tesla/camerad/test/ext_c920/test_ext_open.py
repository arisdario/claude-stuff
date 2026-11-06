"""End-to-end test of the camera format switch in camera_ext_c920.cc against a fake camera. Needs g++, libjpeg-dev, libusb-1.0-0-dev, libavcodec-dev (headers only), ffmpeg+libx264, and a stock openpilot 0.7.5 selfdrive/ for headers:
   OPENPILOT_SELFDRIVE=/path/to/selfdrive python3 test_ext_open.py
Proves: MJPEG is what runs by default and in every failure case; H.264 only when asked for AND everything works; a build without H.264 support ignores the request."""
import os, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.join(HERE, '..', '..')
OPS = os.environ['OPENPILOT_SELFDRIVE']
tmp = tempfile.mkdtemp(prefix='extopen_')
inc = ['-I' + os.path.join(CAM, 'cameras'), '-I' + os.path.join(CAM, 'libuvc', 'include'), '-I' + os.path.join(CAM, 'include'), '-I' + CAM, '-I/usr/include/libusb-1.0', '-I/usr/include/x86_64-linux-gnu',
       '-I' + OPS, '-I' + OPS + '/common', '-I' + OPS + '/..']
libuvc = [os.path.join(CAM, 'libuvc', 'src', f) for f in ('ctrl.c', 'ctrl-gen.c', 'device.c', 'diag.c', 'frame.c', 'init.c', 'misc.c', 'stream.c')]
WRAP = ['uvc_init', 'uvc_find_device', 'uvc_open', 'uvc_close', 'uvc_unref_device', 'uvc_exit', 'uvc_get_extension_units', 'uvc_set_ae_priority', 'uvc_set_focus_auto', 'uvc_set_focus_abs',
        'uvc_get_stream_ctrl_format_size', 'uvc_start_streaming', 'uvc_stop_streaming']
def build(name, extra):
    objs = []
    for s, cc in [(os.path.join(CAM, 'cameras', 'camera_ext_c920.cc'), 'g++'), (os.path.join(HERE, 'ext_open_test.cc'), 'g++')] + [(l, 'gcc') for l in libuvc]:
        o = os.path.join(tmp, name + '_' + os.path.basename(s) + '.o')
        flags = ['-std=c++11'] if cc == 'g++' else []
        subprocess.check_call([cc, '-w', '-c', '-D__user=', '-DEXT_C920_TEST'] + flags + extra + inc + [s, '-o', o]); objs.append(o)
    exe = os.path.join(tmp, name)
    subprocess.check_call(['g++'] + objs + ['-ljpeg', '-lusb-1.0', '-lpthread', '-ldl'] + ['-Wl,--wrap=' + w for w in WRAP] + ['-o', exe])
    return exe
with_h264 = build('with_h264', ['-DEXT_WANT_AVCODEC'])
without = build('without_h264', [])
clip = os.path.join(tmp, 'clip.h264'); jpg = os.path.join(tmp, 'frame.jpg')
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i', 'testsrc2=size=1280x720:rate=20', '-t', '3', '-c:v', 'libx264', '-profile:v', 'baseline', '-pix_fmt', 'yuvj420p',
                       '-x264-params', 'keyint=20:min-keyint=20:bframes=0:aud=1:repeat-headers=1', '-f', 'h264', clip])
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i', 'testsrc2=size=1280x720', '-frames:v', '1', '-q:v', '3', jpg])
ok = True
def run(label, exe, scn, env, want_ok, want_fmt):
    global ok
    e = dict(os.environ); e.pop('CAMERA_FORMAT', None); e.pop('EXT_AVCODEC_DIR', None); e.update(env)
    r = subprocess.run([exe, scn, clip, jpg], stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True, env=e, timeout=60)
    res = [l for l in r.stdout.splitlines() if l.startswith('RESULT')]
    good = bool(res) and res[0] == 'RESULT ok=%d started=%s' % (want_ok, want_fmt)
    ok = ok and good
    print('%-66s %s   (%s)' % (label, 'OK' if good else 'FAIL', res[0] if res else 'no result: ' + r.stderr[-200:]))
H = {'CAMERA_FORMAT': 'h264'}
run('default (no format asked): MJPEG, H.264 never touched', with_h264, 'normal', {}, 1, 'mjpeg')
run('CAMERA_FORMAT=mjpeg: MJPEG', with_h264, 'normal', {'CAMERA_FORMAT': 'mjpeg'}, 1, 'mjpeg')
run('h264 asked, everything works: H.264', with_h264, 'normal', H, 1, 'h264')
run('h264 asked, camera has no H.264 mode: MJPEG', with_h264, 'no_h264', H, 1, 'mjpeg')
run('h264 asked, decoder library missing: MJPEG', with_h264, 'normal', dict(H, EXT_AVCODEC_DIR='/nonexistent'), 1, 'mjpeg')
run('h264 asked, H.264 stream will not start: MJPEG', with_h264, 'start_fails', H, 1, 'mjpeg')
run('h264 asked, camera sends junk (no picture ever): MJPEG', with_h264, 'garbage', H, 1, 'mjpeg')
run('build without H.264 support, h264 asked: MJPEG', without, 'normal', H, 1, 'mjpeg')
sys.exit(0 if ok else 1)
