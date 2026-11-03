"""Builds pack_test (needs g++, libjpeg-dev, libusb-1.0-dev, ffmpeg, numpy) and checks ext_c920_pack_bgr against ffmpeg's crop+pad on a 1280x720 JPEG:  python3 test_pack.py frame.jpg"""
import os
import subprocess
import sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.join(HERE, '..', '..')
src = [os.path.join(CAM, 'cameras', 'camera_ext_c920.cc'), os.path.join(HERE, 'pack_test.cc')] + [os.path.join(CAM, 'libuvc', 'src', f) for f in ('ctrl.c', 'ctrl-gen.c', 'device.c', 'diag.c', 'frame.c', 'init.c', 'misc.c', 'stream.c')]
OPS = os.environ.get('OPENPILOT_SELFDRIVE', '')            # a stock openpilot 0.7.5 selfdrive/ (for the headers only)
stub = os.environ.get('GL_STUB', '')
inc = ['-I' + os.path.join(CAM, 'cameras'), '-I' + os.path.join(CAM, 'libuvc', 'include'), '-I' + os.path.join(CAM, 'include'), '-I/usr/include/libusb-1.0'] + (['-I' + OPS, '-I' + OPS + '/common', '-I' + OPS + '/..'] if OPS else []) + (['-I' + stub] if stub else [])
exe = os.path.join(HERE, 'pack_test.bin')
objs = []
for s in src:
    o = s + '.o'
    cc = ['g++', '-std=c++11', '-w'] if s.endswith('.cc') else ['gcc', '-w']
    subprocess.check_call(cc + ['-c', '-D__user=', '-DEXT_C920_TEST'] + inc + [s, '-o', o]); objs.append(o)
subprocess.check_call(['g++'] + objs + ['-ljpeg', '-lusb-1.0', '-lpthread', '-o', exe])
jpg = sys.argv[1]
subprocess.check_call([exe, jpg, '/tmp/pack_out.bgr'])
mine = np.fromfile('/tmp/pack_out.bgr', dtype=np.uint8).reshape(874, 1164, 3)
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-i', jpg, '-vf', 'crop=1164:720:58:0,pad=1164:874:0:77:black', '-pix_fmt', 'bgr24', '-f', 'rawvideo', '/tmp/pack_ref.bgr'])
ref = np.fromfile('/tmp/pack_ref.bgr', dtype=np.uint8).reshape(874, 1164, 3)
bars = (mine[:77].max() == 0) and (mine[797:].max() == 0)
d = np.abs(mine.astype(int) - ref.astype(int))
print('black bars exactly empty:', bars, '| mean abs diff vs ffmpeg %.2f, 99.9th percentile %d (JPEG decoders differ slightly)' % (d.mean(), np.percentile(d, 99.9)))
swap = np.abs(mine[..., ::-1].astype(int) - ref.astype(int)).mean()
print('BGR order correct (an R/B swap would give a much larger error: %.2f vs %.2f)' % (swap, d.mean()))
sys.exit(0 if (bars and d.mean() < 3.0 and swap > 3 * d.mean()) else 1)
