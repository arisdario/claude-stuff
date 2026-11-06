"""Off-EON test of cameras/camera_h264.h (the optional H.264 input): needs g++, libavcodec-dev/libavutil-dev (headers only), ffmpeg with libx264, numpy.
   python3 test_h264.py
Checks: (1) pure helpers; (2) decoding a Constrained-Baseline 1280x720 yuvj420p stream like the C920's gives the same picture as ffmpeg's own decode;
(3) nothing is output before the first IDR; (4) after a damaged frame nothing is output until the next IDR (a broken reference chain never reaches the model)."""
import os, re, subprocess, sys, tempfile
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CAM = os.path.join(HERE, '..', '..')
tmp = tempfile.mkdtemp(prefix='h264test_')
exe = os.path.join(tmp, 'h264_test')
inc = ["-DEXT_WANT_AVCODEC", '-I' + os.path.join(CAM, 'cameras'), '-I/usr/include/x86_64-linux-gnu']
subprocess.check_call(['g++', '-std=c++11', '-Wall', '-O2', '-DEXT_WANT_AVCODEC'] + inc + [os.path.join(HERE, 'h264_test.cc'), '-ldl', '-o', exe])
clip = os.path.join(tmp, 'clip.h264')
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-f', 'lavfi', '-i', 'testsrc2=size=1280x720:rate=20', '-t', '4', '-c:v', 'libx264', '-profile:v', 'baseline',
                       '-pix_fmt', 'yuvj420p', '-x264-params', 'keyint=20:min-keyint=20:bframes=0:aud=1:repeat-headers=1', '-f', 'h264', clip])
ref = os.path.join(tmp, 'ref.rgb')
subprocess.check_call(['ffmpeg', '-v', 'error', '-y', '-i', clip, '-pix_fmt', 'rgb24', '-f', 'rawvideo', ref])
refv = np.fromfile(ref, np.uint8).reshape(-1, 720, 1280, 3)
FR = 720 * 1280 * 3
ok = True
def run(mode, n=None):
    out = os.path.join(tmp, 'out_%s.rgb' % mode)
    r = subprocess.run([exe, clip, out, mode] + ([str(n)] if n is not None else []), stdout=subprocess.PIPE, universal_newlines=True)
    lines = r.stdout.splitlines()
    outs = [int(m.group(1)) for l in lines for m in [re.match(r'AU (\d+) out', l)] if m]
    v = np.fromfile(out, np.uint8).reshape(-1, 720, 1280, 3) if os.path.getsize(out) else np.zeros((0, 720, 1280, 3), np.uint8)
    return r.stdout, outs, v
def check(name, cond, extra=''):
    global ok
    print('%-62s %s %s' % (name, 'OK' if cond else 'FAIL', extra)); ok = ok and cond

o, outs, v = run('all')
check('opens libavcodec and decodes all %d access units' % len(refv), outs == list(range(len(refv))), '(%d out)' % len(outs))
if len(v) == len(refv):
    d = np.abs(v.astype(int) - refv.astype(int))
    check('picture equals ffmpeg own decode (mean abs diff < 2.0)', d.mean() < 2.0, 'mean %.2f, 99.9th pct %d' % (d.mean(), np.percentile(d, 99.9)))
    check('no R/B swap', np.abs(v[..., ::-1].astype(int) - refv.astype(int)).mean() > 3 * d.mean())
o, outs, v = run('skipfirst')
check('no output until the next IDR when the stream starts mid-GOP', outs and outs[0] == 20 and outs == list(range(20, len(refv))), 'first out AU %s' % (outs[0] if outs else None))
if len(v) == len(refv) - 20:
    d = np.abs(v.astype(int) - refv[20:].astype(int)).mean()
    check('frames after the resync are correct', d < 2.0, 'mean %.2f' % d)
o, outs, v = run('corrupt', 30)
check('after a truncated frame: nothing until the next IDR (AU 40)', 30 not in outs and all(a not in outs for a in range(30, 40)) and 40 in outs and outs[:30] == list(range(30)), 'outs after 29: %s' % [a for a in outs if a >= 30][:3])
if 40 in outs:
    k = outs.index(40)
    d = np.abs(v[k].astype(int) - refv[40].astype(int)).mean()
    check('the IDR after the damage decodes correctly', d < 2.0, 'mean %.2f' % d)
# pure helpers through python
sys.exit(0 if ok else 1)
