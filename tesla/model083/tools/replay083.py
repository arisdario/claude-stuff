#!/usr/bin/env python3
"""
replay083.py - runs a recorded EON drive (fcamera.hevc) through the openpilot 0.8.3 driving model exactly the way the
EON does, decodes the output with the same C++ adapter the EON runs, and draws the path and lanes on the video.
Checks the 0.8.3 port without a car, a screen or the EON's camera in the way. Runs on a PC.

  python3 replay083.py fcamera.hevc --vp 542.62 440.36 --model supercombo.onnx [--frames 600] [--out replay.mp4]

--vp is the calibration from the EON: cat /data/params/d/CalibrationParams  ->  "vanishing_point": [x, y]
Needs: numpy, onnxruntime, Pillow, ffmpeg, g++ (to build tools/replay_decode.cc against ../model083_adapter.h).

What the EON does, reproduced here (openpilot 0.7.5 sources):
  calibrationd  vp -> yaw = atan(vx_norm), pitch = -atan(vy_norm*cos(yaw)), roll 0; camera height 1.22 m
  modeld.cc     warp = K * [view_from_road | (0,1.22,0)] (ground columns) * ground_from_medmodel  (512x256 input)
  transform.cl  per output pixel: sample the camera image at warp*(u,v,1), bilinear, 0 outside; U/V with the warp
                scaled to half size (in = (M*(out/0.5+0.5)) - 0.5) * 0.5)
  loadyuv.cl    6 channels: Y even/even, Y odd-row, Y odd-col, Y odd/odd, U, V; raw 0..255 floats; prev + current
"""
import argparse
import os
import subprocess
import sys

import numpy as np

W_CAM, H_CAM = 1164, 874
K = np.array([[910., 0., 582.], [0., 910., 437.], [0., 0., 1.]])
MODEL_H = 1.22
GROUND_FROM_MEDMODEL = np.array([[0.0, 0.0, 1.0],
                                 [-1.09890110e-03, 0.0, 2.81318681e-01],
                                 [-1.84808520e-20, 9.00738606e-04, -4.28751576e-02]])
VIEW_FROM_DEVICE = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]]).T
MW, MH = 512, 256
OUT_N = 11327 + 512
HERE = os.path.dirname(os.path.abspath(__file__))


def rot_from_euler(roll, pitch, yaw):
  cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
  rr = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
  rp = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
  ry = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
  return ry @ rp @ rr


def rot_from_euler_quat(roll, pitch, yaw):
  """openpilot's euler2quat -> quat2rot, to check rot_from_euler"""
  g, t, p = roll / 2, pitch / 2, yaw / 2
  q0 = np.cos(g) * np.cos(t) * np.cos(p) + np.sin(g) * np.sin(t) * np.sin(p)
  q1 = np.sin(g) * np.cos(t) * np.cos(p) - np.cos(g) * np.sin(t) * np.sin(p)
  q2 = np.cos(g) * np.sin(t) * np.cos(p) + np.sin(g) * np.cos(t) * np.sin(p)
  q3 = np.cos(g) * np.cos(t) * np.sin(p) - np.sin(g) * np.sin(t) * np.cos(p)
  return np.array([[q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2-q0*q3), 2*(q0*q2+q1*q3)],
                   [2*(q1*q2+q0*q3), q0*q0-q1*q1+q2*q2-q3*q3, 2*(q2*q3-q0*q1)],
                   [2*(q1*q3-q0*q2), 2*(q0*q1+q2*q3), q0*q0-q1*q1-q2*q2+q3*q3]])


def extrinsic_from_vp(vx, vy):
  n = np.linalg.inv(K) @ np.array([vx, vy, 1.0])
  yaw = np.arctan(n[0]); pitch = -np.arctan(n[1] * np.cos(yaw))
  R = rot_from_euler(0.0, pitch, yaw)
  assert np.allclose(R, rot_from_euler_quat(0.0, pitch, yaw)), 'rotation conventions disagree'
  view_from_road = VIEW_FROM_DEVICE @ (R @ np.diag([1., -1., -1.]))       # road frame: x fwd, y LEFT, z UP
  return np.hstack((view_from_road, [[0.], [MODEL_H], [0.]])), np.degrees(pitch), np.degrees(yaw)


def warp_plane(img, M, ow, oh):
  """transform.cl: dst(u,v) = bilinear(src, M*(u,v,1)), 0 outside"""
  u, v = np.meshgrid(np.arange(ow, dtype=np.float64), np.arange(oh, dtype=np.float64))
  X = M[0, 0] * u + M[0, 1] * v + M[0, 2]; Y = M[1, 0] * u + M[1, 1] * v + M[1, 2]; Z = M[2, 0] * u + M[2, 1] * v + M[2, 2]
  x = X / Z; y = Y / Z
  x0 = np.floor(x).astype(np.int64); y0 = np.floor(y).astype(np.int64); fx = x - x0; fy = y - y0
  h, w = img.shape
  def px(yy, xx):
    ok = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
    out = np.zeros(xx.shape, np.float64); out[ok] = img[yy[ok], xx[ok]]; return out
  val = (px(y0, x0) * (1 - fx) * (1 - fy) + px(y0, x0 + 1) * fx * (1 - fy) +
         px(y0 + 1, x0) * (1 - fx) * fy + px(y0 + 1, x0 + 1) * fx * fy)
  return np.clip(np.rint(val), 0, 255).astype(np.float32)


def model_input(yuv, M_y, M_uv):
  Y = yuv[:W_CAM * H_CAM].reshape(H_CAM, W_CAM)
  U = yuv[W_CAM * H_CAM:W_CAM * H_CAM * 5 // 4].reshape(H_CAM // 2, W_CAM // 2)
  V = yuv[W_CAM * H_CAM * 5 // 4:].reshape(H_CAM // 2, W_CAM // 2)
  y = warp_plane(Y, M_y, MW, MH); u = warp_plane(U, M_uv, MW // 2, MH // 2); v = warp_plane(V, M_uv, MW // 2, MH // 2)
  return np.stack([y[0::2, 0::2], y[1::2, 0::2], y[0::2, 1::2], y[1::2, 1::2], u, v])


def uv_matrix(M):
  s = 0.5
  t_out = np.array([[1 / s, 0, 0.5], [0, 1 / s, 0.5], [0, 0, 1]])
  t_in = np.array([[s, 0, -0.5 * s], [0, s, -0.5 * s], [0, 0, 1]])
  return t_in @ M @ t_out


def frames(path, maxn):
  p = subprocess.Popen(['ffmpeg', '-v', 'error', '-i', path, '-f', 'rawvideo', '-pix_fmt', 'yuv420p', '-'],
                       stdout=subprocess.PIPE)
  n = W_CAM * H_CAM * 3 // 2; i = 0
  while i < maxn:
    b = p.stdout.read(n)
    if len(b) < n: break
    yield np.frombuffer(b, np.uint8); i += 1
  p.kill()


def project(ext, xs, ys):
  pts = K @ ext @ np.vstack([xs, ys, np.zeros_like(xs), np.ones_like(xs)])
  ok = pts[2] > 0.1
  return pts[0] / pts[2], pts[1] / pts[2], ok


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('hevc'); ap.add_argument('--vp', nargs=2, type=float, default=[582.0, 437.0])
  ap.add_argument('--model', default='supercombo.onnx'); ap.add_argument('--frames', type=int, default=600)
  ap.add_argument('--out', default='replay083.mp4'); ap.add_argument('--every', type=int, default=1, help='draw every Nth frame')
  ap.add_argument('--rhd', action='store_true')
  a = ap.parse_args()
  import onnxruntime as ort
  from PIL import Image, ImageDraw

  ext, pitch, yaw = extrinsic_from_vp(*a.vp)
  cam_from_ground = (K @ ext)[:, [0, 1, 3]]
  M_y = cam_from_ground @ GROUND_FROM_MEDMODEL; M_uv = uv_matrix(M_y)
  print('calibration: pitch %.2f deg, yaw %.2f deg' % (pitch, yaw))

  sess = ort.InferenceSession(a.model, providers=['CPUExecutionProvider'])
  state = np.zeros((1, 512), np.float32); desire = np.zeros((1, 8), np.float32)
  tc = np.array([[0, 1]] if a.rhd else [[1, 0]], np.float32)
  raw_path = a.out + '.raw.bin'; prev = None; kept = []
  with open(raw_path, 'wb') as fr:
    for i, yuv in enumerate(frames(a.hevc, a.frames)):
      cur = model_input(yuv, M_y, M_uv)
      if prev is None: prev = cur
      out = sess.run(None, {'input_imgs': np.concatenate([prev, cur])[None].astype(np.float32), 'desire': desire,
                            'traffic_convention': tc, 'initial_state': state})[0]
      state = out[:, -512:].copy(); prev = cur
      out[0].astype(np.float32).tofile(fr)
      if i % a.every == 0: kept.append((i, yuv.copy()))
      if i % 100 == 0: print('frame', i, flush=True)
  dec_bin = os.path.join(HERE, 'replay_decode')
  if not os.path.exists(dec_bin):
    subprocess.check_call(['g++', '-std=c++11', '-O2', os.path.join(HERE, 'replay_decode.cc'), '-o', dec_bin])
  dec_path = a.out + '.dec.bin'
  subprocess.check_call([dec_bin, raw_path, dec_path])
  dec = np.fromfile(dec_path, np.float32).reshape(-1, 581)

  xs = np.arange(192, dtype=np.float64)
  enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '%dx%d' % (W_CAM, H_CAM),
                          '-r', str(20 // a.every), '-i', '-', '-pix_fmt', 'yuv420p', '-vf', 'scale=874:-2', a.out],
                         stdin=subprocess.PIPE)
  stats = []
  for i, yuv in kept:
    d = dec[i]; path, ll, rl = d[0:192], d[192:384], d[384:576]; lp, rp, ldp, ldd, ldy = d[576:581]
    stats.append((lp, rp, ll[0], rl[0], path[0], np.interp(30, xs, path)))
    Y = yuv[:W_CAM * H_CAM].reshape(H_CAM, W_CAM).astype(np.float32)
    U = np.repeat(np.repeat(yuv[W_CAM * H_CAM:W_CAM * H_CAM * 5 // 4].reshape(H_CAM // 2, W_CAM // 2), 2, 0), 2, 1).astype(np.float32) - 128
    V = np.repeat(np.repeat(yuv[W_CAM * H_CAM * 5 // 4:].reshape(H_CAM // 2, W_CAM // 2), 2, 0), 2, 1).astype(np.float32) - 128
    rgb = np.clip(np.stack([Y + 1.402 * V, Y - 0.344 * U - 0.714 * V, Y + 1.772 * U], -1), 0, 255).astype(np.uint8)
    im = Image.fromarray(rgb); dr = ImageDraw.Draw(im, 'RGBA')
    m = xs <= 100
    for ys, col, w, pr in ((ll, (255, 220, 40), 5, lp), (rl, (255, 220, 40), 5, rp)):
      px, py, ok = project(ext, xs[m], ys[m]); ok &= ~np.isnan(px)
      if ok.sum() > 1: dr.line(list(zip(px[ok], py[ok])), fill=col + (int(60 + 195 * pr),), width=w)
    for off in (-0.9, 0.9):
      px, py, ok = project(ext, xs[m], path[m] + off)
      if ok.sum() > 1: dr.line(list(zip(px[ok], py[ok])), fill=(40, 130, 255, 230), width=4)
    if ldp > 0.5 and ldd > 1:
      bx, by, ok = project(ext, np.array([ldd]), np.array([ldy]))
      if ok[0]: dr.rectangle([bx[0] - 30, by[0] - 40, bx[0] + 30, by[0]], outline=(255, 60, 60, 255), width=4)
    dr.rectangle([0, 0, 640, 34], fill=(0, 0, 0, 170))
    dr.text((10, 10), 'frame %d  L %+.2fm p%.2f  R %+.2fm p%.2f  path %+.2f / at 30m %+.2f  lead p%.2f' %
            (i, ll[0], lp, rl[0], rp, path[0], np.interp(30, xs, path), ldp), fill=(255, 255, 255, 255))
    enc.stdin.write(im.tobytes())
  enc.stdin.close(); enc.wait()
  s = np.array(stats)
  print('wrote %s (%d frames drawn)' % (a.out, len(kept)))
  print('lane prob  median L %.2f R %.2f | lane y at 0 m median L %+.2f R %+.2f (width %.2f) | path median %+.2f' %
        (np.median(s[:, 0]), np.median(s[:, 1]), np.median(s[:, 2]), np.median(s[:, 3]), np.median(s[:, 2] - s[:, 3]), np.median(s[:, 4])))


if __name__ == '__main__':
  main()
