#!/usr/bin/env python3
"""
yt_replay.py - OFFLINE VISUAL REPLAY of any driving video through our existing 0.8.3 supercombo model, using the same
C920 preprocessing our camerad uses. Read-only experiment: it changes no model, planner, controller, calibration or
vehicle parameter and sends nothing anywhere.

  python3 yt_replay.py URL        --model supercombo_083.onnx --out run1 [--start 60 --duration 120]
  python3 yt_replay.py --video f.mp4 --model supercombo_083.onnx --out run1 [--src-hfov 110 --vp-frac 0.5 0.47 --fps 20]

Reuses replay083.py (warp, calibration maths, projection) and replay_decode (the EON's C++ adapter) - the model
inference pipeline is not duplicated. This tool adds: video intake, the exact C920 packing, std / road-edge / timing
capture, detectors, overlay video, time-series plot, CSV/JSON and a report.

WHAT THIS IS NOT: a video has no speed, steering angle or IMU - none are invented (desire = 0, no pose used). The
source camera's field of view, mounting and horizon are unknown unless you give them: results show how the 0.8.3 model
behaves on that footage, NOT how our car or our C920 behaves.
"""
import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import replay083 as R  # noqa: E402  (warp_plane, model_input, uv_matrix, extrinsic_from_vp, project, K ...)

# ---- our C920 path (tesla/camerad/cameras/camera_ext_c920.cc): 1280x720 MJPEG, crop 58 px per side, 77 px black rows
C920_W, C920_H = 1280, 720
CROP_X, PAD_Y = 58, 77
OUT_W, OUT_H = R.W_CAM, R.H_CAM                      # 1164x874 canvas = the model's camera frame
C920_HFOV_DEG = 70.42                                # datasheet (unmeasured, distortion uncorrected)
C920_FPS = 20.
# ---- 0.8.3 output layout (same as model083_adapter.h)
PLAN_G, PLAN_V, PLAN_N = 2 * 990 + 1, 990, 5
LL = PLAN_N * PLAN_G
LLP = LL + 4 * 2 * 2 * 33
RE = LLP + 4
X_IDXS = np.array([0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75, 22.6875, 27., 31.6875,
                   36.75, 42.1875, 48., 54.1875, 60.75, 67.6875, 75., 82.6875, 90.75, 99.1875, 108., 117.1875, 126.75,
                   136.6875, 147., 157.6875, 168.75, 180.1875, 192.])
CSV_COLS = ['frame', 't', 'dt', 'prep_ms', 'infer_ms', 'luma_mean', 'dark_frac', 'lane_prob_l', 'lane_prob_r',
            'lead_prob', 'path_y_0', 'path_y_10', 'path_y_30', 'path_y_60', 'path_std_10', 'path_std_30',
            'path_std_60', 'plan_reach_m', 'laneL_y0', 'laneR_y0', 'laneL_y30', 'laneR_y30', 'laneL_std0',
            'laneR_std0', 'lane_width0', 'edgeL_y0', 'edgeR_y0', 'edgeL_std', 'edgeR_std']

DEFAULT_THR = {'shift_30m': 0.5,        # m change of the path at 30 m between consecutive frames (scaled to 50 ms)
               'shift_10m': 0.3,        # same at 10 m
               'prob_jump': 0.4,        # lane probability change between consecutive frames
               'prob_flicker_n': 4,     # crossings of 0.5 inside flicker_window_s
               'flicker_window_s': 5.0,
               'hyst_lo': 0.4, 'hyst_hi': 0.6,   # probabilities hovering around 0.5 are not counted as flicker
               'std30_high': 1.0,       # m predicted path std at 30 m
               'reach_low': 25.0,       # m the model plan reaches (short plan = low confidence / slow)
               'lane_lost': 0.25,       # both lane probabilities below this
               'merge_gap_s': 0.5}


class DownloadBlocked(Exception):
  pass


# ------------------------------------------------------------------------------------------------------ intake
def download(url, outdir, start, duration, cookies=None):
  """yt-dlp, one attempt, no workarounds. YouTube's bot check is reported, never bypassed."""
  os.makedirs(outdir, exist_ok=True)
  try:
    import yt_dlp  # noqa: F401
    base = [sys.executable, '-m', 'yt_dlp']
  except ImportError:
    base = ['yt-dlp']
  cmd = base + ['--no-playlist', '-f', 'bv*[height<=1080][ext=mp4]/bv*[height<=1080]/b[height<=1080]',
                '-o', os.path.join(outdir, 'source.%(ext)s')]
  if duration:
    cmd += ['--download-sections', '*%g-%g' % (start, start + duration)]
  if cookies:
    cmd += ['--cookies', cookies]
  p = subprocess.run(cmd + [url], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
  txt = p.stdout or ''
  if p.returncode != 0:
    if 'Sign in to confirm' in txt or 'not a bot' in txt:
      raise DownloadBlocked('YouTube asked for sign-in (bot check) and refused the download. This tool does not work '
                            'around that. Options: pass --cookies cookies.txt exported from your own browser, or '
                            'download the video yourself and pass --video FILE.')
    raise DownloadBlocked('yt-dlp failed:\n' + txt[-800:])
  for f in sorted(os.listdir(outdir)):
    if f.startswith('source.') and not f.endswith('.part'):
      return os.path.join(outdir, f)
  raise DownloadBlocked('yt-dlp produced no file:\n' + txt[-800:])


def probe(path):
  out = subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
                                 'stream=width,height,r_frame_rate,avg_frame_rate,codec_name,duration,nb_frames',
                                 '-of', 'json', path]).decode()
  s = json.loads(out)['streams'][0]
  def fr(x):
    a, b = x.split('/'); return float(a) / float(b) if float(b) else 0.
  return {'w': int(s['width']), 'h': int(s['height']), 'r_fps': fr(s['r_frame_rate']), 'avg_fps': fr(s['avg_frame_rate']),
          'codec': s.get('codec_name'), 'duration': float(s.get('duration') or 0.)}


# ------------------------------------------------------------------------------------------------------ geometry
def source_geometry(w, h, src_hfov=None, y_frac=0.5):
  """crop of the source that plays the role of the C920's 1280x720 picture: 16:9, and (if the source field of view is
  given) narrowed to the C920's horizontal field of view. Returns the crop and everything we must report."""
  cw, ch = w, h
  if w * 9 > h * 16:
    cw = int(h * 16 / 9) // 2 * 2
  else:
    ch = int(w * 9 / 16) // 2 * 2
  note = []
  if w * 9 != h * 16:
    note.append('source aspect %dx%d is not 16:9: centre-cropped to %dx%d' % (w, h, cw, ch))
  fov_applied = False
  if src_hfov:
    f = math.tan(math.radians(C920_HFOV_DEG / 2)) / math.tan(math.radians(src_hfov / 2))
    if f < 1.:
      cw = int(cw * f) // 2 * 2; ch = int(ch * f) // 2 * 2; fov_applied = True
      note.append('source HFOV %.0f deg narrowed to the C920 %.1f deg by a %.0f%% crop (lens distortion NOT corrected)' % (
          src_hfov, C920_HFOV_DEG, f * 100))
    else:
      note.append('source HFOV %.0f deg is narrower than the C920 %.1f deg: cannot widen, used as is' % (src_hfov, C920_HFOV_DEG))
  else:
    note.append('source field of view UNKNOWN: assumed equal to the C920 (not verified - geometry may be wrong)')
  cx = (w - cw) // 2; cy = int((h - ch) * y_frac) // 2 * 2
  scale = C920_W / float(cw)
  if scale > 1.0:
    note.append('source crop is only %d px wide: UPSCALED %.2fx to 1280 (softer than a real C920 frame)' % (cw, scale))
  ok_res = cw >= 960
  if not ok_res:
    note.append('INSUFFICIENT RESOLUTION: crop under 960 px wide, results are not meaningful')
  return {'cw': cw, 'ch': ch, 'cx': cx, 'cy': cy, 'scale': scale, 'fov_applied': fov_applied, 'res_ok': ok_res, 'notes': note}


def canvas_to_source(px, py, g):
  """canvas pixel (1164x874) -> source video pixel"""
  sx, sy = px + CROP_X, py - PAD_Y
  return g['cx'] + sx * g['cw'] / float(C920_W), g['cy'] + sy * g['ch'] / float(C920_H)


def vp_canvas(vp_frac):
  """vanishing point given as a fraction of the C920-equivalent (cropped) picture -> canvas pixels"""
  return vp_frac[0] * C920_W - CROP_X, vp_frac[1] * C920_H + PAD_Y


def vf_filter(g, fps=None):
  f = ('fps=%g,' % fps) if fps else ''
  return '%scrop=%d:%d:%d:%d,scale=%d:%d:flags=bicubic' % (f, g['cw'], g['ch'], g['cx'], g['cy'], C920_W, C920_H)


# ------------------------------------------------------------------------------------------------------ preprocessing
def rgb_to_i420(rgb):
  """camerad rgb_to_yuv.cl: BT.601 limited range, 2x2 chroma average -> flat I420 (Y, U, V)"""
  r, g, b = [rgb[..., i].astype(np.float32) for i in range(3)]
  Y = 0.257 * r + 0.504 * g + 0.098 * b + 16
  U = -0.148 * r - 0.291 * g + 0.439 * b + 128
  V = 0.439 * r - 0.368 * g - 0.071 * b + 128
  c = lambda p: (p[0::2, 0::2] + p[1::2, 0::2] + p[0::2, 1::2] + p[1::2, 1::2]) / 4
  q = lambda p: np.rint(np.clip(p, 0, 255)).astype(np.uint8)
  return np.concatenate([q(Y).ravel(), q(c(U)).ravel(), q(c(V)).ravel()])


def c920_canvas(rgb720):
  """ext_c920_pack_bgr: 1280x720 -> crop 58 px per side -> rows 77..796 of a black 1164x874 canvas (no scaling)"""
  cv = np.zeros((OUT_H, OUT_W, 3), np.uint8)
  cv[PAD_Y:PAD_Y + C920_H] = rgb720[:, CROP_X:CROP_X + OUT_W]
  return cv


def read_rgb_frames(video, start, duration, g, fps, maxn):
  cmd = ['ffmpeg', '-v', 'error']
  if start: cmd += ['-ss', '%g' % start]
  cmd += ['-i', video]
  if duration: cmd += ['-t', '%g' % duration]
  cmd += ['-vf', vf_filter(g, fps), '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']
  p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
  n = C920_W * C920_H * 3; i = 0
  while i < maxn:
    b = p.stdout.read(n)
    if len(b) < n: break
    yield np.frombuffer(b, np.uint8).reshape(C920_H, C920_W, 3); i += 1
  p.kill()


# ------------------------------------------------------------------------------------------------------ model outputs
def plan_best(o):
  sel = [o[(i + 1) * PLAN_G - 1] for i in range(PLAN_N)]
  return o[int(np.argmax(sel)) * PLAN_G:(int(np.argmax(sel)) + 1) * PLAN_G]


def parse_extras(o):
  """what replay_decode does not export: path stds, plan reach, lane stds, road edges (adapter formulas)"""
  pl = plan_best(o)
  px = pl[0:990:30].astype(np.float64); py = -pl[1:990:30].astype(np.float64)
  ps = np.exp(pl[PLAN_V + 1:PLAN_V + 990:30].astype(np.float64))
  for i in range(1, 33):
    if px[i] < px[i - 1] + 1e-3: px[i] = px[i - 1] + 1e-3
  lanes, lstd = [], []
  for k in (1, 2):                                   # inner left / inner right, as the adapter
    ln = o[LL + k * 66:LL + (k + 1) * 66]; sd = o[LL + 66 * (4 + k):LL + 66 * (5 + k)]
    lanes.append(-ln[0::2].astype(np.float64)); lstd.append(np.exp(sd[0::2].astype(np.float64)))
  edges, estd = [], []
  for k in (0, 1):
    e = o[RE + k * 66:RE + (k + 1) * 66]
    edges.append(-e[0::2].astype(np.float64)); estd.append(float(np.exp(o[RE + 66 * (2 + k)])))
  return {'px': px, 'py': py, 'ps': ps, 'lanes': lanes, 'lstd': lstd, 'edges': edges, 'estd': estd}


def sigmoid(x): return 1. / (1. + math.exp(-x))


# ------------------------------------------------------------------------------------------------------ detectors
def _events(flag_idx, t, kind, detail, gap):
  out = []
  if not len(flag_idx): return out
  s = e = flag_idx[0]
  for i in flag_idx[1:]:
    if t[i] - t[e] <= gap: e = i
    else: out.append((s, e)); s = e = i
  out.append((s, e))
  return [{'kind': kind, 't_start': float(t[a]), 't_end': float(t[b]), 'frame_start': int(a), 'frame_end': int(b),
           'detail': detail(a, b)} for a, b in out]


def detect(t, cols, thr=None):
  """cols: dict of arrays (path_y_10, path_y_30, path_std_30, plan_reach_m, lane_prob_l, lane_prob_r).
  Sudden path shifts, unstable lane probabilities, high path uncertainty, lost lanes. Returns a list of events."""
  th = dict(DEFAULT_THR); th.update(thr or {})
  t = np.asarray(t, float); n = len(t); ev = []
  if n < 3: return ev
  dt = np.maximum(np.diff(t, prepend=t[0] - 0.05), 1e-3)
  def rate(x):                                       # change per frame, normalised to a 50 ms frame
    d = np.diff(x, prepend=x[0]); return d * (0.05 / dt)
  for name, key, lim in (('path_shift_30m', 'path_y_30', th['shift_30m']), ('path_shift_10m', 'path_y_10', th['shift_10m'])):
    d = rate(np.asarray(cols[key], float)); idx = np.where(np.abs(d) > lim)[0]
    ev += _events(idx, t, name, lambda a, b, d=d: {'max_jump_m': float(np.max(np.abs(d[a:b + 1])))}, th['merge_gap_s'])
  lp, rp = np.asarray(cols['lane_prob_l'], float), np.asarray(cols['lane_prob_r'], float)
  jump = np.maximum(np.abs(np.diff(lp, prepend=lp[0])), np.abs(np.diff(rp, prepend=rp[0])))
  ev += _events(np.where(jump > th['prob_jump'])[0], t, 'lane_prob_jump',
                lambda a, b: {'max_jump': float(np.max(jump[a:b + 1]))}, th['merge_gap_s'])
  cross = np.zeros(n, bool)
  for p in (lp, rp):                                  # hysteresis: a crossing = from below 0.4 to above 0.6 or back
    st = 1 if p[0] > 0.5 else 0
    for i in range(1, n):
      if st == 0 and p[i] > th['hyst_hi']: st = 1; cross[i] = True
      elif st == 1 and p[i] < th['hyst_lo']: st = 0; cross[i] = True
  fl = []
  for i in range(n):
    w = (t >= t[i] - th['flicker_window_s']) & (t <= t[i])
    if cross[w].sum() >= th['prob_flicker_n']: fl.append(i)
  ev += _events(np.array(fl, int), t, 'lane_prob_flicker',
                lambda a, b: {'crossings_in_window': int(max(cross[(t >= t[j] - th['flicker_window_s']) & (t <= t[j])].sum() for j in range(a, b + 1)))},
                th['merge_gap_s'])
  s30 = np.asarray(cols['path_std_30'], float); reach = np.asarray(cols['plan_reach_m'], float)
  ev += _events(np.where(s30 > th['std30_high'])[0], t, 'high_path_uncertainty',
                lambda a, b: {'max_std30_m': float(np.max(s30[a:b + 1]))}, th['merge_gap_s'])
  ev += _events(np.where(reach < th['reach_low'])[0], t, 'short_plan_reach',
                lambda a, b: {'min_reach_m': float(np.min(reach[a:b + 1]))}, th['merge_gap_s'])
  ev += _events(np.where((lp < th['lane_lost']) & (rp < th['lane_lost']))[0], t, 'both_lanes_lost',
                lambda a, b: {'frames': int(b - a + 1)}, th['merge_gap_s'])
  return sorted(ev, key=lambda e: e['t_start'])


# ------------------------------------------------------------------------------------------------------ rendering
def render_overlay(video, start, duration, g, fps, ext, dec, extras, flagged, out_mp4, eff_fps, t_list, maxn):
  from PIL import Image, ImageDraw
  disp_w = 960
  info = probe(video); sc = disp_w / float(info['w']); disp_h = int(round(info['h'] * sc / 2.)) * 2
  cmd = ['ffmpeg', '-v', 'error']
  if start: cmd += ['-ss', '%g' % start]
  cmd += ['-i', video]
  if duration: cmd += ['-t', '%g' % duration]
  cmd += ['-vf', ('fps=%g,' % fps if fps else '') + 'scale=%d:%d' % (disp_w, disp_h), '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-']
  rd = subprocess.Popen(cmd, stdout=subprocess.PIPE)
  enc = subprocess.Popen(['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', '%dx%d' % (disp_w, disp_h),
                          '-r', '%g' % eff_fps, '-i', '-', '-pix_fmt', 'yuv420p', out_mp4], stdin=subprocess.PIPE)
  xs = np.arange(192, dtype=np.float64); m = xs <= 100
  def to_disp(px, py):
    X, Y = canvas_to_source(px, py, g); return X * sc, Y * sc
  nfr = 0; nb = disp_w * disp_h * 3
  for i in range(min(maxn, len(dec))):
    b = rd.stdout.read(nb)
    if len(b) < nb: break
    im = Image.fromarray(np.frombuffer(b, np.uint8).reshape(disp_h, disp_w, 3).copy()); dr = ImageDraw.Draw(im, 'RGBA')
    d = dec[i]; path, ll, rl = d[0:192], d[192:384], d[384:576]; lp, rp = d[576], d[577]
    for ys, pr in ((ll, lp), (rl, rp)):
      px, py, ok = R.project(ext, xs[m], ys[m]); ok &= ~np.isnan(px)
      if ok.sum() > 1:
        X, Y = to_disp(px[ok], py[ok]); dr.line(list(zip(X, Y)), fill=(255, 220, 40, int(60 + 195 * pr)), width=4)
    for off in (-0.9, 0.9):
      px, py, ok = R.project(ext, xs[m], path[m] + off)
      if ok.sum() > 1:
        X, Y = to_disp(px[ok], py[ok]); dr.line(list(zip(X, Y)), fill=(40, 130, 255, 230), width=3)
    ex = extras[i]; xe = X_IDXS[X_IDXS <= 100]
    for k in (0, 1):
      px, py, ok = R.project(ext, xe, ex['edges'][k][:len(xe)]); ok &= ~np.isnan(px)
      if ok.sum() > 1:
        X, Y = to_disp(px[ok], py[ok]); dr.line(list(zip(X, Y)), fill=(255, 70, 70, 200), width=3)
    dr.rectangle([0, 0, disp_w, 26], fill=(0, 0, 0, 170))
    dr.text((8, 7), 't=%.2fs  lanes L p%.2f R p%.2f  path@30m %+.2f (std %.2f)  edges red  %s' % (
        t_list[i], lp, rp, np.interp(30, xs, path), np.interp(30, ex['px'], ex['ps']), 'FLAG' if flagged[i] else ''),
            fill=(255, 255, 255, 255))
    if flagged[i]: dr.rectangle([0, 0, disp_w - 1, disp_h - 1], outline=(255, 60, 60, 255), width=5)
    enc.stdin.write(im.tobytes()); nfr += 1
  enc.stdin.close(); enc.wait(); rd.kill()
  return nfr


def render_timeseries(t, cols, events, out_png):
  import matplotlib
  matplotlib.use('Agg')
  import matplotlib.pyplot as plt
  fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
  c = {10: '#1f77b4', 30: '#d62728', 60: '#2ca02c'}
  for d_ in (10, 30, 60):
    y = np.asarray(cols['path_y_%d' % d_]); sd = np.asarray(cols['path_std_%d' % d_])
    ax[0].plot(t, y, color=c[d_], lw=1, label='path y at %d m' % d_)
    ax[0].fill_between(t, y - sd, y + sd, color=c[d_], alpha=0.12)
  ax[0].set_ylabel('lateral offset (m, left +)\nband = predicted 1-sigma'); ax[0].legend(loc='upper right', ncol=3)
  ax[1].plot(t, cols['path_std_10'], label='std 10 m'); ax[1].plot(t, cols['path_std_30'], label='std 30 m'); ax[1].plot(t, cols['path_std_60'], label='std 60 m')
  ax[1].set_ylabel('path std (m)'); ax[1].set_yscale('log'); ax[1].legend(loc='upper right', ncol=3)
  ax[2].plot(t, cols['lane_prob_l'], label='left lane prob'); ax[2].plot(t, cols['lane_prob_r'], label='right lane prob')
  ax[2].axhline(0.5, color='grey', lw=0.5); ax[2].set_ylabel('lane probability'); ax[2].legend(loc='upper right', ncol=2)
  ax[3].plot(t, cols['infer_ms'], color='k', lw=0.8, label='inference ms (this PC, CPU)'); ax[3].set_ylabel('ms')
  a2 = ax[3].twinx(); a2.plot(t, cols['luma_mean'], color='orange', lw=0.8); a2.set_ylabel('mean luma (orange)')
  ax[3].set_xlabel('video time (s)')
  col = {'path_shift_30m': 'red', 'path_shift_10m': 'red', 'lane_prob_jump': 'purple', 'lane_prob_flicker': 'purple',
         'high_path_uncertainty': 'orange', 'short_plan_reach': 'orange', 'both_lanes_lost': 'brown'}
  for e in events:
    for a in ax: a.axvspan(e['t_start'], e['t_end'] + 0.05, color=col.get(e['kind'], 'grey'), alpha=0.12)
  fig.suptitle('0.8.3 model on video: VISUAL REPLAY (not vehicle behaviour). shaded = detector events')
  fig.tight_layout(); fig.savefig(out_png, dpi=110); plt.close(fig)


# ------------------------------------------------------------------------------------------------------ main
def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('url', nargs='?'); ap.add_argument('--video'); ap.add_argument('--cookies')
  ap.add_argument('--model', default='supercombo.onnx'); ap.add_argument('--out', default='yt_replay_out')
  ap.add_argument('--start', type=float, default=0.); ap.add_argument('--duration', type=float, default=120.)
  ap.add_argument('--fps', type=float, default=None, help='resample to this rate (e.g. 20 = our C920 rate). Default: keep the native rate')
  ap.add_argument('--src-hfov', type=float, default=None, help='horizontal FOV of the source camera in degrees, if known')
  ap.add_argument('--crop-y-frac', type=float, default=0.5)
  ap.add_argument('--vp-frac', nargs=2, type=float, default=[0.5, 0.5], help='vanishing point as a fraction of the cropped picture (default centre = zero calibration)')
  ap.add_argument('--rhd', action='store_true'); ap.add_argument('--max-frames', type=int, default=4000)
  ap.add_argument('--no-video', action='store_true')
  a = ap.parse_args()
  if not a.url and not a.video: ap.error('give a YouTube URL or --video FILE')
  os.makedirs(a.out, exist_ok=True)
  src = a.video; source_desc = a.video
  if not src:
    try:
      src = download(a.url, a.out, a.start, a.duration, a.cookies)
    except DownloadBlocked as e:
      print('DOWNLOAD NOT POSSIBLE: %s' % e); return 3
    source_desc = a.url
    a.start = 0.                                   # the section was cut at download
  info = probe(src)
  g = source_geometry(info['w'], info['h'], a.src_hfov, a.crop_y_frac)
  fps_eff = a.fps or info['avg_fps'] or 30.
  vfr = abs(info['r_fps'] - info['avg_fps']) > 0.5
  print('source %dx%d %.2f fps (%s); crop %dx%d -> 1280x720 (x%.2f); effective %.2f fps' % (
      info['w'], info['h'], info['avg_fps'], info['codec'], g['cw'], g['ch'], g['scale'], fps_eff))

  import onnxruntime as ort
  sess = ort.InferenceSession(a.model, providers=['CPUExecutionProvider'])
  ins = {i.name: [d for d in i.shape] for i in sess.get_inputs()}
  outs = {o.name: [d for d in o.shape] for o in sess.get_outputs()}
  if ins.get('input_imgs') != [1, 12, 128, 256] or outs.get('outputs') != [1, 11327]:
    print('REFUSING: %s is not the 0.8.3 supercombo (inputs %s outputs %s). Newer models (0.8.11 ...) have a different '
          'layout and normalization and are not supported here.' % (a.model, ins, outs)); return 2
  sha = hashlib.sha256(open(a.model, 'rb').read()).hexdigest()

  vpc = vp_canvas(a.vp_frac)
  ext, pitch, yaw = R.extrinsic_from_vp(*vpc)
  M_y = (R.K @ ext)[:, [0, 1, 3]] @ R.GROUND_FROM_MEDMODEL; M_uv = R.uv_matrix(M_y)
  state = np.zeros((1, 512), np.float32); desire = np.zeros((1, 8), np.float32)
  tc = np.array([[0, 1]] if a.rhd else [[1, 0]], np.float32)
  prev = None; rows = []; raw = []; extras = []; t_list = []
  raw_path = os.path.join(a.out, 'raw_outputs.bin'); fr_raw = open(raw_path, 'wb')
  for i, rgb in enumerate(read_rgb_frames(src, a.start, a.duration, g, a.fps, a.max_frames)):
    t0 = time.perf_counter()
    cv = c920_canvas(rgb); yuv = rgb_to_i420(cv); cur = R.model_input(yuv, M_y, M_uv)
    if prev is None: prev = cur
    inp = np.concatenate([prev, cur])[None].astype(np.float32)
    t1 = time.perf_counter()
    o = sess.run(None, {'input_imgs': inp, 'desire': desire, 'traffic_convention': tc, 'initial_state': state})[0]
    t2 = time.perf_counter()
    state = o[:, -512:].copy(); prev = cur
    o = o[0].astype(np.float32); o.tofile(fr_raw)
    Yp = yuv[:OUT_W * OUT_H].reshape(OUT_H, OUT_W)[PAD_Y:PAD_Y + C920_H]
    ex = parse_extras(o)
    t_list.append(a.start + i / fps_eff); extras.append(ex)
    rows.append({'prep_ms': (t1 - t0) * 1e3, 'infer_ms': (t2 - t1) * 1e3, 'luma_mean': float(Yp.mean()),
                 'dark_frac': float((Yp < 40).mean()), 'o': o})
    if i % 100 == 0: print('frame', i, flush=True)
  fr_raw.close()
  n = len(rows)
  if n < 3:
    print('too few frames decoded'); return 1
  dec_bin = os.path.join(HERE, 'replay_decode')
  if not os.path.exists(dec_bin):
    subprocess.check_call(['g++', '-std=c++11', '-O2', os.path.join(HERE, 'replay_decode.cc'), '-o', dec_bin])
  dec_path = os.path.join(a.out, 'adapter_decoded.bin')
  subprocess.check_call([dec_bin, raw_path, dec_path])
  dec = np.fromfile(dec_path, np.float32).reshape(-1, 581)
  t = np.array(t_list)
  cols = {c: np.zeros(n) for c in CSV_COLS}
  xs = np.arange(192.)
  for i in range(n):
    d = dec[i]; path, ll, rl = d[0:192], d[192:384], d[384:576]; ex = extras[i]; r = rows[i]
    cols['frame'][i] = i; cols['t'][i] = t[i]; cols['dt'][i] = t[i] - t[i - 1] if i else 0.
    cols['prep_ms'][i] = r['prep_ms']; cols['infer_ms'][i] = r['infer_ms']; cols['luma_mean'][i] = r['luma_mean']; cols['dark_frac'][i] = r['dark_frac']
    cols['lane_prob_l'][i], cols['lane_prob_r'][i], cols['lead_prob'][i] = d[576], d[577], d[578]
    for k in (0, 10, 30, 60): cols['path_y_%d' % k][i] = path[k]
    for k in (10, 30, 60): cols['path_std_%d' % k][i] = np.interp(k, ex['px'], ex['ps'])
    cols['plan_reach_m'][i] = min(192., ex['px'][-1])
    cols['laneL_y0'][i], cols['laneR_y0'][i] = ll[0], rl[0]; cols['laneL_y30'][i], cols['laneR_y30'][i] = ll[30], rl[30]
    cols['laneL_std0'][i], cols['laneR_std0'][i] = ex['lstd'][0][0], ex['lstd'][1][0]
    cols['lane_width0'][i] = ll[0] - rl[0]
    cols['edgeL_y0'][i], cols['edgeR_y0'][i] = ex['edges'][0][0], ex['edges'][1][0]; cols['edgeL_std'][i], cols['edgeR_std'][i] = ex['estd']
  dt_med = float(np.median(np.diff(t)))
  events = detect(t, cols)
  flagged = np.zeros(n, bool)
  for e in events: flagged[e['frame_start']:e['frame_end'] + 1] = True

  with open(os.path.join(a.out, 'frames.csv'), 'w') as f:
    f.write(','.join(CSV_COLS) + '\n')
    for i in range(n): f.write(','.join('%.5g' % cols[c][i] for c in CSV_COLS) + '\n')
  np.savez_compressed(os.path.join(a.out, 'arrays.npz'), t=t, path_1m=dec[:, 0:192], lane_left_1m=dec[:, 192:384],
                      lane_right_1m=dec[:, 384:576], plan_x=np.array([e['px'] for e in extras]), plan_y=np.array([e['py'] for e in extras]),
                      plan_std=np.array([e['ps'] for e in extras]), lane_left_33=np.array([e['lanes'][0] for e in extras]),
                      lane_right_33=np.array([e['lanes'][1] for e in extras]), road_edge_left_33=np.array([e['edges'][0] for e in extras]),
                      road_edge_right_33=np.array([e['edges'][1] for e in extras]), x_idxs=X_IDXS)
  try:
    render_timeseries(t, cols, events, os.path.join(a.out, 'timeseries.png'))
  except Exception as e:
    print('timeseries plot skipped: %s' % e)
  nover = 0
  if not a.no_video:
    nover = render_overlay(src, a.start, a.duration, g, a.fps, ext, dec, extras, flagged, os.path.join(a.out, 'overlay.mp4'),
                           fps_eff, t, n)

  def stat(x): x = np.asarray(x); return {'median': float(np.median(x)), 'p5': float(np.percentile(x, 5)), 'p95': float(np.percentile(x, 95))}
  jit30 = float(np.std(np.diff(cols['path_y_30'])))
  summary = {'source': source_desc, 'file': os.path.basename(src), 'frames': n, 'duration_s': float(t[-1] - t[0]),
             'effective_fps': fps_eff, 'median_dt_s': dt_med, 'model': os.path.basename(a.model), 'model_sha256': sha,
             'infer_ms': stat(cols['infer_ms']), 'prep_ms': stat(cols['prep_ms']),
             'lane_prob_l': stat(cols['lane_prob_l']), 'lane_prob_r': stat(cols['lane_prob_r']),
             'path_std_30m': stat(cols['path_std_30']), 'lane_width0_m': stat(cols['lane_width0']),
             'path_30m_frame_to_frame_std_m': jit30, 'luma_mean': stat(cols['luma_mean']),
             'dark_frac_median': float(np.median(cols['dark_frac'])), 'events': events,
             'events_by_kind': {k: sum(1 for e in events if e['kind'] == k) for k in sorted({e['kind'] for e in events})},
             'frames_flagged_pct': float(100. * flagged.mean()), 'thresholds': DEFAULT_THR,
             'geometry': g, 'vp_frac_assumed': a.vp_frac, 'vp_canvas_px': list(vpc), 'calib_pitch_yaw_deg': [pitch, yaw],
             'traffic_convention': 'RHD-traffic flag' if a.rhd else 'default (index 0)', 'variable_frame_rate': bool(vfr)}
  json.dump(summary, open(os.path.join(a.out, 'summary.json'), 'w'), indent=1, default=float)

  L = []
  L.append('# 0.8.3 model: visual replay report')
  L.append('')
  L.append('**VISUAL REPLAY ON A VIDEO. Not vehicle behaviour.** No speed, steering angle or IMU exists in a video and none was '
           'invented (desire input = 0, pose output unused). The source camera is not our C920: its field of view, mounting '
           'height and horizon are unknown unless you supplied them, so absolute path/lane positions are NOT validated; this run '
           'tells you how steady and sensible the 0.8.3 model output is on this footage.')
  L.append('')
  L.append('Source: `%s`, %d frames, %.1f s at %.2f fps (%s). Model `%s` sha256 `%s...`. Raw 0-255 input (no 0.8.11 normalization).' % (
      source_desc, n, t[-1] - t[0], fps_eff, info['codec'], os.path.basename(a.model), sha[:16]))
  L.append('')
  L.append('## Differences from our real C920 input')
  L.append('| | our C920 | this replay |')
  L.append('|---|---|---|')
  L.append('| field of view | 70.4 deg HFOV (datasheet), distortion uncorrected | %s |' % ('source HFOV %.0f deg narrowed to match' % a.src_hfov if g['fov_applied'] else 'UNKNOWN / assumed equal'))
  L.append('| picture | 1280x720 MJPEG, crop 58 px/side, 77 black rows top and bottom, no scaling | source crop %dx%d scaled x%.2f to 1280x720, then the same crop/padding |' % (g['cw'], g['ch'], g['scale']))
  L.append('| compression | MJPEG from the camera | %s (typically heavier, smoother) |' % info['codec'])
  L.append('| frame rate | 20 fps (model expects two frames 50 ms apart) | %.2f fps%s |' % (fps_eff, ' - differs: the model sees a different inter-frame time' if abs(fps_eff - C920_FPS) > 0.5 else ''))
  L.append('| timestamps | camera capture time | nominal i/fps%s |' % (' (variable frame rate detected: nominal times may drift)' if vfr else ''))
  L.append('| vanishing point | from our saved calibration | ASSUMED %.2f, %.2f of the picture -> canvas (%.0f, %.0f); pitch %.2f deg yaw %.2f deg |' % (
      a.vp_frac[0], a.vp_frac[1], vpc[0], vpc[1], pitch, yaw))
  L.append('| inference | EON SNPE DSP/GPU | onnxruntime on CPU: timings below are NOT EON timings |')
  L.append('')
  for nn in g['notes']: L.append('- ' + nn)
  L.append('')
  L.append('## Results')
  L.append('- lane probability: left median %.2f (p5 %.2f), right median %.2f (p5 %.2f)' % (
      summary['lane_prob_l']['median'], summary['lane_prob_l']['p5'], summary['lane_prob_r']['median'], summary['lane_prob_r']['p5']))
  L.append('- predicted lane width at 0 m: median %.2f m (p5 %.2f, p95 %.2f)' % (summary['lane_width0_m']['median'], summary['lane_width0_m']['p5'], summary['lane_width0_m']['p95']))
  L.append('- path std at 30 m: median %.2f m, p95 %.2f m' % (summary['path_std_30m']['median'], summary['path_std_30m']['p95']))
  L.append('- path at 30 m, frame-to-frame change (std): %.3f m' % jit30)
  L.append('- mean luma median %.0f/255, dark-pixel fraction median %.2f (night/low-light indicator)' % (summary['luma_mean']['median'], summary['dark_frac_median']))
  L.append('- inference on this PC (CPU): median %.0f ms; preprocessing median %.0f ms' % (summary['infer_ms']['median'], summary['prep_ms']['median']))
  L.append('')
  L.append('## Detector events (%d, %.1f%% of frames flagged)' % (len(events), summary['frames_flagged_pct']))
  L.append('Thresholds: ' + ', '.join('%s=%g' % kv for kv in DEFAULT_THR.items()))
  for k, v in summary['events_by_kind'].items(): L.append('- %s: %d' % (k, v))
  for e in events[:40]:
    L.append('  - %s  %.1f-%.1f s  %s' % (e['kind'], e['t_start'], e['t_end'], json.dumps(e['detail'])))
  if len(events) > 40: L.append('  - ... %d more in summary.json' % (len(events) - 40))
  L.append('')
  L.append('## Limits')
  L.append('- Cannot say whether a path is *correct* (no ground truth), only whether it is stable and plausible.')
  L.append('- Flagged events at this threshold are leads for looking at overlay.mp4, not failures by themselves.')
  L.append('- No claim that the source geometry matches our C920; a wrong vanishing point biases lane/path offsets.')
  L.append('- Files: frames.csv, arrays.npz (1 m path/lanes, 33-point plan stds, lanes, road edges), summary.json, timeseries.png%s.' % (', overlay.mp4' if nover else ''))
  open(os.path.join(a.out, 'report.md'), 'w').write('\n'.join(L) + '\n')
  print('\n'.join(L[:3] + L[-12:]))
  print('done: %s' % a.out)
  return 0


if __name__ == '__main__':
  sys.exit(main())
