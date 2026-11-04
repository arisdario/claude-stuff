#!/usr/bin/env python3
"""Before/after comparison of C920 image profiles on the EON, with openpilot running (read-only except for the profile mode file that radarweb also uses).

  python3 camera_ab.py ab   --a off --b night --period 8 --rounds 6 --out /data/camera_ab/run1   # alternate A and B on the SAME scene, save frames + model outputs
  python3 camera_ab.py capture --label A --seconds 30 --out DIR                                  # just record what the camera does now
  python3 camera_ab.py compare DIR_A DIR_B [--out report_dir]                                    # metrics + side-by-side html (works on a PC)

Interleaving A and B every few seconds on the same road/scene is what makes the comparison fair: a profile recorded on another day or in other light is not comparable.
Metrics (scene-independent where possible; a brighter picture is NOT scored as better):
  brightness mean/std, histogram percentiles p1/p50/p99, % clipped white/black, noise sigma (residual in flat areas), sharpness (Laplacian variance), directional motion-blur
  ratio, saturation, colour-light separation (bright saturated pixels: how many keep their colour vs clip to white, red/green hue distance), dark-region local contrast
  (pedestrians in shadow), and the model's own outputs captured at the same time: lane probabilities, path offset stability.
Needs numpy + PIL (present on the EON image used for the diagnostics; if not, run `compare` on a PC). Model capture needs cereal (EON only).
"""
import glob
import json
import math
import os
import sys
import time

LIVE = '/dev/c920_live.jpg'
STATUS = '/data/camera_status.json'
MODE = '/data/camera_profile'


# ------------------------------------------------------------------ metrics (pure numpy; tested with synthetic images)
def load_rgb(path_or_bytes):
  import io
  import numpy as np
  from PIL import Image
  im = Image.open(io.BytesIO(path_or_bytes) if isinstance(path_or_bytes, (bytes, bytearray)) else path_or_bytes).convert('RGB')
  return np.asarray(im).astype('float32')


def metrics(rgb):
  import numpy as np
  r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
  y = 0.299 * r + 0.587 * g + 0.114 * b
  mx, mn = rgb.max(axis=2), rgb.min(axis=2)
  sat = (mx - mn) / np.maximum(mx, 1.0)
  out = {'mean': float(y.mean()), 'std': float(y.std())}
  for p in (1, 50, 99):
    out['p%d' % p] = float(np.percentile(y, p))
  out['clip_white_pct'] = float(100 * (mx >= 250).mean())
  out['clip_black_pct'] = float(100 * (y <= 5).mean())
  # noise: std of (pixel - 3x3 mean) over flat blocks (lowest-gradient 20% of 16x16 blocks)
  h, w = y.shape
  k = 16
  yb = y[:h // k * k, :w // k * k]
  blur = (np.roll(yb, 1, 0) + np.roll(yb, -1, 0) + np.roll(yb, 1, 1) + np.roll(yb, -1, 1) + yb) / 5.0
  res = (yb - blur)[1:-1, 1:-1]
  gx = np.abs(np.diff(yb, axis=1))[1:-1, :-1][:res.shape[0], :res.shape[1]]
  bh, bw = res.shape[0] // k, res.shape[1] // k
  res_b = res[:bh * k, :bw * k].reshape(bh, k, bw, k).transpose(0, 2, 1, 3).reshape(bh, bw, -1)
  gr_b = gx[:bh * k, :bw * k].reshape(bh, k, bw, k).transpose(0, 2, 1, 3).reshape(bh, bw, -1).mean(axis=2)
  lum_b = yb[1:-1, 1:-1][:bh * k, :bw * k].reshape(bh, k, bw, k).transpose(0, 2, 1, 3).reshape(bh, bw, -1).mean(axis=2)
  ok = (lum_b > 8) & (lum_b < 240)
  thr = np.percentile(gr_b[ok], 20) if ok.any() else 0
  flat = ok & (gr_b <= thr)
  out['noise_sigma'] = float(res_b[flat].std()) if flat.any() else float('nan')
  lap = y[1:-1, 1:-1] * 4 - y[:-2, 1:-1] - y[2:, 1:-1] - y[1:-1, :-2] - y[1:-1, 2:]
  out['sharpness_lapvar'] = float(lap.var())
  out['blur_dir_ratio'] = float(np.abs(np.diff(y, axis=1)).mean() / max(1e-6, np.abs(np.diff(y, axis=0)).mean()))   # >>1: vertical edges smeared horizontally = motion blur / panning
  out['saturation_mean'] = float(sat.mean())
  # colour lights: bright pixels (V>=0.75) -> split into 'keeps colour' (sat>=0.35) and 'clipped to white-ish' (sat<0.15 and V>=0.98)
  bright = mx >= 190
  nb = int(bright.sum())
  out['bright_px'] = nb
  if nb >= 20:
    col = bright & (sat >= 0.35)
    wht = bright & (sat < 0.15) & (mx >= 250)
    out['light_colour_kept_pct'] = float(100 * col.sum() / nb)
    out['light_clipped_white_pct'] = float(100 * wht.sum() / nb)
    # hue of the coloured light pixels; red (hue ~0/360) vs green (~120): circular distance of the two dominant clusters
    hue = np.zeros_like(mx)
    d = np.maximum(mx - mn, 1e-6)
    hr = ((g - b) / d) % 6
    hg = (b - r) / d + 2
    hb = (r - g) / d + 4
    hue = np.where(mx == r, hr, np.where(mx == g, hg, hb)) * 60.0
    hc = hue[col]
    if hc.size >= 10:
      red = ((hc < 25) | (hc > 335)).sum(); grn = ((hc > 80) & (hc < 180)).sum(); amb = ((hc >= 25) & (hc < 70)).sum()
      out['light_red_px'], out['light_green_px'], out['light_amber_px'] = int(red), int(grn), int(amb)
  # dark-region local contrast: std of luma inside the darkest 25% of blocks (pedestrians / road users in shadow)
  if ok.any():
    dark = lum_b <= np.percentile(lum_b[ok], 25)
    blk = yb[1:-1, 1:-1][:bh * k, :bw * k].reshape(bh, k, bw, k).transpose(0, 2, 1, 3).reshape(bh, bw, -1)
    out['dark_local_contrast'] = float(blk[dark & ok].std(axis=1).mean()) if (dark & ok).any() else float('nan')
  return out


def histogram(rgb, bins=32):
  import numpy as np
  y = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
  return np.histogram(y, bins=bins, range=(0, 256))[0].astype(float) / y.size


# ------------------------------------------------------------------ capture on the EON
def read_json(p):
  try:
    return json.load(open(p))
  except Exception:
    return None


def set_mode(mode, settle=4.0, timeout=20.0):
  """same files radarweb writes. Returns the status after camerad confirmed (or None)."""
  st0 = read_json(STATUS) or {}
  n0 = st0.get('apply_count', 0)
  tmp = MODE + '.tmp'
  open(tmp, 'w').write(mode + '\n')
  os.replace(tmp, MODE)
  end = time.time() + timeout
  while time.time() < end:
    time.sleep(0.5)
    st = read_json(STATUS)
    if st and st.get('mode') == mode and (mode == 'off' or st.get('apply_count', 0) != n0 or st.get('active') in ('day', 'night')):
      time.sleep(settle)                       # let the camera's auto exposure settle on the new settings
      return read_json(STATUS)
  return None


def capture(label, seconds, out, with_model=True):
  os.makedirs(out, exist_ok=True)
  fdir = os.path.join(out, 'frames')
  os.makedirs(fdir, exist_ok=True)
  meta = {'label': label, 'start': time.time(), 'status_before': read_json(STATUS)}
  sock = None
  if with_model:
    try:
      sys.path.insert(0, '/data/openpilot')
      import cereal.messaging as m
      sock = (m, m.sub_sock('model', conflate=False))
    except Exception as e:
      meta['model_error'] = str(e)
  last_m, n = None, 0
  model = []
  end = time.time() + seconds
  nxt = 0
  while time.time() < end:
    now = time.time()
    if now >= nxt:
      try:
        st = os.stat(LIVE)
        if st.st_mtime != last_m:
          b = open(LIVE, 'rb').read()
          if len(b) > 1000 and b[:2] == b'\xff\xd8' and b[-2:] == b'\xff\xd9':
            last_m = st.st_mtime
            open(os.path.join(fdir, 'f_%03d_%d.jpg' % (n, int(now * 1000))), 'wb').write(b)
            n += 1
            nxt = now + 0.5
      except OSError:
        pass
    if sock:
      for ev in sock[0].drain_sock(sock[1], wait_for_one=False):
        md = ev.model
        try:
          model.append({'t': now, 'path': [float(x) for x in md.path.poly], 'll': float(md.leftLane.prob), 'rl': float(md.rightLane.prob),
                        'lpoly': [float(x) for x in md.leftLane.poly], 'rpoly': [float(x) for x in md.rightLane.poly]})
        except Exception:
          pass
    time.sleep(0.02)
  meta['status_after'] = read_json(STATUS)
  meta['frames'] = n
  json.dump(meta, open(os.path.join(out, 'meta.json'), 'w'), indent=1)
  with open(os.path.join(out, 'model.jsonl'), 'w') as f:
    for r in model:
      f.write(json.dumps(r) + '\n')
  return n, len(model)


def ab(a, b, period, rounds, out):
  settings = {}
  for rd in range(rounds):
    for lab, mode in (('A', a), ('B', b)):
      st = set_mode(mode)
      if st is None:
        print('camerad did not confirm mode %s - is the profile build installed and the camera streaming? stopping' % mode)
        return 1
      settings[lab] = {'mode': mode, 'status': st}
      n, nm = capture(lab, period, os.path.join(out, lab, 'r%02d' % rd))
      print('round %d %s (%s): %d frames, %d model msgs' % (rd, lab, mode, n, nm))
  json.dump(settings, open(os.path.join(out, 'settings.json'), 'w'), indent=1)
  print('done. python3 camera_ab.py compare %s/A %s/B' % (out, out))
  return 0


# ------------------------------------------------------------------ comparison
def collect(d):
  frames = sorted(glob.glob(os.path.join(d, '**', 'frames', '*.jpg'), recursive=True))
  ms, hist = [], None
  for f in frames:
    try:
      rgb = load_rgb(f)
    except Exception:
      continue
    m = metrics(rgb)
    ms.append(m)
    h = histogram(rgb)
    hist = h if hist is None else hist + h
  model = []
  for f in glob.glob(os.path.join(d, '**', 'model.jsonl'), recursive=True):
    for line in open(f):
      try:
        model.append(json.loads(line))
      except ValueError:
        pass
  return frames, ms, (hist / len(ms) if ms and hist is not None else None), model


def agg(ms, key):
  v = [m[key] for m in ms if key in m and not (isinstance(m[key], float) and math.isnan(m[key]))]
  return (sum(v) / len(v), min(v), max(v), len(v)) if v else None


def model_stats(model):
  if not model:
    return {}
  c0 = [r['path'][3] for r in model if len(r['path']) == 4]
  mean = sum(c0) / len(c0)
  return {'msgs': len(model), 'lane_left_prob_mean': sum(r['ll'] for r in model) / len(model), 'lane_right_prob_mean': sum(r['rl'] for r in model) / len(model),
          'lanes_both_gt_0.5_pct': 100.0 * sum(1 for r in model if r['ll'] > 0.5 and r['rl'] > 0.5) / len(model),
          'path_c0_std_m': math.sqrt(sum((x - mean) ** 2 for x in c0) / len(c0)),
          'path_c0_jitter_m': (sum(abs(a - b) for a, b in zip(c0, c0[1:])) / max(1, len(c0) - 1))}


KEYS = [('mean', 'brightness mean'), ('std', 'brightness std (contrast)'), ('p1', 'luma p1 (shadows)'), ('p50', 'luma median'), ('p99', 'luma p99 (highlights)'),
        ('clip_white_pct', '% clipped white'), ('clip_black_pct', '% crushed black'), ('noise_sigma', 'noise sigma (flat areas)'), ('sharpness_lapvar', 'sharpness (Laplacian var)'),
        ('blur_dir_ratio', 'horizontal-blur ratio (1 = isotropic)'), ('saturation_mean', 'saturation mean'), ('light_colour_kept_pct', 'bright lights keeping colour %'),
        ('light_clipped_white_pct', 'bright lights clipped to white %'), ('dark_local_contrast', 'local contrast in dark areas')]


def compare(da, db, out=None):
  fa, ma, ha, mda = collect(da)
  fb, mb, hb, mdb = collect(db)
  L = ['CAMERA A/B COMPARISON  A=%s (%d frames)  B=%s (%d frames)' % (da, len(fa), db, len(fb)),
       'A brighter picture is NOT scored as better. Read each row against what you need: lane/edge contrast, no clipped lights, low noise, low blur.', '']
  L.append('%-40s %12s %12s' % ('metric', 'A mean', 'B mean'))
  for k, lab in KEYS:
    a, b = agg(ma, k), agg(mb, k)
    L.append('%-40s %12s %12s' % (lab, '%.2f' % a[0] if a else '-', '%.2f' % b[0] if b else '-'))
  L.append('')
  sa, sb = model_stats(mda), model_stats(mdb)
  L.append('MODEL OUTPUTS captured with the frames (same time):')
  for k in sorted(set(sa) | set(sb)):
    L.append('%-40s %12s %12s' % (k, '%.3f' % sa[k] if k in sa else '-', '%.3f' % sb[k] if k in sb else '-'))
  L.append('')
  L.append('Limits: lane/road-edge visibility and pedestrians are judged through the model outputs and the dark-area contrast; traffic-light colour separation needs lights in the')
  L.append('scene (rows are empty otherwise). Frames of both sets are in the html so you can look at them.')
  text = '\n'.join(L) + '\n'
  print(text)
  if out:
    os.makedirs(out, exist_ok=True)
    open(os.path.join(out, 'compare.txt'), 'w').write(text)
    import shutil
    rows = []
    for i in range(min(8, len(fa), len(fb))):
      ia, ib = fa[i * len(fa) // min(8, len(fa))], fb[i * len(fb) // min(8, len(fb))]
      shutil.copy(ia, os.path.join(out, 'A_%d.jpg' % i)); shutil.copy(ib, os.path.join(out, 'B_%d.jpg' % i))
      rows.append('<tr><td><img src="A_%d.jpg" width=480></td><td><img src="B_%d.jpg" width=480></td></tr>' % (i, i))
    open(os.path.join(out, 'index.html'), 'w').write('<!doctype html><meta charset=utf-8><title>A/B</title><pre>%s</pre><table><tr><th>A</th><th>B</th></tr>%s</table>' % (text, ''.join(rows)))
  return text


if __name__ == '__main__':
  a = sys.argv[1:]
  if not a:
    print(__doc__); sys.exit(1)
  arg = lambda n, d=None: a[a.index(n) + 1] if n in a else d
  if a[0] == 'compare':
    compare(a[1], a[2], arg('--out'))
  elif a[0] == 'capture':
    print(capture(arg('--label', 'X'), float(arg('--seconds', 30)), arg('--out', '/data/camera_ab/capture')))
  elif a[0] == 'ab':
    sys.exit(ab(arg('--a', 'off'), arg('--b', 'night'), float(arg('--period', 8)), int(arg('--rounds', 6)), arg('--out', '/data/camera_ab/run')))
  else:
    print(__doc__)
