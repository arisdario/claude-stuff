import os, sys, tempfile, io, json
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'tools'))
import camera_ab as ab

rng = np.random.RandomState(1)


def base():
  img = np.full((360, 640, 3), 90.0, np.float32)
  img[200:, :] = 60
  img[250:300, 300:340] = 200           # a bright patch (lane marking)
  return img


def test_noise_detected_and_brightness_not_equated_with_quality():
  clean = base()
  noisy = np.clip(clean + rng.normal(0, 12, clean.shape), 0, 255)
  mc, mn = ab.metrics(clean), ab.metrics(noisy)
  assert mn['noise_sigma'] > mc['noise_sigma'] + 5
  bright = np.clip(clean * 1.8 + 30, 0, 255)
  mb = ab.metrics(bright)
  assert mb['mean'] > mc['mean'] and mb['clip_white_pct'] >= mc['clip_white_pct']       # brighter is reported as brighter, with its clipping, not as 'better'


def test_motion_blur_and_sharpness():
  from scipy.ndimage import uniform_filter1d
  sharp = base()
  sharp[:, ::8] = 200
  blur = uniform_filter1d(sharp, 15, axis=1)
  ms, mb = ab.metrics(sharp), ab.metrics(blur)
  assert mb['sharpness_lapvar'] < ms['sharpness_lapvar'] / 2
  assert mb['blur_dir_ratio'] < ms['blur_dir_ratio']          # horizontal smear lowers the horizontal gradient relative to the vertical one


def test_traffic_light_colour_vs_clipped_white():
  img = np.full((360, 640, 3), 20.0, np.float32)
  img[50:70, 100:120] = (255, 40, 40)      # red light keeps colour
  img[50:70, 300:320] = (40, 255, 60)      # green light keeps colour
  m = ab.metrics(img)
  assert m['light_colour_kept_pct'] > 90 and m['light_red_px'] > 100 and m['light_green_px'] > 100
  img2 = np.full((360, 640, 3), 20.0, np.float32)
  img2[40:80, 90:130] = 255; img2[40:80, 290:330] = 255          # same lights blown out to white
  m2 = ab.metrics(img2)
  assert m2['light_clipped_white_pct'] > 90 and m2['light_colour_kept_pct'] < 10


def test_compare_runs_on_saved_frames():
  from PIL import Image
  d = tempfile.mkdtemp()
  for lab, gain in (('A', 1.0), ('B', 1.6)):
    os.makedirs(os.path.join(d, lab, 'frames'))
    for i in range(3):
      im = np.clip(base() * gain + rng.normal(0, 3, (360, 640, 3)), 0, 255).astype(np.uint8)
      Image.fromarray(im).save(os.path.join(d, lab, 'frames', 'f_%03d.jpg' % i))
    with open(os.path.join(d, lab, 'model.jsonl'), 'w') as f:
      for i in range(30):
        f.write(json.dumps({'t': i, 'path': [0, 0, 0, 0.1 * (i % 2)], 'll': 0.8, 'rl': 0.7, 'lpoly': [0, 0, 0, 1.6], 'rpoly': [0, 0, 0, -1.6]}) + '\n')
  text = ab.compare(os.path.join(d, 'A'), os.path.join(d, 'B'), os.path.join(d, 'out'))
  assert 'brightness mean' in text and 'MODEL OUTPUTS' in text and os.path.exists(os.path.join(d, 'out', 'index.html'))
