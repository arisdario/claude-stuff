#!/usr/bin/env python3
"""Offline report for lightlog CSVs (run anywhere: python3 lightlog_report.py /data/lightlog/*.csv).

Answers, from the light log alone:
  1. PARAMETERS   when each learned value (steerRatio / stiffnessFactor / angleOffset, valid) was active, how long, and
                  its gain vs a reference ratio (default 15, BogGyver's nominal), because angle = ratio x curvature.
  2. WIGGLE       the 0.2-0.8 Hz steering swing (amplitude, frequency, SNR) per parameter set, ENGAGED (openpilot
                  steering) and MANUAL (you steering) separately.
  3. SHADOW       while you drive: how far the planner's desired angle is from your real angle and how much it jitters
                  (a steady planner while you hold straight = the wiggle is made in the loop, not by the model/planner).
  4. RATIO CHECK  steerRatio implied by your own driving: speed x angle / (wheelbase x yaw rate), no control involved.
  5. DELAY        command -> measured angle lag (engaged only; closed loop, so read it as an estimate, not a spec).
Never changes anything.
"""
import math
import sys

import numpy as np

WHEELBASE = 2.959
REF_RATIO = 15.0
BAND = (0.2, 0.8)
WIN_S = 30.


def load(paths):
  rows, meta = [], []
  cols = None
  for p in paths:
    with open(p) as fh:
      for line in fh:
        line = line.strip()
        if not line:
          continue
        if line.startswith('#'):
          meta.append(line); continue
        if line.startswith('t,'):
          cols = line.split(','); continue
        if cols is None:
          continue
        parts = line.split(',')
        if len(parts) != len(cols):
          continue
        rows.append([float(x) if x != '' else float('nan') for x in parts])
  if not rows:
    return None, meta
  a = np.array(rows)
  d = {c: a[:, i] for i, c in enumerate(cols)}
  return d, meta


def _hz(t):
  dt = np.diff(t)
  dt = dt[dt > 0]
  return 1. / float(np.median(dt)) if len(dt) else 20.


def _highpass(x, hz, seconds=5.):
  k = max(3, int(seconds * hz))
  ker = np.ones(k) / k
  return x - np.convolve(np.pad(x, (k // 2, k - 1 - k // 2), mode='edge'), ker, mode='valid')


def band_stats(t, x):
  """(rms in the wiggle band deg, dominant freq Hz, SNR = peak / median of the band) for a window"""
  hz = _hz(t)
  tu = np.arange(t[0], t[-1], 1. / hz)
  if len(tu) < int(10 * hz):
    return None
  y = np.interp(tu, t, x)
  y = _highpass(y, hz)
  w = np.hanning(len(y))
  sp = np.abs(np.fft.rfft(y * w)) * 2. / w.sum()
  fr = np.fft.rfftfreq(len(y), 1. / hz)
  m = (fr >= BAND[0]) & (fr <= BAND[1])
  if m.sum() < 3:
    return None
  pk = int(np.argmax(sp[m])); f = float(fr[m][pk]); amp = float(sp[m][pk])
  med = float(np.median(sp[m])) + 1e-9
  rms = float(math.sqrt(np.sum(sp[m] ** 2) / 2.))
  return rms, f, amp / med


def _runs(mask, t, min_s):
  out, i, n = [], 0, len(mask)
  while i < n:
    if mask[i]:
      j = i
      while j + 1 < n and mask[j + 1] and t[j + 1] - t[j] < 1.0:
        j += 1
      if t[j] - t[i] >= min_s:
        out.append((i, j + 1))
      i = j + 1
    else:
      i += 1
  return out


def param_sets(d):
  """consecutive spans with the same (sR, sF, ao, valid) rounded; -> list of (i0, i1, key)"""
  key = [(round(a, 2), round(b, 3), round(c, 2), d_) for a, b, c, d_ in zip(d['sR'], d['sF'], d['ao'], d['lp_valid'])]
  out, i = [], 0
  while i < len(key):
    j = i
    while j + 1 < len(key) and key[j + 1] == key[i]:
      j += 1
    if key[i][0] == key[i][0]:        # skip spans with no liveParameters yet (nan)
      out.append((i, j + 1, key[i]))
    i = j + 1
  return out


def xcorr_lag(cmd, meas, hz, max_lag=0.8):
  """lag (s) at which meas best follows cmd, and the correlation there (both high-passed)"""
  a = _highpass(cmd, hz); b = _highpass(meas, hz)
  best = (None, 0.)
  for k in range(0, int(max_lag * hz) + 1):
    x = a[:len(a) - k] if k else a; y = b[k:]
    if len(x) < 20 or x.std() < 1e-6 or y.std() < 1e-6:
      continue
    c = float(np.corrcoef(x, y)[0, 1])
    if c > best[1]:
      best = (k / hz, c)
  return best


def ratio_check(d, idx):
  """steerRatio implied by driving: v * angle_rad / (wheelbase * yaw), over steady cornering samples"""
  v = d['v'][idx]; ang = np.radians(d['ang'][idx]); yaw = d['yaw'][idx]
  m = (v > 8) & (np.abs(yaw) > 0.03) & (np.abs(ang) > 0.01) & np.isfinite(yaw) & np.isfinite(ang)
  if m.sum() < 50:
    return None
  r = v[m] * np.abs(ang[m]) / (WHEELBASE * np.abs(yaw[m]))
  r = r[(r > 5) & (r < 40)]
  if len(r) < 50:
    return None
  return float(np.median(r)), len(r)


def report(d, meta, out=print):
  t = d['t']
  hz = _hz(t)
  out('LIGHTLOG REPORT  %d rows, %.1f min, %.1f Hz' % (len(t), (t[-1] - t[0]) / 60., hz))
  for m in meta[:4]:
    out(m[:200])
  eng = (d['enabled'] == 1) & (d['active'] == 1) & (d['v'] > 5) & (d['pressed'] != 1)
  man = (d['enabled'] != 1) & (d['v'] > 5)
  out('')
  out('1. PARAMETER SETS (gain = steerRatio / %.1f, a ratio of 1.0 = angle per curvature as the car really needs)' % REF_RATIO)
  for i0, i1, key in param_sets(d):
    dur = t[i1 - 1] - t[i0]
    if dur < 20:
      continue
    sR, sF, ao, valid = key
    out('  t=%7.0f..%7.0f (%5.1f min)  sR=%6.2f  gain=%4.2f  sF=%5.3f  ao=%6.2f  valid=%s | engaged %.1f min, manual %.1f min' % (
        t[i0] - t[0], t[i1 - 1] - t[0], dur / 60., sR, sR / REF_RATIO, sF, ao, valid,
        (eng[i0:i1].sum() / hz) / 60., (man[i0:i1].sum() / hz) / 60.))
  out('')
  out('2. WIGGLE (0.2-0.8 Hz swing of the measured angle, 30 s windows; SNR>4 = a real peak) per parameter set')
  for i0, i1, key in param_sets(d):
    if t[i1 - 1] - t[i0] < 20:
      continue
    for label, mask in (('engaged', eng), ('manual ', man)):
      res = []
      for a, b in _runs(mask[i0:i1], t[i0:i1], 10.):
        a += i0; b += i0
        s = a
        while s < b:
          e = s
          while e < b and t[e] - t[s] < WIN_S:
            e += 1
          r = band_stats(t[s:e], d['ang'][s:e]) if e - s > 10 * hz else None
          if r:
            res.append(r)
          s = e
      if res:
        rms = np.array([r[0] for r in res]); snr = np.array([r[2] for r in res]); fr = np.array([r[1] for r in res])
        out('  sR=%6.2f sF=%5.3f %s: %2d windows  rms median %.2f deg max %.2f  SNR median %.1f  windows with SNR>4: %d  freq median %.2f Hz' % (
            key[0], key[1], label, len(res), np.median(rms), rms.max(), np.median(snr), int((snr > 4).sum()), np.median(fr)))
  out('')
  out('3. SHADOW (manual driving): planner desired angle vs your angle')
  mi = np.where(man & np.isfinite(d['pp_ang']) & np.isfinite(d['ang']))[0]
  if len(mi) > 20 * hz:
    diff = d['pp_ang'][mi] - d['ang'][mi]
    jit = _highpass(d['pp_ang'][mi], hz, 2.)
    out('  %d s manual: planner - yours: median %.2f deg, p95 |diff| %.2f deg; planner jitter (>0.5 Hz) rms %.2f deg, yours %.2f deg' % (
        len(mi) / hz, np.median(diff), np.percentile(np.abs(diff - np.median(diff)), 95), np.std(jit),
        np.std(_highpass(d['ang'][mi], hz, 2.))))
    r = band_stats(t[mi], d['pp_ang'][mi]) if len(mi) > 10 * hz else None
    if r:
      out('  planner angle 0.2-0.8 Hz band: rms %.2f deg, freq %.2f Hz, SNR %.1f (a steady road should be ~0)' % r)
  else:
    out('  not enough manual driving')
  out('')
  out('4. RATIO CHECK: steerRatio implied by your own driving (no control involved)')
  allv = np.arange(len(t))
  rc = ratio_check(d, allv)
  out('  %s' % (('median %.2f (%d samples)  vs learned/nominal above' % rc) if rc else 'not enough cornering samples (needs v>8 m/s and some turns)'))
  out('')
  out('5. DELAY command -> measured angle (engaged, closed loop; windows of 30 s with real steering movement)')
  lags = []
  for a, b in _runs(eng, t, 20.):
    s = a
    while s + int(WIN_S * hz) <= b:
      e = s + int(WIN_S * hz)
      c, m = d['ang_cmd'][s:e], d['ang'][s:e]
      if np.isfinite(c).all() and np.isfinite(m).all() and np.std(c) > 0.5:
        lag, cor = xcorr_lag(c, m, hz)
        if lag is not None and cor >= 0.6:
          lags.append((lag, cor))
      s = e
  if lags:
    L = np.array([x[0] for x in lags])
    out('  %d windows: median %.2f s  (p25 %.2f, p75 %.2f); resolution %.0f ms. hard-coded steerActuatorDelay in the car code: see interface.py' % (
        len(L), np.median(L), np.percentile(L, 25), np.percentile(L, 75), 1000. / hz))
  else:
    out('  no engaged windows with enough steering movement / correlation')
  e_i = np.where(eng & np.isfinite(d['ang_cmd']) & (np.abs(d['ang']) < 3))[0]
  if len(e_i) > 10 * hz:
    out('  steady straight, engaged: mean(measured - commanded) = %.2f deg (a frame offset between the two angle signals)' % float(np.mean(d['ang'][e_i] - d['ang_cmd'][e_i])))


def main():
  if len(sys.argv) < 2:
    print(__doc__); return 1
  d, meta = load(sys.argv[1:])
  if d is None:
    print('no data'); return 1
  report(d, meta)
  return 0


if __name__ == '__main__':
  sys.exit(main())
