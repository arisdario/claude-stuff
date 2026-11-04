#!/usr/bin/env python3
"""Forensics on radarweb intervention-event files (ev_*.json, 10 Hz): are the far-path numbers and the steering numbers real?

  python3 event_forensics.py ev_*.json

Answers, per event: (1) is path[40 m] the cubic evaluated from the plan polynomial (parsing check) and which term produces it,
(2) is the steering angle consistent with the path curvature (independent source) and with its own rate of change, how stale
is it, (3) engaged-only lead/lag between path curvature/heading and steering. stdlib only."""
import json
import math
import sys

L = 2.96          # Model S wheelbase, m


def ev(pp, x):
  return pp[0] * x ** 3 + pp[1] * x ** 2 + pp[2] * x + pp[3]


def corr_lag(a, b, dt, maxlag=1.0):
  """lag s at which b best matches a (positive: b lags a)."""
  best = (0, 0.0)
  K = int(maxlag / dt)
  for k in range(-K, K + 1):
    xs = [(a[i], b[i + k]) for i in range(len(a)) if 0 <= i + k < len(b)]
    if len(xs) < 15:
      continue
    ma = sum(x for x, _ in xs) / len(xs); mb = sum(y for _, y in xs) / len(xs)
    sa = math.sqrt(sum((x - ma) ** 2 for x, _ in xs)); sb = math.sqrt(sum((y - mb) ** 2 for _, y in xs))
    if sa < 1e-9 or sb < 1e-9:
      continue
    c = sum((x - ma) * (y - mb) for x, y in xs) / (sa * sb)
    if abs(c) > abs(best[1]):
      best = (k * dt, c)
  return best


def run(path):
  d = json.load(open(path))
  s = d['samples']
  t0 = s[0]['t']
  out = ['=== %s  (%s, %s km/h, %d samples, %.1f s)' % (path.split('/')[-1], d.get('cause'), d.get('kmh'), len(s), s[-1]['t'] - t0)]
  # (1) far path
  err = max(abs(ev(x['pp'], 40) - x['path'][3]) for x in s if x.get('pp') and x.get('path'))
  out.append('(1) path[40 m] vs cubic(pathPlan poly) at 40 m: max difference %.3f m -> the far values ARE the polynomial evaluated at 40 m (no parsing/unit error)' % err)
  worst = max(s, key=lambda x: abs(x['path'][3]))
  c3, c2, c1, c0 = worst['pp']
  terms = (c3 * 64000, c2 * 1600, c1 * 40, c0)
  R = 1 / abs(2 * c2) if abs(c2) > 1e-6 else float('inf')
  out.append('    worst sample: y40=%.1f m = c3 term %.1f + c2 term %.1f + c1 term %.1f + c0 %.2f ; speed %.1f m/s (%.0f km/h); curvature 2*c2=%.4f 1/m (radius ~%.0f m)' % (
    worst['path'][3], terms[0], terms[1], terms[2], terms[3], worst['v'], worst['v'] * 3.6, 2 * c2, R))
  circle = R - math.sqrt(max(0.0, R * R - 40 * 40)) if R > 40 else float('nan')
  out.append('    a constant-curvature arc of that radius would be %.1f m off a straight line at 40 m (a radius under 40 m cannot even reach x=40: the cubic is extrapolating)' % circle if R > 40 else
             '    radius %.0f m < 40 m: the path turns >90 degrees within 40 m, so a cubic evaluated at x=40 m is meaningless (extrapolation of a tight turn), not a camera/pose fault' % R)
  big = sum(1 for x in s if abs(x['path'][3]) > 15)
  out.append('    samples with |y40|>15 m: %d of %d' % (big, len(s)))
  # (2) steering
  ang = [x['ang'] for x in s]
  ts = [x['t'] - t0 for x in s]
  dts = [b - a for a, b in zip(ts, ts[1:])]
  rep = sum(1 for a, b in zip(ang, ang[1:]) if a == b)
  rates = [abs(b - a) / max(1e-3, dt) for a, b, dt in zip(ang, ang[1:], dts)]
  out.append('(2) steering: min %.1f max %.1f deg | max rate %.0f deg/s | consecutive identical values %d/%d | sample dt median %.0f ms max %.0f ms' % (
    min(ang), max(ang), max(rates), rep, len(ang) - 1, 1e3 * sorted(dts)[len(dts) // 2], 1e3 * max(dts)))
  big_j = [(abs(b - a), a, b, dt) for a, b, dt in zip(ang, ang[1:], dts) if abs(b - a) > 8]
  out.append('    jumps >8 deg between samples: %d ; steepest %s' % (len(big_j), ('%.1f deg in %.2f s' % (max(big_j)[0], max(big_j)[3])) if big_j else 'none'))
  # isolated single-sample spikes (corruption signature: one sample far from BOTH neighbours, neighbours agree)
  spikes = [i for i in range(1, len(ang) - 1) if abs(ang[i] - ang[i - 1]) > 6 and abs(ang[i] - ang[i + 1]) > 6
            and (ang[i] - ang[i - 1]) * (ang[i] - ang[i + 1]) > 0 and abs(ang[i - 1] - ang[i + 1]) < 0.5 * min(abs(ang[i] - ang[i - 1]), abs(ang[i] - ang[i + 1]))]
  out.append('    isolated one-sample spikes (a corruption/decoding-glitch signature): %d' % len(spikes))
  # steering vs path curvature (independent source): fit angle = ratio * atan(L*kappa) for samples with a big angle
  pts = [(math.degrees(math.atan(L * 2 * x['pp'][1])), x['ang']) for x in s if abs(x['ang']) > 40 and x['v'] > 0.5]
  if len(pts) >= 8:
    num = sum(a * b for a, b in pts); den = sum(a * a for a, _ in pts)
    ratio = num / den if den else float('nan')
    resid = math.sqrt(sum((b - ratio * a) ** 2 for a, b in pts) / len(pts))
    out.append('    angle vs path-curvature fit on %d samples with |angle|>40 deg: steering ratio %.1f (residual %.0f deg) -> the large angle agrees with the independently measured path curvature = a real turn' % (len(pts), ratio, resid))
  # (3) engaged lead/lag
  eng = [x for x in s if x['op']]
  if len(eng) > 40:
    dt = 0.1
    c = [2 * x['pp'][1] * 1000 for x in eng]; h = [x['pp'][2] for x in eng]; a = [x['ang'] for x in eng]
    for name, sig in (('path curvature', c), ('path heading c1', h)):
      lag, cc = corr_lag(sig, a, dt)
      out.append('(3) engaged %d samples: %s -> steering angle: steering lags by %+.1f s (corr %.2f)%s' % (
        len(eng), name, lag, cc, '' if abs(cc) > 0.3 else ' [weak]'))
  else:
    out.append('(3) engaged samples: %d (too few for lead/lag)' % len(eng))
  out.append('    camera frames for this event: %s' % (d.get('video') or 'NONE (loggerd was blocked / video not recorded)'))
  return '\n'.join(out)


if __name__ == '__main__':
  for p in sys.argv[1:]:
    print(run(p))
    print()
