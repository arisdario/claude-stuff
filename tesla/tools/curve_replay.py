#!/usr/bin/env python3
"""
curve_replay.py - score curve_fusion.fuse() against a recorded drive (a drivetrace_*.csv from tools/drivetrace.py,
columns: t,v_kmh,a_ego,...,pL,pR,vis_curve,map_curve,curv,dist,lead_d,lead_vrel). No car, no openpilot; pure replay.

For each row it builds the three fusion sources with confidences derived from the log, runs fuse(), and also runs the
OLD behaviour (bare min of the valid sources) for comparison. Then it reports the spec's metrics:
  - false-positive braking: rows that would command real deceleration on a STRAIGHT road (|curv| small), old vs fused
  - real-curve catch: rows on a genuine curve where the target is below ego speed
  - effective deceleration = approach_decel(...) * decel_authority, so a low-confidence phantom brakes gently
Pass criterion (printed): the fusion reduces false-positive hard braking vs the old min() while still slowing for real
curves.   usage:  python3 tesla/tools/curve_replay.py drivetrace_XXXX.csv [more.csv ...]
"""
import csv
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import curve_fusion as F

KMH = 1 / 3.6
STRAIGHT_CURV = 0.002     # |map curvature| below this = effectively straight road (radius > 500 m)
CURVE_CURV = 0.006        # above this = a real curve (radius < ~165 m)
BRAKE_A = 0.6             # m/s^2 effective decel above which we call it a "braking event"
HARD_A = 1.2             # m/s^2 = a firm brake
A_MAX = 2.0              # regen ceiling used for the urgency calc


def _f(x):
  try: return float(x)
  except (TypeError, ValueError): return None


def vision_conf(pL, pR):
  # the model path (and so its curve estimate) is trustworthy roughly with lane visibility: jitter was 6x worse with
  # no lane. Map lane prob -> confidence, with a floor so vision still counts on an unmarked road.
  p = max(pL or 0., pR or 0.)
  return max(0.25, min(0.95, 0.3 + 0.7 * p))


def map_conf(curv, dist):
  # map curvature on this car is low-res; trust it only as corroboration, and less when the turn is far/uncertain
  if curv is None or dist is None: return 0.0
  return 0.5 if dist < 200 else 0.35


def lead_follow_speed(v_ego, lead_d, lead_vrel):
  # a simple safe-approach speed from the lead: if closing on a near lead, cap speed; else no constraint.
  if not lead_d or lead_d <= 0 or lead_d > 90: return None
  v_lead = max(0., v_ego + (lead_vrel or 0.))
  # desired gap ~ 1.8 s; if we are closer than that, target the lead's speed (or below)
  if lead_d < max(6., 1.8 * v_ego):
    return v_lead
  return None


def row_sources(r):
  v = (_f(r['v_kmh']) or 0.) * KMH
  vis = _f(r['vis_curve']); mp = _f(r['map_curve']); curv = _f(r['curv']); dist = _f(r['dist'])
  pL = _f(r['pL']); pR = _f(r['pR']); ld = _f(r['lead_d']); lv = _f(r['lead_vrel'])
  srcs = []
  if vis is not None:
    srcs.append({'name': 'vision', 'kind': 'vision', 'speed': vis * KMH, 'conf': vision_conf(pL, pR), 'valid': True})
  if mp is not None and curv is not None:
    srcs.append({'name': 'map', 'kind': 'map', 'speed': mp * KMH, 'conf': map_conf(curv, dist), 'valid': True})
  lf = lead_follow_speed(v, ld, lv)
  if lf is not None:
    srcs.append({'name': 'lead', 'kind': 'lead', 'speed': lf, 'conf': 1.0, 'valid': True})
  return v, curv, dist, srcs


def old_min(srcs, v):
  vals = [s['speed'] for s in srcs if s.get('valid') and s['speed'] is not None]
  return min(vals) if vals else None


def score(path):
  rows = [r for r in csv.DictReader(open(path)) if _f(r['v_kmh']) and _f(r['v_kmh']) > 5]
  n = len(rows)
  stat = {'rows': n, 'fused_fp': 0, 'fused_fp_hard': 0, 'old_fp': 0, 'old_fp_hard': 0,
          'curve_rows': 0, 'curve_caught': 0, 'fused_brake': 0, 'old_brake': 0}
  for r in rows:
    v, curv, dist, srcs = row_sources(r)
    if not srcs: continue
    fus = F.fuse(srcs, v, v)
    oldt = old_min(srcs, v)
    d = dist if dist and dist > 0 else 50.
    # effective decel each would command now
    fused_a = F.approach_decel(v, fus.target, d, 1.0, A_MAX) * fus.decel_authority if fus.target else 0.
    old_a = F.approach_decel(v, oldt, d, 1.0, A_MAX) if oldt else 0.    # old code: full authority always
    straight = curv is not None and abs(curv) < STRAIGHT_CURV
    real_curve = curv is not None and abs(curv) > CURVE_CURV
    if fused_a > BRAKE_A: stat['fused_brake'] += 1
    if old_a > BRAKE_A: stat['old_brake'] += 1
    if straight:
      if fused_a > BRAKE_A: stat['fused_fp'] += 1
      if fused_a > HARD_A: stat['fused_fp_hard'] += 1
      if old_a > BRAKE_A: stat['old_fp'] += 1
      if old_a > HARD_A: stat['old_fp_hard'] += 1
    if real_curve:
      stat['curve_rows'] += 1
      if fus.target is not None and fus.target < v - 0.5: stat['curve_caught'] += 1
  return stat


def main():
  if len(sys.argv) < 2:
    sys.exit(__doc__)
  tot = None
  for p in sys.argv[1:]:
    s = score(p)
    print('\n=== %s ===' % os.path.basename(p))
    print('rows(moving): %d' % s['rows'])
    print('FALSE-POSITIVE braking on straight road (|curv|<%.3f):' % STRAIGHT_CURV)
    print('   old min():   %d events (%d hard >%.1f m/s2)' % (s['old_fp'], s['old_fp_hard'], HARD_A))
    print('   fused:       %d events (%d hard)   <- should be fewer' % (s['fused_fp'], s['fused_fp_hard']))
    print('real curves (|curv|>%.3f): %d rows, caught (target<ego): %d (%.0f%%)' % (
        CURVE_CURV, s['curve_rows'], s['curve_caught'], 100. * s['curve_caught'] / max(1, s['curve_rows'])))
    passed = s['fused_fp_hard'] <= s['old_fp_hard'] and (s['curve_rows'] == 0 or s['curve_caught'] > 0)
    print('VERDICT: %s' % ('PASS - fewer/equal hard false brakes, still catches curves' if passed else 'REVIEW'))
    if tot is None: tot = dict(s)
    else:
      for k in s: tot[k] += s[k]
  if len(sys.argv) > 2 and tot:
    print('\n=== TOTAL over %d files ===' % (len(sys.argv) - 1))
    print('fp hard: old %d -> fused %d ; curve catch %d/%d' % (tot['old_fp_hard'], tot['fused_fp_hard'], tot['curve_caught'], tot['curve_rows']))


if __name__ == '__main__':
  main()
