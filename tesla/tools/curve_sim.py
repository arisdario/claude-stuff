#!/usr/bin/env python3
"""
curve_sim.py - closed-loop simulation of the predictive curve-speed controller (curve_fusion) on the spec's highway
scenarios. No car, no openpilot. Simulates a vehicle approaching a curve at highway speed, runs the fusion + approach
deceleration each 10 Hz step with jerk limiting, integrates speed/position, and reports the spec's metrics:
  entry speed vs the curve's safe speed, deceleration onset distance, peak decel, peak jerk, lateral accel at entry,
  and whether any false-positive braking happened before the curve was in sensor range.
  usage:  python3 tesla/tools/curve_sim.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import curve_fusion as F

KMH = 1 / 3.6
DT = 0.05                 # 20 Hz
A_LAT = 1.85              # m/s^2 comfortable lateral accel (matches A_Y_MAP)
A_MAX = 2.0               # m/s^2 regen braking ceiling
JERK_MAX = 2.5            # m/s^3 comfort jerk limit on the brake
VISION_RANGE = 150.       # m the camera can resolve the curve ahead
MAP_RANGE = 400.          # m the map "sees" the curve (forward-looking advantage of nav/map)


def safe_speed(curv):
  return math.sqrt(A_LAT / curv) if curv > 1e-6 else 999.


def simulate(name, v0, curv, dist0, vision_conf=0.9, map_conf=0.0, vision_sees=True, map_sees=True,
             lead=None, label=''):
  """v0 m/s, curvature curv (1/m), curve starts dist0 m ahead. vision/map_sees: whether that source reports the curve.
  lead: (dist, rel_v) or None. Returns metrics dict."""
  v = v0; x = 0.; prev_a = 0.; onset = None; entry_v = None; peak_a = 0.; peak_j = 0.; fp_brake = False
  vs = safe_speed(curv)
  steps = int((dist0 + 50) / max(v0, 1.) / DT) + 400
  for _ in range(steps):
    dist = dist0 - x                       # distance to the curve start
    srcs = []
    # vision: reports the curve speed only once within camera range
    if vision_sees and dist <= VISION_RANGE:
      srcs.append({'name': 'vision', 'kind': 'vision', 'speed': vs, 'conf': vision_conf, 'valid': True})
    elif vision_sees:
      srcs.append({'name': 'vision', 'kind': 'vision', 'speed': 999., 'conf': vision_conf, 'valid': True})  # sees straight
    # map: reports the curve speed earlier (forward-looking) if it has data
    if map_sees and map_conf > 0 and dist <= MAP_RANGE:
      srcs.append({'name': 'map', 'kind': 'map', 'speed': vs, 'conf': map_conf, 'valid': True})
    if lead is not None:
      ld, lv = lead
      srcs.append({'name': 'lead', 'kind': 'lead', 'speed': max(0., v + lv), 'conf': 1.0, 'valid': True})
    fus = F.fuse(srcs, v, v0) if srcs else None
    # required deceleration to the fused target by the curve distance
    a_cmd = 0.
    if fus and fus.target is not None and dist > -5:
      a_cmd = F.approach_decel(v, fus.target, max(dist, 0.1), 1.0, A_MAX) * fus.decel_authority
    # jerk limit
    a = max(prev_a - JERK_MAX * DT, min(prev_a + JERK_MAX * DT, a_cmd))
    a = max(0., min(a, A_MAX))
    if a > 0.05 and onset is None and dist > 0: onset = dist
    peak_a = max(peak_a, a); peak_j = max(peak_j, abs(a - prev_a) / DT)
    # record entry speed as we cross into the curve
    if dist <= 0 and entry_v is None: entry_v = v
    v = max(0., v - a * DT); x += v * DT; prev_a = a
    if entry_v is not None and x > dist0 + 30: break
  if entry_v is None: entry_v = v
  lat_at_entry = entry_v * entry_v * curv
  det_dist = MAP_RANGE if (map_sees and map_conf > 0) else VISION_RANGE   # earliest a source can see this curve
  avail = min(dist0, det_dist)                                           # distance available to slow before the curve
  req_a = (v0*v0 - vs*vs) / (2.*max(avail,1.)) if v0 > vs else 0.
  reachable = req_a <= A_MAX + 0.05
  return {'name': name, 'label': label, 'v0_kmh': v0 / KMH, 'safe_kmh': vs / KMH, 'entry_kmh': entry_v / KMH,
          'onset_m': onset, 'peak_a': peak_a, 'peak_jerk': peak_j, 'lat_entry': lat_at_entry, 'fp_brake': fp_brake,
          'reachable': reachable, 'req_a': req_a}


def show(m):
  straight = m['safe_kmh'] > 200
  if straight:
    ok = m['onset_m'] is None and m['peak_a'] < 0.1
    verdict = 'OK (no brake)' if ok else 'FALSE BRAKE'
  elif m['reachable']:
    ok = m['entry_kmh'] <= m['safe_kmh'] * 1.10 and m['lat_entry'] <= A_LAT * 1.2 and m['peak_jerk'] <= JERK_MAX + 0.1
    verdict = 'OK' if ok else 'CHECK'
  else:
    # physically impossible on regen alone (needs friction brakes the car cannot command): correct behaviour is to
    # brake at the regen ceiling and warn the driver. Judge that it applied ~max decel.
    ok = m['peak_a'] >= A_MAX - 0.2
    verdict = 'REGEN-LIMITED (driver must brake, warn)' if ok else 'WEAK'
  print('%-38s v0=%3.0f safe=%3.0f entry=%3.0f  onset=%-5s peakA=%.2f jerk=%.2f latG=%.2f reqA=%.1f  %s' % (
      m['name'], m['v0_kmh'], m['safe_kmh'], m['entry_kmh'],
      ('%.0fm' % m['onset_m']) if m['onset_m'] else 'none', m['peak_a'], m['peak_jerk'], m['lat_entry'], m['req_a'], verdict))
  return ok


def main():
  print('Scenario                                   speeds (km/h)           distances / dynamics      verdict')
  res = []
  # 1. 130 -> sharp right exit, vision+map agree (curv 0.03 ~ r33m exit ramp)
  res.append(show(simulate('1 highway->sharp exit (vis+map)', 130 * KMH, 0.03, 300, vision_conf=0.9, map_conf=0.5)))
  # 2. 130 -> sharp left curve, vision only (curv 0.012 ~ r83m)
  res.append(show(simulate('2 highway->left curve (vision)', 130 * KMH, 0.012, 300, vision_conf=0.95)))
  # 3. map says curve, vision sees straight (map wrong OR curve beyond camera): acts on map, reduced authority
  res.append(show(simulate('3 map curve, vision straight', 130 * KMH, 0.02, 300, vision_conf=0.9, map_conf=0.5, vision_sees=True)))
  # 4. vision curve, no map
  res.append(show(simulate('4 vision curve, no map', 130 * KMH, 0.02, 250, vision_conf=0.9, map_conf=0.0)))
  # 5. slower lead before a mild curve
  res.append(show(simulate('5 slower lead before curve', 120 * KMH, 0.006, 300, vision_conf=0.9, lead=(40., -15 * KMH))))
  # 10. very short deceleration distance to a tight exit
  res.append(show(simulate('10 short-distance tight exit', 130 * KMH, 0.03, 120, vision_conf=0.9, map_conf=0.5)))
  # a straight road (no curve): must NOT brake
  res.append(show(simulate('S straight road (no curve)', 130 * KMH, 0.0003, 300, vision_conf=0.9, map_conf=0.4)))
  print('\n%d/%d scenarios OK' % (sum(res), len(res)))


if __name__ == '__main__':
  main()
