#!/usr/bin/env python3
"""check_pairing: read-only. Do the car-side steering limits (steer_limits.py, tesla-unity table) and the panda firmware's
per-message angle check agree? Run it BEFORE flashing and again after.

  python3 /data/claude-stuff/tesla/panda/check_pairing.py
        [--fw /data/openpilot/panda/board/safety/safety_tesla.h] [--flag /data/steer_limits_on]

Verdicts:
  UNPATCHED   firmware still has the old x25 check (about 626 deg per message = no limit). Nothing can trip. Car-side layer is the only limit.
  PAIRED      patched firmware AND car-side layer at full strength AND firmware limit >= car limit at every speed: safe to run together.
  WILL TRIP   patched firmware but the car-side layer is off/partial: legitimate steering can exceed the firmware limit and disengage.
  MISMATCH    patched firmware tables are tighter than the car-side limits at some speed.
Exit code 0 = UNPATCHED or PAIRED, 1 = WILL TRIP / MISMATCH / unreadable.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import steer_limits as S  # noqa: E402

SPEEDS = [0., 2., 5., 7., 10., 15., 17., 20., 30., 38.]


def _interp(x, xp, fp):
  if x <= xp[0]:
    return fp[0]
  for i in range(len(xp) - 1):
    if x < xp[i + 1]:
      return fp[i] + (fp[i + 1] - fp[i]) * (x - xp[i]) / (xp[i + 1] - xp[i])
  return fp[-1]


def parse_table(src, name):
  m = re.search(r'lookup_t\s+' + name + r'\s*=\s*\{\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*\}', src)
  if not m:
    return None
  f = lambda s: [float(x) for x in s.replace('\n', ' ').split(',') if x.strip()]
  return f(m.group(1)), f(m.group(2))


def analyse(fw_src, strength):
  """-> (verdict, [lines])"""
  out = []
  up = parse_table(fw_src, 'TESLA_LOOKUP_ANGLE_RATE_UP_MSG')
  dn = parse_table(fw_src, 'TESLA_LOOKUP_ANGLE_RATE_DOWN_MSG')
  uses_msg = bool(re.search(r'tesla_interpolate\(\s*TESLA_LOOKUP_ANGLE_RATE_UP_MSG', fw_src))
  old = re.search(r'tesla_interpolate\(\s*TESLA_LOOKUP_ANGLE_RATE_UP\s*,\s*tesla_speed\s*\)\s*\*\s*25', fw_src)
  if not (up and dn and uses_msg):
    out.append('firmware: per-message tables not present%s' % (' (old x25 check found)' if old else ''))
    out.append('car side: steer_limits strength %.2f' % strength)
    return 'UNPATCHED', out
  out.append('firmware: per-message tables up %s down %s' % (up[1], dn[1]))
  out.append('car side: steer_limits strength %.2f (1.00 needed for the pair)' % strength)
  verdict = 'PAIRED'
  if strength < 0.99:
    verdict = 'WILL TRIP'
    out.append('the car-side layer is not at full strength: steps larger than the firmware limit will reach the panda')
  worst = None
  for v in SPEEDS:
    # car side: per-call limit x 2 calls per 0x659 message = the per-step table value
    car_up = _interp(v, S.RATE_BP, S.RATE_UP); car_dn = _interp(v, S.RATE_BP, S.RATE_DOWN)
    fw_up = int(_interp(v, up[0], up[1]) + 1.); fw_dn = int(_interp(v, dn[0], dn[1]) + 1.)
    ok = fw_up >= car_up and fw_dn >= car_dn
    out.append('  %4.0f m/s  car up %.2f / down %.2f   firmware up %d / down %d  %s' % (v, car_up, car_dn, fw_up, fw_dn, 'ok' if ok else 'FIRMWARE TIGHTER'))
    if not ok:
      worst = v
  if worst is not None:
    verdict = 'MISMATCH'
  return verdict, out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--fw', default='/data/openpilot/panda/board/safety/safety_tesla.h')
  ap.add_argument('--flag', default=S.FLAG_FILE)
  a = ap.parse_args()
  try:
    src = open(a.fw).read()
  except Exception as e:
    print('cannot read firmware source %s: %s' % (a.fw, e)); return 1
  strength = S.read_strength(a.flag)
  verdict, lines = analyse(src, strength)
  print('\n'.join(lines))
  print('VERDICT: %s' % verdict)
  return 0 if verdict in ('UNPATCHED', 'PAIRED') else 1


if __name__ == '__main__':
  sys.exit(main())
