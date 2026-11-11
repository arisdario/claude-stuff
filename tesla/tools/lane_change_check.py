#!/usr/bin/env python3
"""lane_change_check.py - READ-ONLY check of everything ALCA (tap) and the manual-lane-change hold need.
Never sends anything, never changes a setting.

  python3 lane_change_check.py          # setup checks only (config, button, deployed code, processes)
  python3 lane_change_check.py --live   # + live view (run from /data/openpilot with openpilot running):
                                        #   one line per 0.5 s showing what would block ALCA right now

Rules mirrored from the code (ALCA_module.py / carcontroller.py / carstate.py):
  ALCA starts on a SHORT half-press tap when: alca button on, ALCA enabled (config), openpilot engaged,
  speed >= CL_MIN_V (8.9 m/s = 32 km/h), |steering angle| < CL_MAX_A (20 deg), driver not steering.
  Latched blinker (full press): no ALCA - openpilot releases steering while the stalk is latched (stays
  engaged, speed control continues), then holds steering off after the stalk returns until the car is
  steady in the lane (max steer_reengage_numb_period s), then shows "Lane Centering Resumed".
"""
import configparser
import os
import sys
import time

OP = os.environ.get('OP', '/data/openpilot')
CFG = '/data/bb_openpilot.cfg'
BTN_STATUS = OP + '/selfdrive/car/tesla/buttons.cc.msg'   # one char per button, '1' = on; button 0 = alca
CC = OP + '/selfdrive/car/tesla/carcontroller.py'
CL_MIN_V = 8.9    # m/s, carstate.py
CL_MAX_A = 20.    # deg, carstate.py

ok_all = True


def report(ok, what, detail=''):
  global ok_all
  ok_all &= ok
  print('%-4s %-42s %s' % ('OK' if ok else 'FAIL', what, detail))


def cfg_get(cp, key):
  for sec in cp.sections():
    if cp.has_option(sec, key):
      return cp.get(sec, key).strip()
  return None


def setup_checks():
  cp = configparser.ConfigParser()
  try:
    cp.read(CFG)
  except Exception as e:
    report(False, 'read %s' % CFG, str(e)); return
  ic = cfg_get(cp, 'has_tesla_ic_integration')
  alca = cfg_get(cp, 'enable_alca')
  ign = cfg_get(cp, 'alca_ignore_ic_toggle')
  report(alca == 'True', 'config enable_alca = True', 'is %s' % alca)
  if ic == 'True':
    report(ign == 'True', 'config alca_ignore_ic_toggle = True',
           'is %s (with IC integration, False = ALCA follows the car display toggle)' % ign)
  hso = cfg_get(cp, 'enable_hso')
  print('INFO enable_hso = %s (optional: hands the wheel to you when you steer by hand; the latched-stalk'
        ' release and the hold work either way)' % hso)
  print('     steer_reengage_numb_period = %s s, tap_blinker_extension = %s, hso_numb_period = %s s'
        % (cfg_get(cp, 'steer_reengage_numb_period'), cfg_get(cp, 'tap_blinker_extension'), cfg_get(cp, 'hso_numb_period')))

  try:
    st = open(BTN_STATUS, 'rb').read()
    report(len(st) >= 1 and st[0:1] == b'1', 'alca button on (EON screen)', 'status file: %r' % st)
  except Exception as e:
    report(False, 'alca button on (EON screen)', 'cannot read %s (%s)' % (BTN_STATUS, e))

  try:
    src = open(CC).read()
    report('steer_reengage_active' in src, 'deployed: steer-reengage hold')
    report('self.blinker) is True' in src, 'deployed: ALCA return-value fix')
  except Exception as e:
    report(False, 'deployed carcontroller.py', str(e))
  try:
    report('alcaIgnoreIcToggle' in open(OP + '/selfdrive/car/tesla/carstate.py').read(), 'deployed: alca_ignore_ic_toggle')
  except Exception as e:
    report(False, 'deployed carstate.py', str(e))

  # read /proc directly: `ps` run from Python on NEOS may list only this terminal's processes
  cmdlines = []
  for pid in os.listdir('/proc'):
    if pid.isdigit():
      try:
        cmdlines.append(open('/proc/%s/cmdline' % pid, 'rb').read().replace(b'\0', b' ').decode('utf-8', 'replace'))
      except Exception:
        pass
  for p in ('controlsd', 'boardd', '_modeld'):
    # match the program name only (argv[0] basename, e.g. ./boardd or selfdrive.controls.controlsd),
    # not any word in a command line
    report(any(os.path.basename(c.split(' ')[0]).endswith(p) for c in cmdlines if c), 'process %s running' % p)


def live():
  sys.path.insert(0, OP)
  import cereal.messaging as messaging
  sm = messaging.SubMaster(['carState', 'controlsState', 'pathPlan'])
  print('\nlive (Ctrl-C to stop).  READY = a half-press tap would start ALCA now')
  last_print = 0
  while True:
    sm.update(100)
    if time.monotonic() - last_print < 0.5:
      continue
    last_print = time.monotonic()
    cs, ctl, pp = sm['carState'], sm['controlsState'], sm['pathPlan']
    blockers = []
    if not ctl.enabled: blockers.append('not engaged')
    if cs.vEgo < CL_MIN_V: blockers.append('speed %.0f<%.0f km/h' % (cs.vEgo * 3.6, CL_MIN_V * 3.6))
    if abs(cs.steeringAngle) >= CL_MAX_A: blockers.append('angle %.0f>=%.0f' % (cs.steeringAngle, CL_MAX_A))
    if cs.steeringPressed: blockers.append('driver steering')
    try:
      if open(BTN_STATUS, 'rb').read(1) != b'1': blockers.append('alca button off')
    except Exception:
      blockers.append('alca button ?')
    blink = 'L' if cs.leftBlinker else ('R' if cs.rightBlinker else '-')
    print('%-6s v=%5.1f km/h angle=%6.1f blinker=%s lanes L=%.2f R=%.2f  %s'
          % ('READY' if not blockers else 'wait', cs.vEgo * 3.6, cs.steeringAngle, blink,
             pp.lProb, pp.rProb, ', '.join(blockers)))


if __name__ == '__main__':
  setup_checks()
  print('\nSETUP: %s' % ('all OK' if ok_all else 'fix the FAIL lines above'))
  if '--live' in sys.argv:
    try:
      live()
    except KeyboardInterrupt:
      pass
