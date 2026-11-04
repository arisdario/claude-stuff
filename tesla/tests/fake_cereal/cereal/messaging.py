"""Fake cereal.messaging for testing the diagnostics workflow off the EON: a time-driven synthetic car (no real sockets).
FAKE_OSC="a,b"  steering oscillates (0.35 Hz, 8 deg) from a to b seconds after start; FAKE_FRAME_DROP="a,b"; FAKE_CAN_GAP="a,b"."""
import math
import os
import time
from types import SimpleNamespace as N

T0 = time.clock_gettime(time.CLOCK_BOOTTIME)
PERIOD = {'frame': 0.05, 'model': 0.05, 'cameraOdometry': 0.05, 'controlsState': 0.01, 'carState': 0.01, 'carControl': 0.01,
          'can': 0.01, 'liveLocationKalman': 0.05, 'sensorEvents': 0.01, 'gpsLocationExternal': 1.0, 'liveParameters': 0.25,
          'liveCalibration': 0.25, 'pathPlan': 0.05, 'health': 0.5}
_next = {}
_fid = {'n': 0}


def _win(name):
  v = os.environ.get(name)
  return tuple(float(x) for x in v.split(',')) if v else None


def _enc(angle):
  raw = int(round((angle + 819.2) / 0.1))
  v, i = 0, 37
  for k in range(14):
    v |= ((raw >> (13 - k)) & 1) << ((i // 8) * 8 + i % 8)
    i = i + 15 if i % 8 == 0 else i - 1
  return v.to_bytes(8, 'little')


def sub_sock(name, conflate=False):
  if name not in PERIOD:
    raise KeyError(name)
  _next[name] = T0
  return name


def _angle(t):
  osc = _win('FAKE_OSC')
  tt = t - T0
  return 8 * math.sin(2 * math.pi * 0.35 * tt) if osc and osc[0] <= tt < osc[1] else 0.0


def _mk(name, t):
  lt = int((t - 0.002) * 1e9)
  a = _angle(t)
  tt = t - T0
  if name == 'frame':
    _fid['n'] += 1
    w = _win('FAKE_FRAME_DROP')
    if w and w[0] <= tt < w[1]:
      return None
    return N(logMonoTime=lt, frame=N(frameId=_fid['n'], timestampEof=int((t - 0.03) * 1e9), frameLength=1000, integLines=100, globalGain=1, lensPos=0))
  if name == 'model':
    return N(logMonoTime=int((t + 0.02) * 1e9), model=N(path=N(poly=[0, 0, 0.0, 0.1], std=1), leftLane=N(poly=[0, 0, 0, 1.6], prob=0.5),
                                                          rightLane=N(poly=[0, 0, 0, -1.6], prob=0.5), frameId=_fid['n'], frameDropPerc=0.0,
                                                          timestampEof=int((t - 0.03) * 1e9), modelExecutionTime=0.04, gpuExecutionTime=0.03))
  if name == 'cameraOdometry':
    return N(logMonoTime=lt, cameraOdometry=N(trans=[13, 0, 0], rot=[0, 0.001, 0.002], transStd=[1, 1, 1], rotStd=[.1, .1, .1], frameId=_fid['n']))
  if name == 'controlsState':
    return N(logMonoTime=lt, controlsState=N(angleSteersDes=a + 0.5, angleSteers=a, enabled=True, vEgo=13.0, steerRatio=11.5, curvature=0.0, cumLagMs=5.0, active=True))
  if name == 'carState':
    return N(logMonoTime=lt, carState=N(vEgo=13.0, steeringAngle=a, steeringRate=0, steeringTorque=0.1, yawRate=0.0, aEgo=0.0, gearShifter='drive'))
  if name == 'carControl':
    return N(logMonoTime=lt, carControl=N(enabled=True, active=True, actuators=N(steer=0.0, steerAngle=a, gas=0, brake=0)))
  if name == 'can':
    w = _win('FAKE_CAN_GAP')
    if w and w[0] <= tt < w[1]:
      return None
    return N(logMonoTime=lt, can=[N(address=ad, src=0, dat=d) for ad, d in ((0x370, _enc(-a)), (0x118, bytes(8)), (0x108, bytes(8)), (0x155, bytes(8)), (0x0E, bytes(8)))])
  if name == 'liveLocationKalman':
    return N(logMonoTime=lt, liveLocationKalman=N(inputsOK=True, sensorsOK=True, gpsOK=True, angularVelocityCalibrated=N(value=[0, 0, 0.01], std=[0, 0, 0.001])))
  if name == 'sensorEvents':
    return N(logMonoTime=lt, sensorEvents=[N(gyro=N(v=[0, 0, 0.01]), acceleration=None), N(gyro=None, acceleration=N(v=[0, 0.1, 9.8]))])
  if name == 'gpsLocationExternal':
    return N(logMonoTime=lt, gpsLocationExternal=N(latitude=44.4, longitude=26.1, altitude=80, bearing=10, speed=13, accuracy=3, flags=1))
  if name == 'liveParameters':
    return N(logMonoTime=lt, liveParameters=N(steerRatio=11.5, stiffnessFactor=1.0, angleOffsetAverage=0.0, valid=True))
  if name == 'liveCalibration':
    return N(logMonoTime=lt, liveCalibration=N(rpyCalib=[0.0, 0.08, 0.02], calStatus=1, calPerc=100))
  if name == 'pathPlan':
    return N(logMonoTime=lt, pathPlan=N(angleSteers=a, paramsValid=True, modelValid=True))
  if name == 'health':
    return N(logMonoTime=lt, health=N(canRxErrs=0, canSendErrs=0, canFwdErrs=0, controlsAllowed=True))


def drain_sock(sock, wait_for_one=False):
  now = time.clock_gettime(time.CLOCK_BOOTTIME)
  out = []
  while _next[sock] + PERIOD[sock] <= now and len(out) < 200:
    _next[sock] += PERIOD[sock]
    ev = _mk(sock, _next[sock])
    if ev is not None:
      out.append(ev)
  return out
