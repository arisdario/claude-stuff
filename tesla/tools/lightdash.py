#!/usr/bin/env python3
"""lightdash (Python version) - the C++ twin lightdash.cc is much lighter (~0.3% of a core, 6 MB; build: sh build_lightdash.sh) and is the one to run on the EON;
this file stays as the readable reference / off-car --demo.

lightdash - ONE small read-only web page that shows what the panda tells the instrument cluster (lanes, lead cars, status).

  PYTHONPATH=/data/openpilot nohup nice -n 19 python3 /data/claude-stuff/tesla/tools/lightdash.py > /data/lightdash.log 2>&1 &
  open  http://<EON-IP>:8090/        (stop: pkill -f lightdash.py)       off-car test: python3 lightdash.py --demo

What it shows is NOT a guess: it reads the frames openpilot sends to the panda (`sendcan`: 0x557 lanes+lead, 0x559 objects, 0x659
status, 0x554 switches) and applies, byte for byte, what the firmware (safety_tesla.h, v1.7.3: do_fake_DAS) turns them into -
DAS_lanes 0x239 and DAS_object 0x309 - then decodes those frames with the DBC scaling. The faint cyan lines are the model's own lanes,
for comparison. It sends NOTHING (no CAN, no cereal publish), touches no file, and does no work while no browser is connected.
Cluster units: the car code draws the picture at half scale (IC_LANE_SCALE = 2), so distances in the cluster frames are real/2; this page
multiplies back by 2 and says so.   Env: LIGHTDASH_PORT (8090), LIGHTDASH_BIND (0.0.0.0; use 127.0.0.1 to keep it on the EON).
Python 3.7, standard library only; cereal is imported only when not in --demo.
"""
import argparse
import json
import os
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

IC_LANE_SCALE = 2.0
WANT = (0x557, 0x559, 0x659, 0x554)
CAR_HOLD_S = 3.0                                       # 0x30A comes every 1 s: hold the last value across one lost frame
BODY_ADDRS = (0x318, 0x283, 0x238, 0x2F8)                     # read-only body frames, see class Body


class CarState(object):
  """read-only from the car's own 'can' stream (never sent): 0x30A byte 0 = lamp state (1 Hz + on change), 0x283 byte 7 = outside temperature (~1 Hz)"""
  def __init__(self):
    self.hb = self.fl = 0; self.t30a = 0.; self.out = None; self.t283 = 0.

  def feed(self, addr, d, now):
    if addr == 0x30A and len(d) >= 1:
      self.hb = d[0] >> 3 & 1; self.fl = d[0] >> 2 & 1; self.t30a = now
    elif addr == 0x283 and len(d) >= 8:
      self.out = d[7] * 0.5 - 40 if d[7] < 0xFE else None        # 0xFE init / 0xFF SNA = no value
      self.t283 = now

  def snapshot(self, now):
    live = self.t30a and now - self.t30a <= CAR_HOLD_S
    return {'hb': self.hb if live else None, 'fl': self.fl if live else None,
            'out': round(self.out, 1) if (self.out is not None and self.t283 and now - self.t283 <= 5.) else None}

BODY_HOLD_S = 3.0             # 0x318 / 0x283 come several times a second: older than this = unknown, never "closed"/"off"
LIMIT_HOLD_S = 10.0           # 0x238 cycles through several record types: the speed-limit record is not in every frame


class Body(object):
  """BODY-BEGIN read-only, the car's own body/gateway frames on bus 0 (opendbc tesla_can.dbc), never sent:
  0x318 GTW_carState doors/frunk/trunk, 0x283 BODY_R1 lamps/light sensor/wipers/park brake/reverse/inside temp,
  0x238 UI_driverAssistRoadSign record 3 = map speed limit (m/s), 0x2F8 UI_gpsVehicleSpeed bit 46 = limit units (1 = km/h)"""
  def __init__(self):
    self.t318 = self.t283 = self.tlim = 0.; self.d318 = {}; self.d283 = {}; self.lim_ms = 0.; self.kph = True

  def feed(self, addr, d, now):
    if addr == 0x318 and len(d) >= 8:
      self.d318 = {'fl': d[1] >> 4 & 3, 'fr': d[1] >> 6 & 3, 'rl': d[2] >> 6 & 3, 'rr': d[3] >> 5 & 3, 'trunk': d[5] >> 6 & 3, 'frunk': d[6] >> 2 & 3}
      self.t318 = now
    elif addr == 0x283 and len(d) >= 8:
      self.d283 = {'lo': d[4] & 1, 'hi': d[4] >> 1 & 1, 'night': d[2] >> 6 & 1, 'tunnel': d[2] >> 4 & 1, 'sens_bad': (d[2] >> 3 & 1) | (d[2] >> 5 & 1),
                   'wiper': d[3] >> 5 & 1, 'pbrake': d[3] >> 4 & 1, 'rev': d[4] >> 6 & 3, 'inside': d[5]}
      self.t283 = now
    elif addr == 0x238 and len(d) >= 2 and d[0] == 3:
      self.lim_ms = d[1] * 0.25; self.tlim = now
    elif addr == 0x2F8 and len(d) >= 6:
      self.kph = (d[5] >> 6 & 1) == 1

  def snapshot(self, now):
    a = self.t318 and now - self.t318 <= BODY_HOLD_S
    b = self.t283 and now - self.t283 <= BODY_HOLD_S
    st = lambda v: (True if v == 1 else False if v == 0 else None) if a else None     # 0 closed 1 open 2 init 3 SNA
    o = {'doors': [st(self.d318.get(k)) for k in ('fl', 'fr', 'rl', 'rr')], 'frunk': st(self.d318.get('frunk')), 'trunk': st(self.d318.get('trunk'))}
    for k in ('lo', 'hi', 'wiper', 'pbrake'):
      o[k] = bool(self.d283[k]) if b else None
    o['rev'] = (self.d283['rev'] == 1) if b else None
    o['light'] = None if not b or self.d283['sens_bad'] else ('tunnel' if self.d283['tunnel'] else 'night' if self.d283['night'] else 'day')
    o['inside'] = round(self.d283['inside'] * 0.25, 1) if b and self.d283['inside'] != 0xFF else None
    lim = None
    if self.tlim and now - self.tlim <= LIMIT_HOLD_S and self.lim_ms > 0:
      lim = int(round(self.lim_ms * (3.6 if self.kph else 2.236936) / 5.) * 5)            # map limits are multiples of 5 (0.25 m/s steps are noisy)
    o['limit'] = lim; o['unit'] = 'km/h' if self.kph else 'mph'
    return o
  # BODY-END


# DYN-BEGIN read-only, the car's own drive-unit / brake / airbag / MCU frames on bus 0 (tesla_can.dbc), never sent. Checked against
# ~20 000 logged frames from this car: motor rpm follows road speed (gear 9.73), torque + / - with pedal / regen, standing objects 0 m/s.
DYN_ADDRS = (0x108, 0x118, 0x145, 0x368, 0x201, 0x246)
DYN_HOLD_S = 1.0              # 0x108 / 0x118 / 0x145 / 0x368 are 10-100 Hz: older than this = no value
DYN_SLOW_HOLD_S = 3.0         # 0x201 seat belts / 0x246 sun are slow
GEARS = {1: 'P', 2: 'R', 3: 'N', 4: 'D'}


def _u(v, start, n):
  return v >> start & ((1 << n) - 1)


def _s(v, start, n):
  x = _u(v, start, n)
  return x - (1 << n) if x >> (n - 1) else x


class Dyn(object):
  """0x108 DI_torque1 motor torque (Nm) x motor rpm -> motor power kW (+ drive / - regen; motor output, not battery), pedal %;
  0x118 DI_torque2 gear; 0x145 ESP_145h long/lat acceleration m/s2, brake master cylinder bar; 0x368 DI_state cruise state /
  set speed / units, regen-limited light, vehicle hold; 0x201 SDM1 driver belt; 0x246 UI_solarData sun up"""
  def __init__(self):
    self.f = {}

  def feed(self, addr, d, now):
    d = bytes(d[:8]).ljust(8, b'\0')       # some are shorter (0x118: 6 bytes, 0x201 / 0x246: 5): pad, signals sit in the first bytes
    if any(d):                            # an all-zero frame = the sender is not awake yet: no value, not "0"
      self.f[addr] = (int.from_bytes(d, 'little'), d, now)

  def _get(self, addr, now, hold=DYN_HOLD_S):
    r = self.f.get(addr)
    return r if r and now - r[2] <= hold else None

  def snapshot(self, now):
    o = dict.fromkeys(('kw', 'ped', 'gear', 'ax', 'ay', 'bp', 'cs', 'set', 'su', 'regen', 'hold', 'belt', 'sun'))
    r = self._get(0x108, now)
    if r:
      v = r[0]; o['kw'] = round(_s(v, 16, 13) * 0.25 * _s(v, 32, 16) * 0.10471976 / 1000., 1); o['ped'] = round(_u(v, 48, 8) * 0.4)
    r = self._get(0x118, now)
    if r:
      o['gear'] = GEARS.get(_u(r[0], 12, 3))
    r = self._get(0x145, now)
    if r:
      v = r[0]; o['ax'] = round(_u(v, 0, 10) * 0.05 - 25.6, 2); o['ay'] = round(_u(v, 16, 10) * 0.05 - 25.6, 2); o['bp'] = round(_u(v, 32, 10) * 0.25 - 42.5, 1)
    r = self._get(0x368, now)
    if r:
      v = r[0]; o['cs'] = _u(v, 12, 4); o['set'] = _u(v, 48, 8); o['su'] = 'km/h' if _u(v, 31, 1) else 'mph'
      o['regen'] = bool(_u(v, 8, 1)); o['hold'] = _u(v, 3, 3)
    r = self._get(0x201, now, DYN_SLOW_HOLD_S)
    if r:
      o['belt'] = r[1][0] >> 4 & 3            # SDM_bcklDrivStatus (big-endian 5|2): 1 = buckled
    r = self._get(0x246, now, DYN_SLOW_HOLD_S)
    if r:
      o['sun'] = _u(r[0], 25, 1)
    return o
  # DYN-END


# RADAR-BEGIN the Bosch radar's own object list (radar bus, src 1), read-only. Written fresh from tesla_radar.dbc (BO_ 784 / 785,
# little-endian) and checked on 560 000 logged frame pairs from this car: identical to cantools. No class guessing, no history:
# the radar's class only when it is >= 50 % sure (it rarely is), otherwise physics - moving now, or seen moving and stopped (a car
# at a light) = vehicle; standing things (rails, signs, poles) are left out.
RADAR_SRC = 1
RADAR_STALE_S = 0.3


class Radar(object):
  def __init__(self):
    self.a = {}; self.b = {}

  def feed(self, addr, d, now):
    if 0x310 <= addr < 0x36F and len(d) == 8:
      k, q = divmod(addr - 0x310, 3)
      if q == 0: self.a[k] = (int.from_bytes(bytes(d), 'little'), now)
      elif q == 1: self.b[k] = (int.from_bytes(bytes(d), 'little'), now)

  def snapshot(self, now, v_ego):
    """[[slot, d m, y m (left +), ground speed m/s, class 0 unknown / 1 vehicle / 2 two-wheeler / 3 pedestrian, moving state], ...]"""
    out = []
    for k in range(32):
      ra, rb = self.a.get(k), self.b.get(k)
      if not ra or not rb or now - ra[1] > RADAR_STALE_S or now - rb[1] > RADAR_STALE_S:
        continue
      a, b = ra[0], rb[0]
      d = _u(a, 0, 12) * 0.0625
      if not (_u(a, 55, 1) or _u(a, 62, 1)) or not 0 < d < 250 or _u(a, 63, 1) != _u(b, 63, 1) or _u(a, 35, 5) * 3.125 < 50:
        continue
      gs = _u(a, 12, 12) * 0.0625 - 128 + v_ego
      cls = _u(b, 53, 3) if _u(b, 48, 5) * 3.125 >= 50 else 0
      cls = cls if cls in (1, 2, 3) else 0
      ms = _u(b, 22, 2)
      if cls == 3 and abs(gs) >= 4: cls = 1          # nobody walks at 14 km/h
      if cls == 2 and abs(gs) >= 15: cls = 1         # nor cycles at 54 km/h
      if not (cls or abs(gs) >= 3 or ms in (1, 2)):
        continue
      out.append([k, round(d, 1), round(_u(a, 24, 11) * 0.125 - 128, 2), round(gs, 1), cls, ms])
    return out
  # RADAR-END


SIGN_DETECT_FILE = '/data/sign_detect.json'
SIGN_HOLD_S = 2.0             # the detector writes at ~2.5 Hz when enabled


def read_sign_state():
  """OPTIONAL: camerad's own stop-sign/traffic-light detector output, read fresh every call (see
  camerad/cameras/SIGN_DETECT.md). Read-only: never opens the camera or any process, just a file that
  may not exist. Mirrors lightdash.cc's read_sign_state() (C++ twin) field-for-field."""
  try:
    st = os.stat(SIGN_DETECT_FILE)
    if time.time() - st.st_mtime > SIGN_HOLD_S:
      return {'stop': False, 'stop_dist': None, 'light': False, 'light_color': None}
    with open(SIGN_DETECT_FILE) as f:
      d = json.load(f)
  except Exception:
    return {'stop': False, 'stop_dist': None, 'light': False, 'light_color': None}
  return {
    'stop': bool(d.get('stop_sign')), 'stop_dist': d.get('stop_sign_dist_m') if d.get('stop_sign') else None,
    'light': bool(d.get('light_present')), 'light_color': d.get('light_color') if d.get('light_present') else None,
  }

PANDA_STALE_S = 2.0          # firmware: no 0x659 for 2 s -> reset_DAS_data()
X_IDXS = [0., 0.1875, 0.75, 1.6875, 3., 4.6875, 6.75, 9.1875, 12., 15.1875, 18.75, 22.6875, 27., 31.6875, 36.75, 42.1875, 48., 54.1875,
          60.75, 67.6875, 75., 82.6875, 90.75, 99.1875, 108.]
OP_STATUS = {0: 'disabled', 1: 'unavailable', 2: 'available', 3: 'active', 4: 'restricted', 5: 'active (nav)', 8: 'aborting', 9: 'aborted'}


class PandaMirror(object):
  """The firmware's lane/object/status state (defaults from safety_tesla.h lines 154-186) and its expansion into 0x239 / 0x309."""

  def __init__(self):
    self.reset()
    self.lane_idx = 0
    self.t_659 = 0.
    self.emu = None

  def reset(self):
    self.lWidth, self.curv, self.laneRange = 29, [0x64, 0x7F, 0x7F, 0x7F], 160     # firmware defaults (lWidth 29 overflows its 4-bit field)
    self.lLine = self.rLine = 0
    self.lead_class = 0
    self.have_557 = False
    self.obj = {0: (0xFFFFFF00, 0x03FFFF83), 1: (0xFFFFFF01, 0x03FFFF83), 2: (0xFFFFFF02, 0x03FFFF83)}   # lead / left / right slots
    self.status = {'op': 1, 'cc': 0, 'alca': 5, 'hands': 0, 'acc': 0, 'long': 0}

  def feed(self, addr, d, now):
    """addr, d = payload bytes. Mirrors tesla_tx_hook for the frames openpilot sends. 0x554 (switches) is only 3 bytes long on the wire; the others are 8."""
    if addr == 0x554:
      if len(d) >= 1:
        self.emu = bool(d[0] >> 5 & 1)
      return
    if len(d) < 8:
      return
    lo, hi = struct.unpack('<II', bytes(d[:8]))
    if addr == 0x557:
      rq, lq = lo >> 16 & 3, lo >> 18 & 3
      self.rLine = 1 if rq > 1 else 0
      self.lLine = 1 if lq > 1 else 0
      self.lWidth = lo >> 20 & 0xF
      self.curv = [lo >> 24 & 0xFF, hi & 0xFF, hi >> 8 & 0xFF, hi >> 16 & 0xFF]
      self.laneRange = (hi >> 24 & 0x3F) * 4
      self.have_557 = True
    elif addr == 0x559:
      lane = lo & 3
      if lane in self.obj:
        if (lo >> 8) & 0xFF == 0:
          self.obj[lane] = (0xFFFFFF00 | lane, 0x03FFFF83)
        else:
          self.obj[lane] = (lo, hi)
    elif addr == 0x659:
      b = bytes(d[:8])
      if self.t_659 and now - self.t_659 > PANDA_STALE_S:
        self.reset()
      self.t_659 = now
      self.status = {'op': b[0] & 0x0F, 'long': b[0] >> 7 & 1, 'cc': b[3] >> 6 & 3, 'alca': b[3] & 0x1F, 'hands': b[2] & 7,
                     'acc': b[1]}

  def stale(self, now):
    return (not self.t_659) or now - self.t_659 > PANDA_STALE_S

  def lanes_frame(self):
    """exactly safety_tesla.h:705-711 (32-bit int arithmetic, counter in the top nibble of byte 7)"""
    fuse = 2
    c = self.curv
    mlb = ((self.lWidth << 4) + (self.rLine << 1) + self.lLine + (self.laneRange << 8) + (c[0] << 16) + (c[1] << 24)) & 0xFFFFFFFF
    mhb = (0x00F00000 + (self.lane_idx << 28) + c[2] + (c[3] << 8) + ((((self.rLine * fuse) << 2) + (self.lLine * fuse)) << 16)) & 0xFFFFFFFF
    return struct.pack('<II', mlb, mhb)

  def snapshot(self, now):
    stale = self.stale(now)
    st = self.status
    out = {'present': not stale, 'emu': self.emu, 'age': round(now - self.t_659, 1) if self.t_659 else None,
           'default': (not self.have_557) or stale, 'status': dict(st, text=OP_STATUS.get(st['op'], str(st['op']))),
           'lanes': decode_lanes(self.lanes_frame()), 'obj': []}
    for lane in (0, 1, 2):
      lo, hi = self.obj[lane]
      b = struct.pack('<II', lo & 0xFFFFFFFF, hi & 0xFFFFFFFF)
      for v in decode_object(b):
        v['slot'] = ('lead', 'left', 'right')[lane]
        out['obj'].append(v)
    return out


def decode_lanes(b):
  """DAS_lanes 0x239, DBC scaling"""
  return {'width': 2 + 0.3125 * (b[0] >> 4), 'left': b[0] & 1, 'right': b[0] >> 1 & 1, 'range': b[1],
          'c': [round(b[2] * 0.035 - 3.5, 3), round(b[3] * 0.0016 - 0.2, 5), round(b[4] * 2e-5 - 0.0025, 7), round(b[5] * 2.4e-7 - 3e-5, 9)],
          'use': [b[6] & 3, b[6] >> 2 & 3], 'counter': b[7] >> 4}


def decode_object(b):
  """DAS_object 0x309 -> the vehicles present in the frame. Cluster distances are real/IC_LANE_SCALE: returned in REAL metres"""
  v = struct.unpack('<Q', bytes(b[:8]))[0]
  def bits(start, n):                                  # DBC '@1+' little-endian signals
    return v >> start & ((1 << n) - 1)
  res = []
  for k, (typ, rel, dx, vx, dy, vid) in enumerate(((3, 7, 8, 16, 20, 27), (34, 38, 39, 47, 51, 58))):
    dx_raw = bits(dx, 8)
    if dx_raw == 0xFF or dx_raw == 0:
      continue
    res.append({'n': k + 1, 'type': bits(typ, 3), 'relevant': bits(rel, 1), 'dx': dx_raw * 0.5 * IC_LANE_SCALE,
                'dy': round(bits(dy, 7) * 0.35 - 22.05, 2), 'vx': round((bits(vx, 4) * 4 - 30) * IC_LANE_SCALE, 1) if bits(vx, 4) != 15 else None,
                'id': bits(vid, 7 if k == 0 else 6)})
  return res


def build_model(mm):
  """the model's own lanes/path for comparison (every 4 m up to 100 m) and the road edges the adapter packs in freePath"""
  def pts(pd):
    p = list(pd.points)
    return [round(p[i], 2) for i in range(0, min(len(p), 100), 4)] if len(p) >= 100 else None
  out = {'p': pts(mm.path), 'l': pts(mm.leftLane), 'r': pts(mm.rightLane), 'pl': round(mm.leftLane.prob, 2), 'pr': round(mm.rightLane.prob, 2)}
  try:
    fp = list(mm.freePath)
    if len(fp) >= 69 and fp[0] == 1.0:          # road edges first; 0.8.12 adapter appends model FCW after them
      out['e'] = {'x': X_IDXS, 'l': [round(x, 2) for x in fp[1:1 + len(X_IDXS)]], 'r': [round(x, 2) for x in fp[34:34 + len(X_IDXS)]]}
  except Exception:
    pass
  return out


# --------------------------------------------------------------------------------------------- sources
class Frame(object):
  cond = threading.Condition()
  ver = 0
  data = b'{}'
  clients = 0


def publish(obj):
  with Frame.cond:
    Frame.data = json.dumps(obj, separators=(',', ':')).encode()
    Frame.ver += 1
    Frame.cond.notify_all()


def poll_cereal(hz=10.):
  import cereal.messaging as messaging
  sc = messaging.sub_sock('sendcan')
  subs = {n: messaging.sub_sock(n, conflate=True) for n in ('model', 'carState', 'controlsState')}
  scan = messaging.sub_sock('can')
  mirror = PandaMirror(); car = CarState(); body = Body(); dyn = Dyn(); radar = Radar(); model = None
  extra = {'v': 0., 'ang': 0., 'en': False, 'a': 0., 'bl': False, 'br': False, 'gas': False, 'brk': False, 'act': False, 'al1': '', 'al2': '', 'ast': 0}
  while True:
    if not Frame.clients:
      time.sleep(0.3); continue
    now = time.monotonic()
    for ev in messaging.drain_sock(sc, wait_for_one=False):
      for c in ev.sendcan:
        if c.address in WANT:
          mirror.feed(c.address, bytes(c.dat), now)
    for ev in messaging.drain_sock(scan, wait_for_one=False):
      for c in ev.can:
        if c.src == 0 and c.address in (0x30A, 0x283):
          car.feed(c.address, bytes(c.dat), now)
        if c.src == 0 and c.address in BODY_ADDRS:
          body.feed(c.address, bytes(c.dat), now)
        if c.src == 0 and c.address in DYN_ADDRS:
          dyn.feed(c.address, bytes(c.dat), now)
        elif c.src == RADAR_SRC and 0x310 <= c.address < 0x36F:
          radar.feed(c.address, bytes(c.dat), now)
    m = messaging.recv_one_or_none(subs['carState'])
    if m is not None:
      cs = m.carState; extra['v'] = float(cs.vEgo)
      extra['ang'] = float(getattr(cs, 'steeringAngle', getattr(cs, 'steeringAngleDeg', 0.)))
      extra.update(a=round(float(cs.aEgo), 2), bl=bool(cs.leftBlinker), br=bool(cs.rightBlinker), gas=bool(cs.gasPressed), brk=bool(cs.brakePressed))
    m = messaging.recv_one_or_none(subs['controlsState'])
    if m is not None:
      ct = m.controlsState
      extra.update(en=bool(ct.enabled), act=bool(ct.active), al1=str(ct.alertText1)[:80], al2=str(ct.alertText2)[:80],
                   ast={'normal': 0, 'userPrompt': 1, 'critical': 2}.get(str(ct.alertStatus), 0))
    m = messaging.recv_one_or_none(subs['model'])
    if m is not None:
      try: model = build_model(m.model)
      except Exception: model = None
    snap = mirror.snapshot(now); mirror.lane_idx = (mirror.lane_idx + 1) % 16
    snap.update(extra); snap['car'] = car.snapshot(now); snap['body'] = body.snapshot(now); snap['dyn'] = dyn.snapshot(now); snap['rad'] = radar.snapshot(now, extra['v']); snap['model'] = model; snap['sign'] = read_sign_state(); snap['t'] = round(time.time(), 2)
    publish(snap)
    time.sleep(1. / hz)


def car_packers():
  """tesla/teslacan.py - the car's own packers (off-car it needs a stub for openpilot's clip)"""
  import importlib.util
  try:
    import common.numpy_fast  # noqa: F401
  except Exception:
    import types
    m = types.ModuleType('common.numpy_fast'); m.clip = lambda x, a, b: max(a, min(b, x))
    c = types.ModuleType('common'); c.numpy_fast = m
    sys.modules['common'] = c; sys.modules['common.numpy_fast'] = m
  spec = importlib.util.spec_from_file_location('teslacan_lightdash', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'teslacan.py'))
  mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
  return mod


def poll_demo(hz=10.):
  """synthetic frames packed by the car's real teslacan functions, pushed through the same mirror"""
  import math
  tc = car_packers(); mirror = PandaMirror(); t0 = time.monotonic()
  while True:
    if not Frame.clients:
      time.sleep(0.3); continue
    now = time.monotonic(); t = now - t0
    c0 = 0.4 * math.sin(t * 0.3); k = 0.00008 * math.sin(t * 0.17)
    lead = 40 + 10 * math.sin(t * 0.4)
    lq = 3 if t % 12 > 2 else 0
    mirror.feed(0x557, tc.create_fake_DAS_obj_lane_msg(lead, 0.3, 0, 3, lq, -c0, 0.0, -k * 1.0, 0.0, 30, 3.7)[2], now)
    mirror.feed(0x559, tc.create_DAS_LR_object_msg(0, 0, 7, lead, 0.3, -1.5 * math.sin(t * 0.5), 0, 0, 0, 0, 0)[2], now)
    mirror.feed(0x559, tc.create_DAS_LR_object_msg(1, 0, 9, 26, -3.7, 1., 0, 0, 0, 0, 0)[2], now)   # cluster frame: right-positive, so a left car is negative
    mirror.feed(0x659, bytes([5 if t % 30 > 4 else 2, 90, 0, 0xC0 | 8, 0, 0, 0, 0]), now)
    mirror.feed(0x554, bytes([0x20, 0, 0]), now)
    snap = mirror.snapshot(now); mirror.lane_idx = (mirror.lane_idx + 1) % 16
    xs = list(range(0, 100, 4))
    snap.update({'v': 24 + 4 * math.sin(t * 0.2), 'ang': 300 * k, 'en': t % 30 > 4, 'act': t % 30 > 4, 'a': round(1.5 * math.sin(t * 0.5), 2),
                 'bl': t % 20 < 3, 'br': False, 'gas': False, 'brk': False, 'al1': 'TAKE CONTROL' if t % 40 > 37 else '', 'al2': '', 'ast': 2,
                 'body': {'doors': [t % 25 < 2, False, False, False], 'frunk': False, 'trunk': False, 'lo': True, 'hi': t % 12 > 6, 'wiper': t % 50 > 40,
                          'pbrake': False, 'rev': False, 'light': 'night', 'inside': 21.5, 'limit': 50, 'unit': 'km/h'},
                 'car': {'hb': int(t % 12 > 6), 'fl': 0, 'out': 9.5},
                 'dyn': {'kw': round(60 * math.sin(t * 0.5), 1), 'ped': 20, 'gear': 'D', 'ax': round(1.5 * math.sin(t * 0.5), 2), 'ay': round(1.2 * math.sin(t * 0.23), 2),
                         'bp': 0.0, 'cs': 2 if t % 30 > 4 else 1, 'set': 90, 'su': 'km/h', 'regen': t % 20 > 17, 'hold': 0, 'belt': 1, 'sun': 0},
                 'rad': [[3, round(lead, 1), 0.3, 22.0, 0, 1], [5, round(70 + 20 * math.sin(t * 0.1), 1), 3.6, 26.0, 0, 1], [8, 140.0, -0.4, 24.0, 1, 1],
                         [11, round(18 + 4 * math.sin(t * 0.3), 1), -3.4, 23.0, 0, 1], [14, 230.0, 0.2, 25.0, 0, 1], [17, 30.0, 5.8, 1.0, 3, 1]], 't': round(time.time(), 2), 'model': {
        'p': [round(c0 + k * 3 * x ** 3 / 1e0 * 0 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs], 'pl': 0.9, 'pr': 0.8,
        'l': [round(c0 + 1.85 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs], 'r': [round(c0 - 1.85 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs]}})
    publish(snap)
    time.sleep(1. / hz)


# --------------------------------------------------------------------------------------------- web
PAGE = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>cluster view</title><style>html,body{margin:0;height:100%;background:#0b0d10;color:#e8edf2;font:14px system-ui,Arial,sans-serif;overflow:hidden}
canvas{display:block;width:100vw;height:100vh}</style></head><body><canvas id=c></canvas><script>
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,tdv=false,mode=0,W=0,H=0,SC=2;
function rs(){var d=Math.min(window.devicePixelRatio||1,1.5);W=innerWidth;H=innerHeight;cv.width=W*d;cv.height=H*d;g.setTransform(d,0,0,d,0,0);draw()}
addEventListener('resize',rs);cv.addEventListener('click',function(){mode=(mode+1)%3;tdv=(mode==1);draw()});   // drive -> top-down -> diagnostic
function P(x,y){ // real metres (x ahead, y left) -> screen
 if(tdv){var s=H*0.8/110;return [W/2-y*s*4,H*0.92-x*s]}
 var K=H*0.9,d=x+5;return [W/2-y*K/d,H*0.38+2.5*K/d]}
function line(pts,col,w,dash){if(pts.length<2)return;g.beginPath();g.setLineDash(dash||[]);g.strokeStyle=col;g.lineWidth=w;pts.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke();g.setLineDash([])}
function poly(c,x){return c[0]+c[1]*x+c[2]*x*x+c[3]*x*x*x}
function rr(x,y,w,h,r){g.beginPath();g.moveTo(x+r,y);g.lineTo(x+w-r,y);g.quadraticCurveTo(x+w,y,x+w,y+r);g.lineTo(x+w,y+h-r);g.quadraticCurveTo(x+w,y+h,x+w-r,y+h);g.lineTo(x+r,y+h);g.quadraticCurveTo(x,y+h,x,y+h-r);g.lineTo(x,y+r);g.quadraticCurveTo(x,y,x+r,y);g.closePath()}
function carBack(cx,y0,w,rel){ // a car seen from behind; y0 = where its tyres touch the road, w = 1.8 m of real width
 var h=w*0.8,dark='#0e1217',gr;
 if(w<14){g.fillStyle='#aab3bf';g.fillRect(cx-w/2,y0-h,w,h);return}
 g.save();g.translate(cx,y0);g.scale(1,0.12);g.fillStyle='rgba(0,0,0,.55)';g.beginPath();g.arc(0,0,w*0.58,0,6.2832);g.fill();g.restore();
 if(rel){g.save();g.translate(cx,y0);g.scale(1,0.18);gr=g.createRadialGradient(0,0,0,0,0,w*0.8);gr.addColorStop(0,'rgba(255,60,60,.38)');gr.addColorStop(1,'rgba(255,60,60,0)');g.fillStyle=gr;g.beginPath();g.arc(0,0,w*0.8,0,6.2832);g.fill();g.restore()}
 g.fillStyle=dark;rr(cx-w*0.46,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();rr(cx+w*0.29,y0-h*0.30,w*0.17,h*0.30,w*0.03);g.fill();
 gr=g.createLinearGradient(0,y0-h,0,y0-h*0.15);gr.addColorStop(0,'#eef1f5');gr.addColorStop(0.55,'#bcc3cd');gr.addColorStop(1,'#7c8592');g.fillStyle=gr;
 rr(cx-w/2,y0-h*0.58,w,h*0.40,w*0.08);g.fill();
 g.beginPath();g.moveTo(cx-w*0.42,y0-h*0.56);g.quadraticCurveTo(cx-w*0.36,y0-h*0.99,cx-w*0.28,y0-h);g.lineTo(cx+w*0.28,y0-h);g.quadraticCurveTo(cx+w*0.36,y0-h*0.99,cx+w*0.42,y0-h*0.56);g.closePath();g.fill();
 gr=g.createLinearGradient(0,y0-h*0.94,0,y0-h*0.60);gr.addColorStop(0,'#2b3747');gr.addColorStop(1,'#0f151d');g.fillStyle=gr;
 g.beginPath();g.moveTo(cx-w*0.36,y0-h*0.62);g.quadraticCurveTo(cx-w*0.31,y0-h*0.92,cx-w*0.25,y0-h*0.94);g.lineTo(cx+w*0.25,y0-h*0.94);g.quadraticCurveTo(cx+w*0.31,y0-h*0.92,cx+w*0.36,y0-h*0.62);g.closePath();g.fill();
 g.fillStyle=rel?'#ff4d52':'#9e4a4e';if(rel){g.shadowColor='rgba(255,70,70,.95)';g.shadowBlur=w*0.12}
 rr(cx-w*0.47,y0-h*0.51,w*0.22,h*0.09,w*0.02);g.fill();rr(cx+w*0.25,y0-h*0.51,w*0.22,h*0.09,w*0.02);g.fill();g.shadowBlur=0;
 g.fillStyle='#d5dbe2';g.fillRect(cx-w*0.10,y0-h*0.33,w*0.20,h*0.07);
 g.fillStyle='rgba(0,0,0,.30)';rr(cx-w/2,y0-h*0.23,w,h*0.05,w*0.02);g.fill()}
function person(cx,y0,h,col){var r=h*0.13;g.fillStyle=col;g.beginPath();g.arc(cx,y0-h+r,r,0,6.2832);g.fill();rr(cx-h*0.13,y0-h+r*2.3,h*0.26,h*0.42,h*0.08);g.fill();
 g.fillRect(cx-h*0.11,y0-h*0.32,h*0.08,h*0.32);g.fillRect(cx+h*0.03,y0-h*0.32,h*0.08,h*0.32)}
function leadOf(){var ld=null;for(var i=0;i<SO.length;i++){var o=SO[i];if(o.slot=='lead')return o;if(o.kind=='car'&&Math.abs(o.dy)<1.6&&(!ld||o.dx<ld.dx))ld=o}return ld}   // the cluster lead, else the nearest radar car in the lane (to 250 m)
function carTop(cx,yr,w,l,rel){ // seen from above, rear edge at screen y=yr, nose pointing up (4.5 m long, 1.8 m wide)
 var dark='#0e1217',gl='#18212c',gr;
 g.fillStyle=dark;rr(cx-w*0.53,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.84,w*0.10,l*0.18,w*0.03);g.fill();rr(cx-w*0.53,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();rr(cx+w*0.43,yr-l*0.20,w*0.10,l*0.18,w*0.03);g.fill();
 gr=g.createLinearGradient(cx-w/2,0,cx+w/2,0);gr.addColorStop(0,'#aeb6c1');gr.addColorStop(0.5,'#e6e9ee');gr.addColorStop(1,'#aeb6c1');g.fillStyle=gr;rr(cx-w/2,yr-l,w,l,w*0.30);g.fill();
 g.fillStyle=gl;rr(cx-w*0.38,yr-l*0.66,w*0.76,l*0.22,w*0.10);g.fill();rr(cx-w*0.36,yr-l*0.30,w*0.72,l*0.13,w*0.08);g.fill();
 g.fillStyle=rel?'#ff4d52':'#9e4a4e';if(rel){g.shadowColor='rgba(255,70,70,.95)';g.shadowBlur=w*0.5}
 rr(cx-w*0.46,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill();rr(cx+w*0.24,yr-l*0.06,w*0.22,l*0.04,w*0.02);g.fill();g.shadowBlur=0}
function drawDiag(){g.fillStyle='#0b0d10';g.fillRect(0,0,W,H);if(!S||!S.lanes){g.fillStyle='#9aa4af';g.fillText('waiting for data...',20,30);return}
 var L=S.lanes,R=Math.min(110,L.range*SC),pw=[],i;
 // grid
 g.strokeStyle='#1b2128';g.lineWidth=1;for(i=10;i<=100;i+=10){var a=P(i,-6),b=P(i,6);g.beginPath();g.moveTo(a[0],a[1]);g.lineTo(b[0],b[1]);g.stroke()}
 var M=SMD; // the model's own, faint (smoothed)
 if(M){var xs=[];for(i=0;i<25;i++)xs.push(i*4);
  if(M.l)line(M.l.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.r)line(M.r.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.p){var cl=M.p.map(function(y,k){return P(xs[k],y-.9)}),cr=M.p.map(function(y,k){return P(xs[k],y+.9)}).reverse();g.fillStyle='rgba(60,200,120,.12)';g.beginPath();cl.concat(cr).forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.fill()}
  if(M.e){line(M.e.l.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2);line(M.e.r.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2)}}
 // what the cluster is told: virtual lane = centre curve +- width/2 (cluster x = real/2)
 var cc=[],lf=[],rt=[];for(i=0;i<=R;i+=2){var xc=i/SC,yc=-poly(L.c,xc);cc.push(P(i,yc));lf.push(P(i,yc+L.width/2));rt.push(P(i,yc-L.width/2))}
 line(cc,'rgba(255,255,255,.18)',1,[6,8]);
 line(lf,L.left?'#ffd23f':'#555e68',L.left?5:2,L.left?null:[10,10]);line(rt,L.right?'#ffd23f':'#555e68',L.right?5:2,L.right?null:[10,10]);
 // lead / side vehicles
 SO.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w,h;g.globalAlpha=o.al;
  var pd=o.kind=='ped'||o.kind=='ped?';if(o.kind=='ped?')g.globalAlpha=o.al*0.55;
  if(tdv){h=Math.max(26,4.5*H*0.8/110*1.6);w=Math.max(12,h/2.2);if(pd){h=w;g.fillStyle='#f0b429';g.beginPath();g.arc(p[0],p[1]-w/2,w/2.5,0,6.2832);g.fill()}else carTop(p[0],p[1],o.kind=='bike'?w*0.5:w,h,o.relevant)}   // an icon with car proportions: the top view stretches the sideways scale 4x, so true scale would look squashed
  else{w=Math.max(12,1.8*H*0.9/(o.dx+5));h=w*0.8;if(pd){h=w*1.1;person(p[0],p[1],h,'#f0b429')}else carBack(p[0],p[1],o.kind=='bike'?w*0.45:w,o.relevant)}
  g.fillStyle='#e8edf2';g.textAlign='center';g.fillText((o.slot=='r'?(o.kind=='ped'?'person':o.kind=='ped?'?'person?':o.kind):o.slot)+' '+Math.round(o.dx)+' m',p[0],p[1]-h-6);g.textAlign='left';g.globalAlpha=1});
 // header
 var st=S.status,y0=22;g.font='15px system-ui';g.fillStyle='#e8edf2';
 g.fillText('openpilot '+(S.en?'ENGAGED':'off')+'   speed '+Math.round((S.v||0)*3.6)+' km/h   wheel '+(S.ang||0).toFixed(1)+' deg',16,y0);
 g.fillStyle=S.present?'#7bd88f':'#e5383b';g.fillText('panda data: '+(S.present?'live':('STALE ('+S.age+' s) - cluster is shown defaults')),16,y0+20);
 g.fillStyle='#e8edf2';g.fillText('cluster status: '+st.text+'   emulation: '+(S.emu===null?'?':S.emu?'on':'OFF (panda sends nothing)'),16,y0+40);
 g.fillText('lanes: width '+L.width.toFixed(2)+' m  left '+(L.left?'drawn':'not drawn')+'  right '+(L.right?'drawn':'not drawn')+'  range '+L.range+' (x'+SC+')'+(S.default?'   [DEFAULT frame - no lane data from openpilot]':''),16,y0+60);
 var B=S.body||{};g.fillText('accel '+(S.a||0).toFixed(2)+' m/s2   blinker '+(S.bl?'L ':'')+(S.br?'R':'')+'   doors '+JSON.stringify(B.doors||null)+'   limit '+(B.limit||'-')+' '+(B.unit||'')+'   light '+(B.light||'?')+'   wipers '+(B.wiper?'on':'off')+(S.al1?'   alert: '+S.al1:''),16,y0+80);
 g.fillStyle='#9aa4af';g.font='12px system-ui';g.fillText('yellow = lane lines sent to the cluster   cyan = model lanes   green = model path   red = road edges   tap = next view (drive / top / diagnostic)',16,H-14)}
function grad(y0,y1,st){var gr=g.createLinearGradient(0,y0,0,y1);for(var i=0;i<st.length;i++)gr.addColorStop(st[i][0],st[i][1]);return gr}
function ribbon(a,b,fill){if(a.length<2)return;g.beginPath();a.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});for(var i=b.length-1;i>=0;i--)g.lineTo(b[i][0],b[i][1]);g.closePath();g.fillStyle=fill;g.fill()}
function chip(x,y,t,col,bg){g.font='600 12px system-ui';var w=g.measureText(t).width+20;g.fillStyle=bg;rr(x,y,w,24,12);g.fill();g.fillStyle=col;g.textAlign='left';g.fillText(t,x+10,y+16);return w}
function pill(cx,cy,t){g.font='600 12px system-ui';var w=g.measureText(t).width+14;g.fillStyle='rgba(0,0,0,.45)';rr(cx-w/2,cy-11,w,20,10);g.fill();g.fillStyle='#dfe5ec';g.textAlign='center';g.fillText(t,cx,cy+4);g.textAlign='left'}
function cruiseIcon(cx,cy,r,col){g.strokeStyle=col;g.lineWidth=3;g.beginPath();g.arc(cx,cy,r,Math.PI*0.80,Math.PI*2.20);g.stroke();g.beginPath();g.moveTo(cx,cy+r*0.55);g.lineTo(cx+r*0.45,cy-r*0.45);g.stroke();g.fillStyle=col;g.beginPath();g.arc(cx,cy+r*0.55,3,0,6.2832);g.fill()}
function wheelIcon(cx,cy,r,col){g.strokeStyle=col;g.lineWidth=3.2;g.beginPath();g.arc(cx,cy,r,0,6.2832);g.stroke();g.beginPath();g.moveTo(cx-r,cy);g.lineTo(cx-r*0.35,cy);g.moveTo(cx+r,cy);g.lineTo(cx+r*0.35,cy);g.moveTo(cx,cy+r*0.35);g.lineTo(cx,cy+r);g.stroke();g.beginPath();g.arc(cx,cy,r*0.22,0,6.2832);g.fillStyle=col;g.fill()}
function beamIcon(cx,cy,r,col){g.strokeStyle=col;g.fillStyle=col;g.lineWidth=2.6;g.beginPath();g.moveTo(cx+r*0.15,cy-r*0.8);g.quadraticCurveTo(cx-r*0.95,cy-r*0.8,cx-r*0.95,cy);g.quadraticCurveTo(cx-r*0.95,cy+r*0.8,cx+r*0.15,cy+r*0.8);g.closePath();g.fill();for(var k=-1;k<=1;k++){g.beginPath();g.moveTo(cx+r*0.45,cy+k*r*0.62);g.lineTo(cx+r*1.1,cy+k*r*0.62);g.stroke()}}
function drawDrive(){
 g.fillStyle=grad(0,H,[[0,'#06090d'],[0.45,'#0b131b'],[1,'#10202c']]);g.fillRect(0,0,W,H);
 var eng=!!(S&&S.en),BLUE='rgba(64,156,255,0.95)';
 g.save();g.translate(W/2,H*0.72);g.scale(0.55,1);var rg=g.createRadialGradient(0,0,0,0,0,H*0.62);rg.addColorStop(0,eng?'rgba(40,120,210,.16)':'rgba(110,135,175,.10)');rg.addColorStop(1,'rgba(40,120,210,0)');g.fillStyle=rg;g.beginPath();g.arc(0,0,H*0.62,0,6.2832);g.fill();g.restore();
 if(!S||!S.lanes){g.fillStyle='#7d8792';g.font='16px system-ui';g.textAlign='center';g.fillText('waiting for data...',W/2,H/2);g.textAlign='left';return}
 var M=SMD,i,xs=[];for(i=0;i<25;i++)xs.push(i*4);
 var yb=P(0,0)[1],yh=P(110,0)[1];
 if(M&&M.l&&M.r){
  var lw=Math.max(2.6,Math.min(4.4,(M.l[2]-M.r[2])||3.4));
  var mk=function(arr,off){var q=arr.map(function(y,k){return P(xs[k],y+off)});q.unshift(P(-1.5,arr[0]+off));q.unshift(P(-3.5,arr[0]+off));return q};   // two extra points behind the car so the road runs to the bottom of the screen
  var lp=mk(M.l,0),rp=mk(M.r,0),ll=mk(M.l,lw),rl=mk(M.r,-lw);
  ribbon(lp,rp,grad(yb,yh,[[0,eng?'rgba(60,140,230,.22)':'rgba(150,165,190,.16)'],[1,'rgba(60,140,230,0)']]));
  var sd=grad(yb,yh,[[0,'rgba(150,165,190,.07)'],[1,'rgba(150,165,190,0)']]);ribbon(ll,lp,sd);ribbon(rp,rl,sd);
  var path=function(a){g.beginPath();a.forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke()};
  g.save();g.lineCap='round';
  g.strokeStyle=grad(yb,yh,[[0,'rgba(190,200,215,.28)'],[1,'rgba(190,200,215,0)']]);g.lineWidth=2;[ll,rl].forEach(path);
  if(eng){g.shadowColor='rgba(64,156,255,.85)';g.shadowBlur=14}
  g.strokeStyle=grad(yb,yh,[[0,eng?BLUE:'rgba(235,240,248,.80)'],[0.75,eng?'rgba(64,156,255,.45)':'rgba(235,240,248,.30)'],[1,'rgba(64,156,255,0)']]);g.lineWidth=5;[lp,rp].forEach(path);
  g.restore();
  if(M.e){g.save();g.strokeStyle=grad(yb,yh,[[0,'rgba(170,180,195,.30)'],[1,'rgba(170,180,195,0)']]);g.lineWidth=2;[M.e.l,M.e.r].forEach(function(a){g.beginPath();a.forEach(function(y,k){var p=P(M.e.x[k],y);k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke()});g.restore()}
 }
 var LD=leadOf();SO.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w=Math.max(12,1.8*H*0.9/(o.dx+5)),hh=w*0.8;g.globalAlpha=o.al;
  if(o.kind=='ped'||o.kind=='ped?'){hh=w*1.1;g.globalAlpha=o.al*(o.kind=='ped'?1:0.55);person(p[0],p[1],hh,'#f0b429')}else carBack(p[0],p[1],o.kind=='bike'?w*0.45:w,o.relevant);
  if(o.slot!='r'||o===LD)pill(p[0],p[1]-hh-14,Math.round(o.dx)+' m');g.globalAlpha=1});   // distance on the cluster cars and the lead; the extra radar objects stay quiet
 var ew=Math.min(H*0.27,W*0.2),ey=H*0.92;
 if(eng){g.save();g.translate(W/2,ey);g.scale(1,0.14);var eg=g.createRadialGradient(0,0,0,0,0,ew*0.95);eg.addColorStop(0,'rgba(64,156,255,.55)');eg.addColorStop(1,'rgba(64,156,255,0)');g.fillStyle=eg;g.beginPath();g.arc(0,0,ew*0.95,0,6.2832);g.fill();g.restore()}
 carBack(W/2,ey,ew,false);
 // the middle of the instrument panel: icons either side of the speed
 var pw=Math.min(W*0.5,380),py=14,ph=96,px=W/2-pw/2;
 g.fillStyle='rgba(8,14,20,.72)';rr(px,py,pw,ph,18);g.fill();g.strokeStyle='rgba(255,255,255,.06)';g.lineWidth=1;g.stroke();
 var ic=eng?'rgba(64,156,255,1)':'rgba(120,130,142,.9)';
 cruiseIcon(px+pw*0.14,py+ph*0.46,15,ic);wheelIcon(px+pw*0.86,py+ph*0.46,16,ic);
 var cr=S.car||{},bdy=S.body||{};if(cr.hb||cr.fl||bdy.lo)beamIcon(px+pw*0.14,py+ph*0.46+34,9,cr.hb?'rgba(64,156,255,1)':bdy.lo&&!cr.fl?'rgba(110,220,130,1)':'rgba(235,240,248,.85)');   // blue = high, green = low, white = flash
 if(cr.out!==null&&cr.out!==undefined){g.fillStyle='#aab4c2';g.font='500 14px system-ui';g.textAlign='center';g.fillText(cr.out.toFixed(1)+' \u00b0C',px+pw*0.86,py+ph*0.46+38);g.textAlign='left'}
 var sg=S.sign||{};if(sg.stop||sg.light){g.font='600 12px system-ui';var parts=[];
  if(sg.stop)parts.push({t:'STOP '+(sg.stop_dist!=null?Math.round(sg.stop_dist)+'m':''),bg:'rgba(180,30,30,.85)',col:'#fff'});
  if(sg.light){var lc=['rgba(220,40,40,.9)','rgba(230,180,20,.9)','rgba(40,180,70,.9)'][sg.light_color]||'rgba(140,140,140,.9)';parts.push({t:'●',bg:lc,col:'#fff'})}
  var tw=0;for(var k=0;k<parts.length;k++)tw+=g.measureText(parts[k].t).width+20+(k?6:0);
  var cx0=W/2-tw/2,cy0=py+ph+8;
  for(var k=0;k<parts.length;k++){var w=chip(cx0,cy0,parts[k].t,parts[k].col,parts[k].bg);cx0+=w+6}
 }
 extras(px,py,pw,ph);
 g.fillStyle='#f3f6fa';g.font='300 58px system-ui';g.textAlign='center';g.fillText(Math.round((S.v||0)*3.6),W/2,py+62);g.fillStyle='#8590a0';g.font='500 13px system-ui';g.fillText('km/h',W/2,py+84);
 g.fillStyle=S.present?'rgba(130,150,170,.55)':'rgba(255,120,120,.9)';g.font='11px system-ui';
 g.fillText((S.present?'panda live':'PANDA STALE')+'  ·  cluster '+S.status.text+'  ·  emulation '+(S.emu===null?'?':S.emu?'on':'off')+(S.default?'  ·  no lane data':'')+'  ·  tap: next view',W/2,H-12);g.textAlign='left'}
var SM={},SMM={},ST=0,SO=[],SMD=null,SU=0,SP=0,GN=0;
function ease(dt,tau){return 1-Math.exp(-dt/tau)}
function cl(v,m){return Math.max(-m,Math.min(m,v))}
function smooth(){ // cluster-style cars: glide (predicted between the 10 Hz updates), hold through short gaps, coast + fade out when lost, fade / slide in when new
 var now=Date.now()/1000,dt=ST?Math.min(0.25,now-ST):0.1;ST=now;SO=[];if(!S)return;var id,q,k=ease(dt,0.5);
 if(SU!==SP){SP=SU;var seen={};   // a new data frame arrived
  var c0=(S.lanes&&S.lanes.c)?S.lanes.c[0]:0,cur={},L=[];
  (S.obj||[]).forEach(function(o){L.push({key:o.slot+':'+o.id,slot:o.slot,dx:o.dx,y:c0-o.dy,rel:o.relevant,kind:'car',rid:(o.id>0&&o.id<61)?'c'+o.id:null})});
  var nc=L.length;   // + the radar's own objects (up to 250 m) that the cluster frame does not already have
  (S.rad||[]).forEach(function(r){for(var i=0;i<nc;i++)if(Math.abs(L[i].dx-r[1])<5&&Math.abs(L[i].y-r[2])<1.8)return;
   L.push({key:'r:'+r[0],slot:'r',dx:r[1],y:r[2],rel:0,kind:r[4]==3?'ped':r[4]==2?'bike':(r[4]==0&&r[5]==1&&Math.abs(r[3])<3)?'ped?':'car',rid:'r'+r[0]})});   // ped? = moving at walking pace, radar not sure what it is
  L.forEach(function(o){cur[o.key]=1});
  L.forEach(function(o){id=o.key;q=SM[id];
   var oy=o.y;   // the cluster frame is RIGHT-positive and centred on the lane (car code: dy = curv0 - yRel): back to real metres, left-positive, like the lanes drawn here
   if(q&&(Math.abs(q.tx-o.dx)>20||Math.abs(q.ty-oy)>3)){SM[id+'#'+(GN++)]=q;q.lost=q.lost||now;q=null}   // a jump = another car: the old one coasts out, the new one starts
   if(!q){var j,c,hit=null;for(j in SM){c=SM[j];if(cur[j]||seen[j])continue;   // a track this frame no longer has: the same car moved to another slot (right lane -> lead on a cut-in) or came back under a new id
     if((o.rid&&c.rid===o.rid&&Math.abs(c.tx-o.dx)<15)||(Math.abs(c.tx-o.dx)<6&&Math.abs(c.ty-oy)<2.5)){hit=j;break}}
    if(hit!==null){q=SM[hit];delete SM[hit];q.lost=0}}   // keep it: it glides from where it was instead of a new car fading in
   if(!q){var near=o.dx<40,side=oy>=0?1:-1;q={dx:o.dx,dy:near?oy+side*1.6:oy,tx:o.dx,ty:oy,vx:0,vy:0,tu:now,al:0,fin:near?0.6:0.25}}   // close (cut-in): slow fade, slides in from its own side; far: quick fade, it is small anyway
   else{var du=Math.max(0.05,now-q.tu);q.vx=cl(q.vx+((o.dx-q.tx)/du-q.vx)*0.5,30);q.vy=cl(q.vy+((oy-q.ty)/du-q.vy)*0.5,3);q.tx=o.dx;q.ty=oy;q.tu=now}
   q.lost=0;q.slot=o.slot;q.rel=o.rel;q.rid=o.rid;q.kind=o.kind;SM[id]=q;seen[id]=1});
  for(id in SM)if(!seen[id]&&!SM[id].lost)SM[id].lost=now}
 for(id in SM){q=SM[id];
  if(q.lost){q.tx+=q.vx*dt;q.ty+=q.vy*dt;q.tu=now;if(now-q.lost>0.5)q.al-=dt/0.7;if(q.al<=0||q.tx<-3){delete SM[id];continue}}   // hold 0.5 s, then fade over 0.7 s while it keeps moving
  else q.al=Math.min(1,q.al+dt/q.fin);
  var pt=Math.min(0.3,now-q.tu)+0.45;q.dx+=(q.tx+q.vx*pt-q.dx)*k;q.dy+=(q.ty+q.vy*pt-q.dy)*k;   // aim ahead by the smoothing lag: smooth AND where the car really is
  SO.push({dx:q.dx,dy:q.dy,slot:q.slot,relevant:q.rel,kind:q.kind||'car',al:Math.max(0,Math.min(1,q.al))})}
 var M=S.model;if(!M){SMD=null;return}
 ['l','r','p'].forEach(function(n){var a=M[n],b=SMM[n];if(!a){SMM[n]=null;return}if(!b||b.length!=a.length){SMM[n]=a.slice();return}for(var i=0;i<a.length;i++)b[i]+=(a[i]-b[i])*ease(dt,0.2)});
 SMD={l:SMM.l,r:SMM.r,p:SMM.p,e:M.e,pl:M.pl,pr:M.pr}}
function limitSign(cx,cy,r,v){g.fillStyle='#fff';g.beginPath();g.arc(cx,cy,r,0,6.2832);g.fill();g.strokeStyle='#d0202a';g.lineWidth=r*0.18;g.beginPath();g.arc(cx,cy,r*0.86,0,6.2832);g.stroke();g.fillStyle='#111';g.font='700 '+Math.round(r*0.85)+'px system-ui';g.textAlign='center';g.fillText(v,cx,cy+r*0.3);g.textAlign='left'}
function arrow(cx,cy,s,dir,on){g.fillStyle=on?'#33d17a':'rgba(120,130,142,.22)';g.beginPath();g.moveTo(cx+dir*s,cy);g.lineTo(cx,cy-s*0.8);g.lineTo(cx,cy-s*0.35);g.lineTo(cx-dir*s*0.9,cy-s*0.35);g.lineTo(cx-dir*s*0.9,cy+s*0.35);g.lineTo(cx,cy+s*0.35);g.lineTo(cx,cy+s*0.8);g.closePath();g.fill()}
function extras(px,py,pw,ph){var B=S.body||{},blink=(Date.now()%800)<450,i;
 arrow(px-24,py+ph*0.46,14,-1,S.bl&&blink);arrow(px+pw+24,py+ph*0.46,14,1,S.br&&blink);
 if(B.limit){if(px>=100)limitSign(px-72,py+ph*0.46,24,B.limit);else if(px>=56)limitSign(Math.max(23,px-62),py+ph*0.46,20,B.limit);else limitSign(W-26,py+ph+16,20,B.limit)}
 var Y=S.dyn||{},a=Math.max(-3,Math.min(3,S.a||0)),bw=pw*0.36,bx=px+pw/2,by=py+ph-9;
 g.fillStyle='rgba(255,255,255,.08)';g.fillRect(bx-bw/2,by,bw,4);
 if(Y.kw!=null){var f=Y.kw>=0?Math.min(1,Y.kw/250):Math.max(-1,Y.kw/60);   // motor power: right/white = drive (full 250 kW), left/green = regen (full 60 kW)
  g.fillStyle=Y.kw>=0?'#e8edf2':'#33d17a';g.fillRect(f>=0?bx:bx+f*bw/2,by,Math.abs(f)*bw/2,4);
  g.fillStyle=Y.kw>=0?'#aab4c2':'#33d17a';g.font='600 11px system-ui';g.textAlign='left';g.fillText(Math.round(Y.kw)+' kW',bx+bw/2+6,by+5)}
 else{g.fillStyle=a>=0?'#33d17a':'#ff5a5f';g.fillRect(a>=0?bx:bx+a/3*bw/2,by,Math.abs(a)/3*bw/2,4)}   // no drive-unit data: acceleration, full = 3 m/s2
 if(Y.ax!=null){var gx=40,gr=28,gy=H-110,fs=9.81*0.6;   // G-meter: dot = where the car accelerates (up = forward, left = left), ring = 0.3 g, edge = 0.6 g
  g.strokeStyle='rgba(255,255,255,.16)';g.lineWidth=1;g.beginPath();g.arc(gx,gy,gr,0,6.2832);g.stroke();g.beginPath();g.arc(gx,gy,gr/2,0,6.2832);g.stroke();
  g.beginPath();g.moveTo(gx-gr,gy);g.lineTo(gx+gr,gy);g.moveTo(gx,gy-gr);g.lineTo(gx,gy+gr);g.stroke();
  g.fillStyle='#ffd23f';g.beginPath();g.arc(gx-cl(Y.ay/fs,1)*gr,gy-cl(Y.ax/fs,1)*gr,4,0,6.2832);g.fill();
  g.fillStyle='#8590a0';g.font='11px system-ui';g.textAlign='center';g.fillText((Math.sqrt(Y.ax*Y.ax+Y.ay*Y.ay)/9.81).toFixed(2)+' g',gx,gy+gr+14);
  if(Y.bp!=null&&Y.bp>1)g.fillText('brake '+Math.round(Y.bp)+' bar',gx,gy+gr+28);else if(Y.ped)g.fillText('pedal '+Y.ped+'%',gx,gy+gr+28);g.textAlign='left'}
 var ch=[],D=B.doors||[],nm=['FL','FR','RL','RR'],warn='rgba(205,120,20,.92)';
 if(Y.gear)ch.push({t:Y.gear,bg:'rgba(50,58,70,.92)'});
 ch.push(S.en?{t:'openpilot ON',bg:'rgba(40,110,210,.92)'}:{t:'openpilot off',bg:'rgba(70,78,90,.85)'});
 if(Y.cs>=2&&Y.cs<=4&&Y.set)ch.push({t:'cruise '+Y.set+' '+(Y.su||''),bg:Y.cs==4?'rgba(70,78,90,.85)':'rgba(40,110,210,.92)'});   // 2 on, 3 standstill, 4 override
 if(Y.regen&&Y.gear=='D')ch.push({t:'regen limited',bg:warn});if(Y.hold==3)ch.push({t:'HOLD',bg:'rgba(40,140,80,.92)'});
 if(Y.belt===0&&Y.gear=='D')ch.push({t:'driver seatbelt!',bg:'rgba(180,40,40,.92)'});
 for(i=0;i<4;i++)if(D[i])ch.push({t:'door '+nm[i]+' open',bg:warn});
 if(B.frunk)ch.push({t:'frunk open',bg:warn});if(B.trunk)ch.push({t:'trunk open',bg:warn});
 if(B.pbrake)ch.push({t:'park brake',bg:'rgba(180,40,40,.92)'});if(B.rev)ch.push({t:'R',bg:'rgba(90,90,90,.92)'});
 if(S.brk)ch.push({t:'brake',bg:'rgba(180,40,40,.92)'});if(B.wiper)ch.push({t:'wipers',bg:'rgba(60,120,170,.92)'});
 if(B.light&&B.light!='day')ch.push({t:B.light,bg:'rgba(60,70,110,.92)'});
 if(B.inside!=null)ch.push({t:'in '+B.inside.toFixed(1)+' \u00b0C',bg:'rgba(50,58,70,.92)'});
 g.font='600 12px system-ui';var rows=[[]],rw=[0],maxw=W-16;   // chips wrap onto more rows on a narrow screen
 ch.forEach(function(c){var w=g.measureText(c.t).width+20,r=rows.length-1;if(rows[r].length&&rw[r]+6+w>maxw){rows.push([]);rw.push(0);r++}rw[r]+=(rows[r].length?6:0)+w;rows[r].push(c)});
 rows.forEach(function(row,k){var cx0=W/2-rw[k]/2,cy0=py+ph+40+k*30;row.forEach(function(c){cx0+=chip(cx0,cy0,c.t,'#fff',c.bg)+6})});
 if(S.al1){var col=S.ast==2?'rgba(200,30,40,.93)':S.ast==1?'rgba(210,140,20,.93)':'rgba(30,40,55,.93)',bw2=Math.min(W*0.8,560),bh=S.al2?62:42,bx2=W/2-bw2/2,by2=H-bh-34;
  g.fillStyle=col;rr(bx2,by2,bw2,bh,12);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 18px system-ui';g.fillText(S.al1,W/2,by2+26);
  if(S.al2){g.font='500 14px system-ui';g.fillText(S.al2,W/2,by2+48)}g.textAlign='left'}}
var DIM=false;function draw(){smooth();var dm=!!(S&&S.dyn&&S.dyn.sun===0);if(dm!==DIM){DIM=dm;cv.style.filter=dm?'brightness(0.8)':''}if(mode==0)drawDrive();else drawDiag()}   // dimmer after sunset
function conn(){var es=new EventSource('/stream');es.onmessage=function(e){S=JSON.parse(e.data);SU=Date.now()/1000};es.onerror=function(){es.close();setTimeout(conn,1500)}}
var LF=0;function loop(t){if(t-LF>=33){LF=t;draw()}requestAnimationFrame(loop)}   // ~30 fps: data arrives at 10 Hz, the smoothing fills the frames in between
rs();conn();requestAnimationFrame(loop);</script></body></html>"""


class Server(ThreadingMixIn, HTTPServer):
  daemon_threads = True
  allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
  protocol_version = 'HTTP/1.1'

  def log_message(self, *a):
    pass

  def _send(self, code, body, ctype):
    self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(body)))
    self.send_header('Cache-Control', 'no-store'); self.end_headers(); self.wfile.write(body)

  def do_GET(self):
    p = self.path.split('?')[0]
    if p in ('/', '/index.html'):
      return self._send(200, PAGE.encode(), 'text/html; charset=utf-8')
    if p == '/s':
      with Frame.cond:
        Frame.clients += 1
      try:
        time.sleep(0.6)            # let the poller produce a frame if idle
        return self._send(200, Frame.data, 'application/json')
      finally:
        with Frame.cond:
          Frame.clients -= 1
    if p == '/health':
      return self._send(200, b'ok', 'text/plain')
    if p == '/stream':
      self.send_response(200); self.send_header('Content-Type', 'text/event-stream'); self.send_header('Cache-Control', 'no-store')
      self.send_header('Connection', 'keep-alive'); self.end_headers()
      with Frame.cond:
        Frame.clients += 1; seen = -1
      try:
        while True:
          with Frame.cond:
            Frame.cond.wait_for(lambda: Frame.ver != seen, timeout=5)
            data, seen, fresh = Frame.data, Frame.ver, Frame.ver != seen
          self.wfile.write((b'data: ' + data + b'\n\n') if fresh else b': keepalive\n\n'); self.wfile.flush()
      except (BrokenPipeError, ConnectionResetError, OSError):
        pass
      finally:
        with Frame.cond:
          Frame.clients -= 1
      return
    self._send(404, b'not found', 'text/plain')


def main():
  ap = argparse.ArgumentParser(); ap.add_argument('--demo', action='store_true'); ap.add_argument('--hz', type=float, default=10.)
  a = ap.parse_args()
  try:
    os.nice(19)
  except Exception:
    pass
  threading.Thread(target=poll_demo if a.demo else poll_cereal, args=(a.hz,), daemon=True).start()
  port = int(os.environ.get('LIGHTDASH_PORT', '8090')); bind = os.environ.get('LIGHTDASH_BIND', '0.0.0.0')
  print('lightdash on http://%s:%d/  (%s)' % (bind, port, 'demo data' if a.demo else 'sendcan + model'), flush=True)
  try:
    Server((bind, port), Handler).serve_forever()
  except KeyboardInterrupt:
    pass


if __name__ == '__main__':
  main()
