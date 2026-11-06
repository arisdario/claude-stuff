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
    if len(fp) == 69 and fp[0] == 1.0:
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
  mirror = PandaMirror(); car = CarState(); extra = {'v': 0., 'ang': 0., 'en': False}; model = None
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
    m = messaging.recv_one_or_none(subs['carState'])
    if m is not None:
      cs = m.carState; extra['v'] = float(cs.vEgo)
      extra['ang'] = float(getattr(cs, 'steeringAngle', getattr(cs, 'steeringAngleDeg', 0.)))
    m = messaging.recv_one_or_none(subs['controlsState'])
    if m is not None:
      extra['en'] = bool(m.controlsState.enabled)
    m = messaging.recv_one_or_none(subs['model'])
    if m is not None:
      try: model = build_model(m.model)
      except Exception: model = None
    snap = mirror.snapshot(now); mirror.lane_idx = (mirror.lane_idx + 1) % 16
    snap.update(extra); snap['car'] = car.snapshot(now); snap['model'] = model; snap['t'] = round(time.time(), 2)
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
    mirror.feed(0x559, tc.create_DAS_LR_object_msg(1, 0, 9, 26, 3.7, 1., 0, 0, 0, 0, 0)[2], now)
    mirror.feed(0x659, bytes([5 if t % 30 > 4 else 2, 90, 0, 0xC0 | 8, 0, 0, 0, 0]), now)
    mirror.feed(0x554, bytes([0x20, 0, 0]), now)
    snap = mirror.snapshot(now); mirror.lane_idx = (mirror.lane_idx + 1) % 16
    xs = list(range(0, 100, 4))
    snap.update({'v': 24 + 4 * math.sin(t * 0.2), 'ang': 300 * k, 'en': t % 30 > 4, 'car': {'hb': int(t % 12 > 6), 'fl': 0, 'out': 9.5}, 't': round(time.time(), 2), 'model': {
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
 var M=S.model; // the model's own, faint
 if(M){var xs=[];for(i=0;i<25;i++)xs.push(i*4);
  if(M.l)line(M.l.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.r)line(M.r.map(function(y,k){return P(xs[k],y)}),'rgba(80,200,230,.35)',2);
  if(M.p){var cl=M.p.map(function(y,k){return P(xs[k],y-.9)}),cr=M.p.map(function(y,k){return P(xs[k],y+.9)}).reverse();g.fillStyle='rgba(60,200,120,.12)';g.beginPath();cl.concat(cr).forEach(function(p,k){k?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.fill()}
  if(M.e){line(M.e.l.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2);line(M.e.r.map(function(y,k){return P(M.e.x[k],y)}),'rgba(229,72,77,.6)',2)}}
 // what the cluster is told: virtual lane = centre curve +- width/2 (cluster x = real/2)
 var cc=[],lf=[],rt=[];for(i=0;i<=R;i+=2){var xc=i/SC,yc=poly(L.c,xc);cc.push(P(i,yc));lf.push(P(i,yc+L.width/2));rt.push(P(i,yc-L.width/2))}
 line(cc,'rgba(255,255,255,.18)',1,[6,8]);
 line(lf,L.left?'#ffd23f':'#555e68',L.left?5:2,L.left?null:[10,10]);line(rt,L.right?'#ffd23f':'#555e68',L.right?5:2,L.right?null:[10,10]);
 // lead / side vehicles
 S.obj.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w,h;
  if(tdv){h=Math.max(26,4.5*H*0.8/110*1.6);w=Math.max(12,h/2.2);carTop(p[0],p[1],w,h,o.relevant)}   // an icon with car proportions: the top view stretches the sideways scale 4x, so true scale would look squashed
  else{w=Math.max(12,1.8*H*0.9/(o.dx+5));h=w*0.8;carBack(p[0],p[1],w,o.relevant)}
  g.fillStyle='#e8edf2';g.textAlign='center';g.fillText(o.slot+' '+Math.round(o.dx)+' m',p[0],p[1]-h-6);g.textAlign='left'});
 // header
 var st=S.status,y0=22;g.font='15px system-ui';g.fillStyle='#e8edf2';
 g.fillText('openpilot '+(S.en?'ENGAGED':'off')+'   speed '+Math.round((S.v||0)*3.6)+' km/h   wheel '+(S.ang||0).toFixed(1)+' deg',16,y0);
 g.fillStyle=S.present?'#7bd88f':'#e5383b';g.fillText('panda data: '+(S.present?'live':('STALE ('+S.age+' s) - cluster is shown defaults')),16,y0+20);
 g.fillStyle='#e8edf2';g.fillText('cluster status: '+st.text+'   emulation: '+(S.emu===null?'?':S.emu?'on':'OFF (panda sends nothing)'),16,y0+40);
 g.fillText('lanes: width '+L.width.toFixed(2)+' m  left '+(L.left?'drawn':'not drawn')+'  right '+(L.right?'drawn':'not drawn')+'  range '+L.range+' (x'+SC+')'+(S.default?'   [DEFAULT frame - no lane data from openpilot]':''),16,y0+60);
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
 var M=S.model,i,xs=[];for(i=0;i<25;i++)xs.push(i*4);
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
 S.obj.slice().sort(function(a,b){return b.dx-a.dx}).forEach(function(o){var p=P(o.dx,o.dy),w=Math.max(12,1.8*H*0.9/(o.dx+5));carBack(p[0],p[1],w,o.relevant);pill(p[0],p[1]-w*0.8-14,Math.round(o.dx)+' m')});
 var ew=Math.min(H*0.27,W*0.2),ey=H*0.92;
 if(eng){g.save();g.translate(W/2,ey);g.scale(1,0.14);var eg=g.createRadialGradient(0,0,0,0,0,ew*0.95);eg.addColorStop(0,'rgba(64,156,255,.55)');eg.addColorStop(1,'rgba(64,156,255,0)');g.fillStyle=eg;g.beginPath();g.arc(0,0,ew*0.95,0,6.2832);g.fill();g.restore()}
 carBack(W/2,ey,ew,false);
 // the middle of the instrument panel: icons either side of the speed
 var pw=Math.min(W*0.5,380),py=14,ph=96,px=W/2-pw/2;
 g.fillStyle='rgba(8,14,20,.72)';rr(px,py,pw,ph,18);g.fill();g.strokeStyle='rgba(255,255,255,.06)';g.lineWidth=1;g.stroke();
 var ic=eng?'rgba(64,156,255,1)':'rgba(120,130,142,.9)';
 cruiseIcon(px+pw*0.14,py+ph*0.46,15,ic);wheelIcon(px+pw*0.86,py+ph*0.46,16,ic);
 var cr=S.car||{};if(cr.hb||cr.fl)beamIcon(px+pw*0.14,py+ph*0.46+34,9,cr.hb?'rgba(64,156,255,1)':'rgba(235,240,248,.85)');
 if(cr.out!==null&&cr.out!==undefined){g.fillStyle='#aab4c2';g.font='500 14px system-ui';g.textAlign='center';g.fillText(cr.out.toFixed(1)+' \u00b0C',px+pw*0.86,py+ph*0.46+38);g.textAlign='left'}
 g.fillStyle='#f3f6fa';g.font='300 58px system-ui';g.textAlign='center';g.fillText(Math.round((S.v||0)*3.6),W/2,py+62);g.fillStyle='#8590a0';g.font='500 13px system-ui';g.fillText('km/h',W/2,py+84);
 g.fillStyle=S.present?'rgba(130,150,170,.55)':'rgba(255,120,120,.9)';g.font='11px system-ui';
 g.fillText((S.present?'panda live':'PANDA STALE')+'  ·  cluster '+S.status.text+'  ·  emulation '+(S.emu===null?'?':S.emu?'on':'off')+(S.default?'  ·  no lane data':'')+'  ·  tap: next view',W/2,H-12);g.textAlign='left'}
function draw(){if(mode==0)drawDrive();else drawDiag()}
function conn(){var es=new EventSource('/stream');es.onmessage=function(e){S=JSON.parse(e.data);draw()};es.onerror=function(){es.close();setTimeout(conn,1500)}}
rs();conn();</script></body></html>"""


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
