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
    """addr, d = payload bytes (8). Mirrors tesla_tx_hook for the frames openpilot sends."""
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
    elif addr == 0x554:
      self.emu = bool(d[0] >> 5 & 1)

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
  mirror = PandaMirror(); extra = {'v': 0., 'ang': 0., 'en': False}; model = None
  while True:
    if not Frame.clients:
      time.sleep(0.3); continue
    now = time.monotonic()
    for ev in messaging.drain_sock(sc, wait_for_one=False):
      for c in ev.sendcan:
        if c.address in WANT:
          mirror.feed(c.address, bytes(c.dat), now)
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
    snap.update(extra); snap['model'] = model; snap['t'] = round(time.time(), 2)
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
    mirror.feed(0x554, bytes([0x20, 0, 0, 0, 0, 0, 0, 0]), now)
    snap = mirror.snapshot(now); mirror.lane_idx = (mirror.lane_idx + 1) % 16
    xs = list(range(0, 100, 4))
    snap.update({'v': 24 + 4 * math.sin(t * 0.2), 'ang': 300 * k, 'en': t % 30 > 4, 't': round(time.time(), 2), 'model': {
        'p': [round(c0 + k * 3 * x ** 3 / 1e0 * 0 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs], 'pl': 0.9, 'pr': 0.8,
        'l': [round(c0 + 1.85 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs], 'r': [round(c0 - 1.85 + 0.5 * k * 2 * x * x * 0.1, 2) for x in xs]}})
    publish(snap)
    time.sleep(1. / hz)


# --------------------------------------------------------------------------------------------- web
PAGE = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>cluster view</title><style>html,body{margin:0;height:100%;background:#0b0d10;color:#e8edf2;font:14px system-ui,Arial,sans-serif;overflow:hidden}
canvas{display:block;width:100vw;height:100vh}</style></head><body><canvas id=c></canvas><script>
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,tdv=false,W=0,H=0,SC=2;
function rs(){var d=Math.min(window.devicePixelRatio||1,1.5);W=innerWidth;H=innerHeight;cv.width=W*d;cv.height=H*d;g.setTransform(d,0,0,d,0,0);draw()}
addEventListener('resize',rs);cv.addEventListener('click',function(){tdv=!tdv;draw()});
function P(x,y){ // real metres (x ahead, y left) -> screen
 if(tdv){var s=H*0.8/110;return [W/2-y*s*4,H*0.92-x*s]}
 var K=H*0.9,d=x+5;return [W/2-y*K/d,H*0.38+2.5*K/d]}
function line(pts,col,w,dash){if(pts.length<2)return;g.beginPath();g.setLineDash(dash||[]);g.strokeStyle=col;g.lineWidth=w;pts.forEach(function(p,i){i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])});g.stroke();g.setLineDash([])}
function poly(c,x){return c[0]+c[1]*x+c[2]*x*x+c[3]*x*x*x}
function draw(){g.fillStyle='#0b0d10';g.fillRect(0,0,W,H);if(!S||!S.lanes){g.fillStyle='#9aa4af';g.fillText('waiting for data...',20,30);return}
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
 S.obj.forEach(function(o){var p=P(o.dx,o.dy),w=Math.max(10,40*(tdv?H/900:1/Math.max(.25,o.dx/25)));g.fillStyle=o.relevant?'#e5383b':'#8b949e';g.fillRect(p[0]-w/2,p[1]-w*.6,w,w*.6);g.fillStyle='#e8edf2';g.fillText(o.slot+' '+Math.round(o.dx)+' m',p[0]-w/2,p[1]-w*.6-4)});
 // header
 var st=S.status,y0=22;g.font='15px system-ui';g.fillStyle='#e8edf2';
 g.fillText('openpilot '+(S.en?'ENGAGED':'off')+'   speed '+Math.round((S.v||0)*3.6)+' km/h   wheel '+(S.ang||0).toFixed(1)+' deg',16,y0);
 g.fillStyle=S.present?'#7bd88f':'#e5383b';g.fillText('panda data: '+(S.present?'live':('STALE ('+S.age+' s) - cluster is shown defaults')),16,y0+20);
 g.fillStyle='#e8edf2';g.fillText('cluster status: '+st.text+'   emulation: '+(S.emu===null?'?':S.emu?'on':'OFF (panda sends nothing)'),16,y0+40);
 g.fillText('lanes: width '+L.width.toFixed(2)+' m  left '+(L.left?'drawn':'not drawn')+'  right '+(L.right?'drawn':'not drawn')+'  range '+L.range+' (x'+SC+')'+(S.default?'   [DEFAULT frame - no lane data from openpilot]':''),16,y0+60);
 g.fillStyle='#9aa4af';g.font='12px system-ui';g.fillText('yellow = lane lines sent to the cluster   cyan = model lanes   green = model path   red = road edges   tap = switch view',16,H-14)}
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
