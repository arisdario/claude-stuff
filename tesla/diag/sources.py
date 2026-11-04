"""Data sources for the collector: cereal subscriptions (read-only), /proc + /sys sampling, USB, camera frames.
Nothing here publishes a message, writes a Param, or imports control code."""
import glob
import io
import json
import os
import sys
import time

try:
  from . import canspec  # noqa
except ImportError:
  import canspec  # noqa

SERVICES = ('frame', 'model', 'cameraOdometry', 'sensorEvents', 'liveLocationKalman', 'gpsLocationExternal',
            'controlsState', 'carState', 'carControl', 'liveParameters', 'liveCalibration', 'pathPlan', 'health', 'can')


def _g(o, name, default=None):
  try:
    return getattr(o, name)
  except Exception:
    return default


def _lst(v, n=None):
  try:
    out = [round(float(x), 5) for x in v]
  except Exception:
    return None
  return out[:n] if n else out


def trim(x, maxlist=24, depth=0):
  """JSON-able, rounded, size-bounded copy of a capnp to_dict() result."""
  if depth > 6:
    return None
  if isinstance(x, float):
    return round(x, 6)
  if isinstance(x, dict):
    return {k: trim(v, maxlist, depth + 1) for k, v in x.items() if not isinstance(v, (bytes, bytearray))}
  if isinstance(x, (list, tuple)):
    return [trim(v, maxlist, depth + 1) for v in list(x)[:maxlist]]
  if isinstance(x, (bytes, bytearray)):
    return None
  return x


def gen(sub, maxlist=24):
  try:
    return trim(sub.to_dict(), maxlist)
  except Exception:
    return None


def _sat(d, path=''):
  """first truthy leaf whose key contains 'saturated' in a nested dict -> its path, else None."""
  if isinstance(d, dict):
    for k, v in d.items():
      if 'saturat' in k.lower() and v:
        return path + k
      r = _sat(v, path + k + '.') if isinstance(v, dict) else None
      if r:
        return r
  return None


class Extractor(object):
  """cereal event -> flat record (detector fields) + optional generic full dump at <= full_hz per service."""

  def __init__(self, full_hz=1.0, ls_hz=5.0):
    self.full_dt = 1.0 / full_hz            # capnp to_dict() is slow on the EON: the full generic dump of a message is only taken ~1 Hz
    self.ls_dt = 1.0 / ls_hz
    self.last_full = {}
    self.last_ls = -1e9

  def _full(self, svc, rt):
    if rt - self.last_full.get(svc, -1e9) >= self.full_dt:
      self.last_full[svc] = rt
      return True
    return False

  def __call__(self, svc, ev, rt):
    d = {'s': svc, 'rt': round(rt, 5)}
    lt = _g(ev, 'logMonoTime')
    if lt:
      d['lt'] = round(lt / 1e9, 5)
    full = self._full(svc, rt)
    f = getattr(self, 'x_' + svc, None)
    if f is None:
      return None
    return f(ev, d, full)

  def x_frame(self, ev, d, full):
    f = ev.frame
    d.update(fid=_g(f, 'frameId'), eof=(_g(f, 'timestampEof') or 0) / 1e9, ft=_g(f, 'frameLength'))
    d['integ'], d['gain'], d['lens'] = _g(f, 'integLines'), _g(f, 'globalGain'), _g(f, 'lensPos')
    if full:
      d['all'] = gen(f)
    return d

  def x_model(self, ev, d, full):
    m = ev.model
    p = _g(m, 'path')
    d['path'] = _lst(_g(p, 'poly'))
    d['pstd'] = _g(p, 'std')
    for k, n in (('lane_l', 'leftLane'), ('lane_r', 'rightLane')):
      ln = _g(m, n)
      d[k], d[k + '_p'] = _lst(_g(ln, 'poly')), _g(ln, 'prob')
    for k, n in (('edge_l', 'leftEdge'), ('edge_r', 'rightEdge'), ('edge_l2', 'roadEdgeLeft'), ('edge_r2', 'roadEdgeRight')):
      ln = _g(m, n)
      if ln is not None:
        d[k], d[k + '_p'] = _lst(_g(ln, 'poly')), _g(ln, 'prob')
    d['fid'] = _g(m, 'frameId')
    d['drop'] = _g(m, 'frameDropPerc')
    d['eof'] = (_g(m, 'timestampEof') or 0) / 1e9
    d['exec'] = _g(m, 'modelExecutionTime')
    d['gpu'] = _g(m, 'gpuExecutionTime')
    if full:
      meta = _g(m, 'meta')
      if meta is not None:
        d['meta'] = gen(meta)
    return d

  def x_cameraOdometry(self, ev, d, full):
    c = ev.cameraOdometry
    d.update(trans=_lst(_g(c, 'trans')), rot=_lst(_g(c, 'rot')), transStd=_lst(_g(c, 'transStd')),
             rotStd=_lst(_g(c, 'rotStd')), fid=_g(c, 'frameId'))
    return d

  def x_sensorEvents(self, ev, d, full):
    for s in ev.sensorEvents:
      gy, ac = _g(s, 'gyro'), _g(s, 'acceleration')
      if gy is not None:
        d['gyro'] = _lst(_g(gy, 'v'))
      if ac is not None:
        d['accel'] = _lst(_g(ac, 'v'))
    return d if ('gyro' in d or 'accel' in d) else None

  def x_liveLocationKalman(self, ev, d, full):
    k = ev.liveLocationKalman
    for key, n in (('yawrate', 'angularVelocityCalibrated'), ('orient', 'orientationNED'),
                   ('vel', 'velocityCalibrated'), ('accel', 'accelerationCalibrated'), ('pos', 'positionGeodetic')):
      sub = _g(k, n)
      if sub is not None:
        d[key] = _lst(_g(sub, 'value'))
        if key == 'yawrate':
          d['yawrate_std'] = _lst(_g(sub, 'std'))
    d['ok'] = [_g(k, 'inputsOK'), _g(k, 'sensorsOK'), _g(k, 'gpsOK')]
    return d

  def x_gpsLocationExternal(self, ev, d, full):
    g = ev.gpsLocationExternal
    for k, n in (('lat', 'latitude'), ('lon', 'longitude'), ('alt', 'altitude'), ('brg', 'bearing'), ('spd', 'speed'),
                 ('acc', 'accuracy'), ('vacc', 'verticalAccuracy'), ('flags', 'flags'), ('gt', 'timestamp')):
      d[k] = _g(g, n)
    return d

  def x_controlsState(self, ev, d, full):
    c = ev.controlsState
    d.update(des=_g(c, 'angleSteersDes'), act=_g(c, 'angleSteers'), en=_g(c, 'enabled'), v=_g(c, 'vEgo'),
             sr=_g(c, 'steerRatio'), curv=_g(c, 'curvature'), cum_lag=_g(c, 'cumLagMs'), act_cs=_g(c, 'active'))
    ls = _g(c, 'lateralControlState')
    lsd = None
    if ls is not None and d['rt'] - self.last_ls >= self.ls_dt:
      self.last_ls = d['rt']
      lsd = gen(ls)
    if lsd:
      d['ls'] = lsd
      sf = _sat(lsd)
      d['sat'] = bool(sf)
      d['sat_field'] = sf
    if full:
      d['all'] = gen(c, 12)
    return d

  def x_carState(self, ev, d, full):
    c = ev.carState
    d.update(v=_g(c, 'vEgo'), ang=_g(c, 'steeringAngle'), rate=_g(c, 'steeringRate'), tq=_g(c, 'steeringTorque'),
             tqeps=_g(c, 'steeringTorqueEps'), yaw=_g(c, 'yawRate'), ax=_g(c, 'aEgo'), gas=_g(c, 'gas'),
             brake=_g(c, 'brakePressed'), gear=str(_g(c, 'gearShifter')), steerErr=_g(c, 'steerError'),
             steerWarn=_g(c, 'steerWarning'), blinker=[_g(c, 'leftBlinker'), _g(c, 'rightBlinker')])
    if full:
      d['all'] = gen(c, 12)
    return d

  def x_carControl(self, ev, d, full):
    c = ev.carControl
    a = _g(c, 'actuators')
    d.update(en=_g(c, 'enabled'), active=_g(c, 'active'), steer=_g(a, 'steer'), steerAngle=_g(a, 'steerAngle'),
             gas=_g(a, 'gas'), brake=_g(a, 'brake'))
    if full:
      d['all'] = gen(c, 12)
    return d

  def x_liveParameters(self, ev, d, full):
    p = ev.liveParameters
    d.update(sr=_g(p, 'steerRatio'), sf=_g(p, 'stiffnessFactor'), ao=_g(p, 'angleOffsetAverage'),
             aod=_g(p, 'angleOffset'), valid=_g(p, 'valid'), sra=_g(p, 'steerRatioAverage'))
    return d

  def x_liveCalibration(self, ev, d, full):
    c = ev.liveCalibration
    d.update(rpy=_lst(_g(c, 'rpyCalib')), status=_g(c, 'calStatus'), perc=_g(c, 'calPerc'),
             rpy_std=_lst(_g(c, 'rpyCalibSpread')), extr=_lst(_g(c, 'extrinsicMatrix')))
    return d

  def x_pathPlan(self, ev, d, full):
    p = ev.pathPlan
    d.update(des=_g(p, 'angleSteers'), off=_g(p, 'angleOffset'), cpoly=_lst(_g(p, 'cPoly')), dpoly=_lst(_g(p, 'dPoly')),
             lpoly=_lst(_g(p, 'lPoly')), rpoly=_lst(_g(p, 'rPoly')), lprob=_g(p, 'lProb'), rprob=_g(p, 'rProb'),
             width=_g(p, 'laneWidth'), valid=_g(p, 'paramsValid'), mvalid=_g(p, 'modelValid'),
             cprob=_g(p, 'cProb'), sr=_g(p, 'steerRatio'))
    return d

  def x_health(self, ev, d, full):
    h = ev.health
    d.update(rx=_g(h, 'canRxErrs'), tx=_g(h, 'canSendErrs'), fwd=_g(h, 'canFwdErrs'), allowed=_g(h, 'controlsAllowed'),
             ign=_g(h, 'ignitionLine'), volt=_g(h, 'voltage'))
    return d


# ------------------------------------------------------------------ cereal
# services read at full rate; everything else is conflated (newest message only) and read every slow_every-th poll (~20 Hz),
# which keeps the Python cost on the EON small: the steering-loop signals are what matter at full rate.
FULL_RATE = ('frame', 'model', 'cameraOdometry', 'controlsState', 'carState', 'can')


class CerealSource(object):
  """read-only subscriptions; poll() drains what is due and returns [(svc, event, receive_time)]."""

  def __init__(self, messaging, clock, services=SERVICES, slow_every=5):
    self.m = messaging
    self.clock = clock
    self.socks, self.skipped = {}, []
    self.slow_every, self.n = slow_every, 0
    for s in services:
      try:
        self.socks[s] = messaging.sub_sock(s, conflate=(s not in FULL_RATE))
      except Exception as e:
        self.skipped.append((s, str(e)))
    self.depth_max = {}

  def poll(self):
    out = []
    self.n += 1
    for name, sock in self.socks.items():
      if name not in FULL_RATE and self.n % self.slow_every:
        continue
      msgs = self.m.drain_sock(sock, wait_for_one=False)
      if msgs:
        rt = self.clock()
        if len(msgs) > self.depth_max.get(name, 0):
          self.depth_max[name] = len(msgs)
        out.extend((name, ev, rt) for ev in msgs)
    return out

  def take_depth(self):
    d, self.depth_max = self.depth_max, {}
    return d


def can_frames(ev):
  out = []
  for c in ev.can:
    out.append((_g(c, 'address'), _g(c, 'src'), bytes(_g(c, 'dat') or b'')))
  return out


# ------------------------------------------------------------------ system sampling
def _rd(path):
  try:
    with open(path) as f:
      return f.read()
  except (OSError, IOError):
    return ''


class SystemSampler(object):
  """1 Hz: CPU per process / memory / thermal / GPU via traffic.sysstats, plus scheduling delay + context switches per key process."""
  KEY = ('camerad', 'modeld', 'controlsd', 'plannerd', 'radard', 'boardd', 'loggerd', 'paramsd', 'locationd', 'sensord',
         'calibrationd', 'thermald', 'ui', 'mapcurved', 'uploader', 'deleter')

  def __init__(self, proc_every=5):
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.path.join(here, '..', 'traffic'), os.path.join(here, '..')):
      if p not in sys.path:
        sys.path.insert(0, p)
    import sysstats
    self.S = sysstats.Sampler(proc_every=proc_every)
    self.prev_sched = {}

  def _sched(self, now):
    out = {}
    for pid, label in list(self.S.labels.items()):
      base = label.split('/')[-1].replace('.py', '')
      if not any(k in base for k in self.KEY):
        continue
      ss = _rd('/proc/%d/schedstat' % pid).split()
      st = _rd('/proc/%d/status' % pid)
      if not st:
        continue
      rec = {}
      for line in st.splitlines():
        if line.startswith('Threads:'):
          rec['threads'] = int(line.split()[1])
        elif line.startswith('voluntary_ctxt_switches'):
          rec['vol'] = int(line.split()[1])
        elif line.startswith('nonvoluntary_ctxt_switches'):
          rec['nonvol'] = int(line.split()[1])
        elif line.startswith('VmRSS:'):
          rec['rss_mb'] = round(int(line.split()[1]) / 1024.0, 1)
      if len(ss) >= 2:
        rec['run_delay_ms_total'] = round(int(ss[1]) / 1e6, 1)       # time spent runnable but waiting for a CPU
        prev = self.prev_sched.get(pid)
        if prev:
          rec['run_delay_ms_per_s'] = round((int(ss[1]) - prev[0]) / 1e6 / max(1e-3, now - prev[1]), 2)
        self.prev_sched[pid] = (int(ss[1]), now)
      out[base] = rec
    return out

  def sample(self, now):
    s = self.S.sample(now=now)
    if s is None:
      return None
    s = {k: v for k, v in s.items() if k not in ('t',)}
    s['s'], s['rt'] = 'perf', round(now, 5)
    s['sched'] = self._sched(now)
    s['throttle_mhz'] = s.get('cpu_mhz')
    return s


def usb_scan(root='/sys/bus/usb/devices', vid='046d', pid='082d'):
  """presence/speed/bus address of the C920 (plus every hub on the way up) from sysfs; no lsusb needed."""
  out = {'present': False, 'devices': []}
  for d in glob.glob(os.path.join(root, '*')):
    v = _rd(os.path.join(d, 'idVendor')).strip()
    if not v:
      continue
    rec = {'name': os.path.basename(d), 'vid': v, 'pid': _rd(os.path.join(d, 'idProduct')).strip(),
           'speed_mbps': _rd(os.path.join(d, 'speed')).strip(), 'busnum': _rd(os.path.join(d, 'busnum')).strip(),
           'devnum': _rd(os.path.join(d, 'devnum')).strip(), 'class': _rd(os.path.join(d, 'bDeviceClass')).strip()}
    if rec['vid'] == vid and rec['pid'] == pid:
      out['present'] = True
      out['c920'] = rec
    elif rec['class'] == '09':
      out['devices'].append(rec)
  return out


# ------------------------------------------------------------------ camera image quality
def image_quality(jpg, flip=False):
  """brightness/contrast/sharpness/saturation/over+under exposure/directional-blur from a (reduced-size) decode. None if no decoder."""
  try:
    import numpy as np
  except ImportError:
    return None
  rgb = None
  try:
    import cv2
    a = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_REDUCED_COLOR_4)
    if a is not None:
      rgb = a[:, :, ::-1]
  except ImportError:
    pass
  if rgb is None:
    try:
      from PIL import Image
      im = Image.open(io.BytesIO(jpg))
      im.draft('RGB', (im.size[0] // 4, im.size[1] // 4))
      rgb = np.asarray(im.convert('RGB'))
    except Exception:
      return None
  rgb = rgb.astype(np.float32)
  g = 0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
  lap = g[1:-1, 1:-1] * 4 - g[:-2, 1:-1] - g[2:, 1:-1] - g[1:-1, :-2] - g[1:-1, 2:]
  gx, gy = np.abs(np.diff(g, axis=1)).mean(), np.abs(np.diff(g, axis=0)).mean()
  mx, mn = rgb.max(axis=2), rgb.min(axis=2)
  k = max(1, g.shape[0] * 3 // 10)
  top, bot = float(g[:k].mean()), float(g[-k:].mean())
  if flip:                                   # /dev/c920_live.jpg is the raw frame; camerad rotates it 180 deg before the model when /data/camera_flip=1
    top, bot = bot, top
  return {'h': int(g.shape[0]), 'w': int(g.shape[1]), 'mean': round(float(g.mean()), 2), 'contrast': round(float(g.std()), 2),
          'sharp': round(float(lap.var()), 2), 'sat': round(float((mx - mn).mean() / max(1.0, mx.mean())), 4),
          'over_frac': round(float((g > 245).mean()), 4), 'under_frac': round(float((g < 10).mean()), 4),
          'blur_dir': round(float(gx / max(1e-6, gy)), 3),
          'model_view_top': round(top, 1), 'model_view_bottom': round(bot, 1), 'flip_applied': bool(flip)}


class FrameGrabber(object):
  """camerad writes the full-size MJPEG of every ~2nd frame to /dev/c920_live.jpg; we only READ it."""

  def __init__(self, path='/dev/c920_live.jpg'):
    self.path = path
    self.last_m = None
    self.stale_since = None

  def grab(self, now):
    try:
      st = os.stat(self.path)
    except OSError:
      return None, None
    if st.st_mtime == self.last_m:
      return None, st.st_mtime
    try:
      with open(self.path, 'rb') as f:
        b = f.read()
    except OSError:
      return None, None
    if len(b) < 1000 or b[:2] != b'\xff\xd8' or b[-2:] != b'\xff\xd9':   # half-written / not a JPEG
      return None, None
    self.last_m = st.st_mtime
    return b, st.st_mtime
