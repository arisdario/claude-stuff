"""Diagnostic collector core: clock, async disk writer, ring buffers, anomaly detectors, event manager.

Design rules
  * One clock for everything: CLOCK_BOOTTIME seconds = the same reference as cereal logMonoTime and camerad timestampEof.
    Each record carries 'rt' (receive time here) and, when the message has it, 'lt' (logMonoTime) -> 'rt-lt' is the IPC delay.
  * The data path never blocks: records go into a bounded queue; a writer thread does ALL disk I/O. A full queue drops the
    record and counts it (reported in the performance stream), it never waits.
  * Full-rate telemetry goes to a rolling ring on disk (segments of seg_s seconds, only the newest keep_s are retained);
    a decimated continuous trace per group is kept for the whole session; an anomaly copies [t-pre, t+post] out of the ring.
  * Pure logic (no cereal import): everything is driven through Collector.ingest()/tick() so it can be replayed and tested.
"""
import collections
import gzip
import json
import math
import os
import shutil
import threading
import time

try:
  from . import canspec
except ImportError:      # run as a plain script directory
  import canspec


def boot_now():
  """seconds since boot, same reference as cereal logMonoTime (CLOCK_BOOTTIME)."""
  try:
    return time.clock_gettime(time.CLOCK_BOOTTIME)
  except (AttributeError, OSError):
    return time.monotonic()


GROUPS = {'frame': 'camera', 'cam_quality': 'camera', 'usb': 'camera', 'model': 'model', 'cameraOdometry': 'pose',
          'liveCalibration': 'calibration', 'can': 'can', 'can_raw': 'can', 'health': 'can',
          'liveLocationKalman': 'localization', 'gpsLocationExternal': 'localization', 'sensorEvents': 'localization',
          'liveParameters': 'localization', 'controlsState': 'steering', 'carState': 'steering', 'pathPlan': 'steering',
          'carControl': 'steering', 'perf': 'performance', 'diag': 'performance'}
GROUP_DIRS = ('metadata', 'camera', 'model', 'pose', 'calibration', 'can', 'localization', 'steering', 'performance',
              'events', 'plots', 'ring')

DEFAULT_CFG = {
  'pre_s': 30.0, 'post_s': 30.0, 'seg_s': 10.0, 'keep_s': 100.0,
  'trace_hz': 10.0,                 # continuous per-group trace is decimated to this rate per service
  'frame_hz': 3.0, 'frame_keep_s': 100.0,
  'max_ring_mb': 250.0, 'max_trace_mb': 400.0, 'max_events': 40, 'event_cooldown_s': 12.0, 'max_event_s': 150.0,
  'min_free_mb': 1500.0, 'hard_free_mb': 600.0,
  'queue_max': 30000, 'poll_s': 0.01, 'work_s': 0.03, 'max_backlog': 800,
  # the steering loop (0.3-0.4 Hz wiggle) needs ~20 Hz, not 100 Hz: every Nth message of these services is extracted (all are still counted for timing)
  'stride': {'controlsState': 5, 'carState': 5, 'can': 2},
  # full-rate ring: the steering loop signals stay at full rate, bulk/auxiliary streams are capped (Hz). 0 = unlimited
  'ring_hz': {'carControl': 50, 'sensorEvents': 50, 'pathPlan': 25, 'liveParameters': 4, 'liveCalibration': 4, 'can_raw_other': 25},
  'can_raw_full': [0x370, 0x488, 0x0E],
  'thr': {
    'frame_nominal_s': 0.05, 'frame_gap_factor': 2.2, 'frame_gap_min_s': 0.11, 'frame_age_s': 0.15,
    'model_latency_s': 0.20, 'model_exec_s': 0.06, 'model_drop_perc': 10.0,
    'path_c0_jump_m': 0.45, 'path_c1_jump': 0.05, 'path_y20_jump_m': 1.2,
    'pose_floor': 0.03, 'pose_sigma': 8.0,
    'steer_err_deg': 5.0, 'steer_err_s': 0.5,
    'osc_window_s': 8.0, 'osc_min_p2p_deg': 4.0, 'osc_swing_p2p_deg': 8.0, 'osc_min_cross': 4, 'osc_period_min_s': 0.8, 'osc_period_max_s': 6.0,
    'can_gap_factor': 3.0, 'can_gap_min_s': 0.15, 'can_gap_min_n': 12, 'ipc_delay_s': 0.06,
    'ctl_gap_s': 0.06, 'cam_dark': 25.0, 'cam_bright_frac': 0.35, 'cam_blur_var': 20.0,
  },
}


def merged_cfg(over=None):
  cfg = json.loads(json.dumps(DEFAULT_CFG))
  for k, v in (over or {}).items():
    if k == 'thr':
      cfg['thr'].update(v)
    else:
      cfg[k] = v
  return cfg


# ------------------------------------------------------------------ async writer
class AsyncWriter(object):
  """All disk I/O. put_line/put_bytes never block: past qmax queued items they drop and count. put_cmd (event copies, pruning)
  is never dropped, so evidence is not lost when the disk is slow."""

  def __init__(self, qmax=30000, fsync_probe_dir=None):
    self.dq = collections.deque()
    self.cv = threading.Condition()
    self.qmax = qmax
    self.dropped = 0
    self.files = {}
    self.wlat_ms = collections.deque(maxlen=50)     # batch write+flush latency
    self.fsync_ms = None
    self.bytes_written = 0
    self.errors = 0
    self.max_q = 0
    self._probe_dir, self._last_probe = fsync_probe_dir, 0.0
    self._stop = threading.Event()
    self.t = threading.Thread(target=self._run, name='diag-writer')
    self.t.daemon = True

  def start(self):
    self.t.start()

  def qsize(self):
    return len(self.dq)

  def _put(self, item, droppable=True):
    n = len(self.dq)
    if droppable and n >= self.qmax:
      self.dropped += 1
      return False
    with self.cv:
      self.dq.append(item)
      self.cv.notify()
    if n > self.max_q:
      self.max_q = n
    return True

  def put_line(self, path, line):
    return self._put(('line', path, line))

  def put_bytes(self, path, data):
    return self._put(('bytes', path, data))

  def put_cmd(self, fn, *a):
    return self._put(('cmd', fn, a), droppable=False)

  def _fh(self, path):
    fh = self.files.get(path)
    if fh is None:
      os.makedirs(os.path.dirname(path), exist_ok=True)
      fh = self.files[path] = open(path, 'a')
    return fh

  def close_file(self, path):
    fh = self.files.pop(path, None)
    if fh:
      try:
        fh.close()
      except OSError:
        pass

  def flush_all(self):
    for fh in list(self.files.values()):
      try:
        fh.flush()
      except OSError:
        self.errors += 1

  def _run(self):
    while True:
      with self.cv:
        if not self.dq:
          if self._stop.is_set():
            break
          self.cv.wait(0.1)
        batch = []
        while self.dq and len(batch) < 500:
          batch.append(self.dq.popleft())
      t0 = time.time()
      touched = set()
      for it in batch:
        try:
          if it[0] == 'line':
            self._fh(it[1]).write(it[2] + '\n')
            self.bytes_written += len(it[2]) + 1
            touched.add(it[1])
          elif it[0] == 'bytes':
            os.makedirs(os.path.dirname(it[1]), exist_ok=True)
            tmp = it[1] + '.tmp'
            with open(tmp, 'wb') as f:
              f.write(it[2])
            os.rename(tmp, it[1])
            self.bytes_written += len(it[2])
          elif it[0] == 'cmd':
            self.flush_all()
            it[1](*it[2])
        except Exception:       # a failing disk must never take the collector down
          self.errors += 1
      if touched:
        for p in touched:
          try:
            self.files[p].flush()
          except (OSError, KeyError):
            self.errors += 1
        self.wlat_ms.append(1000.0 * (time.time() - t0))
      now = time.time()
      if self._probe_dir and now - self._last_probe > 5.0:
        self._last_probe = now
        try:
          t1 = time.time()
          p = os.path.join(self._probe_dir, '.fsync_probe')
          with open(p, 'w') as f:
            f.write('x')
            f.flush()
            os.fsync(f.fileno())
          self.fsync_ms = 1000.0 * (time.time() - t1)
        except OSError:
          self.errors += 1

  def stop(self, timeout=15.0):
    """drain the queue, flush, close. Returns True if everything was written."""
    end = time.time() + timeout
    while self.dq and time.time() < end:
      time.sleep(0.05)
    self._stop.set()
    with self.cv:
      self.cv.notify()
    self.t.join(max(0.1, end - time.time()))
    self.flush_all()
    for p in list(self.files):
      self.close_file(p)
    return not self.dq


# ------------------------------------------------------------------ detectors
def _median(x):
  s = sorted(x)
  n = len(s)
  return s[n // 2] if n else None


class Detectors(object):
  """feed(rec) -> list of (kind, detail dict). Pure logic; thresholds from cfg['thr']."""

  def __init__(self, thr):
    self.thr = thr
    self.fr_last = None
    self.fr_int = collections.deque(maxlen=40)
    self.m_last = None
    self.po_last = None
    self.po_d = collections.deque(maxlen=200)
    self.err_since = None
    self.sa = collections.deque()                 # (t, steering angle) for oscillation
    self.osc_next = 0.0
    self.can_t = {}
    self.can_int = {}
    self.loc_ok = {}
    self.sat_n = 0
    self.ctl_last = None
    self.stride_ctl = 1                           # controlsState is sampled every Nth message: gap thresholds scale with it
    self.vego = 0.0
    self.enabled = False
    self.en_since = None
    self.health_last = None
    self.cam_bad = 0
    self.orient_bad = 0
    self.lagging = False                          # set by the collector when it reads late: rt-based checks are then meaningless and skipped

  def feed(self, r):
    s = r.get('s')
    fn = getattr(self, 'd_' + s, None)
    return fn(r) if fn else []

  # ---- camera frame stream (camerad 'frame' message)
  def d_frame(self, r):
    T, out = self.thr, []
    fid, eof = r.get('fid'), r.get('eof')
    if fid is None or eof is None:
      return out
    if self.fr_last is not None:
      lf, le = self.fr_last
      if fid == lf or eof == le:
        out.append(('camera_duplicate_frame', {'frameId': fid}))
      elif fid > lf + 1:
        out.append(('camera_frame_drop', {'frameId': fid, 'missing': fid - lf - 1}))
      dt = eof - le
      if fid > lf and dt > 0:
        self.fr_int.append(dt)
        nom = _median(self.fr_int) if len(self.fr_int) >= 10 else T['frame_nominal_s']
        if dt > max(T['frame_gap_factor'] * nom, T['frame_gap_min_s']):
          out.append(('camera_frame_late', {'frameId': fid, 'interval_s': round(dt, 4), 'nominal_s': round(nom, 4)}))
    self.fr_last = (fid, eof)
    age = r['rt'] - eof
    if age > T['frame_age_s'] and not self.lagging:
      out.append(('camera_frame_delayed', {'frameId': fid, 'age_s': round(age, 4)}))
    return out

  # ---- model output
  def d_model(self, r):
    T, out = self.thr, []
    if r.get('eof') and r.get('lt') is not None:
      lat = r['lt'] - r['eof']
      if lat > T['model_latency_s']:
        out.append(('model_latency_high', {'frameId': r.get('fid'), 'latency_s': round(lat, 4)}))
    if (r.get('exec') or 0) > T['model_exec_s']:
      out.append(('model_exec_slow', {'exec_s': r['exec']}))
    if (r.get('drop') or 0) > T['model_drop_perc']:
      out.append(('model_frame_drop', {'frameDropPerc': r['drop']}))
    p = r.get('path')
    if p and len(p) == 4:
      y20 = p[0] * 8000 + p[1] * 400 + p[2] * 20 + p[3]
      cur = (p[3], p[2], y20, r['rt'])
      if self.m_last is not None and cur[3] - self.m_last[3] < 0.5:
        d0, d1, d20 = abs(cur[0] - self.m_last[0]), abs(cur[1] - self.m_last[1]), abs(cur[2] - self.m_last[2])
        if d0 > T['path_c0_jump_m'] or d1 > T['path_c1_jump'] or d20 > T['path_y20_jump_m']:
          out.append(('model_path_jump', {'d_c0_m': round(d0, 3), 'd_c1_rad': round(d1, 4), 'd_y20_m': round(d20, 3)}))
      self.m_last = cur
    return out

  # ---- pose (cameraOdometry)
  def d_cameraOdometry(self, r):
    T, out = self.thr, []
    rot = r.get('rot')
    if rot and len(rot) == 3:
      if self.po_last is not None:
        d = max(abs(a - b) for a, b in zip(rot, self.po_last))
        if len(self.po_d) >= 40:
          m = _median(self.po_d) or 0.0
          if d > max(T['pose_floor'], T['pose_sigma'] * m):
            out.append(('pose_discontinuity', {'d_rot': round(d, 4), 'typical': round(m, 5)}))
        self.po_d.append(d)
      self.po_last = rot
    return out

  # ---- controls / steering
  def d_controlsState(self, r):
    T, out = self.thr, []
    t = r['rt']
    was = self.enabled
    self.enabled = bool(r.get('en'))
    if was and not self.enabled:
      out.append(('disengagement', {'after_engaged_s': round(t - (self.en_since or t), 2), 'angle': r.get('act'), 'v': r.get('v')}))
    if self.enabled and self.en_since is None:
      self.en_since = t
    if not self.enabled:
      self.en_since = None
      self.err_since = None
      self.sa.clear()
    des, act = r.get('des'), r.get('act')
    if self.enabled and des is not None and act is not None:
      if abs(des - act) > T['steer_err_deg']:
        if self.err_since is None:
          self.err_since = t
        elif t - self.err_since > T['steer_err_s']:
          out.append(('steer_error_large', {'des': des, 'act': act, 'err_deg': round(des - act, 2)}))
          self.err_since = t + 2.0          # do not re-trigger every sample
      else:
        self.err_since = None
    if r.get('sat'):
      self.sat_n += 1
      if self.sat_n == 3:
        out.append(('controller_saturation', {'field': r.get('sat_field')}))
    else:
      self.sat_n = 0
    tl = r.get('lt') or t                          # loop gaps on the producer's own clock
    if self.ctl_last is not None and tl - self.ctl_last > T['ctl_gap_s'] * max(1, self.stride_ctl):
      out.append(('control_loop_gap', {'gap_s': round(tl - self.ctl_last, 4)}))
    self.ctl_last = tl
    if r.get('lt') is not None and t - r['lt'] > T['ipc_delay_s'] and not self.lagging:
      out.append(('ipc_delay_high', {'service': 'controlsState', 'delay_s': round(t - r['lt'], 4)}))
    if act is not None and self.enabled:
      out += self._osc(t, act)
    return out

  @staticmethod
  def _crossings(ts, hp, amp):
    h = max(0.25 * amp / 2, 0.5)           # hysteresis: ignore noise around zero
    state, cr = 0, []
    for tt, v in zip(ts, hp):
      if state <= 0 and v > h:
        if state < 0:
          cr.append(tt)
        state = 1
      elif state >= 0 and v < -h:
        if state > 0:
          cr.append(tt)
        state = -1
    return cr

  def _osc(self, t, x):
    """two detectors on the high-passed steering angle (engaged only):
       steering_swing      last 4 s: >= 2 sign changes and peak-to-peak >= osc_swing_p2p_deg  (catches short, fast-growing wiggles)
       steering_oscillation last 8 s: >= osc_min_cross sign changes, period 0.8-6 s, p2p >= osc_min_p2p_deg"""
    T = self.thr
    self.sa.append((t, x))
    while self.sa and t - self.sa[0][0] > T['osc_window_s']:
      self.sa.popleft()
    if t < self.osc_next or len(self.sa) < 20 or self.sa[-1][0] - self.sa[0][0] < 3.5:
      return []
    self.osc_next = t + 0.5
    ts = [a for a, _ in self.sa]
    xs = [b for _, b in self.sa]
    # high-pass: subtract a ~3 s centred moving average (removes the slow turn, keeps the wiggle)
    hp, j0, j1, acc = [], 0, 0, 0.0
    n = len(xs)
    for i in range(n):
      while ts[j0] < ts[i] - 1.5:
        acc -= xs[j0]; j0 += 1
      while j1 < n and ts[j1] <= ts[i] + 1.5:
        acc += xs[j1]; j1 += 1
      hp.append(xs[i] - acc / max(1, j1 - j0))
    out = []
    k4 = next((i for i, tt in enumerate(ts) if tt >= t - 4.0), 0)
    amp4 = max(hp[k4:]) - min(hp[k4:])
    if amp4 >= T['osc_swing_p2p_deg']:
      cr = self._crossings(ts[k4:], hp[k4:], amp4)
      if len(cr) >= 2:
        per = 2 * (cr[-1] - cr[0]) / (len(cr) - 1)
        if T['osc_period_min_s'] <= per <= T['osc_period_max_s']:
          out.append(('steering_swing', {'period_s': round(per, 2), 'freq_hz': round(1 / per, 2), 'p2p_deg': round(amp4, 1), 'crossings': len(cr)}))
    if self.sa[-1][0] - self.sa[0][0] >= 0.9 * T['osc_window_s']:
      amp = max(hp) - min(hp)
      if amp >= T['osc_min_p2p_deg']:
        cr = self._crossings(ts, hp, amp)
        if len(cr) >= T['osc_min_cross']:
          half = [b - a for a, b in zip(cr, cr[1:])]
          per = 2 * (sum(half) / len(half))
          if T['osc_period_min_s'] <= per <= T['osc_period_max_s']:
            out.append(('steering_oscillation', {'period_s': round(per, 2), 'freq_hz': round(1 / per, 2), 'p2p_deg': round(amp, 1), 'crossings': len(cr)}))
    return out

  def d_carState(self, r):
    if r.get('v') is not None:
      self.vego = r['v']
    out = []
    if r.get('lt') is not None and r['rt'] - r['lt'] > self.thr['ipc_delay_s'] and not self.lagging:
      out.append(('ipc_delay_high', {'service': 'carState', 'delay_s': round(r['rt'] - r['lt'], 4)}))
    return out

  # ---- localization / parameters
  def _edge(self, key, ok, kind, detail):
    prev = self.loc_ok.get(key)
    self.loc_ok[key] = ok
    return [(kind, detail)] if prev is True and ok is False else []

  def d_liveLocationKalman(self, r):
    out = []
    ok = r.get('ok')
    if ok:
      for name, v in zip(('inputsOK', 'sensorsOK', 'gpsOK'), ok):
        out += self._edge('llk_' + name, v, 'localization_invalid', {'flag': name})
    return out

  def d_liveParameters(self, r):
    return self._edge('lp', r.get('valid'), 'localization_invalid', {'flag': 'liveParameters.valid'})

  def d_pathPlan(self, r):
    ok = r.get('valid')
    return self._edge('pp', ok, 'localization_invalid', {'flag': 'pathPlan.paramsValid'})

  # ---- raw CAN
  def d_can_raw(self, r):
    T, out = self.thr, []
    a, t = r['addr'], (r.get('lt') or r['rt'])      # producer time
    last = self.can_t.get(a)
    if last is not None:
      dt = t - last
      iv = self.can_int.setdefault(a, collections.deque(maxlen=60))
      if len(iv) >= T['can_gap_min_n']:
        m = _median(iv)
        if dt > max(T['can_gap_factor'] * m, T['can_gap_min_s']):
          out.append(('can_gap', {'addr': hex(a), 'gap_s': round(dt, 4), 'typical_s': round(m, 4)}))
      iv.append(dt)
    self.can_t[a] = t
    return out

  def d_can_silence(self, r):
    """r: {'addr','silent_s','typical_s'} produced by tick() when a watched address stopped altogether."""
    return [('can_timeout', {'addr': hex(r['addr']), 'silent_s': round(r['silent_s'], 3), 'typical_s': round(r['typical_s'], 4)})]

  def d_health(self, r):
    out = []
    cnt = (r.get('rx'), r.get('tx'), r.get('fwd'))
    if self.health_last and any(a is not None and b is not None and a > b for a, b in zip(cnt, self.health_last)):
      out.append(('can_errors', {'rx_tx_fwd_errs': cnt}))
    self.health_last = cnt
    return out

  def d_cam_quality(self, r):
    T, out = self.thr, []
    bad = None
    if r.get('mean') is not None and r['mean'] < T['cam_dark']:
      bad = 'underexposed'
    elif r.get('over_frac') is not None and r['over_frac'] > T['cam_bright_frac']:
      bad = 'overexposed'
    elif r.get('sharp') is not None and r['sharp'] < T['cam_blur_var']:
      bad = 'blurry'
    t, b = r.get('model_view_top'), r.get('model_view_bottom')
    if t is not None and b is not None and (r.get('mean') or 0) > 90:      # daylight: the sky (top) should be brighter than the road (bottom)
      self.orient_bad = self.orient_bad + 1 if t < 0.85 * b else 0
      if self.orient_bad == 10:
        out.append(('camera_orientation_suspect', {'top': t, 'bottom': b, 'flip_applied': r.get('flip_applied'),
                                                    'meaning': 'model input looks upside down (or a double 180 deg rotation) - check /data/camera_flip vs the mounting'}))
    self.cam_bad = self.cam_bad + 1 if bad else 0
    if bad and self.cam_bad == 2:
      out.append(('camera_image_' + bad, {k: r.get(k) for k in ('mean', 'over_frac', 'sharp')}))
    return out


# ------------------------------------------------------------------ events
class Event(object):
  def __init__(self, eid, t, kinds, first):
    self.id, self.t0, self.t_end = eid, t, t
    self.reasons = [first]
    self.kinds = set(kinds)
    self.last = t


class Collector(object):
  def __init__(self, root, cfg=None, clock=boot_now, writer=None, start_writer=True):
    self.root = root
    self.cfg = merged_cfg(cfg)
    self.clock = clock
    for d in GROUP_DIRS:
      os.makedirs(os.path.join(root, d), exist_ok=True)
    self.w = writer or AsyncWriter(self.cfg['queue_max'], fsync_probe_dir=root)
    if start_writer and not self.w.t.is_alive():
      self.w.start()
    self.det = Detectors(self.cfg['thr'])
    self.ring = os.path.join(root, 'ring')
    self.events = []                # pending
    self.done = []                  # summaries of finished events
    self.n_events = 0
    self.last_kind_t = {}
    self.trace_last = {}
    self.trace_bytes = 0
    self.counts = collections.Counter()
    self.rate = collections.defaultdict(lambda: collections.deque(maxlen=400))
    self.disk_state = 'ok'
    self.frame_last_t = 0.0
    self.start_t = clock()
    self.last_tick = self.start_t
    self.can_stats = collections.defaultdict(lambda: [0, None])
    self.can_last = {}
    self.can_per = {}
    self.proc_t0 = time.process_time()
    self.proc_w0 = time.time()
    self.timeline = collections.deque(maxlen=5000)
    self.ingest_s = 0.0
    self.iv = collections.defaultdict(list)      # per-second interval samples per service (seconds)
    self.iv_last = {}
    self.ring_last = {}
    self.suppressed = collections.Counter()
    self.logged_t = {}
    self.can_prev = {}
    self.can_gapmax = {}

  # ---- paths
  def _seg(self, t):
    return os.path.join(self.ring, 'tel_%08d.jsonl' % int(t // self.cfg['seg_s']))

  def _line(self, rec):
    return json.dumps(rec, separators=(',', ':'), default=str)

  # ---- ingest
  def ingest(self, rec):
    """rec: dict with 's' (service) and 'rt' (receive time on the shared clock)."""
    t0 = time.perf_counter()
    s, rt = rec['s'], rec['rt']
    self.counts[s] += 1
    self.rate[s].append(rt)
    if s in ('frame', 'model', 'cameraOdometry', 'liveLocationKalman'):
      k = rec.get('lt', rt) if s != 'frame' else rec.get('eof', rt)      # use the producer's own timestamp where there is one
      p = self.iv_last.get(s)
      if p is not None and k > p:
        self.iv[s].append(k - p)
      self.iv_last[s] = k
      if s == 'model' and rec.get('eof') and rec.get('lt'):
        self.iv['model_lat'].append(rec['lt'] - rec['eof'])
        if rec.get('exec') is not None:
          self.iv['model_exec'].append(rec['exec'])
      if s == 'frame' and rec.get('eof'):
        self.iv['frame_age'].append(rt - rec['eof'])
      if rec.get('lt') is not None and s == 'model':
        self.iv['ipc_' + s].append(rt - rec['lt'])
    line = self._line(rec)
    rk = s
    lim = self.cfg['ring_hz'].get(s)
    if s == 'can_raw':
      rk = ('can_raw', rec['addr'])
      lim = None if rec['addr'] in self.cfg['can_raw_full'] else self.cfg['ring_hz'].get('can_raw_other')
    ring_ok = True
    if lim:
      tk = rec.get('lt') or rt
      if tk - self.ring_last.get(rk, -1e9) < 1.0 / lim:
        ring_ok = False
      else:
        self.ring_last[rk] = tk
    if self.disk_state != 'hard':
      if ring_ok:
        self.w.put_line(self._seg(rt), line)
      g = GROUPS.get(s, 'performance')
      if s not in ('can_raw',) and self.disk_state == 'ok' and self.trace_bytes < self.cfg['max_trace_mb'] * 1e6:
        tk = rec.get('lt') or rt                         # producer time: a lagging reader must not change what is kept
        if tk - self.trace_last.get(s, -1e9) >= 1.0 / self.cfg['trace_hz'] or s in ('perf', 'diag', 'usb', 'cam_quality', 'health'):
          self.trace_last[s] = tk
          self.w.put_line(os.path.join(self.root, g, 'trace.jsonl'), line)
          self.trace_bytes += len(line)
    for kind, detail in self.det.feed(rec):
      self.trigger(kind, detail, rt)
    self.ingest_s += time.perf_counter() - t0

  def note_msg(self, s, lt, rt):
    """cheap per-message bookkeeping (no JSON, no extraction): interval of the producer's own timestamps and read delay."""
    p = self.iv_last.get('n_' + s)
    if p is not None and lt > p:
      self.iv[s].append(lt - p)
    self.iv_last['n_' + s] = lt
    self.read_lag = rt - lt

  read_lag = 0.0

  def ingest_can(self, rt, lt, frames):
    """frames: list of (addr, bus, data bytes). Counts all, records + decodes the watched/spec'd addresses."""
    CAN_SPEC, decode = canspec.CAN_SPEC, canspec.decode
    for addr, bus, dat in frames:
      c = self.can_stats[(addr, bus)]
      c[0] += 1
      if c[1] is not None and addr in canspec.CAN_SPEC and rt - c[1] > self.can_gapmax.get(addr, 0):
        self.can_gapmax[addr] = rt - c[1]
      c[1] = rt
      if addr in CAN_SPEC:
        d = decode(addr, dat)
        rec = {'s': 'can_raw', 'rt': rt, 'lt': lt, 'addr': addr, 'bus': bus, 'hex': bytes(dat).hex(), 'sig': d}
        self.ingest(rec)
        self.can_last[addr] = rt

  # ---- anomalies
  def trigger(self, kind, detail, t):
    if len(self.done) + len(self.events) >= self.cfg['max_events'] and not self.events:
      return
    self.timeline.append((t, kind, detail))
    if t - self.logged_t.get(kind, -1e9) >= 1.0:          # a detector firing every sample must not flood the log
      self.logged_t[kind] = t
      self.w.put_line(os.path.join(self.root, 'events', 'anomalies.jsonl'),
                      self._line({'rt': t, 'kind': kind, 'detail': detail, 'suppressed_since_last': self.suppressed.pop(kind, 0)}))
    else:
      self.suppressed[kind] += 1
      for ev in self.events:
        if t <= ev.t_end:
          ev.last = t
          if t + self.cfg['post_s'] - ev.t0 <= self.cfg['max_event_s']:
            ev.t_end = t + self.cfg['post_s']
      return
    for ev in self.events:                       # extend the open event instead of opening a second one
      if t <= ev.t_end:
        ev.reasons.append({'t': t, 'kind': kind, 'detail': detail})
        ev.kinds.add(kind)
        ev.last = t
        if t + self.cfg['post_s'] - ev.t0 <= self.cfg['max_event_s']:
          ev.t_end = t + self.cfg['post_s']
        return
    if t - self.last_kind_t.get(kind, -1e9) < self.cfg['event_cooldown_s']:
      return
    self.last_kind_t[kind] = t
    if len(self.done) + len(self.events) >= self.cfg['max_events']:
      return
    self.n_events += 1
    ev = Event(self.n_events, t, [kind], {'t': t, 'kind': kind, 'detail': detail})
    ev.t_end = t + self.cfg['post_s']
    self.events.append(ev)

  # ---- periodic work
  def tick(self, now=None):
    now = self.clock() if now is None else now
    cfg = self.cfg
    # watched CAN addresses that stopped completely
    for a in self.det.can_int:
      last = self.can_last.get(a)
      iv = self.det.can_int[a]
      if last is not None and len(iv) >= cfg['thr']['can_gap_min_n']:
        m = _median(iv)
        silent = now - last
        if silent > max(cfg['thr']['can_gap_factor'] * m, cfg['thr']['can_gap_min_s']) * 2 and a not in self.can_per:
          self.can_per[a] = now
          for k, d in self.det.d_can_silence({'addr': a, 'silent_s': silent, 'typical_s': m}):
            self.trigger(k, d, now)
        elif silent < 2 * m:
          self.can_per.pop(a, None)
    # events whose post window is over
    for ev in list(self.events):
      if now >= ev.t_end:
        self.events.remove(ev)
        self.finalize(ev)
    # disk guard (statvfs is cheap)
    try:
      st = os.statvfs(self.root)
      free = st.f_bavail * st.f_frsize / 1e6
    except OSError:
      free = 1e9
    old = self.disk_state
    self.disk_state = 'hard' if free < cfg['hard_free_mb'] else ('soft' if free < cfg['min_free_mb'] else 'ok')
    if self.disk_state != old:
      self.w.put_line(os.path.join(self.root, 'events', 'anomalies.jsonl'),
                      self._line({'rt': now, 'kind': 'disk_' + self.disk_state, 'detail': {'free_mb': round(free)}}))
    self.free_mb = free
    # ring pruning (never below the oldest pending event's window)
    protect = min([e.t0 - cfg['pre_s'] for e in self.events] + [now - cfg['keep_s']]) - cfg['seg_s']
    self.w.put_cmd(self._prune, protect)
    self.last_tick = now

  def _prune(self, before):
    seg = self.cfg['seg_s']
    try:
      names = sorted(os.listdir(self.ring))
    except OSError:
      return
    total = 0
    for n in names:
      p = os.path.join(self.ring, n)
      try:
        total += os.path.getsize(p)
      except OSError:
        pass
    for n in names:
      p = os.path.join(self.ring, n)
      try:
        if n.startswith('tel_'):
          if (int(n[4:12]) + 1) * seg < before:
            self.w.close_file(p); os.remove(p)
        elif n == 'frames':
          fd = p
          for f in os.listdir(fd):
            if f.endswith('.jpg') and int(f[2:-4]) / 1000.0 < before:
              os.remove(os.path.join(fd, f))
      except (OSError, ValueError):
        pass
    # hard size cap on the ring (drop oldest segments even if protected: storage safety wins)
    if total > self.cfg['max_ring_mb'] * 1e6:
      for n in sorted(x for x in os.listdir(self.ring) if x.startswith('tel_'))[:3]:
        try:
          self.w.close_file(os.path.join(self.ring, n)); os.remove(os.path.join(self.ring, n))
        except OSError:
          pass

  def add_frame(self, rt, jpg):
    """a representative camera frame for the ring (rate limited)."""
    if self.disk_state != 'ok' or rt - self.frame_last_t < 1.0 / self.cfg['frame_hz']:
      return False
    self.frame_last_t = rt
    return self.w.put_bytes(os.path.join(self.ring, 'frames', 'f_%d.jpg' % int(rt * 1000)), jpg)

  def finalize(self, ev):
    cfg = self.cfg
    t0, t1 = ev.t0 - cfg['pre_s'], ev.t_end
    name = 'event_%03d_%s' % (ev.id, sorted(ev.kinds)[0])
    d = os.path.join(self.root, 'events', name)
    summary = {
      'event': name, 'trigger_rt': ev.t0, 'window_rt': [t0, t1], 'pre_s': cfg['pre_s'], 'post_s': round(ev.t_end - ev.t0, 1),
      'kinds': sorted(ev.kinds), 'reasons': ev.reasons[:200], 'n_reasons': len(ev.reasons),
      'clock': 'CLOCK_BOOTTIME seconds (same as cereal logMonoTime); see metadata/clock.json for the wall-clock anchor',
    }
    self.done.append(summary)
    self.w.put_cmd(self._copy_event, d, t0, t1, summary)

  def _copy_event(self, d, t0, t1, summary):
    os.makedirs(os.path.join(d, 'frames'), exist_ok=True)
    n = 0
    segs = sorted(x for x in os.listdir(self.ring) if x.startswith('tel_'))
    with gzip.open(os.path.join(d, 'telemetry_full.jsonl.gz'), 'wt') as out:
      for sname in segs:
        idx = int(sname[4:12])
        s0 = idx * self.cfg['seg_s']
        if s0 + self.cfg['seg_s'] < t0 or s0 > t1:
          continue
        with open(os.path.join(self.ring, sname)) as f:
          for line in f:
            try:
              rt = json.loads(line)['rt']
            except (ValueError, KeyError):
              continue
            if t0 <= rt <= t1:
              out.write(line)
              n += 1
    nf = 0
    fd = os.path.join(self.ring, 'frames')
    if os.path.isdir(fd):
      for f in sorted(os.listdir(fd)):
        if f.endswith('.jpg') and t0 <= int(f[2:-4]) / 1000.0 <= t1:
          shutil.copy2(os.path.join(fd, f), os.path.join(d, 'frames', f))
          nf += 1
    summary['records'], summary['frames'] = n, nf
    with open(os.path.join(d, 'summary.json'), 'w') as f:
      json.dump(summary, f, indent=1)

  # ---- stats for the 1 Hz 'diag' record (the logger watches itself)
  def self_stats(self, now):
    wl = list(self.w.wlat_ms)
    cpu = (time.process_time() - self.proc_t0) / max(1e-6, time.time() - self.proc_w0)
    first = not getattr(self, '_stat_seen', False)
    self._stat_seen = True
    self.proc_t0, self.proc_w0 = time.process_time(), time.time()
    ing = self.ingest_s
    self.ingest_s = 0.0
    stats = {}
    for k, v in self.iv.items():
      if v:
        m = sum(v) / len(v)
        stats[k] = {'n': len(v), 'mean_ms': round(1e3 * m, 2), 'max_ms': round(1e3 * max(v), 2), 'min_ms': round(1e3 * min(v), 2),
                    'std_ms': round(1e3 * math.sqrt(sum((x - m) ** 2 for x in v) / len(v)), 2)}
    self.iv.clear()
    tot = collections.Counter()
    for (a, bus), (n, _) in self.can_stats.items():
      if a in canspec.CAN_SPEC:
        tot[a] += n
    can_hz = {}
    for a, n in tot.items():
      can_hz[hex(a)] = n - self.can_prev.get(a, 0)
      self.can_prev[a] = n
    gaps = {hex(a): round(1e3 * g, 1) for a, g in self.can_gapmax.items()}
    self.can_gapmax = {}
    return {'s': 'diag', 'rt': now, 'iv': stats, 'can_hz': can_hz, 'can_gap_max_ms': gaps, 'self_cpu_core': None if first else round(100 * cpu, 2), 'queue': self.w.qsize(), 'queue_max': self.w.max_q,
            'dropped': self.w.dropped, 'write_ms_p95': round(sorted(wl)[int(0.95 * (len(wl) - 1))], 2) if wl else None,
            'fsync_ms': self.w.fsync_ms, 'write_errors': self.w.errors, 'bytes_written': self.w.bytes_written,
            'disk_state': self.disk_state, 'free_mb': round(getattr(self, 'free_mb', 0)), 'events_open': len(self.events),
            'events_done': len(self.done), 'ingest_s_per_s': round(ing, 4),
            'rate_hz': {k: round((len(v) - 1) / (v[-1] - v[0]), 1) for k, v in self.rate.items()
                        if len(v) > 5 and v[-1] > v[0] and now - v[-1] < 3}}

  def close(self, now=None):
    now = self.clock() if now is None else now
    for ev in list(self.events):          # keep what we have for events still waiting for their post window
      self.events.remove(ev)
      ev.t_end = min(ev.t_end, now)
      ev.reasons.append({'t': now, 'kind': 'shutdown_before_post_window', 'detail': {}})
      self.finalize(ev)
    ok = self.w.stop(timeout=200.0)         # event copies (gzip of up to ~2 min of full-rate data) are slow on the EON flash/CPU
    return ok
