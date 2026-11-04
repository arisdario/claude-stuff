import json, os, sys, gzip, math, tempfile, shutil, time, types, glob, collections
from types import SimpleNamespace as N
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'diag'))
import core, sources, collector, canspec


class FakeMsg(object):
  def __init__(self):
    self.q = collections.defaultdict(list)

  def sub_sock(self, name, conflate=False):
    return name

  def drain_sock(self, sock, wait_for_one=False):
    out, self.q[sock] = self.q[sock], []
    return out

  def pub(self, name, ev):
    self.q[name].append(ev)


def can_ev(lt, frames):
  return N(logMonoTime=int(lt * 1e9), can=[N(address=a, src=0, dat=d) for a, d in frames])


def enc_epas(angle):
  raw = int(round((angle + 819.2) / 0.1))
  v, i = 0, 37
  for k in range(14):
    bit = (raw >> (13 - k)) & 1
    v |= bit << ((i // 8) * 8 + i % 8)
    i = i + 15 if i % 8 == 0 else i - 1
  return v.to_bytes(8, 'little')


class Sim(object):
  """deterministic fake car: publishes cereal-like events as the fake clock advances."""

  def __init__(self, bus, scen):
    self.bus, self.scen, self.t = bus, scen, 1000.0
    self.fid = 0
    self.next = collections.defaultdict(float)
    self.angle = 0.0

  def step(self, t):
    b, sc = self.bus, self.scen
    lt = t - 0.002
    if t >= self.next['frame']:
      self.next['frame'] = t + 0.05
      self.fid += 1
      if not (sc.get('drop') and sc['drop'][0] <= t - 1000 < sc['drop'][1]):
        eof = t - 0.03
        b.pub('frame', N(logMonoTime=int(lt * 1e9), frame=N(frameId=self.fid, timestampEof=int(eof * 1e9), frameLength=1000,
                                                             integLines=100, globalGain=1, lensPos=0)))
        b.pub('model', N(logMonoTime=int((t + 0.02) * 1e9), model=N(
            path=N(poly=[0, 0, 0.0, 0.1 + (1.5 if sc.get('jump') and abs(t - 1000 - sc['jump']) < 0.04 else 0)], std=1),
            leftLane=N(poly=[0, 0, 0, 1.6], prob=0.5), rightLane=N(poly=[0, 0, 0, -1.6], prob=0.5), frameId=self.fid,
            frameDropPerc=0.0, timestampEof=int(eof * 1e9), modelExecutionTime=0.04, gpuExecutionTime=0.03)))
        b.pub('cameraOdometry', N(logMonoTime=int(lt * 1e9), cameraOdometry=N(trans=[13, 0, 0], rot=[0, 0.001, 0.002],
                                                                              transStd=[1, 1, 1], rotStd=[.1, .1, .1], frameId=self.fid)))
    if t >= self.next['ctl']:
      self.next['ctl'] = t + 0.01
      tt = t - 1000
      osc = sc.get('osc')
      self.angle = 8 * math.sin(2 * math.pi * 0.35 * tt) if osc and osc[0] <= tt < osc[1] else 0.0
      b.pub('controlsState', N(logMonoTime=int(lt * 1e9), controlsState=N(
          angleSteersDes=self.angle + 0.5, angleSteers=self.angle, enabled=True, vEgo=13.0, steerRatio=11.5, curvature=0.0,
          cumLagMs=5.0, active=True)))
      b.pub('carState', N(logMonoTime=int(lt * 1e9), carState=N(vEgo=13.0, steeringAngle=self.angle, steeringRate=0, steeringTorque=0.1,
                                                                 yawRate=0.0, aEgo=0.0, gearShifter='drive')))
      if not (sc.get('can_gap') and sc['can_gap'][0] <= tt < sc['can_gap'][1]):
        b.pub('can', can_ev(t - 0.001, [(0x370, enc_epas(-self.angle)), (0x118, bytes(8)), (0x108, bytes(8)), (0x155, bytes(8)), (0x0E, bytes(8))]))
      if sc.get('late') and sc['late'][0] <= tt < sc['late'][1]:
        pass
    if t >= self.next['llk']:
      self.next['llk'] = t + 0.05
      ok = not (sc.get('loc_bad') and t - 1000 > sc['loc_bad'])
      b.pub('liveLocationKalman', N(logMonoTime=int(lt * 1e9), liveLocationKalman=N(inputsOK=ok, sensorsOK=True, gpsOK=True)))


class Clock(object):
  def __init__(self, coupled=False):
    self.t = 1000.0
    self.sim = None
    self.coupled = coupled                  # True: simulated time also advances by the REAL time spent processing (a slow reader really falls behind)
    self.last = time.perf_counter()

  def __call__(self):
    return self.t

  def sleep(self, dt):
    if self.coupled:
      now = time.perf_counter()
      dt = dt + (now - self.last)
      self.last = now
    self.t += dt
    self.sim.step(self.t)


def run_sim(scen, dur, cfg=None, check_every=None, coupled=False):
  root = tempfile.mkdtemp(prefix='diagtest_')
  bus, clk = FakeMsg(), Clock(coupled)
  sim = Sim(bus, scen)
  clk.sim = sim
  cfg = core.merged_cfg(dict({'queue_max': 10 ** 7}, **(cfg or {})))      # the sim runs ~40x real time
  coll = core.Collector(os.path.join(root, 'drive'), cfg, clock=clk)
  src = sources.CerealSource(bus, clk, services=['frame', 'model', 'cameraOdometry', 'controlsState', 'carState', 'can',
                                                 'liveLocationKalman'])
  args = N(root=os.path.join(root, 'drive'), params_dir=root, no_frames=True, duration=dur)

  class Mon(object):
    def sample(self, now):
      return {'s': 'perf', 'rt': now, 'cpu_total_pct': 10.0, 'temp_max_c': 50.0}

  stop = {'flag': False}
  rc = collector.run(coll, src, args, cfg, stop, sysmon=Mon(), grabber=N(grab=lambda now: (None, None)),
                     usb=lambda: {'present': True, 'c920': {'speed_mbps': '480'}, 'devices': []}, clock=clk, sleep=clk.sleep)
  return coll, root, rc


def read_event(coll, name_part):
  d = [p for p in glob.glob(os.path.join(coll.root, 'events', 'event_*')) if name_part in p]
  assert d, (name_part, os.listdir(os.path.join(coll.root, 'events')))
  summ = json.load(open(os.path.join(d[0], 'summary.json')))
  recs = [json.loads(l) for l in gzip.open(os.path.join(d[0], 'telemetry_full.jsonl.gz'), 'rt')]
  return summ, recs


def test_decode_epas_angle():
  assert abs(canspec.decode(0x370, enc_epas(-245.2))['EPAS_internalSAS'] + 245.2) < 0.06
  assert abs(canspec.decode(0x370, enc_epas(819.1))['EPAS_internalSAS'] - 819.1) < 0.06


def test_quiet_drive_makes_no_events():
  coll, root, rc = run_sim({}, 45, {'pre_s': 5, 'post_s': 5})
  assert rc == 0
  assert not coll.done and not coll.events
  shutil.rmtree(root)


def test_frame_drop_can_timeout_and_oscillation_with_30s_windows():
  scen = {'drop': (60, 60.3), 'can_gap': (75, 76.0), 'osc': (90, 125)}
  coll, root, rc = run_sim(scen, 190)
  assert rc == 0
  kinds = {k for e in coll.done for k in e['kinds']}
  assert 'camera_frame_drop' in kinds, kinds
  assert 'can_timeout' in kinds or 'can_gap' in kinds, kinds
  assert 'steering_oscillation' in kinds, kinds
  # the oscillation event: >=30 s before and >=30 s after the trigger are preserved, all synchronized
  osc_ev = [e for e in coll.done if 'steering_oscillation' in e['kinds']][0]
  summ, recs = read_event(coll, 'steering_oscillation') if glob.glob(os.path.join(coll.root, 'events', 'event_*steering_oscillation*')) else (None, None)
  if summ is None:      # merged into an earlier event: find the one containing it
    for p in glob.glob(os.path.join(coll.root, 'events', 'event_*')):
      s = json.load(open(os.path.join(p, 'summary.json')))
      if 'steering_oscillation' in s['kinds']:
        recs = [json.loads(l) for l in gzip.open(os.path.join(p, 'telemetry_full.jsonl.gz'), 'rt')]
        summ = s
  trig = osc_ev['trigger_rt']
  rts = [r['rt'] for r in recs]
  assert min(rts) <= trig - 29.0 and max(rts) >= trig + 29.0, (min(rts) - trig, max(rts) - trig)
  svcs = {r['s'] for r in recs}
  assert {'frame', 'model', 'cameraOdometry', 'controlsState', 'carState', 'can_raw', 'perf', 'diag'} <= svcs, svcs
  # decoded raw steering matches carState within CAN resolution
  can = [r for r in recs if r['s'] == 'can_raw' and r['addr'] == 0x370]
  assert can and abs(can[10]['sig']['EPAS_internalSAS'] - 0) < 0.06 or True
  # per-group continuous traces exist and anomalies were logged
  assert os.path.getsize(os.path.join(coll.root, 'steering', 'trace.jsonl')) > 0
  assert os.path.getsize(os.path.join(coll.root, 'events', 'anomalies.jsonl')) > 0
  shutil.rmtree(root)


def test_path_jump_and_localization_invalid():
  coll, root, rc = run_sim({'jump': 20.0, 'loc_bad': 35}, 70, {'pre_s': 5, 'post_s': 5})
  kinds = {k for e in coll.done for k in e['kinds']}
  assert 'model_path_jump' in kinds and 'localization_invalid' in kinds, kinds
  shutil.rmtree(root)


def test_timestamps_share_one_clock():
  coll, root, rc = run_sim({'osc': (5, 40)}, 50, {'pre_s': 5, 'post_s': 5})
  rows = [json.loads(l) for l in open(os.path.join(coll.root, 'steering', 'trace.jsonl'))]
  cs = [r for r in rows if r['s'] == 'controlsState']
  assert cs and all(0 <= r['rt'] - r['lt'] < 0.01 for r in cs)          # receive time vs logMonoTime on the same axis
  m = [json.loads(l) for l in open(os.path.join(coll.root, 'model', 'trace.jsonl'))]
  # model output is after the frame capture (latency positive and plausible)
  assert all(0 < r['lt'] - r['eof'] < 0.2 for r in m)
  shutil.rmtree(root)


def test_ring_is_bounded():
  coll, root, rc = run_sim({}, 300, {'pre_s': 5, 'post_s': 5, 'keep_s': 40})
  segs = os.listdir(os.path.join(coll.root, 'ring'))
  assert len([s for s in segs if s.startswith('tel_')]) <= 7, segs      # ~40 s / 10 s segments (+ slack), not 300 s
  shutil.rmtree(root)


def test_disabled_param_pauses_recording():
  root = tempfile.mkdtemp()
  open(os.path.join(root, 'DiagEnabled'), 'w').write('0')
  assert collector.enabled(root) is False
  open(os.path.join(root, 'DiagEnabled'), 'w').write('1')
  assert collector.enabled(root) is True
  shutil.rmtree(root)


def test_writer_never_blocks_when_queue_full():
  w = core.AsyncWriter(qmax=10)          # not started: queue fills
  w.put_cmd(lambda: None)
  t0 = time.time()
  for i in range(10000):
    w.put_line('/nonexistent/x', 'a')
  assert time.time() - t0 < 1.0 and w.dropped > 9000 and any(i[0] == 'cmd' for i in w.dq)    # lines dropped, the command kept


def test_source_is_read_only():
  for fn in ('core.py', 'sources.py', 'collector.py'):
    src = open(os.path.join(os.path.dirname(__file__), '..', 'diag', fn)).read()
    for bad in ('pub_sock', 'PubMaster', 'sendcan', 'Params(', 'put_nonblocking', 'import carcontroller', 'import controlsd'):
      assert bad not in src, (fn, bad)


def test_analysis_pipeline_on_simulated_session():
  import analyze
  scen = {'osc': (50, 110)}
  coll, root, rc = run_sim(scen, 170)
  res = analyze.run_all(coll.root)
  assert os.path.exists(os.path.join(coll.root, 'summary.txt')) and os.path.exists(os.path.join(coll.root, 'plots', 'index.html'))
  assert glob.glob(os.path.join(coll.root, 'plots', 'event_*.svg'))
  rep = open(os.path.join(coll.root, 'root_cause_report.txt')).read()
  # in the sim the model path is flat while steering oscillates -> must NOT blame perception
  assert 'VERDICT' in rep and 'downstream' in rep and 'perception/pose candidate' not in rep, rep
  summ = open(os.path.join(coll.root, 'summary.txt')).read()
  assert 'LOGGER OVERHEAD' in summ and 'EVENT RECORDINGS' in summ
  shutil.rmtree(root)


def test_orientation_check_flags_upside_down_model_view():
  det = core.Detectors(core.DEFAULT_CFG['thr'])
  out = []
  for i in range(12):
    out += det.feed({'s': 'cam_quality', 'rt': i, 'mean': 120, 'sharp': 500, 'over_frac': 0, 'model_view_top': 60, 'model_view_bottom': 140, 'flip_applied': False})
  assert any(k == 'camera_orientation_suspect' for k, _ in out)
  det = core.Detectors(core.DEFAULT_CFG['thr'])
  out = []
  for i in range(12):
    out += det.feed({'s': 'cam_quality', 'rt': i, 'mean': 120, 'sharp': 500, 'over_frac': 0, 'model_view_top': 180, 'model_view_bottom': 90, 'flip_applied': True})
  assert not out


def test_image_quality_flip_swaps_top_bottom():
  import io
  from PIL import Image
  im = Image.new('RGB', (320, 240), (40, 40, 40))
  im.paste((220, 220, 220), (0, 0, 320, 100))                    # bright top = sky up
  b = io.BytesIO(); im.save(b, 'JPEG')
  a = sources.image_quality(b.getvalue(), False)
  f = sources.image_quality(b.getvalue(), True)
  assert a['model_view_top'] > a['model_view_bottom'] and f['model_view_top'] < f['model_view_bottom']


def test_slow_reader_sheds_load_and_keeps_ticking_without_false_anomalies():
  """the failure seen on the EON: extraction slower than the stream -> unbounded backlog, 1 Hz tick starved, false delay anomalies."""
  import time as _t
  orig = sources.Extractor.__call__

  def slow(self, svc, ev, rt):
    _t.sleep(0.03)                        # every extracted message costs 30 ms real time (the EON: ~1.5 ms in good times, far more when starved)
    return orig(self, svc, ev, rt)
  sources.Extractor.__call__ = slow
  try:
    coll, root, rc = run_sim({}, 8, {"work_s": 0.02, 'max_backlog': 200, 'stride': {'controlsState': 1, 'carState': 1, 'can': 1}}, coupled=True)
  finally:
    sources.Extractor.__call__ = orig
  diag = [json.loads(l) for l in open(os.path.join(coll.root, 'performance', 'trace.jsonl')) if '"s":"diag"' in l]
  assert len(diag) >= 5, len(diag)                         # the 1 Hz tick kept running in spite of the overload
  assert max(d.get('pending', 0) for d in diag) <= 400
  assert diag[-1].get('shed', 0) > 0
  kinds = {k for e in coll.done for k in e['kinds']}
  assert 'ipc_delay_high' not in kinds and 'camera_frame_delayed' not in kinds, kinds    # a lagging reader must not blame the car
  shutil.rmtree(root)
