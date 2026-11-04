#!/usr/bin/env python3
"""Steering-diagnostics collector (READ-ONLY). Started/stopped by start_diagnostics.sh / stop_diagnostics.sh.

  python3 collector.py --root /data/steering_diagnostics/drive_YYYYMMDD_HHMMSS [--cfg cfg.json] [--params-dir /data/params/d]

It subscribes to cereal sockets, reads /proc, /sys and /dev/c920_live.jpg. It never publishes a message, never writes a Param,
never touches CAN transmit and never imports control code. Runs at nice +10 and drops data (counted) rather than ever waiting.
Pause/resume at any time:  echo 0 > /data/params/d/DiagEnabled   (1 = record, missing = record)
"""
import argparse
import json
import os
import platform
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
  sys.path.insert(0, HERE)

import core  # noqa: E402
import sources  # noqa: E402

CAL_FILES = ('CalibrationParams', 'LiveParameters', 'ControlsParams', 'IsMetric', 'Version', 'GitCommit', 'GitBranch',
             'DongleId', 'CommunityFeaturesToggle')
# constants of the 0.8.3 port (medmodel) and of the camerad C920 backend; reported so geometry can be checked offline
INTRINSICS = {
  'eon_road_camera': {'focal_px': 910.0, 'cx': 582.0, 'cy': 437.0, 'width': 1164, 'height': 874, 'distortion': 'none (EON model)'},
  'c920_after_camerad': {'focal_px_approx': 907.0, 'cx': 582.0, 'cy': 437.0, 'note': 'MJPEG 1280x720 cropped 58 px/side, padded 77 rows -> 1164x874; '
                         'distortion of the C920 is NOT corrected in camerad (no coefficients applied)'},
  'model_083_input': {'width': 512, 'height': 256, 'focal_px': 910.0, 'cx': 256.0, 'cy': 47.6,
                      'assumed_camera_height_m': 1.22, 'note': 'ground-plane homography warp; calibration estimates rotation only'},
}


def rdfile(p):
  try:
    with open(p) as f:
      return f.read()
  except (OSError, IOError):
    return None


def enabled(params_dir):
  v = rdfile(os.path.join(params_dir, 'DiagEnabled'))
  return not (v is not None and v.strip() in ('0', 'false', 'off'))


def vp_from_rpy(rpy, f=910.0, cx=582.0, cy=437.0):
  """same convention calibrationd uses: yaw=atan((vpx-cx)/f), pitch=-atan(vpy_norm*cos(yaw))."""
  import math
  roll, pitch, yaw = rpy
  x = cx + f * math.tan(yaw)
  y = cy - f * math.tan(pitch) / math.cos(yaw)
  return {'vp_px': [round(x, 1), round(y, 1)], 'yaw_deg': round(math.degrees(yaw), 3), 'pitch_deg': round(math.degrees(pitch), 3),
          'roll_deg': round(math.degrees(roll), 3)}


def write_metadata(root, args, cfg, params_dir):
  md = os.path.join(root, 'metadata')
  boot, wall = core.boot_now(), time.time()
  info = {
    'created_wall': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(wall)), 'uname': ' '.join(platform.uname()),
    'python': sys.version.split()[0], 'kernel': (rdfile('/proc/version') or '').strip(),
    'uptime_s': (rdfile('/proc/uptime') or '').split()[:1], 'cpu': [l.strip() for l in (rdfile('/proc/cpuinfo') or '').splitlines()
                                                                       if l.startswith(('Hardware', 'model name'))][:2],
    'meminfo_head': (rdfile('/proc/meminfo') or '').splitlines()[:3], 'pid': os.getpid(),
    'modules': {}, 'openpilot_version': (rdfile('/data/openpilot/selfdrive/common/version.h') or '').strip(),
    'repo_head': (rdfile('/data/openpilot/.git/HEAD') or '').strip(), 'argv': sys.argv,
    'skipped_services': [],
  }
  for m in ('numpy', 'cv2', 'PIL', 'matplotlib', 'capnp', 'zmq'):
    try:
      mod = __import__(m)
      info['modules'][m] = getattr(mod, '__version__', 'yes')
    except Exception:
      info['modules'][m] = None
  for key in ('/data/openpilot/selfdrive/camerad/camerad', '/data/openpilot/selfdrive/modeld/_modeld'):
    try:
      info['mtime_' + os.path.basename(key)] = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(key)))
    except OSError:
      pass
  json.dump(info, open(os.path.join(md, 'system.json'), 'w'), indent=1)
  json.dump({'clock': 'CLOCK_BOOTTIME seconds (== cereal logMonoTime / camerad timestampEof reference)', 'boot_s_at_start': boot,
             'wall_epoch_at_start': wall, 'wall_minus_boot': wall - boot,
             'how_to_get_wall_time': 'wall = rt + wall_minus_boot'}, open(os.path.join(md, 'clock.json'), 'w'), indent=1)
  json.dump(cfg, open(os.path.join(md, 'config.json'), 'w'), indent=1)
  cal = os.path.join(root, 'calibration')
  snap = {'intrinsics': INTRINSICS, 'params': {}}
  for k in CAL_FILES:
    v = rdfile(os.path.join(params_dir, k))
    if v is not None:
      snap['params'][k] = v[:2000]
  try:
    cp = json.loads(snap['params'].get('CalibrationParams', '{}'))
    vp = cp.get('vanishing_point')
    if vp:
      import math
      f, cx, cy = 910.0, 582.0, 437.0
      yaw = math.atan((vp[0] - cx) / f)
      pitch = -math.atan((vp[1] - cy) / f * math.cos(yaw))
      snap['calibration_params_geometry'] = {'vp_px': vp, 'yaw_deg': round(math.degrees(yaw), 3), 'pitch_deg': round(math.degrees(pitch), 3),
                                             'valid_blocks': cp.get('valid_blocks'), 'roll': 'not estimated by calibrationd'}
  except Exception:
    pass
  mount = rdfile('/data/diag_camera_mount.json')
  snap['camera_mount'] = json.loads(mount) if mount else 'not provided (write /data/diag_camera_mount.json: {"x_m":..,"y_m":..,"z_m":..,"note":..})'
  json.dump(snap, open(os.path.join(cal, 'calibration_start.json'), 'w'), indent=1)
  for n in ('/data/camera_focus', '/data/camera_flip', '/data/camera_led_auto'):
    v = rdfile(n)
    if v is not None:
      info.setdefault('camera_settings', {})[os.path.basename(n)] = v.strip()
  json.dump(info, open(os.path.join(md, 'system.json'), 'w'), indent=1)
  return boot, wall


class Trackers(object):
  """change logs: calibration and learned parameters (what changed, when, from what)."""

  def __init__(self, coll):
    self.c = coll
    self.rpy = None
    self.status = None
    self.perc = None
    self.par = {}

  def feed(self, r):
    s = r['s']
    if s == 'liveCalibration':
      rpy = r.get('rpy')
      ev = None
      if self.status is not None and r.get('status') != self.status:
        ev = 'status_change'
      if self.perc is not None and r.get('perc') is not None and r['perc'] < self.perc - 5:
        ev = 'calibration_reset_or_invalidated'
      if rpy and self.rpy and max(abs(a - b) for a, b in zip(rpy, self.rpy)) > 0.0005:
        ev = ev or 'rpy_changed'
      if ev or self.rpy is None:
        row = {'rt': r['rt'], 'event': ev or 'first_value', 'rpy': rpy, 'status': r.get('status'), 'perc': r.get('perc')}
        if rpy:
          row.update(vp_from_rpy(rpy))
        self.c.w.put_line(os.path.join(self.c.root, 'calibration', 'changes.jsonl'), json.dumps(row))
        if ev in ('calibration_reset_or_invalidated',):
          self.c.trigger('calibration_reset', row, r['rt'])
      self.rpy, self.status, self.perc = rpy or self.rpy, r.get('status'), r.get('perc')
    elif s == 'liveParameters':
      ch = {}
      for k in ('sr', 'sf', 'ao'):
        v, o = r.get(k), self.par.get(k)
        if v is not None and (o is None or abs(v - o) > 0.005 * max(1.0, abs(o))):
          ch[k] = [o, v]
          self.par[k] = v
      if ch:
        self.c.w.put_line(os.path.join(self.c.root, 'localization', 'param_changes.jsonl'),
                          json.dumps({'rt': r['rt'], 'change': ch, 'valid': r.get('valid')}))


def preflight(base):
  """free disk + which openpilot processes are running, using only /proc (no ps/pgrep/awk/df needed)."""
  free = -1
  try:
    st = os.statvfs(base)
    free = int(st.f_bavail * st.f_frsize / 1e6)
  except OSError:
    pass
  procs = set()
  for d in os.listdir('/proc'):
    if d.isdigit():
      c = (rdfile('/proc/%s/cmdline' % d) or '').replace('\0', ' ')
      for n in ('controlsd', 'camerad', 'modeld', 'boardd', 'loggerd', 'paramsd'):
        if n in c:
          procs.add(n)
  print('FREE_MB=%d' % free)
  print('PROCS=%s' % ','.join(sorted(procs)))
  return 0


def selfcheck(seconds=6.0):
  """subscribe for a few seconds and report, per service: messages, rate, and extracted fields that were always None
  (= a cereal field name that does not exist in this openpilot build). Read-only."""
  sys.path.insert(0, '/data/openpilot')
  import cereal.messaging as messaging
  src = sources.CerealSource(messaging, core.boot_now)
  ex = sources.Extractor(full_hz=1000)
  seen, fields = {}, {}
  end = time.time() + seconds
  while time.time() < end:
    for svc, ev, rt in src.poll():
      seen[svc] = seen.get(svc, 0) + 1
      if svc == 'can':
        fr = sources.can_frames(ev)
        for a, bus, dat in fr:
          fields.setdefault('can', set()).add(a)
        continue
      rec = ex(svc, ev, rt)
      if rec:
        f = fields.setdefault(svc, {})
        for k, v in rec.items():
          f[k] = f.get(k, False) or (v is not None)
    time.sleep(0.003)
  bad = 0
  print('services skipped (no such socket): %s' % [n for n, _ in src.skipped])
  for svc in sources.SERVICES:
    n = seen.get(svc, 0)
    if svc == 'can':
      got = fields.get('can', set())
      print('%-20s %5d msgs  (%.0f/s)  addresses seen: %d ; spec addresses present: %s missing: %s' % (
        svc, n, n / seconds, len(got), sorted(hex(a) for a in got if a in canspec_addrs()), sorted(hex(a) for a in canspec_addrs() - got)))
      continue
    miss = sorted(k for k, ok in fields.get(svc, {}).items() if not ok)
    print('%-20s %5d msgs  (%.0f/s)  fields always None: %s' % (svc, n, n / seconds, miss or '-'))
    if n == 0 and svc not in ('health', 'gpsLocationExternal'):
      bad += 1
  print('SELFCHECK %s' % ('OK' if not bad else 'WARNING: %d services silent (is openpilot running?)' % bad))
  return 0


def canspec_addrs():
  return set(sources.canspec.CAN_SPEC)


def main(argv=None):
  if argv is None:
    argv = sys.argv[1:]
  if argv and argv[0] == '--selfcheck':
    return selfcheck()
  if argv and argv[0] == '--preflight':
    return preflight(argv[1] if len(argv) > 1 else '/data')
  ap = argparse.ArgumentParser()
  ap.add_argument('--root', required=True)
  ap.add_argument('--cfg', help='json file overriding config defaults')
  ap.add_argument('--params-dir', default='/data/params/d')
  ap.add_argument('--no-frames', action='store_true')
  ap.add_argument('--duration', type=float, default=0, help='stop by itself after N seconds (tests)')
  args = ap.parse_args(argv)
  cfg = core.merged_cfg(json.load(open(args.cfg)) if args.cfg else None)
  try:
    os.nice(10)
  except (OSError, AttributeError):
    pass
  stop = {'flag': False}
  for sg in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(sg, lambda *_: stop.__setitem__('flag', True))
  os.makedirs(args.root, exist_ok=True)
  try:
    sys.path.insert(0, '/data/openpilot')
    import cereal.messaging as messaging
  except Exception as e:                                   # noqa
    print('cannot import cereal.messaging: %s' % e)
    return 2
  coll = core.Collector(args.root, cfg)
  src = sources.CerealSource(messaging, core.boot_now)
  boot, wall = write_metadata(args.root, args, cfg, args.params_dir)
  mi = os.path.join(args.root, 'metadata', 'system.json')
  try:
    j = json.load(open(mi)); j['skipped_services'] = src.skipped; json.dump(j, open(mi, 'w'), indent=1)
  except Exception:
    pass
  return run(coll, src, args, cfg, stop)


def run(coll, src, args, cfg, stop, sysmon=None, grabber=None, usb=sources.usb_scan, clock=core.boot_now, sleep=time.sleep):
  ex = sources.Extractor()
  trk = Trackers(coll)
  sysmon = sysmon or sources.SystemSampler()
  grabber = grabber or sources.FrameGrabber(os.environ.get('DIAG_LIVE_JPG', '/dev/c920_live.jpg'))
  status_p = os.path.join(args.root, 'status.json')
  next_1s = next_usb = next_grab = 0.0
  last_usb = None
  q_last = 0.0
  errs = 0
  paused = False
  t_end = clock() + args.duration if args.duration else None
  while not stop['flag'] and (t_end is None or clock() < t_end):
    try:
      msgs = src.poll() if not paused else []
      for svc, ev, rt in msgs:
        if svc == 'can':
          coll.ingest_can(rt, (getattr(ev, 'logMonoTime', 0) or 0) / 1e9, sources.can_frames(ev))
          continue
        rec = ex(svc, ev, rt)
        if rec:
          coll.ingest(rec)
          trk.feed(rec)
      now = clock()
      if not args.no_frames and now >= next_grab:
        next_grab = now + 0.1
        jpg, mt = grabber.grab(now)
        if jpg and not paused:
          coll.add_frame(now, jpg)
          if now - q_last >= 1.0:
            q_last = now
            fl = (rdfile('/data/camera_flip') or '0').strip() not in ('0', '')
            qd = sources.image_quality(jpg, fl)
            if qd:
              qd.update(s='cam_quality', rt=round(now, 5), bytes=len(jpg))
              coll.ingest(qd)
      if now >= next_usb:
        next_usb = now + 2.0
        u = usb()
        sig = (u.get('present'), json.dumps(u.get('c920'), sort_keys=True))
        row = {'s': 'usb', 'rt': round(now, 5), 'present': u.get('present'), 'c920': u.get('c920'), 'hubs': u.get('devices')}
        if last_usb is not None and sig != last_usb:
          row['change'] = True
          if last_usb[0] and not sig[0]:
            coll.trigger('usb_camera_missing', {'c920': None}, now)
          elif sig[0] and not last_usb[0]:
            coll.trigger('usb_camera_reconnected', {'c920': u.get('c920')}, now)
          else:
            coll.trigger('usb_camera_changed', {'c920': u.get('c920')}, now)      # speed or bus address changed
        last_usb = sig
        if row.get('change') or int(now) % 10 < 2:
          coll.ingest(row)
      if now >= next_1s:
        next_1s = now + 1.0
        paused = not enabled(args.params_dir)
        if not paused:
          coll.tick(now)
          s = sysmon.sample(now)
          if s:
            coll.ingest(s)
          d = coll.self_stats(now)
          d['depth_max'] = src.take_depth() if hasattr(src, 'take_depth') else {}
          d['paused'] = False
          coll.ingest(d)
        coll.w.put_bytes(status_p, json.dumps({'rt': now, 'wall': time.time(), 'paused': paused, 'events': len(coll.done) + len(coll.events),
                                              'counts': dict(coll.counts), 'disk': coll.disk_state, 'free_mb': round(getattr(coll, 'free_mb', 0)),
                                              'dropped': coll.w.dropped, 'pid': os.getpid()}).encode())
      if not msgs:
        sleep(0.004)
    except Exception as e:           # the logger must survive anything
      errs += 1
      if errs <= 5:
        coll.w.put_line(os.path.join(args.root, 'performance', 'collector_errors.log'), '%r' % (e,))
      if errs > 5000:
        break
      sleep(0.05)
  ok = coll.close()
  return 0 if ok else 3


if __name__ == '__main__':
  sys.exit(main())
