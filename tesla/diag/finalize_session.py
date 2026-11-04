#!/usr/bin/env python3
"""Finish a diagnostics session: run the analysis, copy the original maplogs/intervention logs of the session period,
write README.txt + manifest.json, build <session>.zip and verify it. stdlib only; safe to re-run.

  python3 finalize_session.py SESSION_DIR [--no-plots] [--maplog-dir /data/maplog] [--events-dir /data/interventions]
"""
import glob
import hashlib
import json
import os
import shutil
import sys
import time
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import analyze  # noqa: E402

DESCR = {
  'metadata/system.json': ('software/hardware info, module availability, camera settings, versions', 'once at start'),
  'metadata/clock.json': ('wall-clock anchor: wall = rt + wall_minus_boot', 'once at start'),
  'metadata/config.json': ('thresholds and limits used by the anomaly detectors', 'once at start'),
  'calibration/calibration_start.json': ('camera intrinsics (EON, C920 after camerad, 0.8.3 model input), CalibrationParams/LiveParameters at start, mount info', 'once at start'),
  'calibration/changes.jsonl': ('liveCalibration changes: rpy, status, perc, vanishing point; resets/invalidations', 'on change'),
  'localization/param_changes.jsonl': ('learned steerRatio/stiffness/angleOffset changes (old,new), validity', 'on change (>0.5%)'),
  'camera/trace.jsonl': ('camerad frame records (frameId, timestampEof, exposure/gain/lens fields), image quality, USB state', 'frames <=10 Hz decimated, quality 1 Hz, usb 0.5 Hz'),
  'model/trace.jsonl': ('model output: path poly, lanes, edges, frameId, timestampEof, exec time, drop %', '<=10 Hz decimated'),
  'pose/trace.jsonl': ('cameraOdometry: trans/rot and their std', '<=10 Hz decimated'),
  'calibration/trace.jsonl': ('liveCalibration stream', '<=10 Hz decimated'),
  'can/trace.jsonl': ('panda health / CAN error counters', '1 Hz'),
  'localization/trace.jsonl': ('liveLocationKalman (yaw rate, orientation, velocity, flags), GPS, IMU, liveParameters', '<=10 Hz decimated'),
  'steering/trace.jsonl': ('controlsState (desired/actual, lateral controller state), carState, carControl, pathPlan', '<=10 Hz decimated'),
  'performance/trace.jsonl': ('1 Hz: system CPU/mem/temp/GPU, per-process CPU + scheduler delay, logger self-stats, per-second interval/latency/CAN-gap stats', '1 Hz'),
  'events/anomalies.jsonl': ('every anomaly trigger with its detail (rt on the boot clock)', 'on trigger'),
  'ring/': ('last ~100 s of FULL-RATE telemetry (all services, raw+decoded CAN) and camera frames (~3 fps)', 'full rate; rolling'),
  'plots/': ('SVG plots (open plots/index.html)', 'generated at stop'),
  '.fsync_probe': ('1-byte file used to time disk fsync latency (performance/trace.jsonl fsync_ms)', 'every 5 s'),
  'README.txt': ('how to read this session, time reference, known gaps', 'generated at stop'),
  'collector.out': ('stdout/stderr of the collector process (empty = nothing to report)', 'continuous'),
  'status.json': ('last heartbeat of the collector (counts per service, disk state, dropped records)', '1 Hz'),
  'performance/collector_errors.log': ('exceptions the collector survived (should not exist)', 'on error'),
  'events/analysis_errors.log': ('analysis failures for individual events (should not exist)', 'at stop'),
  'summary.txt': ('overview, logger overhead, timing, anomalies', 'generated at stop'),
  'root_cause_report.txt': ('rule-based reading of each oscillation event: onset order, lags, SNR', 'generated at stop'),
}


def sha(p):
  h = hashlib.sha256()
  with open(p, 'rb') as f:
    for b in iter(lambda: f.read(1 << 20), b''):
      h.update(b)
  return h.hexdigest()


def copy_logs(root, maplog_dir, events_dir, t_start, t_end):
  out = {'maplog': [], 'interventions': [], 'missing': []}
  for src, dst, key in ((maplog_dir, 'maplogs', 'maplog'), (events_dir, 'interventions', 'interventions')):
    if not os.path.isdir(src):
      out['missing'].append('%s not found' % src)
      continue
    files = [f for f in glob.glob(os.path.join(src, '*')) if os.path.isfile(f) and t_start - 3600 <= os.path.getmtime(f) <= t_end + 600]
    if not files:
      out['missing'].append('no files in %s modified during the session' % src)
    d = os.path.join(root, 'localization' if key == 'maplog' else 'steering', dst)
    for f in sorted(files)[-30:]:
      os.makedirs(d, exist_ok=True)
      shutil.copy2(f, d)
      out[key].append(os.path.relpath(os.path.join(d, os.path.basename(f)), root))
  return out


def main(argv):
  root = os.path.abspath(argv[1])
  plots = '--no-plots' not in argv
  ml = argv[argv.index('--maplog-dir') + 1] if '--maplog-dir' in argv else '/data/maplog'
  ev = argv[argv.index('--events-dir') + 1] if '--events-dir' in argv else '/data/interventions'
  analyze.run_all(root, plots)
  try:
    clk = json.load(open(os.path.join(root, 'metadata', 'clock.json')))
    t_start = clk['wall_epoch_at_start']
  except Exception:
    t_start = os.path.getctime(root)
  t_end = time.time()
  logs = copy_logs(root, ml, ev, t_start, t_end)
  ring_frames = len(glob.glob(os.path.join(root, 'ring', 'frames', '*.jpg')))
  ev_frames = len(glob.glob(os.path.join(root, 'events', 'event_*', 'frames', '*.jpg')))
  missing = list(logs['missing'])
  if ev_frames == 0 and ring_frames == 0:
    missing.append('no camera frames captured (/dev/c920_live.jpg absent or camera not streaming)')
  for g in ('model', 'steering', 'camera', 'localization', 'pose'):
    if not os.path.exists(os.path.join(root, g, 'trace.jsonl')):
      missing.append('%s/trace.jsonl absent: that service published nothing' % g)
  if not glob.glob(os.path.join(root, 'events', 'event_*')):
    missing.append('no anomaly event recordings (nothing triggered) - the continuous trace + ring are all there is')
  readme = """STEERING DIAGNOSTICS SESSION  %s

Everything here was recorded READ-ONLY (cereal subscriptions, /proc, /sys, /dev/c920_live.jpg). No steering, vehicle parameter or
calibration value was changed by the logger.

TIME: every record has 'rt' = seconds on CLOCK_BOOTTIME (the cereal logMonoTime / camerad timestampEof reference) and, where the
message carries it, 'lt' = logMonoTime. rt-lt = inter-process delay. wall time = rt + wall_minus_boot (metadata/clock.json).

START HERE: summary.txt, root_cause_report.txt, plots/index.html.
  events/event_NNN_*/  one folder per anomaly: summary.json (why), telemetry_full.jsonl.gz (>=30 s before and after, full rate,
                       all services incl. raw+decoded CAN), frames/*.jpg (camera frames in the window)
  <group>/trace.jsonl  continuous whole-session trace, decimated (see manifest.json for rates)
  ring/                the last ~100 s at full rate
  localization/maplogs, steering/interventions  copies of the original radarweb maplogs / intervention events of this period

MISSING / LIMITS: %s
""" % (os.path.basename(root), '; '.join(missing) or 'none known')
  open(os.path.join(root, 'README.txt'), 'w').write(readme)
  files = []
  for dp, dn, fn in os.walk(root):
    for f in sorted(fn):
      p = os.path.join(dp, f)
      rel = os.path.relpath(p, root)
      if rel in ('manifest.json',):
        continue
      d = None
      for k, v in DESCR.items():
        if rel == k or (k.endswith('/') and rel.startswith(k)):
          d = v
      if d is None and rel.startswith('events/event_'):
        d = ('anomaly recording: ' + ('window telemetry (full rate, gzip jsonl)' if rel.endswith('.gz') else 'summary / frame'), 'full rate in window')
      if d is None and ('maplogs/' in rel or 'interventions/' in rel):
        d = ('original radarweb log copy', '1 Hz maplog / 10 Hz intervention samples')
      tref = ('wall-clock epoch seconds in field t (original radarweb log)' if ('maplogs/' in rel or 'interventions/' in rel)
              else 'rt = CLOCK_BOOTTIME seconds' if rel.endswith(('.jsonl', '.jsonl.gz')) else 'n/a')
      files.append({'path': rel, 'bytes': os.path.getsize(p), 'what': (d or ('', ''))[0], 'sampling': (d or ('', ''))[1],
                    'timestamp_reference': tref, 'sha256': sha(p)})
  man = {'session': os.path.basename(root), 'created': time.strftime('%Y-%m-%d %H:%M:%S'),
         'timestamp_reference': 'rt = CLOCK_BOOTTIME seconds (cereal logMonoTime); wall = rt + metadata/clock.json:wall_minus_boot',
         'missing_or_limits': missing, 'original_logs_included': logs, 'files': files}
  json.dump(man, open(os.path.join(root, 'manifest.json'), 'w'), indent=1)
  zp = root + '.zip'
  tmp = zp + '.part'
  with zipfile.ZipFile(tmp, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
    for dp, dn, fn in os.walk(root):
      for f in sorted(fn):
        p = os.path.join(dp, f)
        z.write(p, os.path.join(os.path.basename(root), os.path.relpath(p, root)))
  with zipfile.ZipFile(tmp) as z:
    bad = z.testzip()
    names = len(z.namelist())
  if bad:
    print('ZIP INTEGRITY FAILED at', bad)
    return 4
  os.rename(tmp, zp)
  print('ZIP OK: %s (%d files, %.1f MB, sha256 %s)' % (zp, names, os.path.getsize(zp) / 1e6, sha(zp)))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv))
