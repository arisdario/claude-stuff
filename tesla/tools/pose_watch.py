#!/usr/bin/env python3
"""pose_watch.py - READ-ONLY logger for the steering-wiggle investigation.

Records, at message rate, what the 0.8.3 model + camera pipeline believes and
what the car actually does, so we can tell "path oscillates" (camera/model)
from "path is stable but steering oscillates" (downstream control).

It only SUBSCRIBES to cereal sockets. It never publishes, never writes Params,
never imports control code.  Output: JSON lines in /data/posewatch/ (size capped).

  python3 pose_watch.py log [--dir /data/posewatch] [--max-mb 200]
  python3 pose_watch.py summary FILE.jsonl

Each line: {"t": monotonic_s, "s": service, ...fields}
Fields missing in this openpilot build are simply omitted (getattr-safe).
"""
import json
import math
import os
import sys
import time

SERVICES = ('model', 'cameraOdometry', 'sensorEvents', 'liveLocationKalman',
            'gpsLocationExternal', 'controlsState', 'liveParameters',
            'liveCalibration', 'carState', 'frame', 'pathPlan')


def _g(o, name, default=None):
    try:
        v = getattr(o, name)
    except Exception:
        return default
    return v


def _lst(v, n=None):
    try:
        out = [round(float(x), 5) for x in v]
    except Exception:
        return None
    return out[:n] if n else out


def extract(service, ev):
    """Pure function: cereal event -> small dict (or None). Testable without cereal."""
    d = {}
    if service == 'model':
        m = ev.model
        p = _g(m, 'path')
        d['path'] = _lst(_g(p, 'poly'))          # model path polynomial c3..c0
        d['pstd'] = round(float(_g(p, 'std', 0)), 4)
        for k, n in (('lane_l', 'leftLane'), ('lane_r', 'rightLane')):
            ln = _g(m, n)
            d[k] = _lst(_g(ln, 'poly'))
            d[k + '_p'] = round(float(_g(ln, 'prob', 0)), 3)
        d['fid'] = _g(m, 'frameId')
        d['drop'] = _g(m, 'frameDropPerc')
        d['eof'] = _g(m, 'timestampEof')
        d['exec'] = _g(m, 'modelExecutionTime')
    elif service == 'cameraOdometry':
        c = ev.cameraOdometry
        d['trans'] = _lst(_g(c, 'trans'))
        d['rot'] = _lst(_g(c, 'rot'))
        d['transStd'] = _lst(_g(c, 'transStd'))
        d['rotStd'] = _lst(_g(c, 'rotStd'))
        d['fid'] = _g(c, 'frameId')
    elif service == 'sensorEvents':
        out = []
        for s in ev.sensorEvents:
            gy = _g(s, 'gyro')
            if gy is not None:
                out.append(_lst(_g(gy, 'v')))
        if not out:
            return None
        d['gyro'] = out[-1]
    elif service == 'liveLocationKalman':
        k = ev.liveLocationKalman
        for key, n in (('yawrate', 'angularVelocityCalibrated'),
                       ('orient', 'orientationNED'), ('vel', 'velocityCalibrated')):
            d[key] = _lst(_g(_g(k, n), 'value'))
        d['ok'] = _g(k, 'inputsOK'), _g(k, 'sensorsOK'), _g(k, 'gpsOK')
    elif service == 'gpsLocationExternal':
        g = ev.gpsLocationExternal
        d.update(lat=_g(g, 'latitude'), lon=_g(g, 'longitude'),
                 brg=_g(g, 'bearing'), spd=_g(g, 'speed'), acc=_g(g, 'accuracy'))
    elif service == 'controlsState':
        c = ev.controlsState
        d.update(des=_g(c, 'angleSteersDes'), act=_g(c, 'angleSteers'),
                 en=_g(c, 'enabled'), v=_g(c, 'vEgo'),
                 sr=_g(c, 'steerRatio'), curv=_g(c, 'curvature'))
    elif service == 'liveParameters':
        p = ev.liveParameters
        d.update(sr=_g(p, 'steerRatio'), sf=_g(p, 'stiffnessFactor'),
                 ao=_g(p, 'angleOffsetAverage'), valid=_g(p, 'valid'))
    elif service == 'liveCalibration':
        c = ev.liveCalibration
        d['rpy'] = _lst(_g(c, 'rpyCalib'))
        d['status'] = _g(c, 'calStatus')
        d['perc'] = _g(c, 'calPerc')
    elif service == 'carState':
        c = ev.carState
        d.update(v=_g(c, 'vEgo'), ang=_g(c, 'steeringAngle'),
                 rate=_g(c, 'steeringRate'), tq=_g(c, 'steeringTorque'),
                 yaw=_g(c, 'yawRate'))
    elif service == 'frame':
        f = ev.frame
        d.update(fid=_g(f, 'frameId'), eof=_g(f, 'timestampEof'),
                 ft=_g(f, 'frameLength'))
    elif service == 'pathPlan':
        p = ev.pathPlan
        d.update(des=_g(p, 'angleSteers'), off=_g(p, 'angleOffset'),
                 cpoly=_lst(_g(p, 'cPoly')), valid=_g(p, 'paramsValid'))
    else:
        return None
    return d


class Writer:
    def __init__(self, directory, max_mb):
        os.makedirs(directory, exist_ok=True)
        self.path = os.path.join(directory, time.strftime('pose_%Y%m%d_%H%M%S.jsonl'))
        self.f = open(self.path, 'a')
        self.cap = max_mb * 1e6
        self.n = 0

    def write(self, rec):
        if self.f.tell() > self.cap:
            return False
        self.f.write(json.dumps(rec, separators=(',', ':')) + '\n')
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()
        return True


def run_log(directory, max_mb):
    import cereal.messaging as messaging           # only on the EON
    socks = {}
    for s in SERVICES:
        try:
            socks[s] = messaging.sub_sock(s, conflate=False)
        except Exception as e:
            print('skip', s, e)
    w = Writer(directory, max_mb)
    print('logging to', w.path)
    t0 = time.monotonic()
    while True:
        got = False
        for name, sock in socks.items():
            while True:
                msgs = messaging.drain_sock(sock, wait_for_one=False)
                if not msgs:
                    break
                got = True
                for ev in msgs:
                    try:
                        d = extract(name, ev)
                    except Exception:
                        d = None
                    if d:
                        d['t'] = round(time.monotonic() - t0, 4)
                        d['s'] = name
                        d['lt'] = round(_g(ev, 'logMonoTime', 0) / 1e9, 4)
                        if not w.write(d):
                            print('size cap reached, stopping'); return
        if not got:
            time.sleep(0.005)


def _load(path):
    rows = {}
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue
            rows.setdefault(r['s'], []).append(r)
    return rows


def _std(x):
    x = [v for v in x if v is not None]
    if len(x) < 2:
        return None
    m = sum(x) / len(x)
    return math.sqrt(sum((v - m) ** 2 for v in x) / len(x))


def summarize(path):
    R = _load(path)
    out = []
    mdl = R.get('model', [])
    if mdl:
        ids = [r['fid'] for r in mdl if r.get('fid') is not None]
        gaps = sum(max(0, b - a - 1) for a, b in zip(ids, ids[1:]))
        dt = [b['lt'] - a['lt'] for a, b in zip(mdl, mdl[1:])]
        out.append('model: %d msgs, skipped frameIds %d, dt mean %.1f ms max %.1f ms, '
                   'frameDropPerc max %s' % (len(mdl), gaps, 1e3 * sum(dt) / max(1, len(dt)),
                                             1e3 * max(dt or [0]), max((r.get('drop') or 0) for r in mdl)))
        c0 = [r['path'][3] for r in mdl if r.get('path') and len(r['path']) == 4]
        out.append('model path c0 (offset at car) std %.3f m' % (_std(c0) or 0))
        # spectral power of path offset above 1.5 Hz relative to total (needs ~uniform rate)
        out.append(_band(c0, mdl))
    od = R.get('cameraOdometry', [])
    if od:
        for i, n in enumerate(('roll', 'pitch', 'yaw')):
            v = [r['rot'][i] for r in od if r.get('rot')]
            out.append('cameraOdometry rot %s std %.4f rad/s' % (n, _std(v) or 0))
    gy = R.get('liveLocationKalman', [])
    cs = R.get('controlsState', [])
    if cs:
        e = [r['des'] - r['act'] for r in cs if r.get('des') is not None and r.get('act') is not None and r.get('en')]
        out.append('engaged angle tracking error (des-act) std %.2f deg' % (_std(e) or 0))
        out.append('steering des std %.2f deg (engaged only)' % (_std([r['des'] for r in cs if r.get('en') and r.get('des') is not None]) or 0))
    lp = R.get('liveParameters', [])
    if lp:
        out.append('liveParameters last: sr=%s sf=%s valid=%s' % (lp[-1].get('sr'), lp[-1].get('sf'), lp[-1].get('valid')))
    lc = R.get('liveCalibration', [])
    if lc:
        out.append('liveCalibration last rpy=%s status=%s perc=%s' % (lc[-1].get('rpy'), lc[-1].get('status'), lc[-1].get('perc')))
    print('\n'.join(out))


def _band(x, rows, f0=1.5):
    n = len(x)
    if n < 64:
        return 'spectrum: not enough samples'
    T = rows[-1]['lt'] - rows[0]['lt']
    fs = (n - 1) / T if T > 0 else 0
    m = sum(x) / n
    x = [v - m for v in x]
    tot = hi = 0.0
    for k in range(1, n // 2):
        f = k * fs / n
        re = sum(v * math.cos(2 * math.pi * k * i / n) for i, v in enumerate(x))
        im = sum(v * math.sin(2 * math.pi * k * i / n) for i, v in enumerate(x))
        p = re * re + im * im
        tot += p
        if f > f0:
            hi += p
    return 'path-offset power above %.1f Hz: %.1f%% (fs %.1f Hz)' % (f0, 100 * hi / tot if tot else 0, fs)


if __name__ == '__main__':
    a = sys.argv[1:]
    if not a or a[0] not in ('log', 'summary'):
        print(__doc__); sys.exit(1)
    if a[0] == 'summary':
        summarize(a[1])
    else:
        d = a[a.index('--dir') + 1] if '--dir' in a else '/data/posewatch'
        mb = float(a[a.index('--max-mb') + 1]) if '--max-mb' in a else 200
        run_log(d, mb)
