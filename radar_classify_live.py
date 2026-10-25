#!/usr/bin/env python3
# radar_classify_live.py - the radarml classifier on a standalone Tesla Bosch MRRevo14F radar (Pi/laptop + CAN adapter),
# using SamSkjord/tesla-radar-driver for the keepalive and decoding. Same features/model as radarweb on the EON.
#   git clone https://github.com/SamSkjord/tesla-radar-driver && pip install python-can cantools
#   python3 radar_classify_live.py --driver ./tesla-radar-driver --channel can0 [--model radar_model.json] [--log tracks.jsonl]
# --log writes one row per track per 0.1 s in the labels.jsonl format (with 'c' empty): label them afterwards
# (or move real objects in front of the radar and pass --label car) and train with radarlearn_train.py.
#
# BENCH ONLY. tesla-radar-driver's keepalive invents the vehicle state: speed sweeps 30-120 km/h in a 60 s cycle and
# the steering wheel swings +-20 deg. The radar uses that for ego-motion, so with it every stationary object looks like
# it is approaching at up to 120 km/h and moving state / class / ground speed are nonsense. This script pins the
# simulated speed to --speed (default 0 = radar standing on a bench) and steering to 0. Never connect this to a car's
# CAN bus: there the real modules already talk to the radar, and these frames would conflict with them.
import sys, os, time, json, argparse

ap = argparse.ArgumentParser()
ap.add_argument('--driver', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tesla-radar-driver'), help='tesla-radar-driver checkout')
ap.add_argument('--channel', default='can0')
ap.add_argument('--interface', default='socketcan')
ap.add_argument('--model', default='radar_model.json')
ap.add_argument('--speed', type=float, default=0.0, help='vehicle speed (km/h) the radar is told; the radar is not moving, so 0')
ap.add_argument('--driver-sim', action='store_true', help="keep tesla-radar-driver's 30-120 km/h simulation (gs/class features become meaningless)")
ap.add_argument('--log', help='append every track (labels.jsonl format) to this file')
ap.add_argument('--label', default='', help='with --log: label every logged row with this (e.g. when only one known object is in view)')
A = ap.parse_args()
A.model = os.path.abspath(A.model); A.log = A.log and os.path.abspath(A.log)
sys.path.insert(0, A.driver); os.chdir(A.driver)     # the driver loads opendbc/*.dbc relative to the working directory
import tesla_radar_protocol as trp
from tesla_radar_driver import TeslaRadarDriver, TeslaRadarConfig
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import radarml

if not A.driver_sim:
    def _pinned_state(self):
        now = time.time(); self.actual_speed_kph = self.last_speed_kph = A.speed; self.vehicle_accel = 0.0
        self.accel_active = False; self.brake_active = A.speed < 0.5; self.last_state_update_ts = now
        self.gear_state = 0x04; self.turn_indicator = 0
    def _pinned_wheels(self):
        self.steering_angle_deg = self.steering_rate_dps = self.steering_torque_nm = 0.0
        return [self.actual_speed_kph] * 4
    trp.TeslaRadarProtocol._update_vehicle_state = _pinned_state
    trp.TeslaRadarProtocol._generate_wheel_speeds = _pinned_wheels
V_EGO = A.speed / 3.6

CLS = {0: 'unknown', 1: 'four_wheel', 2: 'two_wheel', 3: 'pedestrian', 4: 'construction'}
def to_obj(t):
    """TeslaRadarTrack -> the object dict radarweb builds (same keys, so the same features and model work)"""
    r = t.raw; gs = t.rel_speed + V_EGO
    return {'s': t.track_id, 'd': round(t.long_dist, 2), 'y': round(t.lat_dist, 2), 'vr': round(t.rel_speed, 2), 'gs': round(gs, 2),
            'pe': t.prob_exist, 'pc': r.get('ProbClass', 0.0), 'len': t.length, 'c': CLS.get(t.object_class, 'unknown'),
            'ls': t.lat_speed, 'la': t.long_accel, 'dz': t.dz, 'po': t.prob_obstacle, 'pno': r.get('ProbNonObstacle', 0.0),
            'dxs': r.get('dxSigma', 0), 'vxs': r.get('vxSigma', 0), 'axs': r.get('axSigma', 0), 'dys': r.get('dySigma', 0),
            'meas': t.meas, 'mst': t.moving_state}

model = radarml.Model(A.model) if os.path.exists(A.model) else None
if model is None: print('no model at %s: showing radar classes only (logging still works)' % A.model)
trk = radarml.TrackHist(); logf = open(A.log, 'a') if A.log else None
drv = TeslaRadarDriver(TeslaRadarConfig(channel=A.channel, interface=A.interface)); drv.start()
print('radar told %.0f km/h%s; Ctrl-C to stop' % (A.speed, ' (driver simulation)' if A.driver_sim else ''))
try:
    while True:
        time.sleep(0.1); now = time.time(); objs = []
        for t in drv.get_tracks().values():
            if not (t.valid or t.tracked) or t.long_dist <= 0: continue
            o = to_obj(t); trk.add(o['s'], now, o); h = trk.get(o['s'], now)
            o['ml'] = model.track_class(o['s'], radarml.features(o, h, V_EGO)) if model else None
            objs.append(o)
            if logf:
                logf.write(json.dumps(dict(o, c=A.label, radar_c=o['c'], radar_pc=o['pc'], t=round(now, 2), v=V_EGO, h=h, hf=trk.fields),
                                      separators=(',', ':')) + '\n')
        alive = set(o['s'] for o in objs); trk.drop_missing(alive)
        if model:
            for k in [k for k in model.smooth if k not in alive]: model.forget(k)
        ka = drv.keepalive_status() or {}
        line = ' '.join('%d:%.1fm/%+.1f %s%s' % (o['s'], o['d'], o['y'], o['c'][:4], ('>%s %.0f%%' % (o['ml'][0][:5], o['ml'][1] * 100)) if o['ml'] else '')
                        for o in sorted(objs, key=lambda o: o['d'])[:6])
        sys.stdout.write('\rradar=%s tracks=%2d  %s\033[K' % (ka.get('radar_status'), len(objs), line)); sys.stdout.flush()
        if logf: logf.flush()
except KeyboardInterrupt:
    pass
finally:
    drv.stop(); print()
