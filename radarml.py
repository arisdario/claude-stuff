#!/usr/bin/env python3
# radarml.py - object classifier shared by radarweb.py / radar_classify_live.py (run it) and radarlearn_train.py (trains it).
# Pure python, no numpy/sklearn/cantools: the EON only needs this file plus the exported model JSON.
#   decode_ab(): every signal of a Bosch MRRevo14F object frame pair (0x310+3k / 0x311+3k), bit layout from
#                tesla-radar-driver's opendbc/tesla_radar.dbc - adds lat speed, long accel, obstacle probabilities,
#                height (dZ) and the sigma fields that radarlog's decoders don't expose
#   features():  one fixed-length vector per track. Method from INFRA3DRC-Radar-Classification: mean/std/min/max/
#                median/span of every signal. INFRA3DRC takes those over the radar points of one object; this radar
#                outputs one point per object, so here they're taken over the last HIST_S seconds of the track instead
#   TrackHist:   per-slot history ring that radarweb fills every 0.1 s and attaches to /learn labels
#   Model:       loads the exported model (logistic regression or a tree forest) and predicts, smoothed per track
import json, math

HIST_S = 2.0         # seconds of track history the features look at (and that /learn stores with each label)
JUMP_D, JUMP_Y = 8.0, 3.0   # a slot whose position jumps more than this between frames is a new object: history restarts
RADAR_CLASSES = ('four_wheel', 'two_wheel', 'pedestrian', 'construction', 'unknown')

# /learn labels -> training targets. Fine labels with only a handful of examples each are merged so every
# class gets enough rows; edit this if you collect lots of one fine class (e.g. split truck from car).
TARGET = {'car': 'four_wheel', 'truck': 'four_wheel', 'bus': 'four_wheel', 'two_wheel': 'two_wheel',
          'pedestrian': 'pedestrian', 'dog': 'animal', 'animal': 'animal', 'construction': 'construction',
          'obstacle': 'construction', 'separator': 'static', 'wall': 'static', 'pole': 'static', 'sign': 'static',
          'not_real': 'not_real', 'four_wheel': 'four_wheel', 'static': 'static'}
# training targets -> what the road/top views can draw (they only have icons for RADAR_CLASSES)
DISPLAY = {'four_wheel': 'four_wheel', 'two_wheel': 'two_wheel', 'pedestrian': 'pedestrian',
           'construction': 'construction', 'animal': 'unknown', 'static': 'unknown', 'not_real': 'unknown'}

# tesla_radar.dbc, BO_ 784 / 785 (all little-endian unsigned): name -> (start bit, length, scale, offset)
SIG_A = {'LongDist': (0, 12, 0.0625, 0), 'LongSpeed': (12, 12, 0.0625, -128), 'LatDist': (24, 11, 0.125, -128),
         'ProbExist': (35, 5, 3.125, 0), 'LongAccel': (40, 10, 0.03125, -16), 'ProbObstacle': (50, 5, 3.125, 0),
         'Valid': (55, 1, 1, 0), 'ProbNonObstacle': (56, 5, 3.125, 0), 'Meas': (61, 1, 1, 0), 'Tracked': (62, 1, 1, 0),
         'Index': (63, 1, 1, 0)}
SIG_B = {'LatSpeed': (0, 10, 0.125, -64), 'Length': (10, 6, 0.125, 0), 'dZ': (16, 6, 0.25, -5), 'MovingState': (22, 2, 1, 0),
         'dxSigma': (24, 6, 1, 0), 'vxSigma': (30, 6, 1, 0), 'axSigma': (36, 6, 1, 0), 'dySigma': (42, 6, 1, 0),
         'ProbClass': (48, 5, 3.125, 0), 'Class': (53, 3, 1, 0), 'dxRearEndLoss': (56, 6, 1, 0), 'Index2': (63, 1, 1, 0)}
def _dec(data, sigs):
    v = int.from_bytes(bytes(data), 'little')
    return {n: ((v >> st) & ((1 << ln) - 1)) * sc + of for n, (st, ln, sc, of) in sigs.items()}
def decode_pair(a_bytes, b_bytes):
    """one A/B object frame pair -> (a, b, extras): a/b with radarlog.decode_a/decode_b's key names (radarweb checks
    they agree on your radar before using them), extras = decode_ab()'s fields; every frame is decoded only once"""
    A = _dec(a_bytes, SIG_A); B = _dec(b_bytes, SIG_B)
    a = {'long_dist': A['LongDist'], 'lat_dist': A['LatDist'], 'long_speed': A['LongSpeed'], 'prob_exist': A['ProbExist'],
         'valid': A['Valid'], 'tracked': A['Tracked'], 'index': A['Index']}
    b = {'cls': B['Class'], 'prob_class': B['ProbClass'], 'length': B['Length'], 'moving': B['MovingState'], 'index2': B['Index2']}
    return a, b, _extras(A, B)
def decode_ab(a_bytes, b_bytes):
    """extra per-object fields (short keys, as stored in the track history) from one A/B frame pair"""
    return _extras(_dec(a_bytes, SIG_A), _dec(b_bytes, SIG_B))
def _extras(a, b):
    return {'ls': b['LatSpeed'], 'la': a['LongAccel'], 'dz': b['dZ'], 'po': a['ProbObstacle'], 'pno': a['ProbNonObstacle'],
            'dxs': b['dxSigma'], 'vxs': b['vxSigma'], 'axs': b['axSigma'], 'dys': b['dySigma'], 'rel': b['dxRearEndLoss'],
            'meas': a['Meas'], 'mst': b['MovingState']}

# history row = [age_s] + SIGNALS (missing signal -> 0, like INFRA3DRC's fillna(0)). The labels file stores the
# field names next to the rows ('hf'), so rows recorded with an older radarweb still line up.
# v2 adds, after Ham et al., Sensors 2025 (object-level radar classification): vy = sideways ground speed (radarweb's
# rotation-compensated estimate), ga = ground acceleration (radar's relative + your own), axs = the tracker's
# acceleration uncertainty - plus 2-D ground speed and correlations, derived below.
SIGNALS = ['d', 'y', 'vr', 'gs', 'ls', 'la', 'len', 'dz', 'pe', 'po', 'pno', 'pc', 'dxs', 'vxs', 'dys', 'axs', 'vy', 'ga']
FEATS_VERSION = 2
HIST_FIELDS = ['age'] + SIGNALS
STATS = ('mean', 'std', 'min', 'max', 'median', 'span')
FEATS = ['v_ego', 'age', 'n', 'abs_y', 'abs_gs', 'y_rate', 'd_rate_err', 'meas',
         'gxy_mean', 'gxy_std', 'corr_d_gs', 'corr_y_vy', 'corr_gs_vy'] + \
        ['%s_now' % s for s in SIGNALS] + ['%s_%s' % (s, st) for s in SIGNALS for st in STATS] + \
        ['rc_' + c for c in RADAR_CLASSES] + ['ms_%d' % i for i in range(4)]


def _stats(a):
    n = len(a); m = sum(a) / n; srt = sorted(a)
    med = srt[n // 2] if n % 2 else (srt[n // 2 - 1] + srt[n // 2]) / 2
    return {'mean': m, 'std': math.sqrt(sum((x - m) ** 2 for x in a) / n), 'min': srt[0], 'max': srt[-1], 'median': med, 'span': srt[-1] - srt[0]}
def _corr(a, b):
    n = len(a)
    if n < 3: return 0.0
    ma = sum(a) / n; mb = sum(b) / n
    va = sum((x - ma) ** 2 for x in a); vb = sum((x - mb) ** 2 for x in b)
    return sum((x - ma) * (y - mb) for x, y in zip(a, b)) / math.sqrt(va * vb) if va > 1e-9 and vb > 1e-9 else 0.0
def _slope(ts, xs):
    if len(ts) < 3: return 0.0
    mt = sum(ts) / len(ts); mx = sum(xs) / len(xs); den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (x - mx) for t, x in zip(ts, xs)) / den if den > 1e-6 else 0.0


def features(o, hist=None, v_ego=None, hist_fields=None):
    """o: object dict as radarweb builds it ({d,y,vr,gs,len,pe,pc,c,ms?,...}) or a labels.jsonl row ({..., radar_c, radar_pc, h, hf, v}).
    hist: history rows (oldest first, age_s <= 0) in hist_fields order (default HIST_FIELDS). Missing -> a brand-new track."""
    if hist is None and o.get('h'):
        hist = o['h']; hist_fields = o.get('hf') or ['age', 'd', 'y', 'vr', 'gs', 'len', 'pe', 'pc']   # first labels.jsonl format had no 'hf'
    cur = dict((s, float(o.get(s, 0) or 0)) for s in SIGNALS)
    cur['pc'] = float(o.get('radar_pc', o.get('pc', 0)) or 0)
    if not hist:
        hist = [[0.0] + [cur[s] for s in SIGNALS]]; hist_fields = HIST_FIELDS
    idx = dict((f, i) for i, f in enumerate(hist_fields or HIST_FIELDS))
    col = lambda s: [float(r[idx[s]]) for r in hist] if s in idx else [cur[s]] * len(hist)
    ts = col('age') if 'age' in idx else [0.0] * len(hist)
    last = dict((s, col(s)[-1]) for s in SIGNALS)
    if v_ego is None: v_ego = float(o.get('v', last['gs'] - last['vr']))
    rc = o.get('radar_c', o.get('c', 'unknown')); mst = int(o.get('mst', -1) if o.get('mst') is not None else -1)
    ds, ys = col('d'), col('y'); gss, vys = col('gs'), col('vy')
    gxy = [math.sqrt(a * a + b * b) for a, b in zip(gss, vys)]; gm = sum(gxy) / len(gxy)
    f = {'gxy_mean': gm, 'gxy_std': math.sqrt(sum((x - gm) ** 2 for x in gxy) / len(gxy)),       # 2-D speed over ground
         'corr_d_gs': _corr(ds, gss), 'corr_y_vy': _corr(ys, vys), 'corr_gs_vy': _corr(gss, vys)}
    f.update({'v_ego': v_ego, 'age': -ts[0], 'n': float(len(hist)), 'abs_y': abs(last['y']), 'abs_gs': abs(last['gs']),
         'y_rate': _slope(ts, ys), 'meas': float(o.get('meas', 0) or 0),
         # range rate the radar reports vs how the distance actually changed: real objects agree, multipath ghosts often don't
         'd_rate_err': abs(_slope(ts, ds) - sum(col('vr')) / len(hist)) if len(hist) >= 3 else 0.0})
    for s in SIGNALS:
        f[s + '_now'] = last[s]
        for st, v in _stats(col(s)).items(): f['%s_%s' % (s, st)] = v
    for c in RADAR_CLASSES: f['rc_' + c] = 1.0 if rc == c else 0.0
    for i in range(4): f['ms_%d' % i] = 1.0 if mst == i else 0.0
    return [f[k] for k in FEATS]


# CROSSING: the radar measures speed along its line of sight, so a pedestrian/cyclist crossing in front of you looks
# like a stationary object to it and keeps class 'unknown' (seen in the 25 Oct logs: 113 sideways-moving tracks, all
# 'unknown 25%'). Their sideways motion is visible in the track position though. lateral_speed() estimates it from
# the last HIST_S of y positions, cross-checked against the radar's own LatSpeed. When you turn, everything ahead sweeps
# sideways by (your yaw rate x distance): scene_yaw() measures that rotation from the stationary tracks themselves
# (median, so the few real movers don't bias it) and ground_vy() subtracts it. Steering angle is only the fallback
# when there are too few stationary tracks; it underestimates rotation at walking speed in tight turns.
WHEELBASE, STEER_RATIO = 2.96, 15.0
CROSS = ((0.6, 1.4, 'pedestrian'), (1.4, 7.0, 'two_wheel'), (7.0, 20.0, 'four_wheel'))  # sideways ground speed (m/s) -> class: walking 2-5 km/h (4 km/h + noise margin), cycling 5-25 km/h, car
def lateral_speed(hist, ls):
    """sideways speed (m/s) of a track relative to the car, or None (too little history, position and LatSpeed disagree)"""
    if not hist or len(hist) < 6 or -hist[0][0] < 0.8: return None
    vy = _slope([r[0] for r in hist], [r[2] for r in hist])
    if ls is not None and 0.3 < abs(ls) < 15 and abs(vy) > 0.3 and (ls > 0) != (vy > 0): return None   # LatSpeed -64 = no data
    return vy
def scene_yaw(objs):
    """your rotation (rad/s, in the radar's own sign convention) from how the stationary tracks sweep sideways, or None"""
    r = sorted(-o['vy_raw'] / o['d'] for o in objs if o.get('vy_raw') is not None and abs(o['gs']) < 1.5 and o['d'] > 3)
    return r[len(r) // 2] if len(r) >= 3 else None
def ground_vy(vy_raw, yaw, d, v_ego, steer_deg):
    """sideways ground speed: relative speed minus the sweep caused by your own rotation; None while it can't be told apart"""
    if vy_raw is None or abs(steer_deg or 0.0) > 90: return None    # manoeuvring (parking, U-turn): no way to separate walkers
    if yaw is not None:
        return None if abs(yaw) > 0.06 else vy_raw + yaw * d     # turning: too much sweep to separate a walker from it
    est = abs(v_ego * math.tan(math.radians((steer_deg or 0.0) / STEER_RATIO)) / WHEELBASE)
    return None if est * max(d, 1.0) > 0.3 else vy_raw
def cross_class(vy, gs, d):
    """class implied by crossing in front of you, or None"""
    if vy is None or abs(gs) >= 3.0 or d >= 40: return None
    for lo, hi, c in CROSS:
        if lo <= abs(vy) < hi: return c
    return None


class TrackHist:
    """Per radar slot: the last HIST_S seconds of [t] + SIGNALS. Restarts when the slot is reused."""
    fields = HIST_FIELDS
    def __init__(self): self.h = {}
    def add(self, k, now, o):
        r = [now] + [float(o.get(s, 0) or 0) for s in SIGNALS]
        h = self.h.get(k)
        if h and (now - h[-1][0] > 0.5 or abs(r[1] - h[-1][1]) > JUMP_D or abs(r[2] - h[-1][2]) > JUMP_Y): h = None
        if h is None: h = self.h[k] = []
        h.append(r)
        while h and now - h[0][0] > HIST_S: h.pop(0)
    def get(self, k, now, rnd=True):
        """history in the features() format (times relative to now), or None. rnd=False skips rounding (live use, faster);
        rounded copies are only needed when the history is saved with a label"""
        h = self.h.get(k)
        if not h or now - h[-1][0] > 0.5: return None
        if not rnd: return [[r[0] - now] + r[1:] for r in h]
        return [[round(r[0] - now, 2)] + [round(x, 3) for x in r[1:]] for r in h]
    def set_last(self, k, field, value):
        """fill in a value computed after add() (radarweb's sideways ground speed needs all objects of the tick first)"""
        h = self.h.get(k)
        if h: h[-1][HIST_FIELDS.index(field)] = float(value or 0.0)
    def drop_missing(self, alive):
        for k in [k for k in self.h if k not in alive]: del self.h[k]


class AutoLabeler:
    """Objects label themselves: follows every track over its whole life and, when it ends, gives it a label only
    if its behaviour clearly fits one (rules below). Snapshots taken during its life (features as they were at that
    moment) all get that label, so the model learns to recognise a vehicle before it moves, a pole before you pass it.
    Rows go to the auto-labels file with 'auto': true and a track id 'tid' (training keeps one track's snapshots on
    the same side of the cross-validation split). Manual /learn labels weigh more in training (MANUAL_WEIGHT)."""
    SNAP_AT = (0.5, 1.5, 3.0, 6.0)   # take a snapshot when the track is this old (s); at most this many per track
    PER_CLASS_S = 15.0               # at most one labelled track per class per this many seconds (keeps the file small, classes balanced)
    def __init__(self): self.tr = {}; self.last = {}; self.n = 0; self.counts = {}
    def update(self, k, now, o, v_ego, hist):
        t = self.tr.get(k)
        if t and (now - t['last'] > 0.5 or abs(o['d'] - t['d']) > JUMP_D or abs(o['y'] - t['y']) > JUMP_Y):
            out = self.finish(k); t = None
        else: out = []
        if t is None:
            self.n += 1
            t = self.tr[k] = {'id': '%x-%d' % (int(now), self.n), 't0': now, 'last': now, 'd': o['d'], 'y': o['y'], 'f': 0,
                              'move_s': 0.0, 'still_f': 0, 'gs_max': 0.0, 'ego_m': 0.0, 'pe_max': 0.0, 'rc': {}, 'derr': 0.0, 'snaps': [],
                              'cross': {}, 'side_f': 0}
        dt = min(0.2, now - t['last']) if t['f'] else 0.0
        gs = abs(o['gs']); t['f'] += 1; t['last'] = now; t['d'] = o['d']; t['y'] = o['y']
        if gs >= 6.0: t['move_s'] += dt
        vy = o.get('vy'); xc = cross_class(vy, o['gs'], o['d'])
        if xc: t['cross'][xc] = t['cross'].get(xc, 0.0) + dt
        if vy is not None and abs(vy) >= 0.3: t['side_f'] += 1
        if gs < 0.5 and (vy is None or abs(vy) < 0.3): t['still_f'] += 1
        t['gs_max'] = max(t['gs_max'], gs); t['ego_m'] += abs(v_ego) * dt; t['pe_max'] = max(t['pe_max'], o.get('pe', 0))
        if o.get('pc', 0) >= 80: t['rc'][o.get('rc', o.get('c'))] = t['rc'].get(o.get('rc', o.get('c')), 0) + 1
        age = now - t['t0']
        if len(t['snaps']) < len(self.SNAP_AT) and age >= self.SNAP_AT[len(t['snaps'])] and hist:
            x = features(o, hist, v_ego); t['derr'] = max(t['derr'], x[FEATS.index('d_rate_err')])
            row = dict((key, o.get(key)) for key in ('s', 'd', 'y', 'vr', 'gs', 'len', 'pe', 'mst', 'meas') + tuple(SIGNALS) if key in o)
            row.update(radar_c=o.get('rc', o.get('c')), radar_pc=o.get('pc'), t=round(now, 2), v=round(v_ego, 2),
                       h=[[round(x, 3) for x in r] for r in hist[::2]], hf=HIST_FIELDS)
            t['snaps'].append(row)
        return out
    def rule(self, t):
        life = t['last'] - t['t0']; f = t['f']
        top = max(t['rc'].items(), key=lambda kv: kv[1]) if t['rc'] else (None, 0)
        sure = top[0] if top[0] in ('four_wheel', 'two_wheel', 'pedestrian', 'construction') and top[1] >= 0.7 * f and life >= 1.0 else None
        if life < 0.8 and t['pe_max'] < 70: return 'not_real'                       # flickered in and out, radar itself unsure
        if t['derr'] > 4.0 and life < 3.0: return 'not_real'                         # range rate and distance change disagree: multipath ghost
        if t['move_s'] >= 1.0:                                                      # moved over the ground at 6+ m/s (22 km/h) for 1 s
            return 'two_wheel' if sure == 'two_wheel' and t['gs_max'] < 12.0 else 'four_wheel'
        xc = max(t['cross'].items(), key=lambda kv: kv[1]) if t['cross'] else (None, 0)
        if xc[1] >= 1.0: return xc[0]                                               # crossed in front of you for 1 s: walking / cycling / car pace
        if sure: return sure                                                        # radar held one class at 80%+ for most of the track
        if life >= 3.0 and t['still_f'] >= 0.95 * f and t['side_f'] == 0 and t['ego_m'] >= 20.0: return 'static'   # never moved (along or across) while you drove 20 m past it
        return None
    def finish(self, k):
        t = self.tr.pop(k, None)
        if not t or not t['snaps']: return []
        lab = self.rule(t)
        if lab is None or t['last'] - self.last.get(lab, 0) < self.PER_CLASS_S: return []
        self.last[lab] = t['last']; self.counts[lab] = self.counts.get(lab, 0) + 1
        return [dict(r, c=lab, auto=True, tid=t['id']) for r in t['snaps']]
    def end_missing(self, alive):
        out = []
        for k in [k for k in self.tr if k not in alive]: out += self.finish(k)
        return out


class Model:
    """Exported model ({'kind': 'logreg' | 'forest', 'feats', 'classes', ...}) + per-track probability smoothing."""
    def __init__(self, path):
        m = json.load(open(path))
        if m['feats'] != FEATS: raise ValueError('model was trained on other features - retrain it with this radarml.py')
        self.m = m; self.classes = m['classes']; self.smooth = {}
        self.min_p = float(m.get('min_p', 0.6))
    def proba(self, x):
        m = self.m
        if m['kind'] == 'mlp': return _mlp_proba(m, x)
        if m['kind'] == 'logreg':
            z = [(xi - mu) / sc for xi, mu, sc in zip(x, m['mean'], m['scale'])]
            s = [b + sum(w * zi for w, zi in zip(row, z)) for row, b in zip(m['coef'], m['intercept'])]
            mx = max(s); e = [math.exp(v - mx) for v in s]; t = sum(e)
            return [v / t for v in e]
        p = [0.0] * len(self.classes)
        for tr in m['trees']:          # arrays from sklearn's tree_: left, right, feature, threshold, leaf class probabilities
            i = 0; L = tr['l']; R = tr['r']; F = tr['f']; T = tr['t']
            while L[i] != -1: i = L[i] if x[F[i]] <= T[i] else R[i]
            for j, v in enumerate(tr['v'][i]): p[j] += v
        n = float(len(m['trees'])); return [v / n for v in p]
    def track_class(self, k, x, alpha=0.35):
        """smoothed (label, probability) for slot k: an exponential average stops the class flickering frame to frame"""
        p = self.proba(x); s = self.smooth.get(k)
        s = p if s is None else [a + alpha * (b - a) for a, b in zip(s, p)]
        self.smooth[k] = s; j = max(range(len(s)), key=s.__getitem__)
        return self.classes[j], s[j]
    def forget(self, k): self.smooth.pop(k, None)


# ---------------------------------------------------------------------------------------------------------------
# AUTOTRAIN: pure-python trainer so the EON can retrain by itself (no numpy/sklearn there). radarweb runs it as
#   nice -n 19 python3 radarml.py --autotrain /data/canraw/labels.jsonl /data/radar_model.json
# only while parked, and kills it when the car moves. Extremely randomized trees (random split thresholds, so no
# sorting: cheap in plain python), class-balanced, scored with cross-validation that never splits one drive (or
# one track) across train and test. The new model is installed only if it beats both what radarweb shows without a
# model and the model already installed; the old one is kept as <model>.prev. radarlearn_train.py on a laptop
# (scikit-learn, nested grid search) is still the better trainer when you have a computer at hand.
import os, random, time

AUTO_MIN_ROWS = 60       # don't train on fewer labelled rows than this
AUTO_MIN_CLASS = 5       # targets with fewer rows are left out (the model can't learn them from 2 examples)
AUTO_MARGIN = 0.05       # cross-validated macro-F1 must beat "no model" by this much
MANUAL_WEIGHT = 3.0      # a label you tapped counts as much as this many automatic ones


def _gini(cnt):
    t = sum(cnt); return 1.0 - sum((c / t) ** 2 for c in cnt) if t > 0 else 0.0

def _tree(X, y, w, idx, K, rnd, max_depth, min_leaf, n_feat):
    L, R, F, T, V = [], [], [], [], []
    def leaf(cnt):
        t = sum(cnt) or 1.0; L.append(-1); R.append(-1); F.append(-2); T.append(0.0); V.append([round(c / t, 4) for c in cnt])
    def grow(idx, depth):
        cnt = [0.0] * K
        for i in idx: cnt[y[i]] += w[i]
        if depth >= max_depth or len(idx) < 2 * min_leaf or max(cnt) >= sum(cnt) - 1e-9: leaf(cnt); return len(L) - 1
        best = None; base = _gini(cnt) * sum(cnt)
        for f in rnd.sample(range(len(X[0])), n_feat):
            vals = [X[i][f] for i in idx]; lo, hi = min(vals), max(vals)
            if hi - lo < 1e-9: continue
            thr = rnd.uniform(lo, hi); cl = [0.0] * K; cr = [0.0] * K; nl = 0
            for i, v in zip(idx, vals):
                if v <= thr: cl[y[i]] += w[i]; nl += 1
                else: cr[y[i]] += w[i]
            if nl < min_leaf or len(idx) - nl < min_leaf: continue
            gain = base - _gini(cl) * sum(cl) - _gini(cr) * sum(cr)
            if best is None or gain > best[0]: best = (gain, f, thr)
        if best is None: leaf(cnt); return len(L) - 1
        _, f, thr = best; me = len(L); L.append(0); R.append(0); F.append(f); T.append(round(thr, 5)); V.append(None)
        li = [i for i in idx if X[i][f] <= thr]; ri = [i for i in idx if X[i][f] > thr]
        L[me] = grow(li, depth + 1); R[me] = grow(ri, depth + 1)
        return me
    grow(idx, 0)
    return {'l': L, 'r': R, 'f': F, 't': T, 'v': [v if v is not None else [] for v in V]}

def train_forest(X, y, K, idx=None, n_trees=40, max_depth=8, min_leaf=2, seed=0, sw=None):
    idx = list(range(len(X))) if idx is None else list(idx)
    sw = sw or [1.0] * len(y); n_c = [0.0] * K
    for i in idx: n_c[y[i]] += sw[i]
    w = [sw[i] * sum(n_c) / (K * n_c[c]) if n_c[c] else 0.0 for i, c in enumerate(y)]     # class_weight='balanced' x row weight
    rnd = random.Random(seed); n_feat = max(1, int(math.sqrt(len(X[0]))))
    return [_tree(X, y, w, idx, K, rnd, max_depth, min_leaf, n_feat) for _ in range(n_trees)]

def _predict(trees, x):
    p = None
    for tr in trees:
        i = 0
        while tr['l'][i] != -1: i = tr['l'][i] if x[tr['f'][i]] <= tr['t'][i] else tr['r'][i]
        p = list(tr['v'][i]) if p is None else [a + b for a, b in zip(p, tr['v'][i])]
    return max(range(len(p)), key=p.__getitem__)

def macro_f1(true, pred, K):
    f = []
    for c in range(K):
        tp = sum(1 for a, b in zip(true, pred) if a == c and b == c)
        fp = sum(1 for a, b in zip(true, pred) if a != c and b == c); fn = sum(1 for a, b in zip(true, pred) if a == c and b != c)
        if tp + fn: f.append(2.0 * tp / (2 * tp + fp + fn))
    return sum(f) / len(f) if f else 0.0

def load_labels(paths):
    """label files (manual /learn + automatic) -> (X, y, classes, groups, shown, weights), or raise ValueError with the reason"""
    rows = []
    for path in paths:
        if not os.path.exists(path): continue
        for ln in open(path):
            try: r = json.loads(ln)
            except ValueError: continue
            if r.get('c') in TARGET and 'd' in r: rows.append(r)
    rows.sort(key=lambda r: r.get('t', 0))
    cnt = {}
    for r in rows: cnt[TARGET[r['c']]] = cnt.get(TARGET[r['c']], 0) + 1
    rows = [r for r in rows if cnt[TARGET[r['c']]] >= AUTO_MIN_CLASS]
    classes = sorted(set(TARGET[r['c']] for r in rows))
    if len(rows) < AUTO_MIN_ROWS: raise ValueError('%d usable labels, need %d' % (len(rows), AUTO_MIN_ROWS))
    if len(classes) < 2: raise ValueError('need 2+ label types with %d+ labels each' % AUTO_MIN_CLASS)
    ci = dict((c, i) for i, c in enumerate(classes))
    # groups: a new drive after 5 min without labels; a new track when the slot changes or 15 s pass
    drive, track = [], []; d = k = 0; pt = None; ps = None
    for r in rows:
        t = r.get('t', 0)
        if pt is not None and t - pt > 300: d += 1
        if pt is None or t - pt > 15 or r.get('s') != ps: k += 1
        drive.append(d); track.append(r.get('tid') or k); pt = t; ps = r.get('s')
    groups = drive if len(set(drive)) >= 5 else track
    X = [features(r) for r in rows]; y = [ci[TARGET[r['c']]] for r in rows]
    shown = [ci.get(DISPLAY.get(r.get('radar_c'), 'unknown'), -1) for r in rows]
    sw = [1.0 if r.get('auto') else MANUAL_WEIGHT for r in rows]
    return X, y, classes, groups, shown, sw

def _folds(groups, folds=5):
    """group k-fold: list of (train idx, test idx); one drive (or track) never on both sides"""
    ug = sorted(set(groups), key=str); folds = min(folds, len(ug))
    if folds < 2: return []
    order = list(ug); random.Random(1).shuffle(order); fold_of = dict((g, i % folds) for i, g in enumerate(order))
    return [([i for i, g in enumerate(groups) if fold_of[g] != fo], [i for i, g in enumerate(groups) if fold_of[g] == fo]) for fo in range(folds)]

def cv_score(X, y, K, groups, folds=5, sw=None, **kw):
    """macro-F1 of the plain-python extra-trees over group k-fold (fallback trainer when numpy is missing)"""
    pred = [0] * len(y); fl = _folds(groups, folds)
    if not fl: return None
    for tr, te in fl:
        if len(set(y[i] for i in tr)) < 2: continue
        trees = train_forest(X, y, K, tr, sw=sw, **kw)
        for i in te: pred[i] = _predict(trees, X[i])
    return macro_f1(y, pred, K)

def metrics(true, pred, classes):
    """the numbers INFRA3DRC reports (accuracy, balanced accuracy, macro-F1, weighted-F1) + per-class F1 + confusion matrix"""
    K = len(classes); cm = [[0] * K for _ in range(K)]
    for a, b in zip(true, pred): cm[a][b] += 1
    f1, rec, sup = [], [], []
    for c in range(K):
        tp = cm[c][c]; fn = sum(cm[c]) - tp; fp = sum(cm[r][c] for r in range(K)) - tp; n = sum(cm[c])
        f1.append(2.0 * tp / (2 * tp + fp + fn) if tp + fp + fn else 0.0); rec.append(tp / float(n) if n else 0.0); sup.append(n)
    tot = float(sum(sup)) or 1.0; present = [c for c in range(K) if sup[c]]
    return {'accuracy': round(sum(cm[c][c] for c in range(K)) / tot, 3), 'balanced_accuracy': round(sum(rec[c] for c in present) / max(1, len(present)), 3),
            'macro_f1': round(sum(f1[c] for c in present) / max(1, len(present)), 3), 'weighted_f1': round(sum(f1[c] * sup[c] for c in range(K)) / tot, 3),
            'f1': dict((classes[c], round(f1[c], 3)) for c in range(K)), 'confusion': cm}


# MODEL V2 TRAINERS (numpy, which openpilot ships on the EON; inference stays plain python in Model.proba)
#   rf : Random Forest as in INFRA3DRC-Radar-Classification - bootstrap sample per tree, best split over sqrt(features)
#        candidates per node (gini), balanced class weights. 100 trees: their 500 would cost too much CPU per object.
#   mlp: the 2 hidden layers x 10 ReLU network of Ham et al. (Sensors 2025, grid-search model): softmax output,
#        class-weighted cross-entropy, RMSprop lr 0.01, batch 64, up to 50 epochs, early stopping on held-out drives.
#        Class weights instead of their SMOTE (no synthetic samples).
RF_TREES, RF_CV_TREES, RF_DEPTH, RF_LEAF = 100, 40, 10, 2

def _rf_tree(np, X, Y, w, ids, rs, max_depth, min_leaf, n_feat):
    L, R, F, T, V = [], [], [], [], []
    nf = X.shape[1]
    def leaf(W):
        t = W.sum() or 1.0; L.append(-1); R.append(-1); F.append(-2); T.append(0.0); V.append([round(float(v) / t, 4) for v in W])
        return len(L) - 1
    def grow(ids, depth):
        Yw = Y[ids] * w[ids, None]; W = Yw.sum(0); tot = W.sum(); n = len(ids)
        if depth >= max_depth or n < 2 * min_leaf or W.max() >= tot - 1e-9: return leaf(W)
        best = None; pos = np.arange(1, n)
        okpos = (pos >= min_leaf) & (pos <= n - min_leaf)
        for f in rs.choice(nf, n_feat, replace=False):
            xv = X[ids, f]; o = np.argsort(xv, kind='mergesort'); xs = xv[o]
            cl = np.cumsum(Yw[o], axis=0)[:-1]; cr = W - cl
            lt = cl.sum(1); rt = cr.sum(1)
            ok = okpos & (xs[1:] > xs[:-1])
            if not ok.any(): continue
            imp = (lt - (cl * cl).sum(1) / np.maximum(lt, 1e-12)) + (rt - (cr * cr).sum(1) / np.maximum(rt, 1e-12))
            imp[~ok] = np.inf; j = int(np.argmin(imp))
            if best is None or imp[j] < best[0]: best = (imp[j], int(f), float((xs[j] + xs[j + 1]) / 2.0))
        if best is None: return leaf(W)
        _, f, thr = best; me = len(L); L.append(0); R.append(0); F.append(f); T.append(round(thr, 5)); V.append([])
        m = X[ids, f] <= thr
        L[me] = grow(ids[m], depth + 1); R[me] = grow(ids[~m], depth + 1)
        return me
    grow(ids, 0)
    return {'l': L, 'r': R, 'f': F, 't': T, 'v': V}

def train_rf(np, X, y, K, idx, sw, n_trees=RF_TREES, seed=0):
    X = np.asarray(X, dtype=float); y = np.asarray(y); idx = np.asarray(idx); swa = np.asarray(sw, dtype=float)
    Y = np.zeros((len(y), K)); Y[np.arange(len(y)), y] = 1.0
    n_c = np.array([swa[idx][y[idx] == c].sum() for c in range(K)])
    w = swa * np.where(n_c[y] > 0, n_c.sum() / (K * np.maximum(n_c[y], 1e-12)), 0.0)        # class_weight='balanced' x row weight
    rs = np.random.RandomState(seed); n_feat = max(1, int(math.sqrt(X.shape[1])))
    return [_rf_tree(np, X, Y, w, idx[rs.randint(0, len(idx), len(idx))], rs, RF_DEPTH, RF_LEAF, n_feat) for _ in range(n_trees)]

def train_mlp(np, X, y, K, idx, sw, groups, hidden=(10, 10), lr=0.01, epochs=50, batch=64, seed=0):
    X = np.asarray(X, dtype=float); y = np.asarray(y); swa = np.asarray(sw, dtype=float); idx = list(idx)
    gl = sorted(set(groups[i] for i in idx), key=str); random.Random(seed).shuffle(gl)
    vg = set(gl[:max(1, len(gl) // 7)]) if len(gl) >= 4 else set()                       # ~15% of drives/tracks held out for early stopping
    tr = np.array([i for i in idx if groups[i] not in vg]); va = np.array([i for i in idx if groups[i] in vg])
    mu = X[tr].mean(0); sd = X[tr].std(0); sd[sd < 1e-9] = 1.0
    Z = (X - mu) / sd
    n_c = np.array([swa[tr][y[tr] == c].sum() for c in range(K)])
    cw = np.where(n_c > 0, n_c.sum() / (K * np.maximum(n_c, 1e-12)), 0.0); w = swa * cw[y]
    rs = np.random.RandomState(seed); sizes = [X.shape[1]] + list(hidden) + [K]
    Ws = [rs.randn(a, b) * math.sqrt(2.0 / a) for a, b in zip(sizes, sizes[1:])]; bs = [np.zeros(b) for b in sizes[1:]]
    cache = [np.zeros_like(p) for p in Ws + bs]
    def forward(A):
        acts = [A]
        for i, (W_, b_) in enumerate(zip(Ws, bs)):
            A = A.dot(W_) + b_
            if i < len(Ws) - 1: A = np.maximum(A, 0.0)
            acts.append(A)
        e = np.exp(A - A.max(1, keepdims=True)); return acts, e / e.sum(1, keepdims=True)
    def wloss(ii):
        if not len(ii): return 0.0
        _, P = forward(Z[ii]); return float(-(w[ii] * np.log(P[np.arange(len(ii)), y[ii]] + 1e-12)).sum() / w[ii].sum())
    best = (np.inf, None); bad = 0
    for ep in range(epochs):
        perm = rs.permutation(tr)
        for s0 in range(0, len(perm), batch):
            bi = perm[s0:s0 + batch]; acts, P = forward(Z[bi])
            G = P.copy(); G[np.arange(len(bi)), y[bi]] -= 1.0; G *= (w[bi] / w[bi].sum())[:, None]
            grads_W, grads_b = [None] * len(Ws), [None] * len(Ws)
            for i in range(len(Ws) - 1, -1, -1):
                grads_W[i] = acts[i].T.dot(G) + 1e-4 * Ws[i]; grads_b[i] = G.sum(0)
                if i: G = G.dot(Ws[i].T) * (acts[i] > 0)
            for j, (p_, g_) in enumerate(zip(Ws + bs, grads_W + grads_b)):          # RMSprop
                cache[j] = 0.9 * cache[j] + 0.1 * g_ * g_; p_ -= lr * g_ / (np.sqrt(cache[j]) + 1e-7)
        vl = wloss(va) if len(va) else wloss(tr)
        if vl < best[0] - 1e-4: best = (vl, ([W_.copy() for W_ in Ws], [b_.copy() for b_ in bs])); bad = 0
        else:
            bad += 1
            if bad >= 5: break                                                         # early stopping, patience 5
    Wb, bb = best[1]
    return {'mean': mu.tolist(), 'scale': sd.tolist(), 'layers': [[W_.tolist(), b_.tolist()] for W_, b_ in zip(Wb, bb)]}

def _mlp_proba(m, x):
    h = [(xi - mu) / sc for xi, mu, sc in zip(x, m['mean'], m['scale'])]
    for li, (W_, b_) in enumerate(m['layers']):
        h = [b_[j] + sum(h[i] * W_[i][j] for i in range(len(h))) for j in range(len(b_))]
        if li < len(m['layers']) - 1: h = [v if v > 0 else 0.0 for v in h]
    mx = max(h); e = [math.exp(v - mx) for v in h]; t = sum(e)
    return [v / t for v in e]

def _forest_pred(trees, x, K):
    p = [0.0] * K
    for tr in trees:
        i = 0
        while tr['l'][i] != -1: i = tr['l'][i] if x[tr['f'][i]] <= tr['t'][i] else tr['r'][i]
        for j, v in enumerate(tr['v'][i]): p[j] += v
    return max(range(K), key=p.__getitem__)

def autotrain(labels, model_path, min_p=0.6, log=print):
    st = {'t': time.time(), 'installed': False, 'feats_version': FEATS_VERSION}
    try:
        X, y, classes, groups, shown, sw = load_labels(labels)
    except (ValueError, IOError) as e:
        st['msg'] = 'not trained: %s' % e; return st
    K = len(classes); st['rows'] = len(y); st['auto_rows'] = sum(1 for w in sw if w == 1.0); t0 = time.time()
    base = metrics(y, [c if c >= 0 else (y[i] + 1) % K for i, c in enumerate(shown)], classes); st['baseline'] = base
    st['baseline_f1'] = base['macro_f1']
    fl = _folds(groups)
    if not fl:
        st['msg'] = 'not trained: not enough separate drives/tracks to cross-validate'; return st
    try: import numpy as np
    except ImportError: np = None
    cands = {}
    if np is not None:
        for name in ('rf', 'mlp'):
            pred = [0] * len(y)
            for k_, (tr, te) in enumerate(fl):
                if len(set(y[i] for i in tr)) < 2: continue
                if name == 'rf':
                    trees = train_rf(np, X, y, K, tr, sw, n_trees=RF_CV_TREES, seed=k_)
                    for i in te: pred[i] = _forest_pred(trees, X[i], K)
                else:
                    m = train_mlp(np, X, y, K, tr, sw, groups, seed=k_)
                    for i in te: p = _mlp_proba(m, X[i]); pred[i] = p.index(max(p))
            cands[name] = metrics(y, pred, classes)
            log('%s: cross-validated %s (no model: macro-F1 %.3f)' % (name, {k: v for k, v in cands[name].items() if k not in ('f1', 'confusion')}, base['macro_f1']))
    else:                                                                              # no numpy: plain-python extra-trees
        for depth in (5, 8):
            f1 = cv_score(X, y, K, groups, sw=sw, max_depth=depth)
            if f1 is not None: cands['extratrees%d' % depth] = {'macro_f1': round(f1, 3)}
    st['candidates'] = cands
    if not cands:
        st['msg'] = 'not trained: cross-validation failed'; return st
    name = max(cands, key=lambda k: cands[k]['macro_f1']); f1 = cands[name]['macro_f1']; st['cv_f1'] = f1; st['chosen'] = name
    old = None
    try:
        om = json.load(open(model_path))
        if om.get('feats') == FEATS: old = om.get('cv_macro_f1')                         # a model on older features doesn't count
    except Exception: pass
    if f1 < base['macro_f1'] + AUTO_MARGIN:
        st['msg'] = 'kept current: best (%s) %.2f vs no model %.2f' % (name, f1, base['macro_f1']); return st
    if old is not None and f1 < old - 0.02:
        st['msg'] = 'kept current: best (%s) %.2f vs installed %.2f' % (name, f1, old); return st
    allidx = list(range(len(y)))
    if name == 'rf': out = {'kind': 'forest', 'trees': train_rf(np, X, y, K, allidx, sw), 'algo': 'random-forest-%dtrees' % RF_TREES}
    elif name == 'mlp': out = dict(train_mlp(np, X, y, K, allidx, sw, groups), kind='mlp', algo='mlp-2x10')
    else: out = {'kind': 'forest', 'trees': train_forest(X, y, K, max_depth=int(name[-1]), sw=sw), 'algo': 'radarml-' + name}
    out.update({'classes': classes, 'feats': FEATS, 'feats_version': FEATS_VERSION, 'min_p': min_p, 'cv_macro_f1': f1,
                'baseline_f1': base['macro_f1'], 'rows': len(y), 'trained': time.time(), 'metrics': cands.get(name), 'candidates':
                dict((k, {kk: v[kk] for kk in v if kk != 'confusion'}) for k, v in cands.items()), 'baseline': base})
    tmp = model_path + '.tmp'
    json.dump(out, open(tmp, 'w'), separators=(',', ':'))
    if os.path.exists(model_path): os.replace(model_path, model_path + '.prev')
    os.replace(tmp, model_path)
    st.update(installed=True, msg='new model (%s): %.2f vs no model %.2f (%d labels, %.0f s)' % (out['algo'], f1, base['macro_f1'], len(y), time.time() - t0))
    return st

if __name__ == '__main__':
    import sys
    if len(sys.argv) == 4 and sys.argv[1] == '--autotrain':
        files = sys.argv[2].split(',')
        st = autotrain(files, sys.argv[3])
        st['labels'] = sum(sum(1 for _ in open(f)) for f in files if os.path.exists(f))   # radarweb compares this line count to decide when to retrain
        json.dump(st, open(sys.argv[3] + '.status.json', 'w')); print(st['msg'])
    else:
        print('usage: python3 radarml.py --autotrain labels.jsonl[,autolabels.jsonl] radar_model.json')
