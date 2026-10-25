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
def decode_ab(a_bytes, b_bytes):
    """extra per-object fields (short keys, as stored in the track history) from one A/B frame pair"""
    a = _dec(a_bytes, SIG_A); b = _dec(b_bytes, SIG_B)
    return {'ls': b['LatSpeed'], 'la': a['LongAccel'], 'dz': b['dZ'], 'po': a['ProbObstacle'], 'pno': a['ProbNonObstacle'],
            'dxs': b['dxSigma'], 'vxs': b['vxSigma'], 'axs': b['axSigma'], 'dys': b['dySigma'], 'rel': b['dxRearEndLoss'],
            'meas': a['Meas'], 'mst': b['MovingState']}

# history row = [age_s] + SIGNALS (missing signal -> 0, like INFRA3DRC's fillna(0)). The labels file stores the
# field names next to the rows ('hf'), so rows recorded with an older radarweb still line up.
SIGNALS = ['d', 'y', 'vr', 'gs', 'ls', 'la', 'len', 'dz', 'pe', 'po', 'pno', 'pc', 'dxs', 'vxs', 'dys']
HIST_FIELDS = ['age'] + SIGNALS
STATS = ('mean', 'std', 'min', 'max', 'median', 'span')
FEATS = ['v_ego', 'age', 'n', 'abs_y', 'abs_gs', 'y_rate', 'd_rate_err', 'meas'] + \
        ['%s_now' % s for s in SIGNALS] + ['%s_%s' % (s, st) for s in SIGNALS for st in STATS] + \
        ['rc_' + c for c in RADAR_CLASSES] + ['ms_%d' % i for i in range(4)]


def _stats(a):
    n = len(a); m = sum(a) / n; srt = sorted(a)
    med = srt[n // 2] if n % 2 else (srt[n // 2 - 1] + srt[n // 2]) / 2
    return {'mean': m, 'std': math.sqrt(sum((x - m) ** 2 for x in a) / n), 'min': srt[0], 'max': srt[-1], 'median': med, 'span': srt[-1] - srt[0]}
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
    ds, ys = col('d'), col('y')
    f = {'v_ego': v_ego, 'age': -ts[0], 'n': float(len(hist)), 'abs_y': abs(last['y']), 'abs_gs': abs(last['gs']),
         'y_rate': _slope(ts, ys), 'meas': float(o.get('meas', 0) or 0),
         # range rate the radar reports vs how the distance actually changed: real objects agree, multipath ghosts often don't
         'd_rate_err': abs(_slope(ts, ds) - sum(col('vr')) / len(hist)) if len(hist) >= 3 else 0.0}
    for s in SIGNALS:
        f[s + '_now'] = last[s]
        for st, v in _stats(col(s)).items(): f['%s_%s' % (s, st)] = v
    for c in RADAR_CLASSES: f['rc_' + c] = 1.0 if rc == c else 0.0
    for i in range(4): f['ms_%d' % i] = 1.0 if mst == i else 0.0
    return [f[k] for k in FEATS]


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
    def get(self, k, now):
        """history in the features() format (times relative to now), or None"""
        h = self.h.get(k)
        if not h or now - h[-1][0] > 0.5: return None
        return [[round(r[0] - now, 2)] + [round(x, 3) for x in r[1:]] for r in h]
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
                              'move_s': 0.0, 'still_f': 0, 'gs_max': 0.0, 'ego_m': 0.0, 'pe_max': 0.0, 'rc': {}, 'derr': 0.0, 'snaps': []}
        dt = min(0.2, now - t['last']) if t['f'] else 0.0
        gs = abs(o['gs']); t['f'] += 1; t['last'] = now; t['d'] = o['d']; t['y'] = o['y']
        if gs >= 6.0: t['move_s'] += dt
        if gs < 0.5: t['still_f'] += 1
        t['gs_max'] = max(t['gs_max'], gs); t['ego_m'] += abs(v_ego) * dt; t['pe_max'] = max(t['pe_max'], o.get('pe', 0))
        if o.get('pc', 0) >= 80: t['rc'][o.get('rc', o.get('c'))] = t['rc'].get(o.get('rc', o.get('c')), 0) + 1
        age = now - t['t0']
        if len(t['snaps']) < len(self.SNAP_AT) and age >= self.SNAP_AT[len(t['snaps'])] and hist:
            x = features(o, hist, v_ego); t['derr'] = max(t['derr'], x[FEATS.index('d_rate_err')])
            row = dict((key, o.get(key)) for key in ('s', 'd', 'y', 'vr', 'gs', 'len', 'pe', 'mst', 'meas') + tuple(SIGNALS) if key in o)
            row.update(radar_c=o.get('rc', o.get('c')), radar_pc=o.get('pc'), t=round(now, 2), v=round(v_ego, 2), h=hist[::2], hf=HIST_FIELDS)
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
        if sure: return sure                                                        # radar held one class at 80%+ for most of the track
        if life >= 3.0 and t['still_f'] >= 0.95 * f and t['ego_m'] >= 20.0: return 'static'   # never moved while you drove 20 m past it
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

def cv_score(X, y, K, groups, folds=5, sw=None, **kw):
    """macro-F1 over group k-fold: every prediction is made by a forest that never saw that drive/track"""
    ug = sorted(set(groups), key=str); folds = min(folds, len(ug)); pred = [0] * len(y)
    if folds < 2: return None
    order = list(ug); random.Random(1).shuffle(order); fold_of = dict((g, i % folds) for i, g in enumerate(order))
    for fo in range(folds):
        tr = [i for i, g in enumerate(groups) if fold_of[g] != fo]; te = [i for i, g in enumerate(groups) if fold_of[g] == fo]
        if len(set(y[i] for i in tr)) < 2: continue
        trees = train_forest(X, y, K, tr, sw=sw, **kw)
        for i in te: pred[i] = _predict(trees, X[i])
    return macro_f1(y, pred, K)

def autotrain(labels, model_path, min_p=0.6, log=print):
    st = {'t': time.time(), 'installed': False}
    try:
        X, y, classes, groups, shown, sw = load_labels(labels)
    except (ValueError, IOError) as e:
        st['msg'] = 'not trained: %s' % e; return st
    K = len(classes); st['rows'] = len(y); t0 = time.time()
    base = macro_f1(y, shown, K); st['baseline_f1'] = round(base, 3)
    best = None
    for depth in (5, 8):
        f1 = cv_score(X, y, K, groups, sw=sw, max_depth=depth)
        log('depth %d: cross-validated macro-F1 %s (no model: %.3f)' % (depth, f1, base))
        if f1 is not None and (best is None or f1 > best[0]): best = (f1, depth)
    if best is None:
        st['msg'] = 'not trained: not enough separate drives/tracks to cross-validate'; return st
    f1, depth = best; st['cv_f1'] = round(f1, 3)
    old = None
    try: old = json.load(open(model_path)).get('cv_macro_f1')
    except Exception: pass
    if f1 < base + AUTO_MARGIN:
        st['msg'] = 'kept current: new model %.2f vs no model %.2f' % (f1, base); return st
    if old is not None and f1 < old - 0.02:
        st['msg'] = 'kept current: new model %.2f vs installed %.2f' % (f1, old); return st
    trees = train_forest(X, y, K, max_depth=depth, sw=sw)
    out = {'kind': 'forest', 'classes': classes, 'trees': trees, 'feats': FEATS, 'min_p': min_p, 'cv_macro_f1': round(f1, 3),
           'baseline_f1': round(base, 3), 'rows': len(y), 'algo': 'radarml-extratrees-depth%d' % depth, 'trained': time.time()}
    tmp = model_path + '.tmp'
    json.dump(out, open(tmp, 'w'), separators=(',', ':'))
    if os.path.exists(model_path): os.replace(model_path, model_path + '.prev')
    os.replace(tmp, model_path)
    st.update(installed=True, msg='new model: %.2f vs no model %.2f (%d labels, %.0f s)' % (f1, base, len(y), time.time() - t0))
    st['auto_rows'] = sum(1 for w in sw if w == 1.0)
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
