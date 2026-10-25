#!/usr/bin/env python3
# radarlearn_train.py - train the radar object classifier from the labels you tap on radarweb's /learn page.
# Runs on a laptop (needs numpy, pandas, scikit-learn); the EON only gets the small JSON it writes.
#   scp eon:/data/canraw/labels.jsonl .       (one file or several: labels from many drives can be combined)
#   python3 radarlearn_train.py labels.jsonl [more.jsonl ...] [--out radar_model.json] [--min-p 0.6]
#   scp radar_model.json eon:/data/radar_model.json      then restart radarweb.py (or set RADAR_MODEL=...)
# What it does, and why it's different from a plain train_test_split:
#   - rows of the same object a few seconds apart are near-copies; splitting them randomly lets the model "see" the
#     test object in training and inflates the score. Rows are grouped by drive (or by track if there's only one
#     drive) and cross-validation never puts one group on both sides.
#   - scaling lives inside a Pipeline, so it's re-fit on each training fold only (no test-set leakage).
#   - it prints the score of what radarweb shows today (radar class + refine_class rules) next to each model,
#     so you can see whether the model is actually an improvement before you put it on the car.
#   - class_weight='balanced' + macro F1: with 300 cars and 15 pedestrians, accuracy alone rewards ignoring pedestrians.
import sys, json, argparse, warnings
warnings.filterwarnings('ignore', message='y_pred contains classes')
import numpy as np, pandas as pd
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier, ExtraTreesClassifier
from sklearn.model_selection import StratifiedGroupKFold, GridSearchCV, cross_val_predict
from sklearn.metrics import classification_report, confusion_matrix, f1_score, balanced_accuracy_score, accuracy_score
from radarml import FEATS, TARGET, DISPLAY, features

ap = argparse.ArgumentParser()
ap.add_argument('files', nargs='+')
ap.add_argument('--out', default='radar_model.json')
ap.add_argument('--min-p', type=float, default=0.6, help='radarweb only overrides the radar class when the model is at least this sure')
ap.add_argument('--min-rows', type=int, default=8, help='targets with fewer labelled rows than this are dropped')
ap.add_argument('--trees', type=int, default=40, help='forest size cap: every tree costs CPU on the EON at 10 Hz x 32 tracks')
ap.add_argument('--plots', action='store_true', help='save confusion matrix + feature importance PNGs')
A = ap.parse_args()

rows = []
for fn in A.files:
    for ln in open(fn):
        try: r = json.loads(ln)
        except ValueError: continue
        if r.get('c') in TARGET and 'd' in r: rows.append(r)
if not rows: sys.exit('no usable labels in ' + ', '.join(A.files))
rows.sort(key=lambda r: r.get('t', 0))
df = pd.DataFrame({'target': [TARGET[r['c']] for r in rows], 't': [r.get('t', 0) for r in rows], 's': [r.get('s', -1) for r in rows],
                   'shown': [DISPLAY.get(r.get('radar_c'), 'unknown') if r.get('radar_c') in DISPLAY else 'unknown' for r in rows],
                   'has_hist': ['h' in r for r in rows]})
X = np.array([features(r) for r in rows], dtype=float)

# groups: a new drive after 5 min without labels; within a drive, a new track when the slot changes or 15 s pass
t = df['t'].values
drive = np.cumsum(np.r_[1, np.diff(t) > 300])
track = np.cumsum(np.r_[1, (np.diff(t) > 15) | (np.diff(df['s'].values) != 0)])
tids = [r.get('tid') for r in rows]          # automatic labels: all snapshots of one track share a 'tid'
if any(tids):
    ids = {}; track = np.array([ids.setdefault(tid, len(ids) + 10 ** 6) if tid else k for tid, k in zip(tids, track)])
counts = df['target'].value_counts()
keep = df['target'].map(counts).values >= A.min_rows
if (~keep).any(): print('dropped (fewer than %d rows): %s' % (A.min_rows, dict(counts[counts < A.min_rows])))
X, df, drive, track = X[keep], df[keep].reset_index(drop=True), drive[keep], track[keep]
y = df['target'].values
classes = sorted(set(y))
if len(classes) < 2: sys.exit('need at least 2 label classes with %d+ rows each - keep labelling on /learn' % A.min_rows)
groups, gname = (drive, 'drive') if len(set(drive)) >= 5 else (track, 'track')
n_split = int(min(5, len(set(groups)), df['target'].value_counts().min()))
if n_split < 2: sys.exit('not enough independent %ss to cross-validate - label objects on more drives' % gname)
cv = StratifiedGroupKFold(n_splits=n_split, shuffle=True, random_state=0)
print('%d labelled rows, %d %ss, %d with track history, classes: %s' % (len(y), len(set(groups)), gname, int(df['has_hist'].sum()), {k: int(v) for k, v in df['target'].value_counts().items()}))
if df['has_hist'].mean() < 0.5: print('note: most rows were labelled before radarweb stored track history - the history features are flat for those')

# what radarweb shows today, scored only on the classes it can actually draw
def score(name, pred):
    # the four numbers INFRA3DRC-Radar-Classification reports, so results can be put side by side
    print('%-24s acc %.3f  balanced acc %.3f  macro-F1 %.3f  weighted-F1 %.3f' % (name, accuracy_score(y, pred), balanced_accuracy_score(y, pred),
          f1_score(y, pred, average='macro', labels=classes, zero_division=0), f1_score(y, pred, average='weighted', labels=classes, zero_division=0)))
score('today (radar + rules)', df['shown'].values)

cands = {
    'logreg': (make_pipeline(StandardScaler(), LogisticRegression(max_iter=2000, class_weight='balanced')),
               {'logisticregression__C': [0.03, 0.1, 0.3, 1, 3, 10]}),
    'forest': (RandomForestClassifier(n_estimators=A.trees, class_weight='balanced_subsample', random_state=0, n_jobs=-1),
               {'max_depth': [4, 6, 8], 'min_samples_leaf': [1, 3, 5], 'max_features': ['sqrt', 0.5]}),
    'extratrees': (ExtraTreesClassifier(n_estimators=A.trees, class_weight='balanced_subsample', random_state=0, n_jobs=-1),
                   {'max_depth': [4, 6, 8], 'min_samples_leaf': [1, 3, 5], 'max_features': ['sqrt', 0.5]}),
}
best = None
for name, (est, grid) in cands.items():
    gs = GridSearchCV(est, grid, scoring='f1_macro', cv=cv)
    # nested CV: the grid search runs inside each outer fold, so the score below is not biased by the tuning
    pred = cross_val_predict(gs, X, y, groups=groups, cv=cv, params={'groups': groups})
    f1 = f1_score(y, pred, average='macro')
    score(name, pred)
    if best is None or f1 > best[1]: best = (name, f1, gs, pred)

name, f1, gs, pred = best
print('\nbest: %s (cross-validated, never scored on rows it trained on)\n' % name)
print(classification_report(y, pred, labels=classes, zero_division=0))
cm = confusion_matrix(y, pred, labels=classes)
print('confusion (rows = your label, columns = model):\n' + pd.DataFrame(cm, index=classes, columns=classes).to_string())

gs.fit(X, y, groups=groups)
model = gs.best_estimator_
print('\nparameters:', gs.best_params_)
if name == 'logreg':
    sc, lr = model.named_steps['standardscaler'], model.named_steps['logisticregression']
    coef, icpt = lr.coef_, lr.intercept_
    if len(lr.classes_) == 2: coef, icpt = np.vstack([-coef / 2, coef / 2]), np.r_[-icpt / 2, icpt / 2]   # binary -> 2-row softmax
    out = {'kind': 'logreg', 'classes': list(lr.classes_), 'mean': sc.mean_.tolist(), 'scale': sc.scale_.tolist(),
           'coef': coef.tolist(), 'intercept': icpt.tolist()}
    imp = np.abs(coef).mean(0)
else:
    trees = []
    for e in model.estimators_:
        tr = e.tree_; v = tr.value[:, 0, :]; v = v / np.maximum(v.sum(1, keepdims=True), 1e-12)
        trees.append({'l': tr.children_left.tolist(), 'r': tr.children_right.tolist(), 'f': tr.feature.tolist(),
                      't': [round(float(x), 5) for x in tr.threshold], 'v': np.round(v, 4).tolist()})
    out = {'kind': 'forest', 'classes': list(model.classes_), 'trees': trees}
    imp = model.feature_importances_
out.update({'feats': FEATS, 'min_p': A.min_p, 'cv_macro_f1': round(f1, 3), 'rows': int(len(y)), 'algo': name})
json.dump(out, open(A.out, 'w'), separators=(',', ':'))
print('top features:', ', '.join('%s %.2f' % (FEATS[i], imp[i]) for i in np.argsort(imp)[::-1][:8]))
print('wrote', A.out, '-> copy it to /data/radar_model.json on the EON and restart radarweb.py')

if A.plots:
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 5)); ax.imshow(cm, cmap='Blues')
    ax.set_xticks(range(len(classes)), classes, rotation=45, ha='right'); ax.set_yticks(range(len(classes)), classes)
    for i in range(len(classes)):
        for j in range(len(classes)): ax.text(j, i, cm[i, j], ha='center', va='center')
    ax.set_xlabel('model'); ax.set_ylabel('your label'); ax.set_title('Confusion matrix (%s, cross-validated)' % name)
    fig.tight_layout(); fig.savefig('radar_confusion.png', dpi=120)
    o = np.argsort(imp); fig, ax = plt.subplots(figsize=(6, 7)); ax.barh([FEATS[i] for i in o], imp[o])
    ax.set_title('Feature importance'); fig.tight_layout(); fig.savefig('radar_features.png', dpi=120)
    print('saved radar_confusion.png, radar_features.png')
