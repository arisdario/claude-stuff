"""Seeded benchmark for the offline navigator. Judges every behaviour against GROUND TRUTH (the edge the simulated car is really on) and the INDEPENDENT
reachability oracle, never against the navigator's own opinion. Advisory tooling only - no vehicle control.

  python3 -m tesla.nav.evaluate --all --seeds 30 --md tesla/nav/BENCHMARK.md --json bench.json     # the complete benchmark
  python3 -m tesla.nav.evaluate --family parallel --filter sep=20 --filter noise=8 --seeds 30      # one scenario family / slice
  python3 -m tesla.nav.evaluate --family parallel --filter sep=8 --filter noise=5 --filter connected=true --seeds 1 --seed-base 17 --trace   # ONE run, per fix
  python3 -m tesla.nav.evaluate --all --override tracker.SWITCH_LLR=6 --json variant.json         # evidence for a parameter change (never changes the defaults)
  python3 -m tesla.nav.evaluate --compare baseline.json variant.json [...]                        # the variants side by side, per ambiguity class (classes are never pooled)

Reproducibility: a run is fully determined by (family, scenario parameters, GPS noise sigma, noise model, seed). The seed drives the GPS noise, the sampling
phase of the drive and (gps_events) the timing of outliers/dropouts; the networks themselves are deterministic. Every record stores seed, parameters, noise
model, git commit and a hash of every navigation constant.

Metric definitions (all per run unless stated; proportions carry Wilson 95% intervals, medians a bootstrap 95% interval):
  condition class  'ambiguous' (wrong road within 2 sigma of the route), 'marginal' (2-3 sigma), 'clear' (>= 3 sigma); judged from the NOMINAL GPS sigma, not from
                   how the navigator performs. Classes are never pooled into one success rate.
  per fix          (judged from SETTLE_M = 60 m after the car leaves the route; the latency before that is measured as 'time to correct identification')
                   correct = confident AND tracker on the true road; wrong_confident = confident AND not on the true road; uncertain = tracker lost or contested, or
                   the navigator reports ambiguous. 'uncertain' is NOT counted as wrong - but persistent ambiguity (an episode longer than 100 m, or unresolved at
                   the end of the drive) is reported separately so it cannot flatter the other numbers.
  detection        the car left the route and the navigator reported off_route/no_route/rerouted; missed = never reported.
  false off-route  an on-route (control) drive in which the navigator reported off_route/no_route/rerouted (a declaration on screen).
  false recalc     an on-route drive in which the navigator actually RECALCULATED (the harmful kind: it replaces a correct route).
  time to correct identification  distance after leaving the route until the tracker is on the true road, confident, for 3 fixes in a row.
  wrong-road recalc  a recalculation whose first edge is not on the true road (+-30 m); 'confident' = not flagged confidence='low'.
  snap-back        a recalculation although the oracle says the destination is unreachable from the true road, or one that starts on the original route's road while the
                   car is not on it.
  arrival          the navigator reached 'arrived' (only where the oracle says the destination is reachable)."""
import argparse
import hashlib
import json
import math
import os
import random
import subprocess
import sys
import time

try:
    from . import geo, scenarios as S, tracker as TK, matcher as M, validate as V
    from .router import find_route, NoRoute
    from .oracle import Oracle
except ImportError:
    import geo
    import scenarios as S
    import tracker as TK
    import matcher as M
    import validate as V
    from router import find_route, NoRoute
    from oracle import Oracle

SPEED = 15.0
WINDOW_M = 30.0                  # a recalculation may start anywhere within +-WINDOW_M of the true position along the driven road
PERSISTENT_M = 100.0             # an ambiguous episode longer than this, or still open at the end of the drive, is 'persistent'
SETTLE_M = 60.0                  # steady-state road identification is judged this far (4 fixes at 15 m/s) after the car leaves the route; the latency before it is
                                 # measured and reported separately (ident_delay_m), and the unsettled raw counts are kept in every record
SUSTAINED_FIXES = 3              # a confident wrong belief held for this many consecutive fixes (after settling) is a failure; one noisy fix is not
ALARM = ('off_route', 'no_route', 'rerouted')
FAMILIES = ('parallel', 'fork', 'ladder', 'repeated', 'diamond', 'oneway', 'gps_events', 'island', 'teleport_island')


# ------------------------------------------------------------------ provenance ------------------------------------------------------
def param_snapshot():
    """every numeric navigation constant (tracker + matcher), so a result can be tied to the exact parameters that produced it"""
    out = {}
    for mod, name in ((TK, 'tracker'), (M, 'matcher')):
        for k in dir(mod):
            v = getattr(mod, k)
            if k.isupper() and isinstance(v, (int, float)) and not isinstance(v, bool):
                out['%s.%s' % (name, k)] = v
    return out


def version_info(overrides=()):
    try:
        here = os.path.dirname(os.path.abspath(__file__))
        commit = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], cwd=here, stderr=subprocess.DEVNULL).decode().strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain', '--', 'tesla/nav'], cwd=here, stderr=subprocess.DEVNULL).decode().strip())
    except Exception:
        commit, dirty = 'unknown', None
    snap = param_snapshot()
    return {'commit': commit, 'nav_dirty': dirty, 'params_hash': hashlib.sha1(json.dumps(snap, sort_keys=True).encode()).hexdigest()[:12],
            'params': snap, 'overrides': list(overrides)}


def apply_overrides(overrides):
    for ov in overrides:
        key, val = ov.split('=', 1)
        mod, name = key.split('.', 1)
        target = {'tracker': TK, 'matcher': M}[mod]
        if not hasattr(target, name):
            raise SystemExit('unknown parameter %s' % key)
        setattr(target, name, type(getattr(target, name))(float(val)))


# ------------------------------------------------------------------ conditions ------------------------------------------------------
class Condition(object):
    def __init__(self, family, params, noise, model='gauss'):
        self.family, self.params, self.noise, self.model = family, dict(params), float(noise), model

    @property
    def id(self):
        return '%s|%s|noise=%g|%s' % (self.family, ','.join('%s=%s' % kv for kv in sorted(self.params.items())), self.noise, self.model)

    def as_dict(self):
        return {'family': self.family, 'params': self.params, 'noise': self.noise, 'noise_model': self.model}


def full_grid():
    c = []
    mixes = ((2, 'gauss'), (5, 'gauss'), (8, 'gauss'), (5, 'ar1'), (5, 'heavy'), (5, 'uniform'))
    for sep in (3, 8, 12, 20, 35):
        for noise, model in mixes:
            for conn in (True, False):
                c.append(Condition('parallel', {'sep': sep, 'connected': conn}, noise, model))
    for angle in (5, 10, 15, 30, 90):
        for merge in (True, False):
            for noise in (2, 5, 8):
                c.append(Condition('fork', {'angle': angle, 'merge': merge}, noise))
    for sep in (8, 12, 20, 35):
        for noise in (2, 5, 8):
            c.append(Condition('ladder', {'sep': sep}, noise))
    for sep in (8, 20):
        for noise in (2, 5, 8):
            c.append(Condition('repeated', {'sep': sep}, noise))
    for sep in (3, 8, 20):
        for noise in (2, 5, 8):
            c.append(Condition('diamond', {'sep': sep}, noise))
    for sep in (12, 20):
        for link in ('into', 'out'):
            for noise in (2, 5, 8):
                c.append(Condition('oneway', {'sep': sep, 'link': link}, noise))
    for kind, ns in (('jump', (1, 2, 3, 5)), ('dropout', (5, 10, 30))):
        for n in ns:
            for noise in (2, 5):
                c.append(Condition('gps_events', {'sep': 20, 'kind': kind, 'n': n}, noise))
    c.append(Condition('island', {'sep': 20}, 5))
    for noise in (2, 5, 8):
        c.append(Condition('teleport_island', {'sep': 20}, noise))
    return c


def quick_grid():
    keep = {('parallel', 20, 5, 'gauss'), ('parallel', 8, 2, 'gauss'), ('parallel', 8, 8, 'gauss'), ('fork', 90, 5, 'gauss'), ('fork', 10, 5, 'gauss')}
    out = []
    for cd in full_grid():
        key = (cd.family, cd.params.get('sep', cd.params.get('angle')), cd.noise, cd.model)
        if key in keep or (cd.family in ('ladder', 'repeated', 'diamond', 'oneway', 'gps_events', 'island', 'teleport_island') and cd.noise in (5, 5.0)
                           and cd.params.get('sep', 20) in (8, 20)):
            out.append(cd)
    return out


def build_scenario(cond, seed=0):
    """Deterministic given the condition; `seed` is accepted (and recorded) for uniformity - the run seed drives GPS noise, sampling phase and event timing."""
    p, f = cond.params, cond.family
    if f == 'parallel':
        return S.parallel(p['sep'], p['connected'])
    if f == 'fork':
        return S.fork(p['angle'], p['merge'])
    if f == 'ladder':
        return S.ladder(p['sep'])
    if f == 'repeated':
        return S.repeated(p['sep'])
    if f == 'diamond':
        return S.diamond(p['sep'])
    if f == 'oneway':
        sc = S.parallel(p['sep'], True, link_oneway=p['link'])
        sc.family = 'oneway'
        return sc
    if f == 'gps_events':
        return S.gps_events(p['sep'])
    if f == 'island':
        return S.island(p['sep'])
    if f == 'teleport_island':
        return S.teleport_to_island(p['sep'])
    raise ValueError('unknown family %r' % f)


# ------------------------------------------------------------------ driving + truth ---------------------------------------------------
def _edges_for_nodes(g, nodes):
    out = []
    for a, b in zip(nodes, nodes[1:]):
        out.append(next(e for e in g.out_edges(a) if e.v == b))
    return out


def _driven(g, edges):
    """(points, offsets, lengths) of driving the given directed edges in order"""
    pts, offs, lens, run = [], [], [], 0.0
    for e in edges:
        gm = g.geom(e.eid)
        offs.append(run)
        lens.append(e.length_m)
        pts.extend(gm[1:] if pts and pts[-1] == gm[0] else gm)
        run += e.length_m
    return pts, offs, lens


class Truth(object):
    def __init__(self, edges, offs, lens):
        self.edges, self.offs, self.lens = edges, offs, lens

    def window(self, s, w=WINDOW_M):
        return {e.eid for e, o, l in zip(self.edges, self.offs, self.lens) if o - w <= s <= o + l + w}

    def edge_at(self, s):
        for e, o, l in zip(self.edges, self.offs, self.lens):
            if o <= s < o + l:
                return e, s - o
        return self.edges[-1], self.lens[-1]


def _route_truth(R0):
    offs = list(R0.edge_offsets)
    lens = [offs[i + 1] - offs[i] for i in range(len(offs) - 1)] + [R0.distance_m - offs[-1]]
    return Truth(list(R0.edges), offs, lens)


def _events(fixes, rnd, kind, n):
    """Inject ONE kind of GPS fault at a seed-dependent time into an on-route drive, so the failure cliff is visible:
         jump     `n` consecutive fixes displaced 80 m sideways (a multipath/urban-canyon excursion lasting n seconds, ~15 m of driving each)
         dropout  `n` seconds with no fix at all"""
    out = list(fixes)
    total = len(out)
    start = rnd.randint(8, max(9, total - n - 8))
    m_lat = (math.pi / 180.0) * geo.R_EARTH
    for i in range(start, min(total, start + n)):
        t, la, lo, h, v, a = out[i]
        out[i] = (t, la + 80.0 / m_lat, lo, h, v, a) if kind == 'jump' else (t, None, None, h, v, a)
    return out


def play(sc, cond, seed, R0, oracle, points, truth, s_leave, s0, fixes, post_from=0.0):
    """Run the navigator over `fixes`, judging each step against `truth` (None = no per-fix truth). Returns one metrics dict."""
    nav = M.Navigator(sc.graph, sc.dest)
    nav.plan(R0.points[0][0], R0.points[0][1], None)
    route0 = {e.eid for e in R0.edges}
    targets = oracle.targets_for(R0.edges[-1].eid, R0.goal.along_m)
    m = {'n_fixes': len(fixes), 'detected': False, 'delay_m': None, 'early_alarm': False, 'alarm_any': False, 'alarm_status': False, 'recalcs': [], 'snap_backs': 0, 'arrived': False,
         'post_fixes': 0, 'correct': 0, 'wrong_conf': 0, 'uncertain': 0, 'episodes': [], 'amb_any_post': False, 'statuses': {},
         'raw_post_fixes': 0, 'raw_correct': 0, 'raw_wrong_conf': 0, 'raw_uncertain': 0, 'ident_delay_m': None, 'sustained_wrong': 0}
    prev_rer, open_ep, ident_run, ident_start, wrong_run = 0, None, 0, 0.0, 0
    for k, f in enumerate(fixes):
        st = nav.update(*f)
        s_true = s0 + f[0] * SPEED
        m['statuses'][st.status] = m['statuses'].get(st.status, 0) + 1
        post = s_leave is None or s_true >= s_leave
        if st.status in ('off_route', 'no_route'):
            m['alarm_status'] = True
        if st.status in ALARM:
            m['alarm_any'] = True
            if s_leave is not None and s_true < s_leave - 20.0:
                m['early_alarm'] = True
            if post and s_leave is not None and not m['detected']:
                m['detected'], m['delay_m'] = True, max(0.0, s_true - s_leave)
        if st.status == 'arrived':
            m['arrived'] = True
        if nav.reroutes > prev_rer:
            first = nav.route.edges[0].eid
            ev = {'k': k, 's': s_true, 'low_conf': st.confidence == 'low', 'first_edge': first}
            if truth is not None:
                te, along = truth.edge_at(s_true)
                ev['wrong'] = first not in truth.window(s_true)
                unreachable = not oracle.reachable(te.eid, targets, along)[0]
                ev['snap_back'] = bool(unreachable or (ev['wrong'] and first in route0 and post))
            else:                                                            # teleport: the destination cannot be reached from the island
                ev['wrong'], ev['snap_back'] = True, True
            m['snap_backs'] += 1 if ev['snap_back'] else 0
            m['recalcs'].append(ev)
            prev_rer = nav.reroutes
        # ---- per-fix road identification (only where there is a truth and a usable fix) -------------------------------------------
        if truth is not None and f[1] is not None and post and st.status not in ('gps_lost', 'gps_poor'):
            tk = nav.tracker
            uncertain = (not tk.known) or tk.contested or st.ambiguous
            on_true = (not uncertain) and tk.edge.eid in truth.window(s_true)
            settled = s_leave is None or s_true >= s_leave + SETTLE_M
            # raw (unsettled) counts: always kept
            m['raw_post_fixes'] += 1
            m['raw_correct'] += 1 if on_true else 0
            m['raw_wrong_conf'] += 1 if (not uncertain and not on_true) else 0
            m['raw_uncertain'] += 1 if uncertain else 0
            # time to correct identification: first fix that starts 3 consecutive correct-confident fixes after leaving
            if s_leave is not None and m['ident_delay_m'] is None:
                if on_true:
                    if ident_run == 0:
                        ident_start = s_true
                    ident_run += 1
                    if ident_run >= 3:
                        m['ident_delay_m'] = max(0.0, ident_start - s_leave)
                else:
                    ident_run = 0
            if uncertain:
                m['amb_any_post'] = m['amb_any_post'] or settled
                if open_ep is None:
                    open_ep = [s_true, s_true]
                open_ep[1] = s_true
            elif open_ep is not None:
                m['episodes'].append({'len_m': open_ep[1] - open_ep[0] + SPEED, 'resolved': True})
                open_ep = None
            if settled:
                m['post_fixes'] += 1
                if uncertain:
                    m['uncertain'] += 1
                    wrong_run = 0
                elif on_true:
                    m['correct'] += 1
                    wrong_run = 0
                else:
                    m['wrong_conf'] += 1
                    wrong_run += 1
                    if wrong_run == SUSTAINED_FIXES:
                        m['sustained_wrong'] += 1
    if open_ep is not None:
        m['episodes'].append({'len_m': open_ep[1] - open_ep[0] + SPEED, 'resolved': False})
    m['persistent_amb'] = any((not e['resolved']) or e['len_m'] > PERSISTENT_M for e in m['episodes'])
    m['reroutes'] = nav.reroutes
    return m


def run_one(cond, seed, trace=False):
    """One seeded run of a condition: returns a record with seed, parameters, noise model and the metrics of each drive (control and/or deviation)."""
    sc = build_scenario(cond, seed)
    rnd = random.Random(seed)
    s0 = rnd.uniform(0.0, SPEED)                                         # sampling phase: where the first fix falls
    rec = {'seed': seed, 'condition': cond.as_dict(), 'id': cond.id, 'class': sc.ambiguity_class(cond.noise), 'eff_sep': sc.eff_sep, 'phase_m': round(s0, 3)}
    g = sc.graph
    if cond.family == 'island':
        try:
            find_route(g, sc.start, sc.dest)
            rec['planning'] = {'routed': True, 'diagnosed': False}
        except NoRoute:
            rec['planning'] = {'routed': False, 'diagnosed': 'DIFFERENT' in V.diagnose_no_route(g, sc.start, sc.dest)}
        return rec
    R0 = find_route(g, sc.start, sc.dest)
    oracle = sc.__dict__.setdefault('_oracle', None) or Oracle(g.path)
    sc._oracle = oracle
    # ---- control: the car follows the planned route --------------------------------------------------------------------------------
    if cond.family != 'teleport_island':
        fx = V.fixes_along(R0.points, SPEED, noise_m=cond.noise, seed=seed, noise_model=cond.model, s0=s0)
        if cond.family == 'gps_events':
            fx = _events(fx, random.Random(seed + 7919), cond.params['kind'], cond.params['n'])
        rec['control'] = play(sc, cond, seed, R0, oracle, R0.points, _route_truth(R0), None, s0, fx)
    # ---- the car takes a different road ----------------------------------------------------------------------------------------------
    if cond.family == 'teleport_island':
        isl = _edges_for_nodes(g, sc.island)
        ipts, _, _ = _driven(g, isl)
        k0 = 14
        a = V.fixes_along(R0.points, SPEED, noise_m=cond.noise, seed=seed, noise_model=cond.model, s0=s0)[:k0]
        b = V.fixes_along(ipts, SPEED, noise_m=cond.noise, seed=seed + 1, noise_model=cond.model, s0=0.0)
        b = [(a[-1][0] + 1.0 + f[0],) + tuple(f[1:]) for f in b]
        s_leave = s0 + a[-1][0] * SPEED
        rec['deviation'] = play(sc, cond, seed, R0, oracle, None, None, s_leave, s0, a + b)
        rec['oracle_reachable'] = False
    elif sc.leave is not None:
        edges = _edges_for_nodes(g, sc.driven)
        pts, offs, lens = _driven(g, edges)
        truth = Truth(edges, offs, lens)
        s_leave = next(o for e, o in zip(edges, offs) if e.u == sc.leave)
        fx = V.fixes_along(pts, SPEED, noise_m=cond.noise, seed=seed, noise_model=cond.model, s0=s0)
        rec['deviation'] = play(sc, cond, seed, R0, oracle, pts, truth, s_leave, s0, fx)
        te, along = truth.edge_at(min(s_leave + 40.0, truth.offs[-1] + truth.lens[-1] - 1.0))
        rec['oracle_reachable'] = oracle.reachable(te.eid, oracle.targets_for(R0.edges[-1].eid, R0.goal.along_m), along)[0]
    return rec


# ------------------------------------------------------------------ statistics ---------------------------------------------------------
def wilson(k, n, z=1.96):
    if n == 0:
        return (float('nan'), float('nan'), float('nan'))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (p, max(0.0, c - h), min(1.0, c + h))


def pct(t):
    p, lo, hi = t
    return 'n/a' if p != p else '%.0f%% [%.0f-%.0f]' % (100 * p, 100 * lo, 100 * hi)


def median_ci(vals, n=1000, seed=12345):
    if not vals:
        return None
    v = sorted(vals)
    med = lambda a: a[len(a) // 2] if len(a) % 2 else 0.5 * (a[len(a) // 2 - 1] + a[len(a) // 2])
    rnd = random.Random(seed)
    boots = sorted(med(sorted(rnd.choice(v) for _ in v)) for _ in range(n))
    return med(v), boots[int(0.025 * n)], boots[int(0.975 * n) - 1]


def fmt_med(t):
    return '-' if t is None else '%.0f [%.0f-%.0f]' % t


def summarise(recs):
    """pooled metrics over a list of run records (one family x class). Never mixes classes: the caller groups."""
    ctl = [r['control'] for r in recs if 'control' in r]
    dev = [r['deviation'] for r in recs if 'deviation' in r]
    out = {'runs': len(recs), 'control_runs': len(ctl), 'deviation_runs': len(dev)}
    out['false_positive'] = wilson(sum(1 for c in ctl if c['alarm_any']), len(ctl))          # any off-route declaration or recalculation on an on-route drive
    out['false_recalc'] = wilson(sum(1 for c in ctl if c['reroutes']), len(ctl))             # the harmful kind
    out['fp_per_1000_fixes'] = (1000.0 * sum(1 for c in ctl if c['alarm_any']) / max(1, sum(c['n_fixes'] for c in ctl)))
    out['detection'] = wilson(sum(1 for d in dev if d['detected']), len(dev))
    out['early_alarm'] = wilson(sum(1 for d in dev if d['early_alarm']), len(dev))
    out['delay'] = median_ci([d['delay_m'] for d in dev if d['detected']])
    rec_runs = [d for d in dev if d['recalcs']]
    out['recalc_runs'] = wilson(len(rec_runs), len(dev))
    out['wrong_confident'] = wilson(sum(1 for d in rec_runs if any(e['wrong'] and not e['low_conf'] for e in d['recalcs'])), len(rec_runs))
    out['wrong_low_conf'] = wilson(sum(1 for d in rec_runs if any(e['wrong'] and e['low_conf'] for e in d['recalcs'])), len(rec_runs))
    out['snap_back'] = wilson(sum(1 for d in dev if d['snap_backs']), len(dev))
    pf = sum(d['post_fixes'] for d in dev) + sum(c['post_fixes'] for c in ctl)
    cor = sum(d['correct'] for d in dev) + sum(c['correct'] for c in ctl)
    wc = sum(d['wrong_conf'] for d in dev) + sum(c['wrong_conf'] for c in ctl)
    un = sum(d['uncertain'] for d in dev) + sum(c['uncertain'] for c in ctl)
    out['fix_correct'], out['fix_wrong_confident'], out['fix_uncertain'] = wilson(cor, pf), wilson(wc, pf), wilson(un, pf)
    out['fix_precision'] = wilson(cor, cor + wc)                       # of the fixes where the navigator was confident, how many were right
    out['sustained_wrong'] = wilson(sum(1 for d in dev + ctl if d['sustained_wrong']), len(dev) + len(ctl))
    out['ident_delay'] = median_ci([d['ident_delay_m'] for d in dev if d['ident_delay_m'] is not None])
    out['never_identified'] = wilson(sum(1 for d in dev if d['ident_delay_m'] is None), len(dev))
    rpf = sum(d['raw_post_fixes'] for d in dev) + sum(c['raw_post_fixes'] for c in ctl)
    out['raw_wrong_confident'] = wilson(sum(d['raw_wrong_conf'] for d in dev) + sum(c['raw_wrong_conf'] for c in ctl), rpf)
    both = dev + ctl
    out['amb_reported'] = wilson(sum(1 for d in both if d['amb_any_post']), len(both))
    out['persistent_amb'] = wilson(sum(1 for d in both if d['persistent_amb']), len(both))
    eps = [e['len_m'] for d in both for e in d['episodes'] if e['resolved']]
    out['resolve_m'] = median_ci(eps)
    reach = [r for r in recs if 'deviation' in r and r.get('oracle_reachable')]
    out['arrival'] = wilson(sum(1 for r in reach if r['deviation']['arrived']), len(reach))
    out['control_arrival'] = wilson(sum(1 for c in ctl if c['arrived']), len(ctl))
    out['unreachable_runs'] = len([r for r in recs if 'deviation' in r and not r.get('oracle_reachable')])
    return out


def run_grid(conds, seeds, seed_base=0, jobs=1):
    recs = []
    for cd in conds:
        for i in range(seeds):
            recs.append(run_one(cd, seed_base + i + 1))
    return recs


# ------------------------------------------------------------------ report -----------------------------------------------------------
CLASS_ORDER = ('clear', 'marginal', 'ambiguous')
CLASS_NOTE = {'clear': 'separation >= 3 sigma (clearly distinguishable)', 'marginal': 'separation 2-3 sigma', 'ambiguous': 'separation < 2 sigma (statistically ambiguous)'}


def failures(recs):
    """conditions with at least one failure, with the failing seeds - so every failure is reproducible"""
    out = {}
    for r in recs:
        why = []
        c, d = r.get('control'), r.get('deviation')
        if c and c['alarm_status']:
            why.append('false off-route declaration (control)')
        if c and c['reroutes']:
            why.append('false recalculation (control)')
        if d:
            if d['early_alarm']:
                why.append('alarm before leaving the route')
            if not d['detected'] and r.get('class') != 'ambiguous':
                why.append('missed deviation')
            if not d['detected'] and r.get('class') == 'ambiguous':
                why.append('missed deviation')
            if any(e['wrong'] and not e['low_conf'] for e in d['recalcs']):
                why.append('wrong-road confident recalculation')
            if d['snap_backs']:
                why.append('snap-back')
            if r.get('oracle_reachable') and not d['arrived']:
                why.append('did not arrive')
            if d['sustained_wrong']:
                why.append('sustained confident wrong road (>= %d fixes after settling)' % SUSTAINED_FIXES)
        if r.get('planning') and (r['planning']['routed'] or not r['planning']['diagnosed']):
            why.append('island destination not reported/diagnosed')
        for w in why:
            out.setdefault((r['id'], r['class'], w), []).append(r['seed'])
    return out


def render_markdown(recs, meta, seeds):
    L = ['# Offline navigation - benchmark report', '',
         'Generated by `python3 -m tesla.nav.evaluate --all --seeds %d` at commit `%s`%s, navigation-parameter hash `%s`, %d seeded runs.' % (
             seeds, meta['commit'], ' (uncommitted changes in tesla/nav)' if meta['nav_dirty'] else '', meta['params_hash'], len(recs)), '',
         '**Read this first.** Conditions are split into *clear* (wrong road >= 3 sigma from the route), *marginal* (2-3 sigma) and *ambiguous* (< 2 sigma), judged from the nominal '
         'GPS sigma, not from how the navigator performed. They are reported separately and **never combined into one success percentage**: a navigator cannot be expected to tell '
         'apart roads that are within one GPS error of each other, and averaging those in would hide how it behaves where it can. Every proportion has a Wilson 95% interval; '
         'with 30 seeds an observed 0/30 still allows a true rate up to ~11%.', '',
         'Reporting `ambiguous` instead of guessing is **not** counted as a wrong road. Persistent ambiguity (an episode longer than %d m, or still open at the end of the drive) is '
         'reported in its own column so that it cannot flatter the other numbers.' % PERSISTENT_M, '',
         'Definitions are in the module docstring of `tesla/nav/evaluate.py`. Overrides applied: %s.' % (', '.join(meta['overrides']) or 'none (defaults)'), '']
    fams = [f for f in FAMILIES if any(r['condition']['family'] == f for r in recs)]
    for fam in fams:
        fr = [r for r in recs if r['condition']['family'] == fam]
        L += ['## %s' % fam, '']
        if fam == 'island':
            n = len(fr)
            ok = sum(1 for r in fr if not r['planning']['routed'] and r['planning']['diagnosed'])
            L += ['Destination on a road component disconnected from the start: %d/%d runs reported NO ROUTE **and** diagnosed it as disconnected data %s.' % (ok, n, pct(wilson(ok, n))), '']
            continue
        L += ['| class | runs | detection | missed | false off-route (control) | false recalc (control) | early alarm | wrong-road recalc (confident) | wrong-road recalc (low-conf) | snap-back | arrival | median delay m |',
              '|---|---|---|---|---|---|---|---|---|---|---|---|']
        rows_b = []
        for cl in CLASS_ORDER:
            cr = [r for r in fr if r['class'] == cl]
            if not cr:
                continue
            s = summarise(cr)
            det = s['detection']
            missed = 'n/a' if det[0] != det[0] else pct(wilson(s['deviation_runs'] - round(det[0] * s['deviation_runs']), s['deviation_runs']))
            arr = pct(s['arrival']) if s['arrival'][0] == s['arrival'][0] else pct(s['control_arrival']) + ' (control)'
            L.append('| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (cl, s['runs'], pct(det), missed, pct(s['false_positive']), pct(s['false_recalc']),
                     pct(s['early_alarm']), pct(s['wrong_confident']), pct(s['wrong_low_conf']), pct(s['snap_back']), arr, fmt_med(s['delay'])))
            rows_b.append((cl, s))
        L += ['', '| class | fixes correct | fixes wrong-but-confident | (raw, incl. first %d m) | fixes uncertain | precision when confident | sustained wrong belief (runs) | time to correct identification m | never identified | runs reporting ambiguity | persistent ambiguity | median time to resolve m |' % SETTLE_M,
              '|---|---|---|---|---|---|---|---|---|---|---|---|']
        for cl, s in rows_b:
            L.append('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |' % (cl, pct(s['fix_correct']), pct(s['fix_wrong_confident']), pct(s['raw_wrong_confident']), pct(s['fix_uncertain']),
                     pct(s['fix_precision']), pct(s['sustained_wrong']), fmt_med(s['ident_delay']), pct(s['never_identified']), pct(s['amb_reported']), pct(s['persistent_amb']), fmt_med(s['resolve_m'])))
        L += ['', '*Class meaning: ' + '; '.join('%s = %s' % (k, v) for k, v in CLASS_NOTE.items()) + '. In the ambiguous class "runs reporting ambiguity" is the ambiguity-detection rate; in the clear class it is the false-ambiguity rate.*', '']
    # ---- remaining failure cases ----------------------------------------------------------------------------------------------------------
    fl = failures(recs)
    L += ['## Remaining failure cases', '',
          'Every condition with at least one failing run, grouped by class. `clear` failures are real defects; `marginal` and `ambiguous` failures are expected to occur at some rate and are listed so the rate can be judged. '
          'Reproduce a single run with `python3 -m tesla.nav.evaluate --family F --filter ... --seeds 1 --seed-base SEED-1 --trace`.', '']
    for cl in CLASS_ORDER:
        rows = [(k, v) for k, v in sorted(fl.items()) if k[1] == cl]
        L += ['### %s (%s)' % (cl, CLASS_NOTE[cl]), '']
        if not rows:
            L += ['none', '']
            continue
        L += ['| condition | failure | failing runs | seeds |', '|---|---|---|---|']
        for (cid, _c, why), seeds_ in rows:
            total = sum(1 for r in recs if r['id'] == cid)
            L.append('| `%s` | %s | %d/%d | %s |' % (cid, why, len(seeds_), total, ', '.join(map(str, seeds_[:8])) + (' ...' if len(seeds_) > 8 else '')))
        L.append('')
    return '\n'.join(L) + '\n'


# ------------------------------------------------------------------ CLI --------------------------------------------------------------
def _parse_filters(fs):
    out = {}
    for f in fs or []:
        k, v = f.split('=', 1)
        out[k] = v
    return out


def _match(cond, flt):
    for k, v in flt.items():
        if k == 'noise':
            ok = abs(cond.noise - float(v)) < 1e-9
        elif k == 'model':
            ok = cond.model == v
        else:
            have = cond.params.get(k)
            ok = str(have).lower() == v.lower() if not isinstance(have, (int, float)) or isinstance(have, bool) else abs(float(have) - float(v)) < 1e-9
        if not ok:
            return False
    return True


def trace_run(cond, seed):
    rec = run_one(cond, seed)
    print(json.dumps({k: v for k, v in rec.items() if k not in ('control', 'deviation')}, indent=1, default=str))
    for key in ('control', 'deviation', 'planning'):
        if key in rec:
            d = {k: v for k, v in rec[key].items() if k != 'episodes'}
            print('--', key, json.dumps(d, default=str))
            if rec[key].get('episodes'):
                print('   ambiguous episodes:', rec[key]['episodes'])


COMPARE_COLS = (('false recalc (control)', 'false_recalc'), ('wrong-road recalc (confident)', 'wrong_confident'), ('snap-back', 'snap_back'),
                ('detection', 'detection'), ('sustained wrong belief', 'sustained_wrong'), ('fixes uncertain', 'fix_uncertain'),
                ('persistent ambiguity', 'persistent_amb'), ('arrival', 'arrival'))


def compare(paths):
    """Side-by-side of several benchmark JSONs, one table per ambiguity class (never pooled across classes). Pools families WITHIN a class."""
    runs = []
    for p in paths:
        d = json.load(open(p))
        runs.append((os.path.basename(p), d['meta'], d['records']))
    L = ['# Benchmark comparison', '']
    for cl in CLASS_ORDER:
        L += ['## %s - %s' % (cl, CLASS_NOTE[cl]), '', '| variant | overrides | runs | ' + ' | '.join(c for c, _ in COMPARE_COLS) + ' | time to correct identification m |',
              '|---|---|---|' + '---|' * (len(COMPARE_COLS) + 1)]
        for name, meta, recs in runs:
            cr = [r for r in recs if r['class'] == cl and 'planning' not in r]
            if not cr:
                continue
            s = summarise(cr)
            cells = []
            for _c, key in COMPARE_COLS:
                t = s[key]
                cells.append(pct(t) if t[0] == t[0] else 'n/a')
            L.append('| %s | %s | %d | %s | %s |' % (name, ', '.join(meta['overrides']) or 'defaults', len(cr), ' | '.join(cells), fmt_med(s['ident_delay'])))
        L.append('')
    return '\n'.join(L) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--all', action='store_true'); ap.add_argument('--quick', action='store_true', help='reduced grid (used by the test-suite)')
    ap.add_argument('--family', choices=FAMILIES); ap.add_argument('--filter', action='append', metavar='KEY=VALUE')
    ap.add_argument('--seeds', type=int, default=30); ap.add_argument('--seed-base', type=int, default=0)
    ap.add_argument('--override', action='append', default=[], metavar='MODULE.CONST=VALUE')
    ap.add_argument('--json'); ap.add_argument('--md'); ap.add_argument('--trace', action='store_true')
    ap.add_argument('--compare', nargs='+', metavar='RESULT.json')
    a = ap.parse_args(argv)
    if a.compare:
        out = compare(a.compare)
        if a.md:
            open(a.md, 'w').write(out)
        else:
            print(out)
        return 0
    apply_overrides(a.override)
    conds = quick_grid() if a.quick else full_grid()
    if a.family:
        conds = [c for c in conds if c.family == a.family]
    conds = [c for c in conds if _match(c, _parse_filters(a.filter))]
    if not (a.all or a.quick or a.family):
        ap.error('choose --all, --quick or --family')
    if not conds:
        ap.error('no condition matches')
    meta = version_info(a.override)
    if a.trace:
        for c in conds:
            for i in range(a.seeds):
                print('=' * 20, c.id, 'seed', a.seed_base + i + 1)
                trace_run(c, a.seed_base + i + 1)
        return 0
    t0 = time.time()
    recs = run_grid(conds, a.seeds, a.seed_base)
    for r in recs:
        r['version'] = {'commit': meta['commit'], 'params_hash': meta['params_hash']}
    sys.stderr.write('%d conditions x %d seeds = %d runs in %.1fs\n' % (len(conds), a.seeds, len(recs), time.time() - t0))
    if a.json:
        with open(a.json, 'w') as f:
            json.dump({'meta': meta, 'seeds': a.seeds, 'seed_base': a.seed_base, 'records': recs}, f, indent=1, default=str)
    md = render_markdown(recs, meta, a.seeds)
    if a.md:
        with open(a.md, 'w') as f:
            f.write(md)
    else:
        print(md)
    return 0


if __name__ == '__main__':
    sys.exit(main())
