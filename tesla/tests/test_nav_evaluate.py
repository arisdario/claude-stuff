#!/usr/bin/env python3
"""Tests for the benchmark harness itself (nav/evaluate.py, nav/scenarios.py): reproducibility, noise models, ambiguity accounting, statistics, coverage.
They test the INSTRUMENT, not the navigator's score.   python3 -m unittest tesla.tests.test_nav_evaluate"""
import contextlib
import io
import json
import math
import os
import random
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import evaluate as E, scenarios as S, validate as V, geo  # noqa: E402


def cond(family='parallel', noise=5, model='gauss', **p):
    defaults = {'parallel': {'sep': 20, 'connected': True}, 'fork': {'angle': 30, 'merge': True}, 'ladder': {'sep': 20}, 'repeated': {'sep': 20},
                'diamond': {'sep': 20}, 'oneway': {'sep': 20, 'link': 'out'}, 'gps_events': {'sep': 20, 'kind': 'jump', 'n': 2},
                'island': {'sep': 20}, 'teleport_island': {'sep': 20}}[family]
    d = dict(defaults); d.update(p)
    return E.Condition(family, d, noise, model)


def canon(rec):
    return json.dumps(rec, sort_keys=True, default=str)


class TestReproducibility(unittest.TestCase):
    def test_same_seed_same_result_for_every_family(self):
        for fam in E.FAMILIES:
            c = cond(fam)
            self.assertEqual(canon(E.run_one(c, 7)), canon(E.run_one(c, 7)), fam)

    def test_different_seeds_differ(self):
        c = cond('parallel', noise=8, connected=False)
        self.assertNotEqual(canon(E.run_one(c, 1)), canon(E.run_one(c, 2)))

    def test_a_run_does_not_depend_on_what_ran_before_it(self):
        c, other = cond('parallel', noise=5), cond('fork', noise=8)
        solo = canon(E.run_one(c, 5))
        E.run_one(other, 3); E.run_one(c, 99)
        self.assertEqual(solo, canon(E.run_one(c, 5)))

    def test_record_carries_seed_parameters_noise_model_and_version(self):
        out = os.path.join(tempfile.mkdtemp(), 'b.json')
        with contextlib.redirect_stderr(io.StringIO()):
            E.main(['--family', 'parallel', '--filter', 'sep=20', '--filter', 'noise=5', '--filter', 'model=ar1', '--filter', 'connected=true', '--seeds', '2', '--seed-base', '10', '--json', out, '--md', out + '.md'])
        d = json.load(open(out))
        self.assertEqual({r['seed'] for r in d['records']}, {11, 12})
        r = d['records'][0]
        self.assertEqual(r['condition'], {'family': 'parallel', 'params': {'sep': 20, 'connected': True}, 'noise': 5.0, 'noise_model': 'ar1'})
        self.assertTrue(r['version']['commit']); self.assertEqual(len(r['version']['params_hash']), 12)
        self.assertIn('tracker.SWITCH_LLR', d['meta']['params'])

    def test_cli_runs_are_identical_twice(self):
        outs = []
        for _ in range(2):
            out = os.path.join(tempfile.mkdtemp(), 'b.json')
            with contextlib.redirect_stderr(io.StringIO()):
                E.main(['--family', 'fork', '--filter', 'angle=30', '--filter', 'noise=5', '--seeds', '3', '--json', out, '--md', out + '.md'])
            outs.append(json.load(open(out))['records'])
        self.assertEqual(canon(outs[0]), canon(outs[1]))

    def test_single_run_trace_mode(self):
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            E.main(['--family', 'parallel', '--filter', 'sep=20', '--filter', 'noise=5', '--filter', 'connected=true', '--filter', 'model=gauss', '--seeds', '1', '--seed-base', '16', '--trace'])
        self.assertIn('seed 17', buf.getvalue()); self.assertIn('-- deviation', buf.getvalue())

    def test_overrides_change_the_recorded_parameter_hash_but_defaults_are_untouched(self):
        before = E.version_info()['params_hash']
        import nav.tracker as TK
        old = TK.SWITCH_LLR
        try:
            E.apply_overrides(['tracker.SWITCH_LLR=6'])
            self.assertNotEqual(E.version_info(['tracker.SWITCH_LLR=6'])['params_hash'], before)
        finally:
            TK.SWITCH_LLR = old
        self.assertEqual(E.version_info()['params_hash'], before)

    def test_unknown_override_is_rejected(self):
        with self.assertRaises(SystemExit):
            E.apply_overrides(['tracker.NO_SUCH_CONSTANT=1'])


class TestNoiseModels(unittest.TestCase):
    def samples(self, model, n=6000, sigma=5.0):
        f = V._noise_sampler(model, sigma, random.Random(1))
        return [f() for _ in range(n)]

    def test_deterministic_and_default_unchanged(self):
        pts = [(45.5, 25.5), (45.5, 25.51)]
        a = V.fixes_along(pts, seed=3)
        b = V.fixes_along(pts, seed=3, noise_model='gauss')
        self.assertEqual(a, b)
        for m in V.NOISE_MODELS:
            self.assertEqual(V.fixes_along(pts, seed=3, noise_model=m), V.fixes_along(pts, seed=3, noise_model=m))

    def test_gauss_scale(self):
        xs = [n for n, _ in self.samples('gauss')]
        sd = math.sqrt(sum(x * x for x in xs) / len(xs))
        self.assertAlmostEqual(sd, 5.0, delta=0.3)

    def test_ar1_is_temporally_correlated_with_the_right_marginal(self):
        xs = [n for n, _ in self.samples('ar1', 8000)]
        m = sum(xs) / len(xs)
        var = sum((x - m) ** 2 for x in xs) / len(xs)
        ac = sum((xs[i] - m) * (xs[i + 1] - m) for i in range(len(xs) - 1)) / (len(xs) - 1) / var
        self.assertAlmostEqual(ac, 0.9, delta=0.05)
        self.assertAlmostEqual(math.sqrt(var), 5.0, delta=0.8)

    def test_heavy_tailed_has_excess_kurtosis(self):
        xs = [n for n, _ in self.samples('heavy', 20000)]
        m = sum(xs) / len(xs)
        v = sum((x - m) ** 2 for x in xs) / len(xs)
        kurt = sum((x - m) ** 4 for x in xs) / len(xs) / v ** 2
        self.assertGreater(kurt, 4.0)                                        # a Gaussian is 3

    def test_uniform_is_bounded(self):
        w = math.sqrt(3.0) * 5.0
        self.assertTrue(all(abs(n) <= w and abs(e) <= w for n, e in self.samples('uniform')))

    def test_unknown_model_rejected(self):
        with self.assertRaises(ValueError):
            V._noise_sampler('pink', 5.0, random.Random(1))

    def test_sampling_phase_shifts_the_first_fix(self):
        pts = [(45.5, 25.5), (45.5, 25.51)]
        a = V.fixes_along(pts, noise_m=0.0, seed=1, s0=0.0)
        b = V.fixes_along(pts, noise_m=0.0, seed=1, s0=7.5)
        self.assertGreater(geo.hav(a[0][1], a[0][2], b[0][1], b[0][2]), 7.0)


class TestStatistics(unittest.TestCase):
    def test_wilson_intervals(self):
        p, lo, hi = E.wilson(0, 30)
        self.assertEqual(p, 0.0); self.assertEqual(lo, 0.0); self.assertAlmostEqual(hi, 0.114, delta=0.005)     # '0/30' still allows ~11%
        p, lo, hi = E.wilson(30, 30)
        self.assertAlmostEqual(lo, 0.886, delta=0.005); self.assertEqual(hi, 1.0)
        p, lo, hi = E.wilson(15, 30)
        self.assertAlmostEqual(p, 0.5); self.assertLess(lo, 0.5); self.assertGreater(hi, 0.5)
        self.assertTrue(math.isnan(E.wilson(0, 0)[0]))

    def test_median_ci_brackets_the_median_and_is_deterministic(self):
        v = [10, 20, 30, 40, 50, 60, 70]
        m, lo, hi = E.median_ci(v)
        self.assertEqual(m, 40); self.assertLessEqual(lo, 40); self.assertGreaterEqual(hi, 40)
        self.assertEqual(E.median_ci(v), E.median_ci(v))
        self.assertIsNone(E.median_ci([]))


class TestAmbiguityAccounting(unittest.TestCase):
    """'uncertain' must never count as a wrong road; persistent ambiguity must be visible on its own."""
    def run_with(self, tracker_state, n_fixes=40):
        """drive an easy on-route network but force the tracker to a given (known, contested, on_true) state every fix"""
        import nav.tracker as TK
        c = cond('parallel', noise=2, connected=True)
        real = TK.EdgeTracker.update
        def forced(self_, *a, **k):
            r = real(self_, *a, **k)
            known, contested = tracker_state
            self_.known, self_.contested = known, contested
            return r
        TK.EdgeTracker.update = forced
        try:
            return E.run_one(c, 3)
        finally:
            TK.EdgeTracker.update = real

    def test_uncertain_fixes_are_not_wrong_but_confident(self):
        rec = self.run_with((True, True))                                    # always contested
        c = rec['control']
        self.assertGreater(c['uncertain'], 0)
        self.assertEqual(c['wrong_conf'], 0)
        self.assertEqual(c['correct'], 0)                                    # and they are not 'correct' either

    def test_lost_track_counts_as_uncertain(self):
        c = self.run_with((False, False))['control']
        self.assertEqual(c['wrong_conf'], 0); self.assertGreater(c['uncertain'], 0)

    def test_persistent_ambiguity_is_reported_separately(self):
        c = self.run_with((True, True))['control']
        self.assertTrue(c['persistent_amb'])                                 # an episode that never resolves cannot hide
        self.assertFalse(c['episodes'][-1]['resolved'])
        s = E.summarise([self.run_with((True, True))])
        self.assertGreater(s['persistent_amb'][0], 0.99)

    def test_confident_correct_run_has_no_ambiguity(self):
        c = E.run_one(cond('parallel', noise=2, connected=True, sep=35), 3)['control']
        self.assertEqual(c['wrong_conf'], 0)
        self.assertGreater(c['correct'], c['uncertain'])
        self.assertFalse(c['persistent_amb'])

    def test_unsettled_counts_are_kept_next_to_the_settled_ones(self):
        rec = E.run_one(cond('parallel', noise=2, connected=False, sep=35), 1)['deviation']
        self.assertGreaterEqual(rec['raw_post_fixes'], rec['post_fixes'])
        self.assertGreater(rec['raw_post_fixes'] - rec['post_fixes'], 0)     # the first SETTLE_M of the departure is excluded from the settled rates, not discarded
        self.assertIsNotNone(rec['ident_delay_m'])                           # and the latency itself is measured

    def test_ambiguity_classes_come_from_the_nominal_sigma_not_from_performance(self):
        sc = S.parallel(12, True)
        self.assertEqual(sc.ambiguity_class(8), 'ambiguous')                 # 1.5 sigma
        self.assertEqual(sc.ambiguity_class(5), 'marginal')                  # 2.4 sigma
        self.assertEqual(sc.ambiguity_class(2), 'clear')                     # 6 sigma
        self.assertEqual(S.island(20).ambiguity_class(8), 'clear')           # no parallel road at all

    def test_report_never_pools_classes_into_one_success_rate(self):
        recs = [E.run_one(c, s) for c in (cond('parallel', noise=2, sep=35), cond('parallel', noise=8, sep=8)) for s in (1, 2)]
        md = E.render_markdown(recs, E.version_info(), 2)
        for cl in ('clear', 'ambiguous'):
            self.assertIn('| %s |' % cl, md)
        self.assertNotIn('overall', md.lower().replace('read this first', ''))
        self.assertIn('never combined', md)


class TestCoverage(unittest.TestCase):
    def test_grid_covers_every_required_condition(self):
        g = E.full_grid()
        fams = {c.family for c in g}
        self.assertTrue(set(E.FAMILIES) <= fams)
        par = [c for c in g if c.family == 'parallel']
        self.assertGreaterEqual(len({c.params['sep'] for c in par}), 5)                       # parallel roads at different separations
        self.assertTrue({'gauss', 'ar1', 'heavy', 'uniform'} <= {c.model for c in par})       # different noise distributions
        self.assertGreaterEqual(len({c.noise for c in par}), 3)
        self.assertTrue({True, False} <= {c.params['connected'] for c in par})                # connected and disconnected
        self.assertTrue({True, False} <= {c.params['merge'] for c in g if c.family == 'fork'})  # merges and forks
        self.assertTrue(any(c.family == 'fork' and c.params['angle'] <= 10 for c in g))       # recovery after ambiguous matching (a slow divergence)
        self.assertTrue(any(c.family == 'parallel' and c.params['sep'] <= 5 for c in g))      # near-tie candidates
        self.assertTrue(any(c.family == 'diamond' and c.params['sep'] <= 5 for c in g))       # near tie at a junction with several predecessors
        self.assertTrue({'jump', 'dropout'} <= {c.params['kind'] for c in g if c.family == 'gps_events'})
        self.assertTrue({'into', 'out'} <= {c.params['link'] for c in g if c.family == 'oneway'})
        for c in g:
            S_ = E.build_scenario(c, 1)
            self.assertIsNotNone(S_.graph)

    def test_every_class_is_populated(self):
        classes = {E.build_scenario(c).ambiguity_class(c.noise) for c in E.full_grid()}
        self.assertEqual(classes, {'clear', 'marginal', 'ambiguous'})

    def test_every_family_runs_and_yields_the_metrics_the_report_needs(self):
        for fam in E.FAMILIES:
            rec = E.run_one(cond(fam), 4)
            if fam == 'island':
                self.assertIn('planning', rec)
                continue
            self.assertTrue('control' in rec or 'deviation' in rec, fam)
            for key in ('control', 'deviation'):
                if key in rec:
                    for need in ('detected', 'recalcs', 'snap_backs', 'correct', 'wrong_conf', 'uncertain', 'episodes', 'persistent_amb', 'arrived'):
                        self.assertIn(need, rec[key], (fam, key))

    def test_island_destination_is_reported_and_diagnosed(self):
        rec = E.run_one(cond('island'), 1)
        self.assertFalse(rec['planning']['routed']); self.assertTrue(rec['planning']['diagnosed'])


if __name__ == '__main__':
    unittest.main()
