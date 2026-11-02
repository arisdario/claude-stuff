#!/usr/bin/env python3
"""KNOWN, UNFIXED defects found by the benchmark (tesla/nav/BENCHMARK.md), pinned as expected failures on the exact seeds that fail. Each test asserts the CORRECT
behaviour. While the defect exists the test is an expected failure and the suite stays green; when the defect is fixed the test starts passing, unittest reports an
'unexpected success', and the @expectedFailure marker must be removed so the test becomes an ordinary regression test.
    python3 -m unittest tesla.tests.test_nav_known_defects
Reproduce any of them by hand with the command in each docstring."""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import evaluate as E  # noqa: E402


class TestKnownDefects(unittest.TestCase):
    @unittest.expectedFailure
    def test_a_three_second_gps_jump_must_not_recalculate_the_route(self):
        """DEFECT D1: three consecutive fixes displaced 80 m (a 3 s multipath excursion) trigger a RECALCULATION in 30/30 runs - exactly at the 3-fix off-route
        minimum - and the recalculation plans from the displaced position (the tracker is lost there). 1-2 fix jumps and dropouts up to 30 s are handled (0/30).
          python3 -m tesla.nav.evaluate --family gps_events --filter kind=jump --filter n=3 --filter noise=2 --seeds 1 --trace"""
        c = E.Condition('gps_events', {'sep': 20, 'kind': 'jump', 'n': 3}, 2, 'gauss')
        bad = [s for s in (1, 2, 3, 4, 5) if E.run_one(c, s)['control']['reroutes']]
        self.assertEqual(bad, [], 'seeds %s recalculated an on-route drive after a 3-fix GPS jump' % bad)

    @unittest.expectedFailure
    def test_a_lost_track_must_not_recalculate_confidently_onto_a_nearby_road(self):
        """DEFECT D2: when the GPS jumps onto a road component that cannot reach the destination, the lost-track fallback plans from COORDINATES, snaps to the nearest road
        20 m away (the original route) and recalculates with normal confidence (snap-back, 4/30 at 5 m noise, 3/30 at 8 m). The 'contested' guard only covers known tracks.
          python3 -m tesla.nav.evaluate --family teleport_island --filter noise=5 --seeds 1 --seed-base 4 --trace"""
        c = E.Condition('teleport_island', {'sep': 20}, 5, 'gauss')
        bad = [s for s in (5, 17, 28, 29) if any(e['snap_back'] and not e['low_conf'] for e in E.run_one(c, s)['deviation']['recalcs'])]
        self.assertEqual(bad, [], 'seeds %s recalculated confidently onto the wrong road' % bad)

    @unittest.expectedFailure
    def test_a_second_deviation_onto_a_close_side_road_must_be_identified_promptly(self):
        """DEFECT D3: in the SECOND deviation (onto a side road 8 m from Main, 2 m GPS noise - clearly distinguishable, 4 sigma) the navigator stays confidently on Main for
        60-90 m (14/30 runs sustain a confident wrong belief), although a single deviation at the same separation and noise is identified correctly in 29/30 runs.
        OBSERVED: the symptom, from traces. CAUSE NOT VERIFIED. Two candidates, neither tested: (a) CUSUM evidence is kept per EDGE, so crossing from the side road's first
        (18 m) edge to its second discards the evidence gathered; (b) the evidence rate at 8 m with the fixed sigma of 6 m is only ~0.9 per fix, i.e. ~5 fixes (75 m) to reach
        SWITCH_LLR - but that alone would not explain why the second deviation is so much worse than the first, so something specific to a track created by a recalculation
        is also suspected.
          python3 -m tesla.nav.evaluate --family repeated --filter sep=8 --filter noise=2 --seeds 1 --seed-base 4 --trace"""
        c = E.Condition('repeated', {'sep': 8}, 2, 'gauss')
        bad = [s for s in (5, 7, 9, 12, 13) if E.run_one(c, s)['deviation']['sustained_wrong']]
        self.assertEqual(bad, [], 'seeds %s held a confident wrong road for >= 3 fixes' % bad)

    @unittest.expectedFailure
    def test_a_near_tie_junction_must_not_cause_a_false_recalculation(self):
        """DEFECT D4 (not yet diagnosed): on the 'diamond' (two roads that split and merge) a control drive along the planned road is declared off-route / recalculated in
        clear-class conditions (8 m separation, 2 m noise: 2/30 and 1/30; seed 12). Cause not investigated.
          python3 -m tesla.nav.evaluate --family diamond --filter sep=8 --filter noise=2 --seeds 1 --seed-base 11 --trace"""
        c = E.Condition('diamond', {'sep': 8}, 2, 'gauss')
        bad = [s for s in (12, 24) if E.run_one(c, s)['control']['alarm_status']]
        self.assertEqual(bad, [], 'seeds %s were declared off-route while on the route' % bad)

    @unittest.expectedFailure
    def test_a_close_parallel_road_with_good_gps_must_never_be_missed(self):
        """DEFECT D5 (not yet diagnosed): at 8 m separation and 2 m noise the deviation is never reported in 1 of 30 runs (seed 2) in parallel and ladder scenarios.
          python3 -m tesla.nav.evaluate --family parallel --filter sep=8 --filter noise=2 --filter connected=false --seeds 1 --seed-base 1 --trace"""
        c = E.Condition('parallel', {'sep': 8, 'connected': False}, 2, 'gauss')
        self.assertTrue(E.run_one(c, 2)['deviation']['detected'])


if __name__ == '__main__':
    unittest.main()
