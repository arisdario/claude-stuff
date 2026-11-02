#!/usr/bin/env python3
"""Permanent regression tests for every EdgeTracker / Navigator defect found while validating wrong-road recalculation. Each test names the defect and
fails on the code that had it.   python3 -m unittest tesla.tests.test_nav_tracker_regressions"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from nav import scenarios as S, tracker as TK, matcher as M, router as RT, validate as V, geo  # noqa: E402


def par_chain(g, sc):
    e = next(x for x in g.out_edges(sc.leave) if x.name == 'Par')
    chain = [e]
    while True:
        nxt = [x for x in g.out_edges(chain[-1].v) if x.name == 'Par' and not (x.u == chain[-1].v and x.v == chain[-1].u)]
        if not nxt:
            return chain
        chain.append(nxt[0])


def twin(g, e):
    return next(x for x in g.out_edges(e.v) if x.v == e.u and x.way_id == e.way_id and x.rev != e.rev)


def main_east(g, node):
    return next(x for x in g.out_edges(node) if x.name == 'Main' and x.rev == 0)


class Base(unittest.TestCase):
    def setUp(self):
        self.sc = S.parallel(20, True)
        self.g = self.sc.graph
        self.par = par_chain(self.g, self.sc)               # p0 is 44.7 m (the diagonal), then 100 m edges
        self.J = self.sc.leave


class TestReachabilityWindow(Base):
    def test_junction_beyond_the_window_is_not_a_source(self):
        """DEFECT: the backward walk pushed the first node beyond BEHIND_M as a source before checking the bound, so a junction 150-250 m behind still made the
        road it left look reachable (Main became 'connected' to Par and the tracker could flip onto it)."""
        p2 = self.par[2]                                   # starts 144.7 m from J
        beyond = EdgeTracker = TK.EdgeTracker(self.g, p2, along=TK.BEHIND_M - 144.7 + 10.0)    # J is 10 m beyond the window
        mains = {e.eid for n in range(1001, 1011) for e in self.g.out_edges(n) if e.name == 'Main'}
        self.assertFalse(beyond.reachable(60.0) & mains, 'Main edges are reachable through a junction outside the window')
        inside = TK.EdgeTracker(self.g, p2, along=TK.BEHIND_M - 144.7 - 10.0)                 # J is 10 m inside the window
        self.assertTrue(inside.reachable(60.0) & mains, 'a junction inside the window must still offer its other exits')


class TestNoReversalThroughHypotheticalPasts(Base):
    def test_twin_of_walked_edges_is_never_reachable(self):
        """DEFECT: the walk back used every in-edge, including the TWIN of the current edge, so the car's own forward node counted as 'behind' it; from there the
        closure drove back along Par's westbound twin to J and forward onto Main - a U-turn on the spot that the no-reversal rule forbids everywhere else."""
        p0, p1 = self.par[0], self.par[1]
        tk = TK.EdgeTracker(self.g, p1, along=20.0)
        reach = tk.reachable(60.0)
        self.assertNotIn(twin(self.g, p1).eid, reach)       # the opposite carriageway of the road it is on
        self.assertNotIn(twin(self.g, p0).eid, reach)       # back along the road it just walked

    def test_sources_carry_their_own_arrival_edge(self):
        """DEFECT (introduced while fixing the one above): excluding 'anything leading back along a hypothetical past' also excluded Par's FIRST edge from a track
        on Main, because Par's twin made 5001 a 'predecessor' of J. The continuation of a branch must stay reachable from a track on the road it left."""
        main_next = main_east(self.g, self.J)
        tk = TK.EdgeTracker(self.g, main_next, along=55.0)
        reach = tk.reachable(60.0)
        self.assertIn(self.par[0].eid, reach)
        self.assertIn(self.par[1].eid, reach)


class TestSameWayBaseline(Base):
    def test_passing_an_edge_boundary_is_not_evidence_for_a_road_beside_it(self):
        """DEFECT: the evidence baseline was the distance to the CURRENT edge only. When the car passed that edge's end the distance jumped (23 m), inflating the
        evidence for every candidate at once, and a noisy fix 9.4 m from Main (20 m away) beat Par's own continuation 10.6 m away by 6.4 to 6.0 in a single fix."""
        old = TK.BEHIND_M
        TK.BEHIND_M = 600.0                                 # make Main reachable so the baseline logic itself is what is tested
        try:
            p2, p3 = self.par[2], self.par[3]
            tk = TK.EdgeTracker(self.g, p2, along=99.0)
            gm = self.g.geom(p3.eid)
            m_lat = (3.14159265358979 / 180.0) * geo.R_EARTH
            m_lon = m_lat * __import__('math').cos(__import__('math').radians(gm[0][0]))
            # the real failing trace: the car has just passed the end of p2 (21 m into p3) and a noisy fix lies 9.4 m from Main but 10.6 m from Par (Par is 20 m north of Main)
            lat = gm[0][0] - 10.6 / m_lat
            lon = gm[0][1] + 21.0 / m_lon
            tk.update(lat, lon, 90.0, 15.0)
            self.assertEqual(tk.edge.eid, p3.eid, 'the tracker left Par for Main on one noisy fix at an edge boundary')
        finally:
            TK.BEHIND_M = old


class TestIrreversibility(Base):
    def test_a_wrong_track_on_an_unconnected_road_is_recovered(self):
        """DEFECT: connectivity made a wrong assignment permanent. Once on Main, Par is not connected to it, so no evidence about Par was ever collected
        (llr == {}) while the car drove 20 m from the tracked edge, and the navigator then recalculated from the wrong road."""
        M4 = 1001 + 4
        wrong = main_east(self.g, M4)
        tk = TK.EdgeTracker(self.g, wrong, along=50.0)
        par_ids = {e.eid for e in self.par}
        self.assertFalse(tk.reachable(60.0) & par_ids, 'precondition: Par must not be connected to this track')
        east = 450.0
        relocated_at = None
        for i in range(4):
            lat, lon = S.at(20.0, east + 15.0 * i)          # exactly on Par, 20 m from the tracked Main edge
            tk.update(lat, lon, 90.0, 15.0)
            if tk.jumped and relocated_at is None:
                relocated_at = i
        self.assertIsNotNone(relocated_at, 'the track never recovered')
        self.assertLessEqual(relocated_at, 2)
        self.assertEqual(tk.edge.name, 'Par')


class TestContestedTrackIsNotRecalculatedFrom(Base):
    def drive(self, force_contested):
        sc = self.sc
        R0 = RT.find_route(self.g, sc.start, sc.dest)
        edges = [next(e for e in self.g.out_edges(a) if e.v == b) for a, b in zip(sc.driven, sc.driven[1:])]
        pts = []
        for e in edges:
            gm = self.g.geom(e.eid)
            pts.extend(gm[1:] if pts and pts[-1] == gm[0] else gm)
        real = TK.EdgeTracker.update
        def contested(self_, *a, **k):
            r = real(self_, *a, **k)
            if force_contested and self_.known:
                self_.contested = True
            return r
        TK.EdgeTracker.update = contested
        try:
            nav = M.Navigator(self.g, sc.dest)
            nav.plan(R0.points[0][0], R0.points[0][1], None)
            states = [nav.update(*f) for f in V.fixes_along(pts, 15.0, noise_m=2.0, seed=3)]
        finally:
            TK.EdgeTracker.update = real
        return nav, states

    def test_contested_defers_then_recalculates_with_low_confidence(self):
        """DEFECT: the navigator recalculated from the tracked edge even while the tracker held contrary evidence - confident guidance from a guess."""
        nav, states = self.drive(True)
        amb = [s for s in states if s.ambiguous]
        self.assertEqual(len(amb), M.AMBIG_GIVE_UP_FIXES)                       # deferred for exactly the give-up budget
        self.assertTrue(all(s.status == 'off_route' for s in amb))
        rer = [s for s in states if s.status == 'rerouted']
        self.assertTrue(rer)
        self.assertEqual(rer[0].confidence, 'low')                              # and when it finally recalculates it says so
        self.assertEqual(nav.reroutes, 1)

    def test_uncontested_recalculation_is_prompt_and_normal_confidence(self):
        nav, states = self.drive(False)
        self.assertFalse([s for s in states if s.ambiguous])
        rer = [s for s in states if s.status == 'rerouted']
        self.assertTrue(rer)
        self.assertEqual(rer[0].confidence, 'normal')


if __name__ == '__main__':
    unittest.main()
