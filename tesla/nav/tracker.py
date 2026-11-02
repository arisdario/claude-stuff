"""Edge tracker: follows WHICH directed edge the car is on, using graph connectivity as the continuity constraint and accumulated
evidence (not single fixes) to decide when it has moved to another edge.

Why: matching a fix to the route polyline by cross-track distance alone cannot tell the route from a road running 15-25 m beside it
(frontage road, dual carriageway, the other leg of a switchback), and re-snapping to 'the nearest edge' after leaving the route would
happily land back on the original road.

1. CONNECTIVITY: the car may only move to an edge CONNECTED to the one it was on - reachable through shared OSM nodes within the
   distance it could have driven, including the exits of junctions it passed in the last BEHIND_M. A road with no shared node to the
   track is never selected, however close it is.
2. EVIDENCE: for every connected candidate the tracker accumulates a CUSUM of the Gaussian log-likelihood ratio against the current
   edge, sum of (d_cur^2 - d_cand^2) / (2 sigma^2) (+ a heading term), floored at 0. It switches only when that exceeds SWITCH_LLR.
   Two roads 8 m apart cannot be told apart by one fix with 5 m GPS noise, but a few fixes can: a car truly on A drives B's evidence
   strongly negative (it can never accumulate), a car truly on B accumulates it within a few fixes. 20 m apart switches at once.

When no connected edge explains a fix the tracker reports known=False (it does not guess); after LOST_FIXES such fixes in a row it
re-localises to the nearest heading-consistent edge, still known=False, until the track connects again. Advisory only."""
import heapq
import itertools

BEHIND_M = 150.0           # junctions this far behind the track still offer their exits as candidates: a road that leaves at a
                           # shallow angle only separates from the route by more than GPS noise some distance after the junction
LOST_FIXES = 3
MAX_SNAP_M = 35.0
HEADING_MAX_DEG = 70.0
SIGMA_POS_M = 6.0          # GPS error (1 sigma) assumed in the likelihood ratio. Using each fix's REPORTED accuracy instead was tried and measured
                           # WORSE (19 vs 6 false alarms per 144 hard runs, plus a snap-back): a larger sigma shrinks the evidence per fix, so a
                           # wrong switch takes longer to correct and the off-route logic sees it for longer. Do not 'fix' this without re-measuring.
SIGMA_HEAD_DEG = 30.0
SWITCH_LLR = 4.0           # ~ 55:1 odds (6.0 measured worse: slower to correct a wrong switch)
LLR_CAP = 20.0
MAX_BUDGET_M = 3000.0


class EdgeTracker(object):
    def __init__(self, graph, edge, along=0.0):
        self.graph, self.edge, self.along = graph, edge, along
        self.known = True
        self.lost = 0
        self.llr = {}              # candidate edge id -> accumulated evidence that the car moved onto it

    def reachable(self, budget_m):
        """Edge ids the car can be on now: every edge leaving a node the car could have passed - the end of the current edge (ahead by the
        rest of the edge) and the start nodes of the current edge and of its predecessors back to BEHIND_M (so the exits of a junction
        just passed are still candidates) - followed forward through shared nodes while within budget_m of driving, with no immediate
        reversal. Connectivity only: a road with no shared node to the track is never in this set, however close it is."""
        g, e0 = self.graph, self.edge
        tick = itertools.count()
        reach = {e0.eid}
        heap = [(max(0.0, e0.length_m - self.along), next(tick), e0.v, e0)]    # (distance from the car to the node, tiebreak, node, arrival edge)
        seen_back = {e0.u}
        back = [(-self.along, e0.u)]                                           # nodes behind the car, as negative distances
        while back:
            d, node = back.pop()
            heapq.heappush(heap, (d, next(tick), node, None))
            if -d >= BEHIND_M:
                continue
            for p in g.in_edges(node):
                if p.u not in seen_back:
                    seen_back.add(p.u)
                    back.append((d - p.length_m, p.u))
        done = set()
        while heap:
            d, _, node, arrived = heapq.heappop(heap)
            key = (node, arrived.eid if arrived else None)
            if d > budget_m or key in done:
                continue
            done.add(key)
            for n in g.out_edges(node):
                if arrived is not None and n.u == arrived.v and n.v == arrived.u:
                    continue                        # no immediate reversal
                reach.add(n.eid)
                heapq.heappush(heap, (d + n.length_m, next(tick), n.v, n))
        return reach

    def update(self, lat, lon, heading, traveled_m):
        """Advance the track with one fix. traveled_m = how far the car may have driven since the last fix (speed * dt)."""
        g = self.graph
        sigma = SIGMA_POS_M
        budget = min(MAX_BUDGET_M, max(60.0, 1.5 * traveled_m + 40.0))
        cands = [c for c in g.snap(lat, lon, heading, MAX_SNAP_M, limit=16) if c.heading_err is None or c.heading_err <= HEADING_MAX_DEG]
        reach = self.reachable(budget)
        conn = {c.edge.eid: c for c in cands if c.edge.eid in reach}
        d_cur, along_cur, _pl, _pn, h_cur = g.measure(self.edge.eid, lat, lon, heading)
        if not conn and d_cur > MAX_SNAP_M:
            # nothing connected explains this fix and we are nowhere near the current edge: do not guess
            self.lost += 1
            self.known = False
            if cands and self.lost >= LOST_FIXES:
                best = min(cands, key=lambda c: c.dist_m + 0.3 * (c.heading_err or 0.0))
                self.edge, self.along, self.lost, self.llr = best.edge, best.along_m, 0, {}
            return self
        self.lost, self.known = 0, True
        # accumulate evidence that the car is on each connected candidate rather than on the current edge
        new = {}
        hc = min(h_cur, 90.0) if h_cur is not None else None
        for eid, c in conn.items():
            if eid == self.edge.eid:
                continue
            delta = (d_cur ** 2 - c.dist_m ** 2) / (2.0 * sigma ** 2)
            if hc is not None and c.heading_err is not None:
                delta += (hc ** 2 - c.heading_err ** 2) / (2.0 * SIGMA_HEAD_DEG ** 2)
            new[eid] = min(LLR_CAP, max(0.0, self.llr.get(eid, 0.0) + delta))        # CUSUM: evidence that stops being supported is lost
        self.llr = {k: v for k, v in new.items() if v > 0.0}
        if self.llr:
            eid, best = max(self.llr.items(), key=lambda kv: kv[1])
            if best >= SWITCH_LLR:
                c = conn[eid]
                self.edge, self.along, self.llr = c.edge, c.along_m, {}
                return self
        self.along = along_cur
        return self
