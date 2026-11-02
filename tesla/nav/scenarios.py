"""Synthetic road networks with GROUND TRUTH for evaluating the navigator. Every scenario knows the route the planner will choose, the path the
simulated driver ACTUALLY drives (which may leave that route), where it leaves it, and which edge the car is really on at every metre - so a
recalculation can be judged against the truth, not against the navigator's own opinion. Networks are built through the real graph builder.

Geometry is metres north/east of a fixed origin. Nothing here touches vehicle control."""
import math
import os
import tempfile

try:
    from . import geo
    from .routegraph import build_from_edges, Graph
except ImportError:
    import geo
    from routegraph import build_from_edges, Graph

LAT0, LON0 = 45.50, 25.57
_M_LAT = (math.pi / 180.0) * geo.R_EARTH
_M_LON = _M_LAT * math.cos(math.radians(LAT0))


def ll(n, e):
    return (LON0 + e / _M_LON, LAT0 + n / _M_LAT)        # (lon, lat)


def at(n, e):
    return (LAT0 + n / _M_LAT, LON0 + e / _M_LON)        # (lat, lon)


class Net(object):
    def __init__(self):
        self.edges, self.wid, self._nid = [], 0, 1000

    def node(self):
        self._nid += 1
        return self._nid

    def way(self, name, nodes, **tags):
        """nodes: [(node_id, north, east), ...]"""
        self.wid += 1
        t = {'highway': 'residential', 'name': name}
        t.update(tags)
        for (a, an, ae), (b, bn, be) in zip(nodes, nodes[1:]):
            self.edges.append((str(self.wid), a, b, [ll(an, ae), ll(bn, be)], t))

    def build(self):
        path = os.path.join(tempfile.mkdtemp(), 'sc.sqlite')
        build_from_edges(self.edges, path, [])
        return Graph(path)


class Scenario(object):
    """graph; start/dest (lat, lon); driven = node ids the car actually drives, in order; leave = the node where it first departs the planned route (None for a
    control drive); family/params describe the condition; reachable_expected = what the independent oracle must say about the destination from the wrong road."""
    def __init__(self, family, params, net, start, dest, driven, leave=None, island=None, eff_sep=None):
        self.family, self.params, self.graph = family, params, net.build()
        self.start, self.dest, self.driven, self.leave, self.island = start, dest, driven, leave, island
        self.eff_sep = eff_sep          # how far apart the wrong road and the route are (m) once the car has committed: decides how distinguishable they are
        self._oracle = None

    def ambiguity_class(self, noise_m):
        """'ambiguous' (separation < 2 sigma), 'marginal' (2-3 sigma) or 'clear' (>= 3 sigma), judged from the nominal GPS sigma - NOT from how the navigator performs."""
        if self.eff_sep is None:
            return 'clear'
        r = self.eff_sep / max(noise_m, 1e-9)
        return 'ambiguous' if r < 2.0 else ('marginal' if r < 3.0 else 'clear')


def _main(net, n_edges=9, spacing=100.0, name='Main'):
    ids = [net.node() for _ in range(n_edges + 1)]
    net.way(name, [(ids[c], 0.0, spacing * c) for c in range(n_edges + 1)])
    return ids


def parallel(sep, connected, link_oneway=None):
    """Main 0..800 m east. At M2 (200 m) a parallel road leaves, runs `sep` m north of Main and either dead-ends or rejoins Main at M7 (700 m) through a final
    link (link_oneway: 'into' = one-way from Main onto Par - the link exists but cannot be driven towards Main; 'out' = one-way towards Main)."""
    net = Net()
    M = _main(net)
    par = [(M[2], 0.0, 200.0), (net.node(), sep, 200.0 + 2 * sep)]
    par += [(net.node(), sep, 200.0 + 2 * sep + 100.0 * (i + 1)) for i in range(4)]
    last = par[-1]
    net.way('Par', par)
    if connected:
        if link_oneway == 'into':                      # one-way FROM Main ONTO Par: the link exists in the graph but cannot be driven towards Main
            net.way('Link', [(M[7], 0.0, 700.0), (last[0], last[1], last[2])], oneway='yes')
        elif link_oneway == 'out':                     # one-way towards Main (the useful direction)
            net.way('Link', [(last[0], last[1], last[2]), (M[7], 0.0, 700.0)], oneway='yes')
        else:
            net.way('Link', [(last[0], last[1], last[2]), (M[7], 0.0, 700.0)])
    legal_back = connected and link_oneway != 'into'            # a car cannot drive a one-way link against its direction
    driven = [M[0], M[1], M[2]] + [n for n, _, _ in par[1:]] + ([M[7], M[8], M[9]] if legal_back else [])
    return Scenario('parallel', {'sep': sep, 'connected': connected, 'link_oneway': link_oneway}, net, at(0, 0), at(0, 900), driven, leave=M[2], eff_sep=sep)


def fork(angle_deg, merge):
    """A branch leaves Main at M2 at `angle_deg` from its direction (15/30/90), runs 150 m, then either rejoins Main at M6 (a legitimate alternative route, merging
    back) or dead-ends. The car takes the branch."""
    net = Net()
    M = _main(net)
    th = math.radians(angle_deg)
    p1 = (150.0 * math.sin(th), 200.0 + 150.0 * math.cos(th))
    p2 = (p1[0], 800.0 - p1[1])
    ids = [net.node(), net.node()]
    pts = [(M[2], 0.0, 200.0), (ids[0], p1[0], p1[1]), (ids[1], p2[0], p2[1])]
    if merge:
        pts.append((M[6], 0.0, 600.0))
    net.way('Branch', pts)
    driven = [M[0], M[1], M[2], ids[0], ids[1]] + ([M[6], M[7], M[8], M[9]] if merge else [])
    return Scenario('fork', {'angle': angle_deg, 'merge': merge}, net, at(0, 0), at(0, 900), driven, leave=M[2], eff_sep=60.0 * math.sin(th))


def ladder(sep):
    """Main and a second road `sep` m north of it, joined by SHORT connecting roads (length `sep`) at 200, 400 and 600 m. The car changes to the second road through the
    first link, drives it, and comes back through the second link: legitimate route changes through short edges."""
    net = Net()
    M = _main(net)
    Q = [net.node() for _ in range(10)]
    net.way('Second', [(Q[c], sep, 100.0 * c) for c in range(10)])
    for c in (2, 4, 6):
        net.way('Link%d' % c, [(M[c], 0.0, 100.0 * c), (Q[c], sep, 100.0 * c)])
    driven = [M[0], M[1], M[2], Q[2], Q[3], Q[4], M[4], M[5], M[6], M[7], M[8], M[9]]
    return Scenario('ladder', {'sep': sep}, net, at(0, 0), at(0, 900), driven, leave=M[2], eff_sep=sep)


def repeated(sep):
    """Two separate side roads, each leaving Main and rejoining it. The car ignores guidance and drives Main -> side A -> Main -> side B -> Main (two deviations)."""
    net = Net()
    M = _main(net, 12)
    a = [net.node() for _ in range(2)]
    b = [net.node() for _ in range(2)]
    net.way('SideA', [(M[2], 0.0, 200.0), (a[0], sep, 200.0 + 2 * sep), (a[1], sep, 500.0 - 2 * sep), (M[5], 0.0, 500.0)])
    net.way('SideB', [(M[7], 0.0, 700.0), (b[0], sep, 700.0 + 2 * sep), (b[1], sep, 1000.0 - 2 * sep), (M[10], 0.0, 1000.0)])
    driven = [M[0], M[1], M[2], a[0], a[1], M[5], M[6], M[7], b[0], b[1], M[10], M[11], M[12]]
    return Scenario('repeated', {'sep': sep}, net, at(0, 0), at(0, 1200), driven, leave=M[2], eff_sep=sep)


def island(sep):
    """The destination is on a road component that is NOT connected to the start (a clipped extract / missing link)."""
    net = Net()
    M = _main(net, 8)
    isl = [net.node() for _ in range(5)]
    net.way('Island', [(isl[c], sep, 100.0 * c) for c in range(5)])
    return Scenario('island', {'sep': sep}, net, at(0, 0), at(sep, 250), [M[c] for c in range(9)], leave=None, island=isl)


def teleport_to_island(sep):
    """Control drive on Main, then the GPS jumps onto a road component that is not connected to anything the route uses (it stays there)."""
    net = Net()
    M = _main(net, 9)
    isl = [net.node() for _ in range(6)]
    net.way('Island', [(isl[c], sep, 100.0 * c) for c in range(6)])
    return Scenario('teleport_island', {'sep': sep}, net, at(0, 0), at(0, 900), [M[c] for c in range(10)], leave=None, island=isl, eff_sep=sep)


def gps_events(sep):
    """On-route drive beside a parallel (connected) road, with GPS outliers and a 10 s dropout injected by the evaluator; the car never leaves the route."""
    sc = parallel(sep, True)
    sc.family, sc.leave, sc.driven = 'gps_events', None, None
    return sc


def diamond(sep):
    """A junction with SEVERAL possible predecessors: from S two roads run to X, the straight one (the planned route) and one bulging `sep` m to the
    north, and they MERGE at X, where the route continues. The car takes the bulging road: at X the tracker's walk back has two arrival edges."""
    net = Net()
    ids = [net.node() for _ in range(12)]
    m0, s_, a1, a2, x, b1, b2 = ids[0], ids[1], ids[2], ids[3], ids[4], ids[5], ids[6]
    tail = ids[7:12]
    net.way('Start', [(m0, 0.0, 0.0), (s_, 0.0, 100.0)])
    net.way('A', [(s_, 0.0, 100.0), (a1, 0.0, 200.0), (a2, 0.0, 300.0), (x, 0.0, 400.0)])
    net.way('B', [(s_, 0.0, 100.0), (b1, sep, 150.0), (b2, sep, 350.0), (x, 0.0, 400.0)])
    net.way('End', [(x, 0.0, 400.0)] + [(tail[i], 0.0, 500.0 + 100.0 * i) for i in range(5)])
    driven = [m0, s_, b1, b2, x] + tail
    return Scenario('diamond', {'sep': sep}, net, at(0, 0), at(0, 900), driven, leave=s_, eff_sep=sep)
