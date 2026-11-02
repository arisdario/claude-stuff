"""Turn-by-turn instructions and junction summaries from a planned Route. Advisory text only."""
try:
    from . import geo
    from .router import in_bearing, out_bearing, turn_angle
except ImportError:
    import geo
    from router import in_bearing, out_bearing, turn_angle

COMPASS = ('north', 'north-east', 'east', 'south-east', 'south', 'south-west', 'west', 'north-west')
DECISION_ANGLE = 35.0           # at a junction with alternatives, a turn at least this sharp is an instruction


class Step(object):
    def __init__(self, kind, text, label, start_m, node, angle, exits):
        self.kind, self.text, self.label, self.start_m, self.node, self.angle, self.exits = kind, text, label, start_m, node, angle, exits
        self.distance_m = 0.0

    def as_dict(self):
        return {'kind': self.kind, 'text': self.text, 'road': self.label, 'start_m': round(self.start_m), 'distance_m': round(self.distance_m),
                'node': self.node, 'angle': None if self.angle is None else round(self.angle), 'junction_exits': self.exits}


def label(e):
    """what a driver reads on the road: ref and/or name (e.g. 'DN1 (E60)')"""
    if e.ref and e.name:
        return '%s (%s)' % (e.name, e.ref)
    return e.name or e.ref or '(unnamed %s)' % e.highway


def legal_exits(graph, prev):
    """edges a driver could take at the end of `prev` (legal directed edges, excluding an immediate reversal)"""
    return [n for n in graph.out_edges(prev.v) if not (n.u == prev.v and n.v == prev.u)]


def describe_turn(angle):
    a = abs(angle)
    side = 'right' if angle > 0 else 'left'
    if a < 20:
        return 'straight', 'Continue straight'
    if a < 60:
        return 'slight ' + side, 'Bear %s' % side
    if a < 120:
        return side, 'Turn %s' % side
    if a < 170:
        return 'sharp ' + side, 'Take a sharp %s' % side
    return 'u-turn', 'Make a U-turn'


def build_steps(graph, route):
    edges, offs = route.edges, route.edge_offsets
    first = edges[0]
    b0 = in_bearing(graph, first)
    steps = [Step('depart', 'Head %s on %s' % (COMPASS[int((b0 + 22.5) // 45) % 8], label(first)), label(first), 0.0, first.u, None, None)]
    rb_exits, rb_label_in = 0, None
    for i in range(1, len(edges)):
        prev, cur = edges[i - 1], edges[i]
        node = prev.v
        if cur.roundabout and prev.roundabout:
            rb_exits += sum(1 for n in legal_exits(graph, prev) if not n.roundabout)       # arms passed
            continue
        if cur.roundabout and not prev.roundabout:
            rb_exits = 0
            continue
        exits = legal_exits(graph, prev)
        ang = turn_angle(graph, prev, cur)
        if prev.roundabout and not cur.roundabout:
            n = rb_exits + 1
            steps.append(Step('roundabout', 'At the roundabout take exit %d onto %s' % (n, label(cur)), label(cur), offs[i], node, ang, len(exits)))
            continue
        changed = label(prev) != label(cur)
        decision = len(exits) >= 2 and abs(ang) >= DECISION_ANGLE
        if not (changed or decision):
            continue
        kind, phrase = describe_turn(ang)
        if kind == 'straight':
            text = 'Continue onto %s' % label(cur) if changed else phrase
        else:
            text = '%s onto %s' % (phrase, label(cur))
        steps.append(Step('turn' if kind != 'straight' else 'continue', text, label(cur), offs[i], node, ang, len(exits)))
    steps.append(Step('arrive', 'Arrive at your destination', label(edges[-1]), route.distance_m, edges[-1].v, None, None))
    for a, b in zip(steps, steps[1:]):
        a.distance_m = max(0.0, b.start_m - a.start_m)
    return steps


def junctions(graph, route):
    """Every node the route passes, with its decision complexity: (node, legal_exits, turn_angle, from_road, to_road). Used by the
    validation report to inspect junction connectivity."""
    out = []
    for i in range(1, len(route.edges)):
        prev, cur = route.edges[i - 1], route.edges[i]
        ex = legal_exits(graph, prev)
        out.append({'node': prev.v, 'exits': len(ex), 'angle': turn_angle(graph, prev, cur), 'from': label(prev), 'to': label(cur),
                    'at_m': route.edge_offsets[i], 'shares_node': prev.v == cur.u})
    return out
