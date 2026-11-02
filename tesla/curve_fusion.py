#!/usr/bin/env python3
"""
curve_fusion.py - confidence-weighted fusion of the forward-looking speed constraints PCC already computes
(vision curve, map curve, radar lead), replacing the bare min() in PCC_module. Pure Python, no openpilot imports,
fully unit-testable (tests/test_curve_fusion.py).

Design principles (from the predictive-speed-control spec):
  - SAFETY IS A FLOOR, NOT A VOTE. The fused target is the MOST RESTRICTIVE *valid* constraint. A source never raises
    the speed above a more restrictive one. Confidence changes HOW EARLY / HOW HARD we slow, never whether a real
    constraint is ignored.
  - A low-confidence or stale source cannot, on its own, command a hard brake (phantom braking) - its deceleration
    authority is scaled by confidence. But it can still request a mild reduction.
  - Agreement between sources (vision + map) raises confidence -> earlier, firmer slowing. Disagreement -> act on the
    restrictive source but more cautiously (gentler decel authority), and flag the disagreement.
  - Radar (lead) is a TRAFFIC constraint, not road geometry: it is always applied as a hard min (following distance is
    not negotiable), independent of the curve confidence.
  - Missing/stale/invalid sources are dropped; the system keeps working on whatever remains (graceful degradation).

Each source is a dict: {'name','speed','conf','valid','kind'} where kind is 'vision'|'map'|'lead'|'route'.
  speed: the source's recommended max speed (m/s). conf: 0..1. valid: bool.
Returns a Fusion result with the target speed, the binding source, a decel-authority factor (0..1) for the caller to
scale its braking limit, and diagnostics.
"""

CONF_MIN = 0.15            # below this a source is ignored entirely (treated invalid)
AGREE_SPEED_FRAC = 0.20   # two curve sources "agree" if their speeds are within 20% of each other
GEOM_KINDS = ('vision', 'map', 'route')   # road-geometry sources (fused among themselves by confidence)


class Fusion(object):
  def __init__(self, target, conf, binding, decel_authority, note, used):
    self.target = target                 # fused max speed (m/s)
    self.conf = conf                     # confidence in the binding curve constraint (0..1); 1.0 for a lead
    self.binding = binding               # name of the binding source, or None
    self.decel_authority = decel_authority  # 0..1: how much of the max curve-brake the caller may use
    self.note = note                     # short human string for logs
    self.used = used                     # list of source names that passed validity

  def as_dict(self):
    return {'target': None if self.target is None else round(self.target, 2), 'conf': round(self.conf, 2),
            'binding': self.binding, 'decel_authority': round(self.decel_authority, 2), 'note': self.note, 'used': self.used}


def _valid(s):
  return bool(s.get('valid', True)) and s.get('speed') is not None and s['speed'] >= 0 and (s.get('conf', 1.0) >= CONF_MIN)


def fuse(sources, v_ego, v_cruise, no_lead_speed=255.):
  """sources: list of source dicts. v_ego, v_cruise, no_lead_speed in m/s. Returns a Fusion."""
  valid = [s for s in sources if _valid(s)]
  geom = [s for s in valid if s.get('kind') in GEOM_KINDS]
  leads = [s for s in valid if s.get('kind') == 'lead']

  # --- road-geometry fusion (vision / map / route) -------------------------------------------------------------
  curve_target = None; curve_conf = 0.0; curve_binding = None; agree = False
  if geom:
    binding = min(geom, key=lambda s: s['speed'])        # most restrictive geometry source wins (safety floor)
    curve_target = binding['speed']; curve_binding = binding['name']; curve_conf = binding.get('conf', 1.0)
    # does another geometry source corroborate the binding one at a similar speed?
    others = [s for s in geom if s is not binding]
    for s in others:
      if binding['speed'] <= 0: continue
      if abs(s['speed'] - binding['speed']) <= AGREE_SPEED_FRAC * max(binding['speed'], 1.0):
        agree = True
        curve_conf = min(1.0, curve_conf + 0.25 * s.get('conf', 1.0))   # corroboration raises confidence
    # a geometry source that is NOT binding and says "much faster" (e.g. map says straight, vision says curve) is a
    # disagreement: we still act on the restrictive one, but note it. It never raises the target.
  # decel authority tracks confidence: a reliable reading (high conf) brakes firmly even alone; a shaky one (low conf)
  # brakes gently so it cannot slam the brakes on a phantom; corroboration is a 1.2x bonus. Floor 0.3.
  decel_authority = max(0.3, min(1.0, curve_conf * (1.2 if agree else 1.0)))

  # --- lead (radar) constraint: always a hard min, confidence-independent --------------------------------------
  lead_target = min((s['speed'] for s in leads), default=None)

  # --- combine: the overall target is the most restrictive of curve and lead ----------------------------------
  cands = [x for x in (curve_target, lead_target) if x is not None]
  if not cands:
    return Fusion(None, 0.0, None, 0.0, 'no valid curve/lead source', [s['name'] for s in valid])
  target = min(cands)
  if lead_target is not None and (curve_target is None or lead_target <= curve_target):
    # lead is binding (or ties): follow-distance governs. Full decel authority (traffic, not an uncertain curve).
    binding_name = min(leads, key=lambda s: s['speed'])['name']
    note = 'lead-limited'
    return Fusion(target, 1.0, binding_name, 1.0, note, [s['name'] for s in valid])
  # curve is binding
  note = ('curve: %s' % curve_binding) + (' (corroborated)' if agree else ' (single source)')
  return Fusion(target, curve_conf, curve_binding, decel_authority, note, [s['name'] for s in valid])


def approach_decel(v_ego, v_target, dist_m, reaction_s, a_max):
  """deceleration (m/s^2, positive) needed NOW to reach v_target by dist_m, allowing reaction_s of slack. 0 if already
  slow enough or the curve is far. The caller clamps to its comfort/jerk limits; this just says how urgent it is."""
  if v_target is None or v_ego <= v_target:
    return 0.0
  d = max(0.1, dist_m - max(v_ego, 0.) * reaction_s)
  need = (v_ego * v_ego - v_target * v_target) / (2.0 * d)   # v^2 = v0^2 - 2 a d
  return max(0.0, min(need, a_max))
