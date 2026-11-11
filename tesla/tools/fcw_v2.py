#!/usr/bin/env python3
"""fcw_v2.py - a redesigned FCW trigger, built directly from two real failures found by
replay_fcw.py against an actual drive log (see tesla/tools/replay_fcw.py's docstring for the replay
methodology and its own caveats, which apply here too - no fcw_lead/vlat_lead in the log, so this is
also evaluated optimistically on those two inputs):

  1. worst ttc=0.46s (gap=3.59m, closing=6.6m/s) - stock fcw.py never armed; blocked only by
     y_lead (lead drifted >1m laterally at some point) and the 3-second blinker-silence ramp.
     Neither gate is actually about how dangerous that moment was.
  2. worst ttc=0.62s (gap=2.46m, closing=2.1m/s) - stock fcw.py never armed; blocked by
     v_lead_max (lead's speed never exceeded 2.5 m/s anywhere in the encounter) - a gate that, by
     construction, can NEVER be satisfied for a stopped or crawling lead. This structurally excludes
     the single most common severe rear-end scenario (hitting a stopped/very slow car).

WHAT CHANGED FROM STOCK fcw.py, AND WHY:
  - Primary trigger is REQUIRED DECELERATION (a_needed = v_rel^2 / (2*gap)), not raw TTC. This
    answers "can braking from right now still avoid this" directly, and is well-defined at v_lead=0 -
    no v_lead_max gate exists here at all, so a stopped lead can trigger this.
  - Debounce is a PERSISTENCE FILTER (danger true in >= PERSIST_K of the last PERSIST_N frames), not a
    counter that resets to zero on one noisy frame. Tolerates single-frame track glitches mid-danger.
  - Blinker suppression only applies within BLINKER_SUPPRESS_S seconds of ACTUAL blinker activity -
    not a standing 3-second prerequisite for every alert regardless of relevance.
  - y_lead (lateral position) is a SEPARATE, softer check - large lateral offset lowers confidence
    but does not by itself veto an otherwise-clear required-deceleration trigger, since a real lead
    car drifting laterally while still closing fast longitudinally is still a real hazard, not a
    false positive to suppress outright.
  - Two severity stages: EARLY (lower urgency, e.g. a visual cue) and URGENT (full alert), each with
    its own required-deceleration threshold and its own persistence filter.

NOT YET WIRED INTO ANYTHING LIVE. This is shadow-mode logic only, meant to be validated against real
drive logs (see validate_against_log() below) before any discussion of controlsd actually consuming
it. Loosening a safety trigger always trades against false-positive rate; that tradeoff needs your
explicit sign-off, with more validation data than one log, before this goes anywhere near the car.
"""
import json
import sys
from collections import deque

G = 9.81
EARLY_A_NEEDED = 0.3 * G    # ~2.9 m/s^2 - "should start thinking about braking"
URGENT_A_NEEDED = 0.6 * G   # ~5.9 m/s^2 - "hard braking needed now, most drivers/cars can't exceed this comfortably"
MIN_GAP_FLOOR_M = 0.5        # avoid dividing by ~0 at near-zero gap
PERSIST_N = 5                # look at the last N frames
PERSIST_K_EARLY = 3           # >= K of last N must show danger for EARLY
PERSIST_K_URGENT = 4          # >= K of last N must show danger for URGENT (stricter - this is the loud one)
BLINKER_SUPPRESS_S = 1.5      # only suppress within this many seconds of actual blinker use
Y_LEAD_SOFT_LIMIT_M = 2.5     # beyond this, treat the lead track as too uncertain to trust at all


def required_decel(v_ego, x_lead, v_lead):
  """Constant deceleration needed, starting now, to just avoid closing the gap to zero.
  Well-defined for v_lead == 0 (a stopped lead) - that's the whole point versus TTC+v_lead_max."""
  v_rel = v_ego - v_lead
  if v_rel <= 0:
    return 0.0  # not closing at all - no braking need from this alone
  gap = max(x_lead, MIN_GAP_FLOOR_M)
  return (v_rel ** 2) / (2 * gap)


class FCWv2:
  def __init__(self):
    self.early_hist = deque(maxlen=PERSIST_N)
    self.urgent_hist = deque(maxlen=PERSIST_N)
    self.last_blinker_t = None
    self.early_fired_at = None
    self.urgent_fired_at = None

  def step(self, t, v_ego, x_lead, v_lead, y_lead, blinkers):
    if blinkers:
      self.last_blinker_t = t
    blinker_recent = self.last_blinker_t is not None and (t - self.last_blinker_t) <= BLINKER_SUPPRESS_S
    track_too_uncertain = abs(y_lead) > Y_LEAD_SOFT_LIMIT_M

    a_needed = required_decel(v_ego, x_lead, v_lead)

    early_danger = (a_needed >= EARLY_A_NEEDED) and not track_too_uncertain
    urgent_danger = (a_needed >= URGENT_A_NEEDED) and not track_too_uncertain

    self.early_hist.append(early_danger)
    self.urgent_hist.append(urgent_danger)

    early_ok = (not blinker_recent) and len(self.early_hist) == PERSIST_N and sum(self.early_hist) >= PERSIST_K_EARLY
    urgent_ok = (not blinker_recent) and len(self.urgent_hist) == PERSIST_N and sum(self.urgent_hist) >= PERSIST_K_URGENT

    if early_ok and self.early_fired_at is None:
      self.early_fired_at = t
    if urgent_ok and self.urgent_fired_at is None:
      self.urgent_fired_at = t

    return {'a_needed': a_needed, 'early': early_ok, 'urgent': urgent_ok,
            'blinker_suppressed': blinker_recent, 'track_too_uncertain': track_too_uncertain}


def replay_episode_v2(rows):
  rep = FCWv2()
  t0 = rows[0]['t']
  max_a_needed = 0.0
  max_a_needed_t_rel = None
  for d in rows:
    l1 = d['lead1']
    blinkers = bool(d.get('left_blinker') or d.get('right_blinker'))
    out = rep.step(d['t'], d['v_ego'], l1['dRel'], l1['vLeadK'], l1['yRel'], blinkers)
    if out['a_needed'] > max_a_needed:
      max_a_needed = out['a_needed']
      max_a_needed_t_rel = d['t'] - t0
  return {
      'duration_s': rows[-1]['t'] - t0,
      'max_a_needed_g': max_a_needed / G,
      'max_a_needed_t_rel': max_a_needed_t_rel,
      'early_fired_t_rel': (rep.early_fired_at - t0) if rep.early_fired_at is not None else None,
      'urgent_fired_t_rel': (rep.urgent_fired_at - t0) if rep.urgent_fired_at is not None else None,
  }


def validate_against_log(path, min_gap_m=8.0):
  """Reuses replay_fcw's episode-finding (same bridging/duration rules) so results are directly
  comparable to the stock-logic replay, then runs FCWv2 across each episode instead."""
  sys.path.insert(0, __file__.rsplit('/', 1)[0])
  import replay_fcw as RF

  rows = []
  with open(path) as f:
    f.readline()
    for line in f:
      d = json.loads(line)
      if 'lead1' in d and 'v_ego' in d:
        rows.append(d)

  episodes = RF.find_lead_episodes(rows, min_gap_m)
  return [replay_episode_v2(ep) for ep in episodes]


def main(argv):
  if len(argv) < 1:
    print(__doc__)
    return 2
  results = validate_against_log(argv[0], min_gap_m=float(argv[1]) if len(argv) > 1 else 8.0)
  print('%d episodes evaluated' % len(results))
  for r in sorted(results, key=lambda r: -r['max_a_needed_g']):
    early = ('EARLY at t=%.2fs' % r['early_fired_t_rel']) if r['early_fired_t_rel'] is not None else 'early: never'
    urgent = ('URGENT at t=%.2fs' % r['urgent_fired_t_rel']) if r['urgent_fired_t_rel'] is not None else 'urgent: never'
    print('worst a_needed=%.2fg at t=%.2fs (episode %.1fs) -> %s, %s' % (
        r['max_a_needed_g'], r['max_a_needed_t_rel'], r['duration_s'], early, urgent))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
