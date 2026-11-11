#!/usr/bin/env python3
"""replay_fcw.py - replays selfdrive/controls/lib/fcw.py's actual FCW gating logic against a
driving_logger.py drivelog.jsonl, to see whether/when it would have armed during real close-following
events, and if not, which specific gate blocked it.

WHY THIS IS APPROXIMATE, NOT A FAITHFUL REPLAY: fcw.py's real update() needs two fields
driving_logger.py does not currently log - fcw_lead (vision/radar lead classification confidence,
gates the whole thing behind fcw_lead > 0.99) and vlat_lead (lead's lateral velocity). This script
assumes both are always satisfied (fcw_lead=1.0, vlat_lead=0.0) - the OPTIMISTIC case. The real system
can only be equal to or SLOWER/LESS LIKELY to arm than this replay shows, never faster - if this
replay already shows a failure to arm, the real system failed too; if this replay shows it arming,
the real system might still not have (if fcw_lead never crossed 0.99, or vlat_lead was ever >= 0.4).

Everything else (calc_ttc, the six per-danger counters, the two common counters including the
3-second blinker ramp, the future_fcw gate) is copied faithfully from fcw.py.

Usage:
    python3 replay_fcw.py drivelog.jsonl                  # scan whole log, report every episode
    python3 replay_fcw.py drivelog.jsonl --min-gap 5.0    # only episodes where dRel dropped below 5m
"""
import json
import sys

MAX_TTC = 5.0
NOMINAL_DT = 0.05  # 1/20Hz - the cycle rate fcw.py's own ramp-rate constants (10.0/(20*3.0)) assume


def calc_ttc(v_ego, a_ego, x_lead, v_lead, a_lead):
  """Verbatim port of FCWChecker.calc_ttc from selfdrive/controls/lib/fcw.py."""
  v_rel = v_ego - v_lead
  a_rel = a_ego - a_lead
  t_decel = 2.
  a_rel = min(a_rel, v_lead / t_decel) if t_decel else a_rel
  delta = v_rel ** 2 + 2 * x_lead * a_rel
  if delta < 0.1 or (delta ** 0.5 + v_rel < 0.1):
    return MAX_TTC
  return min(2 * x_lead / (delta ** 0.5 + v_rel), MAX_TTC)


class FCWReplay:
  """Mirrors FCWChecker's counter state machine, fed from drivelog.jsonl rows instead of cereal."""

  def __init__(self):
    self.counters = {'ttc': 0.0, 'v_lead_max': 0.0, 'v_ego_lead': 0.0, 'lead_seen': 0.0,
                      'y_lead': 0.0, 'vlat_lead': 0.0}
    self.common = {'blinkers': 0.0, 'v_ego': 0.0}
    self.v_lead_max = 0.0
    self.prev_t = None
    self.armed_at = None
    self.ttc_history = []  # (t, ttc, dRel, blocking_gates) for every qualifying frame

  def step(self, t, v_ego, a_ego, x_lead, v_lead, a_lead, y_lead, blinkers):
    dt = (t - self.prev_t) if self.prev_t is not None else NOMINAL_DT
    self.prev_t = t
    cyc = dt / NOMINAL_DT  # how many nominal 20Hz cycles this real dt represents

    self.v_lead_max = max(self.v_lead_max, v_lead)
    self.common['blinkers'] = self.common['blinkers'] + (10.0 / (20 * 3.0)) * cyc if not blinkers else 0.0
    self.common['v_ego'] = self.common['v_ego'] + cyc if v_ego > 5.0 else 0.0

    ttc = calc_ttc(v_ego, a_ego, x_lead, v_lead, a_lead)

    self.counters['ttc'] = self.counters['ttc'] + cyc if ttc < 2.5 else 0.0
    self.counters['v_lead_max'] = self.counters['v_lead_max'] + cyc if self.v_lead_max > 2.5 else 0.0
    self.counters['v_ego_lead'] = self.counters['v_ego_lead'] + cyc if v_ego > v_lead else 0.0
    self.counters['lead_seen'] = self.counters['lead_seen'] + 0.33 * cyc
    self.counters['y_lead'] = self.counters['y_lead'] + cyc if abs(y_lead) < 1.0 else 0.0
    self.counters['vlat_lead'] = self.counters['vlat_lead'] + cyc  # optimistic: field unavailable

    blocking = [k for k, v in self.counters.items() if v < 10] + \
               ['common:' + k for k, v in self.common.items() if v < 10]
    allowed = not blocking
    if allowed and self.armed_at is None:
      self.armed_at = t

    self.ttc_history.append((t, ttc, x_lead, tuple(blocking)))
    return ttc, blocking


def find_lead_episodes(rows, min_gap_m, max_dropout_s=1.0, min_duration_s=1.0):
  """Groups lead1-active rows into episodes, bridging brief dropouts (lead1.status flips False for
  up to max_dropout_s - the track-reacquisition noise seen in practice, not a real new encounter).
  Keeps only episodes lasting at least min_duration_s (anything shorter can't fairly be judged on
  whether FCW 'should have' armed - a few tenths of a second isn't a real sustained encounter) where
  dRel ever dropped below min_gap_m."""
  episodes = []
  current = []
  last_active_t = None
  for d in rows:
    l1 = d.get('lead1') or {}
    active = l1.get('status') and l1.get('dRel') is not None
    if active:
      if current and last_active_t is not None and (d['t'] - last_active_t) > max_dropout_s:
        episodes.append(current)
        current = []
      current.append(d)
      last_active_t = d['t']
    # an inactive row within max_dropout_s of the last active one is silently bridged (dropped,
    # not appended) - the next active row's dt from the previous active row carries the real gap
  if current:
    episodes.append(current)

  out = []
  for ep in episodes:
    if ep[-1]['t'] - ep[0]['t'] < min_duration_s:
      continue
    if min(d['lead1']['dRel'] for d in ep) >= min_gap_m:
      continue
    out.append(ep)
  return out


def replay_episode(rows):
  """Runs the FCWReplay state machine across one episode's rows in order. Tracks the worst moment
  two different ways - minimum raw gap, and minimum TTC (gap alone hides a fast-closing-but-currently-
  further-away lead; TTC is what FCW actually keys off) - since they aren't always the same frame."""
  rep = FCWReplay()
  t0 = rows[0]['t']
  min_gap = None
  min_gap_t_rel = None
  closing_rate_at_min_gap = None
  min_ttc = None
  min_ttc_t_rel = None
  gap_at_min_ttc = None
  closing_rate_at_min_ttc = None
  blocking_at_min_ttc = None

  for d in rows:
    l1 = d['lead1']
    blinkers = bool(d.get('left_blinker') or d.get('right_blinker'))
    ttc, blocking = rep.step(d['t'], d['v_ego'], d['a_ego'], l1['dRel'], l1['vLeadK'],
                              l1['aLeadK'], l1['yRel'], blinkers)
    if min_gap is None or l1['dRel'] < min_gap:
      min_gap = l1['dRel']
      min_gap_t_rel = d['t'] - t0
      closing_rate_at_min_gap = -l1['vRel']  # positive = closing
    if min_ttc is None or ttc < min_ttc:
      min_ttc = ttc
      min_ttc_t_rel = d['t'] - t0
      gap_at_min_ttc = l1['dRel']
      closing_rate_at_min_ttc = -l1['vRel']
      blocking_at_min_ttc = blocking

  armed_t_rel = (rep.armed_at - t0) if rep.armed_at is not None else None
  return {
      'duration_s': rows[-1]['t'] - t0,
      'min_gap_m': min_gap,
      'min_gap_t_rel': min_gap_t_rel,
      'closing_rate_at_min_gap': closing_rate_at_min_gap,
      'min_ttc_s': min_ttc,
      'min_ttc_t_rel': min_ttc_t_rel,
      'gap_at_min_ttc': gap_at_min_ttc,
      'closing_rate_at_min_ttc': closing_rate_at_min_ttc,
      'armed_t_rel': armed_t_rel,
      'blocking_at_min_ttc': blocking_at_min_ttc,
  }


def scan_log(path, min_gap_m=6.0):
  rows = []
  with open(path) as f:
    f.readline()  # meta header
    for line in f:
      d = json.loads(line)
      if 'lead1' in d and 'v_ego' in d:
        rows.append(d)

  episodes = find_lead_episodes(rows, min_gap_m)
  return [replay_episode(ep) for ep in episodes]


def main(argv):
  if len(argv) < 1:
    print(__doc__)
    return 2
  path = argv[0]
  min_gap = 6.0
  if '--min-gap' in argv:
    min_gap = float(argv[argv.index('--min-gap') + 1])

  results = scan_log(path, min_gap)
  print('%d close-following episodes with gap < %.1fm' % (len(results), min_gap))
  never_armed = [r for r in results if r['armed_t_rel'] is None]
  print('%d of %d never armed within the episode' % (len(never_armed), len(results)))
  print()
  for r in sorted(results, key=lambda r: r['min_ttc_s']):
    status = ('ARMED at t=%.2fs' % r['armed_t_rel']) if r['armed_t_rel'] is not None else 'NEVER ARMED'
    print('worst ttc=%.2fs (gap=%.2fm closing=%.1fm/s) at t=%.2fs (episode %.1fs long) -> %s%s' % (
        r['min_ttc_s'], r['gap_at_min_ttc'], r['closing_rate_at_min_ttc'], r['min_ttc_t_rel'], r['duration_s'],
        status, ('  blocked by: %s' % ','.join(r['blocking_at_min_ttc']) if r['blocking_at_min_ttc'] else '')))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
