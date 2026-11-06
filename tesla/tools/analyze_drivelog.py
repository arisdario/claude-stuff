#!/usr/bin/env python3
"""
analyze_drivelog.py - turns a driving_logger.py JSONL file into a compact text report: time spent
in each engagement state, acceleration/deceleration profile split by state and lead-present/absent,
and steering wiggle diagnostics (torque/rate/commanded-vs-measured, plus the steer_limits strength
comparison). Stdlib only, single streaming pass (safe on a multi-hundred-MB file on the EON itself).

Run THIS ON THE EON, on the actual log file - don't try to copy a multi-MB jsonl through chat:

  python3 /data/cs_new/tesla/tools/analyze_drivelog.py /data/drivelog_20261106_213015.jsonl

Read-only: only reads the file and prints a report. Touches nothing else.
"""
import argparse
import json
import math
import sys


def pct(x, n):
  return 0.0 if n == 0 else 100.0 * x / n


def mean(xs):
  return sum(xs) / len(xs) if xs else None


def stdev(xs, m=None):
  if len(xs) < 2:
    return None
  m = mean(xs) if m is None else m
  return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


def percentile(xs, p):
  if not xs:
    return None
  s = sorted(xs)
  k = (len(s) - 1) * p / 100.0
  f, c = int(k), min(int(k) + 1, len(s) - 1)
  return s[f] if f == c else s[f] + (s[c] - s[f]) * (k - f)


class Analysis(object):
  def __init__(self):
    self.n = 0
    self.t0 = self.t1 = None
    self.state_dt = {}            # state -> seconds
    self.state_n = {}
    self.transitions = 0
    self.prev_state = None
    # accel/decel samples, keyed by (state_bucket, lead_present)
    self.accel = {}               # key -> list of a_ego where a_ego > 0.1 and not standstill
    self.decel = {}               # key -> list of a_ego where a_ego < -0.1 and not standstill
    # steering
    self.torque = []
    self.rate = []
    self.cmd_vs_meas = []          # abs(steering_angle - das_commanded_angle) while steer_enabled
    self.sl20_dev = []             # abs(sl20_angle - das_commanded_angle)
    self.sl25_dev = []
    self.osc_events = 0            # rate sign flips within a short window, see detect_oscillations
    self._rate_hist = []           # (t, rate) ring for oscillation detection
    self.lead_null_n = 0
    self.last_t = None

  def state_bucket(self, state):
    # collapse LONGITUDINAL_ONLY/LATERAL_ONLY/ENGAGED into one "OPENPILOT" bucket for the accel/decel
    # comparison (that question is human vs openpilot, not which axis) - the per-state time table
    # below still reports all states separately.
    if state == 'HUMAN_DRIVING':
      return 'HUMAN'
    if state and state.startswith('OPENPILOT'):
      return 'OPENPILOT'
    return None   # TRANSITION/UNKNOWN excluded from the accel/decel comparison

  def feed(self, s):
    self.n += 1
    t = s.get('t')
    if t is None:
      return
    if self.t0 is None:
      self.t0 = t
    self.t1 = t
    state = s.get('state')
    dt = 0.0 if self.last_t is None else max(0.0, min(1.0, t - self.last_t))  # clamp: ignore gaps/pauses
    self.last_t = t

    if state != self.prev_state and self.prev_state is not None:
      self.transitions += 1
    self.prev_state = state
    self.state_dt[state] = self.state_dt.get(state, 0.0) + dt
    self.state_n[state] = self.state_n.get(state, 0) + 1

    bucket = self.state_bucket(state)
    lead = s.get('lead1') or {}
    lead_present = lead.get('status')
    if lead_present is None:
      self.lead_null_n += 1
    a_ego = s.get('a_ego')
    standstill = s.get('standstill')
    if bucket and a_ego is not None and not standstill and lead_present is not None:
      key = (bucket, bool(lead_present))
      if a_ego > 0.1:
        self.accel.setdefault(key, []).append(a_ego)
      elif a_ego < -0.1:
        self.decel.setdefault(key, []).append(a_ego)

    torque, rate = s.get('steering_torque'), s.get('steering_rate')
    if torque is not None:
      self.torque.append(torque)
    if rate is not None:
      self.rate.append(rate)
      self._rate_hist.append((t, rate))
      self._rate_hist[:] = [(rt, rr) for rt, rr in self._rate_hist if t - rt <= 1.0]
      self._check_oscillation()

    cmd, meas, steer_en = s.get('das_commanded_angle'), s.get('steering_angle'), s.get('das_steer_enabled')
    if cmd is not None and meas is not None and steer_en:
      self.cmd_vs_meas.append(abs(meas - cmd))
    sl20, sl25 = s.get('sl20_angle'), s.get('sl25_angle')
    if sl20 is not None and cmd is not None:
      self.sl20_dev.append(abs(sl20 - cmd))
    if sl25 is not None and cmd is not None:
      self.sl25_dev.append(abs(sl25 - cmd))

  def _check_oscillation(self):
    """an 'event' = the rate crosses zero at least 3 times within the trailing 1 s window, with each
    excursion exceeding 1 deg/s - a simple, conservative stand-in for 'wiggling back and forth', not a
    claim about root cause."""
    hist = self._rate_hist
    if len(hist) < 4:
      return
    crossings = 0
    prev_sign = 0
    for _, r in hist:
      if abs(r) < 1.0:
        continue
      sign = 1 if r > 0 else -1
      if prev_sign != 0 and sign != prev_sign:
        crossings += 1
      prev_sign = sign
    if crossings >= 3:
      self.osc_events += 1

  def report(self):
    out = []
    dur = (self.t1 - self.t0) if (self.t0 is not None and self.t1 is not None) else 0.0
    out.append('=== driving_logger analysis ===')
    out.append('samples: %d   duration: %.1f s (%.1f min)   state transitions: %d' % (self.n, dur, dur / 60.0, self.transitions))
    if self.lead_null_n:
      out.append('note: lead1 was null/unavailable on %d samples (%.1f%%) - radarState timing gaps, see driving_logger docstring' % (
          self.lead_null_n, pct(self.lead_null_n, self.n)))
    out.append('')
    out.append('--- time per state ---')
    for state in sorted(self.state_dt, key=lambda k: -self.state_dt[k]):
      d = self.state_dt[state]
      out.append('  %-28s %7.1f s  (%5.1f%%)  %d samples' % (str(state), d, pct(d, dur), self.state_n[state]))
    out.append('')
    out.append('--- acceleration / deceleration, human vs openpilot, with/without a lead ---')
    out.append('  (a_ego in m/s2; only non-standstill samples with a known lead status; +accel / -decel bucketed separately)')
    for bucket in ('HUMAN', 'OPENPILOT'):
      for lead_present in (True, False):
        key = (bucket, lead_present)
        a, d = self.accel.get(key, []), self.decel.get(key, [])
        label = '%-10s lead=%-5s' % (bucket, lead_present)
        if a:
          out.append('  %s  ACCEL  n=%4d  mean=%.3f  p50=%.3f  p90=%.3f  max=%.3f' % (
              label, len(a), mean(a), percentile(a, 50), percentile(a, 90), max(a)))
        if d:
          out.append('  %s  DECEL  n=%4d  mean=%.3f  p50=%.3f  p90=%.3f  min=%.3f' % (
              label, len(d), mean(d), percentile(d, 50), percentile(d, 90), min(d)))
        if not a and not d:
          out.append('  %s  (no samples)' % label)
    out.append('')
    out.append('--- steering (wiggle diagnostics) ---')
    if self.torque:
      out.append('  steering_torque: n=%d mean=%.3f std=%.3f p5=%.3f p95=%.3f' % (
          len(self.torque), mean(self.torque), stdev(self.torque) or 0., percentile(self.torque, 5), percentile(self.torque, 95)))
    if self.rate:
      out.append('  steering_rate:   n=%d mean=%.3f std=%.3f p5=%.3f p95=%.3f  max|rate|=%.2f' % (
          len(self.rate), mean(self.rate), stdev(self.rate) or 0., percentile(self.rate, 5), percentile(self.rate, 95),
          max(abs(x) for x in self.rate)))
    out.append('  oscillation events (rate sign flips >=3x within 1 s, each >1 deg/s): %d' % self.osc_events)
    if self.cmd_vs_meas:
      out.append('  |measured - commanded| while steering engaged: n=%d mean=%.3f p90=%.3f max=%.3f  (big values = tracking lag or the wiggle itself)' % (
          len(self.cmd_vs_meas), mean(self.cmd_vs_meas), percentile(self.cmd_vs_meas, 90), max(self.cmd_vs_meas)))
    out.append('')
    out.append('--- steer_limits hypothetical: how much would 0.20 / 0.25 have changed the command? ---')
    out.append('  (bigger numbers = that strength would have overridden the raw request more; compare the two, don\'t read either alone)')
    if self.sl20_dev:
      out.append('  strength 0.20: n=%d mean=%.3f p90=%.3f max=%.3f' % (
          len(self.sl20_dev), mean(self.sl20_dev), percentile(self.sl20_dev, 90), max(self.sl20_dev)))
    if self.sl25_dev:
      out.append('  strength 0.25: n=%d mean=%.3f p90=%.3f max=%.3f' % (
          len(self.sl25_dev), mean(self.sl25_dev), percentile(self.sl25_dev, 90), max(self.sl25_dev)))
    return '\n'.join(out)


def analyze(path):
  a = Analysis()
  with open(path) as f:
    for line in f:
      line = line.strip()
      if not line:
        continue
      try:
        s = json.loads(line)
      except Exception:
        continue
      if 'meta' in s:   # the start/fields_seen_once marker lines - not a sample
        continue
      a.feed(s)
  return a


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('path')
  args = ap.parse_args(argv)
  a = analyze(args.path)
  print(a.report())


if __name__ == '__main__':
  main()
