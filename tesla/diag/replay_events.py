#!/usr/bin/env python3
"""Replay recorded radarweb intervention events (ev_*.json, 10 Hz) through the diagnostics Collector (same detectors, same
event/ring code as the live logger) to validate it on real drives and to produce a session folder + analysis.

  python3 replay_events.py OUT_SESSION_DIR ev_*.json

Only the fields the events contain exist (no desired angle, no frame ids, no pose): detectors needing them stay silent."""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import core  # noqa: E402


class SimClock(object):
  t = 0.0

  def __call__(self):
    return self.t


def replay(out, files):
  clk = SimClock()
  cfg = {'queue_max': 10 ** 7, 'thr': {'ctl_gap_s': 1e9, 'ipc_delay_s': 1e9}, 'pre_s': 30, 'post_s': 30, 'seg_s': 10, 'keep_s': 100,
         'trace_hz': 10, 'ring_hz': {}}
  c = core.Collector(out, cfg, clock=clk)
  last = None
  for f in sorted(files):
    d = json.load(open(f))
    t0 = None
    for x in d['samples']:
      t = x['t']
      clk.t = t
      if last is not None and t - last > 2.0 and t0 is None:      # new event file, gap since the previous one
        pass
      t0 = t0 or t
      c.ingest({'s': 'model', 'rt': t, 'path': x['pp'], 'lane_l_p': (x.get('lane_l') or [None] * 5)[4], 'lane_r_p': (x.get('lane_r') or [None] * 5)[4], 'fid': None})
      c.ingest({'s': 'controlsState', 'rt': t, 'des': None, 'act': x['ang'], 'en': bool(x['op']), 'v': x['v']})
      c.ingest({'s': 'carState', 'rt': t, 'v': x['v'], 'ang': x['ang'], 'tq': x.get('tq')})
      c.tick(t)
      last = t
    clk.t += 40.0                                   # let post windows close between events
    c.tick(clk.t)
  c.close(clk.t)
  return c


if __name__ == '__main__':
  out = sys.argv[1]
  c = replay(out, sys.argv[2:])
  print('events:', [(e['event'], e['kinds']) for e in c.done])
  print('triggers by kind:')
  import collections
  print(dict(collections.Counter(k for _, k, _ in c.timeline)))
