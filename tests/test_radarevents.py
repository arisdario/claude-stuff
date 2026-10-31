#!/usr/bin/env python3
"""Tests for radarevents.py (no openpilot needed):  python3 tests/test_radarevents.py"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import radarevents as RE  # noqa: E402

LANE_L = [0, 0, 0, 1.75]; LANE_R = [0, 0, 0, -1.75]; PATH = [0, 0, 0, 0.0]


def state(op=True, v=20.0, sp=False, br=False, gas=False, tq=0.0, edge_l=3.5, edge_r=-3.5, objs=None, gps=(45.0, 9.0, 90.0),
          lead=None, a=0.0, fcw=False, alert=''):
  return {'v': v, 'a': a, 'ang': 1.0, 'op': op, 'acc': op, 'drv': {'sp': sp, 'st': tq, 'br': br, 'gas': gas},
          'lead': lead, 'plan': {'aT': a, 'vT': v, 'fcw': fcw, 'lead': lead is not None}, 'alert': alert,
          'model': {'p': PATH, 'l': LANE_L, 'r': LANE_R, 'pl': 0.9, 'pr': 0.9,
                    'e': {'l': [edge_l] * 33, 'r': [edge_r] * 33, 'sl': 0.3, 'sr': 0.3}},
          'o': objs or [], 'gps': {'lat': gps[0], 'lon': gps[1], 'hdg': gps[2]}}


def run(rec, states, t0=1000.0):
  """feed (n, state) pairs at 10 Hz; returns finished events and the end time"""
  evs = []; t = t0
  for n, st in states:
    for _ in range(n):
      e = rec.update(st, t)
      if e: evs.append(e)
      t += 0.1
  return evs, t


class TestClearances(unittest.TestCase):
  def test_lane_edge_object(self):
    c = RE.clearances(state(edge_r=-1.2, objs=[{'d': 8.0, 'y': 1.5, 'stationary': True}, {'d': 8.0, 'y': -5.0, 'stationary': False}]))
    self.assertAlmostEqual(c['lane_l'], 1.75 - RE.HALF_W, places=2)
    self.assertAlmostEqual(c['edge_r'], 1.2 - RE.HALF_W, places=2)
    self.assertAlmostEqual(c['obj_l'], 1.5 - RE.HALF_W, places=2)
    self.assertIsNone(c['obj_r'])                       # moving objects don't count as walls

  def test_old_model_has_no_edges(self):
    st = state(); del st['model']['e']
    c = RE.clearances(st)
    self.assertIsNone(c['edge_l']); self.assertIsNotNone(c['lane_l'])

  def test_object_measured_from_the_path_in_a_bend(self):
    st = state(objs=[{'d': 10.0, 'y': 2.0, 'stationary': True}])
    st['model']['p'] = [0, 0.02, 0, 0]                  # path curves left: 2 m left at 10 m
    self.assertAlmostEqual(RE.clearances(st)['obj_r'], 0.0 - RE.HALF_W, places=2)


class TestRecorder(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp(); self.rec = RE.Recorder(out_dir=self.dir, segment_dir=self.dir)

  def test_steer_correction_records_pre_and_post(self):
    evs, _ = run(self.rec, [(200, state()), (5, state(sp=True, tq=2.5)), (60, state())])
    self.assertEqual(len(evs), 1); ev = evs[0]
    self.assertEqual(ev['type'], 'steer')
    span = ev['samples'][-1]['t'] - ev['samples'][0]['t']
    self.assertGreater(span, RE.PRE_S + RE.POST_S - 0.6)
    self.assertTrue(os.path.exists(os.path.join(self.dir, ev['file'])))

  def test_resting_hands_are_not_a_correction(self):
    evs, _ = run(self.rec, [(100, state()), (200, state(sp=True, tq=0.6)), (60, state())])   # hands on, light grip
    self.assertEqual(evs, [])

  def test_short_touch_is_not_an_intervention(self):
    evs, _ = run(self.rec, [(100, state()), (2, state(sp=True, tq=2.5)), (80, state())])
    self.assertEqual(evs, [])

  def test_disengage_by_brake(self):
    evs, _ = run(self.rec, [(100, state()), (2, state(br=True)), (70, state(op=False, br=True))])
    self.assertEqual([(e['type'], e['cause'], e['side']) for e in evs], [('disengage', 'brake', None)])   # nothing close

  def test_steer_then_disengage_is_one_event(self):
    evs, _ = run(self.rec, [(100, state()), (5, state(sp=True, tq=2.5)), (70, state(op=False, sp=True, tq=2.5))])
    self.assertEqual(len(evs), 1)
    self.assertEqual(evs[0]['type'], 'steer'); self.assertEqual(evs[0]['disengaged'], 'steer')

  def test_close_pass_without_takeover(self):
    evs, _ = run(self.rec, [(100, state()), (10, state(edge_r=-1.1)), (60, state())])
    self.assertEqual(len(evs), 1)
    self.assertEqual((evs[0]['type'], evs[0]['side']), ('close', 'right'))
    self.assertAlmostEqual(evs[0]['min_clear']['edge_r'], 1.1 - RE.HALF_W, places=2)

  def test_close_to_stationary_object(self):
    wall = [{'d': 6.0, 'y': 1.1, 'stationary': True}]
    evs, _ = run(self.rec, [(100, state()), (10, state(objs=wall)), (60, state())])
    self.assertEqual((evs[0]['type'], evs[0]['side']), ('close', 'left'))

  def test_no_close_events_when_not_engaged_or_slow(self):
    evs, _ = run(self.rec, [(50, state(op=False, edge_r=-1.0)), (50, state(v=1.0, edge_r=-1.0)), (60, state())])
    self.assertEqual(evs, [])

  def test_mark(self):
    _, t = run(self.rec, [(50, state())]); self.rec.mark('wall on the right')
    evs, _ = run(self.rec, [(60, state())], t0=t)
    self.assertEqual((evs[0]['type'], evs[0]['cause']), ('mark', 'wall on the right'))

  def test_rate_limit(self):
    seq = [(100, state())] + [(5, state(sp=True, tq=2.5)), (20, state())] * 3 + [(60, state())]
    evs, _ = run(self.rec, seq)
    self.assertEqual(len(evs), 1)                       # three corrections within MIN_GAP_S -> one event

  def test_side_from_where_it_was_close(self):
    evs, _ = run(self.rec, [(100, state(edge_l=1.4)), (5, state(edge_l=1.4, sp=True, tq=2.5)), (60, state(edge_l=1.4))])
    self.assertEqual(evs[0]['side'], 'left')

  def test_list_and_load(self):
    run(self.rec, [(100, state()), (5, state(sp=True, tq=2.5)), (60, state())])
    rows = RE.list_events(self.dir)
    self.assertEqual(len(rows), 1); self.assertNotIn('samples', rows[0])
    self.assertIsNotNone(RE.load_event(rows[0]['file'], self.dir))
    self.assertIsNone(RE.load_event('../etc/passwd', self.dir))

  def test_pruning(self):
    for i in range(5):
      with open(os.path.join(self.dir, 'ev_2026010%d_000000_steer.json' % i), 'w') as f: json.dump({}, f)
    old = RE.MAX_FILES; RE.MAX_FILES = 3
    try: run(self.rec, [(100, state()), (5, state(sp=True, tq=2.5)), (60, state())])
    finally: RE.MAX_FILES = old
    self.assertEqual(len([x for x in os.listdir(self.dir) if x.startswith('ev_')]), 3)


def truck_run(lead_last=20, slow_last=8, fcw_from=None, brake=True):
  """30 m/s towards a truck at 5 m/s: radar track 7 from 180 m, 60 samples (6 s); openpilot's lead only for the last
  lead_last samples, car decelerating for the last slow_last; then the driver brakes and openpilot disengages"""
  seq = [(100, state())]
  d = 180.0
  for i in range(60):
    left = 60 - i
    truck = [{'d': round(d, 1), 'y': 0.2, 'vy': 0.0, 'c': 'four_wheel', 'stationary': False, 's': 7, 'vr': -25.0}]
    lead = {'d': round(d, 1), 'vr': -25.0, 'radar': True} if left <= lead_last else None
    seq.append((1, state(objs=truck, lead=lead, a=-1.0 if left <= slow_last else 0.0,
                         fcw=fcw_from is not None and left <= fcw_from)))
    d -= 2.5
  truck = lambda dd: [{'d': dd, 'y': 0.2, 'vy': 0.0, 'c': 'four_wheel', 'stationary': False, 's': 7, 'vr': -25.0}]
  ld = (lambda dd: {'d': dd, 'vr': -25.0, 'radar': True}) if lead_last > 0 else (lambda dd: None)
  if brake:
    seq += [(1, state(br=True, objs=truck(d), lead=ld(d), a=-1.0))]
    seq += [(1, state(br=True, objs=truck(d - 2.5), lead=ld(d - 2.5), a=-3.0))]
    seq += [(60, state(op=False, br=True))]
  return seq


class TestAnalysis(unittest.TestCase):
  def setUp(self):
    self.rec = RE.Recorder(out_dir=tempfile.mkdtemp(), segment_dir='/nonexistent')

  def test_openpilot_late_no_warning(self):
    evs, _ = run(self.rec, truck_run())
    self.assertEqual(len(evs), 1); a = evs[0]['analysis']
    self.assertEqual((evs[0]['type'], evs[0]['cause']), ('disengage', 'brake'))
    self.assertEqual(a['hazard']['id'], 7)
    self.assertAlmostEqual(a['radar_saw'], 6.0, delta=0.15)        # radar had it 6 s before you braked
    self.assertAlmostEqual(a['radar_saw_d'], 180.0, delta=0.1)
    self.assertAlmostEqual(a['op_lead'], 2.0, delta=0.15)          # openpilot's lead only 2 s before
    self.assertTrue(a['op_lead_radar'])
    self.assertAlmostEqual(a['op_slowed'], 0.8, delta=0.15)
    self.assertIsNone(a['warned']); self.assertTrue(a['late'])
    self.assertAlmostEqual(a['ttc'], (180.0 - 60 * 2.5) / 25.0, delta=0.15)
    self.assertIn('no warning', a['verdict'])

  def test_acted_is_when_the_brake_went_down(self):
    evs, _ = run(self.rec, truck_run())
    self.assertAlmostEqual(evs[0]['t'] - evs[0]['acted'], 0.2, delta=0.05)

  def test_openpilot_in_time(self):
    evs, _ = run(self.rec, truck_run(lead_last=55, slow_last=40, fcw_from=30))
    a = evs[0]['analysis']
    self.assertAlmostEqual(a['op_slowed'], 4.0, delta=0.15)
    self.assertAlmostEqual(a['warned'], 3.0, delta=0.15)
    self.assertFalse(a['late'])
    self.assertIn('was slowing', a['verdict'])

  def test_never_its_lead(self):
    evs, _ = run(self.rec, truck_run(lead_last=0, slow_last=0))
    a = evs[0]['analysis']
    self.assertIsNone(a['op_lead']); self.assertIsNone(a['op_slowed'])
    self.assertIn('never took it as its lead', a['verdict'])

  def test_lateral_event_has_no_hazard(self):
    evs, _ = run(self.rec, [(100, state(edge_r=-1.4)), (5, state(edge_r=-1.4, sp=True, tq=2.5)), (60, state())])
    a = evs[0]['analysis']
    self.assertIsNone(a['hazard']); self.assertIn('steering / lane-position', a['verdict'])

  def test_object_beside_the_path_is_not_the_hazard(self):
    car_next_lane = [{'d': 30.0, 'y': 3.6, 'vy': 0.0, 'c': 'four_wheel', 'stationary': False, 's': 3, 'vr': -10.0}]
    evs, _ = run(self.rec, [(100, state(objs=car_next_lane)), (2, state(br=True, objs=car_next_lane)), (60, state(op=False))])
    self.assertIsNone(evs[0]['analysis']['hazard'])


if __name__ == '__main__':
  unittest.main()
