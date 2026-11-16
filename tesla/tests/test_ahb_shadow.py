#!/usr/bin/env python3
"""AHB SHADOW decision (tesla/tools/ahb_shadow.py) and its picture budget in the driving logger."""
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import ahb_shadow as A  # noqa: E402
import driving_logger as DL  # noqa: E402

DARK = {'t': 0., 'n': 0, 'lamps_above': 0, 'band_luma': 20., 'horizon': 325, 'lights': []}
KPH = 1 / 3.6


def run(sh, secs, v_kph=80., lead=None, lights=DARK, fresh=True, t0=[0.]):
  out = None
  for _ in range(int(secs * 5)):
    t0[0] += 0.2
    out = sh.step(t0[0], v_kph * KPH, lead, lights, fresh)
  return out


class TestDecision(unittest.TestCase):
  def test_dark_empty_road_goes_high_only_after_the_hold(self):
    sh = A.AhbShadow()
    self.assertEqual(run(sh, A.HOLD_S - 0.6)[0], 'dip')
    self.assertEqual(run(sh, 1.0)[0], 'high')

  def test_each_reason_dips_at_once(self):
    for kw, why in ((dict(lights=dict(DARK, n=1)), 'headlights'), (dict(lead=90.), 'car_ahead'),
                    (dict(lights=dict(DARK, lamps_above=5)), 'lit_road'), (dict(lights=dict(DARK, band_luma=140.)), 'bright'),
                    (dict(fresh=False), 'no_camera'), (dict(lights=None), 'no_camera'), (dict(v_kph=20.), 'slow')):
      sh = A.AhbShadow()
      run(sh, 5)
      self.assertEqual(sh.state, 'high')
      st, w, changed = run(sh, 0.2, **kw)
      self.assertEqual((st, w, changed), ('dip', why, True), kw)

  def test_far_car_ahead_does_not_dip(self):
    sh = A.AhbShadow()
    self.assertEqual(run(sh, 5, lead=180.)[0], 'high')

  def test_speed_hysteresis(self):
    sh = A.AhbShadow()
    self.assertEqual(run(sh, 5, v_kph=35.)[1], 'slow')          # from a stop: needs 40 km/h
    self.assertEqual(run(sh, 5, v_kph=45.)[0], 'high')
    self.assertEqual(run(sh, 1, v_kph=35.)[0], 'high')          # once going, only below 30 dips
    self.assertEqual(run(sh, 0.2, v_kph=25.)[1], 'slow')

  def test_a_flicker_of_light_restarts_the_hold(self):
    sh = A.AhbShadow()
    run(sh, 5)
    run(sh, 0.2, lights=dict(DARK, n=2))
    self.assertEqual(run(sh, A.HOLD_S - 0.6)[0], 'dip')
    self.assertEqual(run(sh, 1.0)[0], 'high')

  def test_line_and_reader(self):
    sh = A.AhbShadow()
    d = json.loads(sh.line(1000., 22.2, None, dict(DARK, lights=[[500, 340, 64, 0.8]])))
    self.assertEqual((d['src'], d['state'], d['lead_d'], d['lights']), ('ahb', 'dip', None, [[500, 340, 64, 0.8]]))
    tmp = tempfile.mkdtemp()
    try:
      p = os.path.join(tmp, 'l.json')
      self.assertEqual(A.read_lights(1000., p), (None, False))                 # feature off: no file
      open(p, 'w').write(json.dumps(dict(DARK, t=999.5)))
      self.assertEqual(A.read_lights(1000., p)[1], True)
      self.assertEqual(A.read_lights(1002., p)[1], False)                      # stale
      open(p, 'w').write('{"t": 99')                                           # torn write: ignored, no crash
      self.assertEqual(A.read_lights(1000., p), (None, False))
    finally:
      shutil.rmtree(tmp)


class TestPictureBudget(unittest.TestCase):
  def test_ahb_pictures_have_their_own_budget(self):
    d = tempfile.mkdtemp()
    try:
      w = DL.ChunkWriter(root=d, background=False)
      n_acc = sum(w.add_frame(1000. + i, b'x', 'acc_press') for i in range(30))
      n_ahb = sum(w.add_frame(2000. + i, b'x', 'ahb_dip') for i in range(30))
      self.assertEqual(n_acc, DL.FRAMES_PER_CHUNK)                              # routine budget used up ...
      self.assertEqual(n_ahb, DL.FRAMES_AHB_PER_CHUNK)                          # ... AHB moments still get theirs
    finally:
      shutil.rmtree(d)


if __name__ == '__main__':
  unittest.main()
