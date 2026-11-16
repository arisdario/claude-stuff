#!/usr/bin/env python3
"""AHBSTALK: the light-lever value in the injected stalk message (teslacan) and the one-shot parked test (ahb_stalk.py)."""
import os
import shutil
import sys
import tempfile
import unittest
from ctypes import create_string_buffer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
try:
  import common.numpy_fast  # noqa: F401
except Exception:
  import types
  _m = types.ModuleType('common.numpy_fast'); _m.clip = lambda x, a, b: max(a, min(b, x))
  _c = types.ModuleType('common'); _c.numpy_fast = _m
  sys.modules['common'] = _c; sys.modules['common.numpy_fast'] = _m
import teslacan  # noqa: E402
import ahb_stalk  # noqa: E402

REAL = {'SpdCtrlLvr_Stat': 0, 'VSL_Enbl_Rq': 1, 'SpdCtrlLvrStat_Inv': 0, 'DTR_Dist_Rq': 255, 'TurnIndLvr_Stat': 0,
        'HiBmLvr_Stat': 0, 'WprWashSw_Psd': 0, 'WprWash_R_Sw_Posn_V2': 0, 'StW_Lvr_Stat': 0, 'StW_Cond_Flt': 0,
        'StW_Cond_Psd': 0, 'HrnSw_Psd': 0, 'WprSw6Posn': 0, 'MC_STW_ACTN_RQ': 7, 'CRC_STW_ACTN_RQ': 0}


class TestStalkBuilder(unittest.TestCase):
  def test_default_is_unchanged(self):
    a = teslacan.create_cruise_adjust_msg(16, 0, dict(REAL))
    b = teslacan.create_cruise_adjust_msg(16, 0, dict(REAL), hiBmLvr_Stat=None)
    self.assertEqual(a, b)
    self.assertEqual((bytes(a[2])[2] >> 2) & 3, 0)                    # the real lever (normal) is copied

  def test_light_lever_override(self):
    real = dict(REAL, WprWashSw_Psd=1, TurnIndLvr_Stat=0)
    for v in (1, 2):
      msg_id, _, raw, bus = teslacan.create_cruise_adjust_msg(None, None, dict(real), hiBmLvr_Stat=v)
      raw = bytes(raw)
      self.assertEqual((msg_id, bus), (0x45, 0))
      self.assertEqual((raw[2] >> 2) & 3, v)                          # HiBmLvr_Stat 18|2
      self.assertEqual((raw[2] >> 4) & 3, 1)                          # wiper wash copied from the real stalk
      self.assertEqual(raw[0] & 0x3F, 0)                              # cruise lever untouched
      self.assertEqual(raw[6] >> 4, 8)                                # counter = real + 1
      self.assertEqual(raw[7], teslacan.add_tesla_crc(msg=create_string_buffer(raw, 8), msg_len=7))   # valid checksum
    self.assertEqual(real['HiBmLvr_Stat'], 0)                         # the caller's copy of the real stalk is not modified


class TestParserFloats(unittest.TestCase):
  def test_floats_from_the_can_parser(self):
    # openpilot's CAN parser gives FLOATS; the test path (spdCtrlLvr_stat=None) copies them (16 Nov 2026: crashed controlsd)
    real = {k: float(v) for k, v in REAL.items()}
    _, _, raw, _ = teslacan.create_cruise_adjust_msg(None, None, dict(real), hiBmLvr_Stat=0)
    self.assertEqual(len(bytes(raw)), 8)
    a = teslacan.create_cruise_adjust_msg(16, 0, dict(real))                 # the ACC's own call: unchanged
    b = teslacan.create_cruise_adjust_msg(16, 0, dict(REAL))
    self.assertEqual(a, b)


class CS(object):
  def __init__(self, v=0., hibm=0, turn=0, cruise=0):
    self.v_ego = v
    self.steering_wheel_stalk = dict(REAL, HiBmLvr_Stat=hibm, TurnIndLvr_Stat=turn, SpdCtrlLvr_Stat=cruise)
    self.ahbHiBeamOn, self.ahbLoBeamOn, self.ahbNightMode = 0, 1, 1


class TestParkedTest(unittest.TestCase):
  def setUp(self):
    self.d = tempfile.mkdtemp()
    self.trig, self.log = os.path.join(self.d, 'ahb_stalk_test'), os.path.join(self.d, 'ahb_stalk_test.log')
    self.now = [100.]
    self.t = ahb_stalk.StalkHighBeamTest(self.trig, self.log, clock=lambda: self.now[0])

  def tearDown(self):
    shutil.rmtree(self.d)

  def run_for(self, s, cs, enabled=False, busy=False):
    out = []
    for _ in range(int(round(s * 20))):
      out.append(self.t.update(cs, enabled, busy))
      self.now[0] += 0.05
    return out

  def test_nothing_without_the_file(self):
    self.assertTrue(all(x is None for x in self.run_for(5, CS())))

  def test_push_once_for_half_a_second_then_done(self):
    open(self.trig, 'w').close()
    out = self.run_for(8, CS())
    self.assertEqual(sum(1 for x in out if x == 1), 10)              # 0.5 s at 20 Hz
    self.assertTrue(all(x in (None, 1) for x in out))
    self.assertFalse(os.path.exists(self.trig)); self.assertTrue(os.path.exists(self.trig + '.done'))
    self.assertIn('push): done', open(self.log).read())
    self.assertTrue(all(x is None for x in self.run_for(5, CS())))     # one-shot: never again

  def test_pull(self):
    open(self.trig, 'w').write('pull\n')
    out = self.run_for(8, CS())
    self.assertEqual(sum(1 for x in out if x == 2), 10)

  def test_dip_needs_the_lever_out(self):
    open(self.trig, 'w').write('dip')
    self.assertTrue(all(x is None for x in self.run_for(3, CS(hibm=0))))   # lever normal: waits
    out = self.run_for(8, CS(hibm=1))                                      # you push it out: 0.5 s of "normal"
    self.assertEqual(sum(1 for x in out if x == 0), 10)
    self.assertTrue(all(x in (None, 0) for x in out))
    self.assertTrue(os.path.exists(self.trig + '.done'))

  def test_waits_for_conditions_then_gives_up(self):
    open(self.trig, 'w').close()
    out = self.run_for(70, CS(v=5.))                                    # moving the whole time
    self.assertTrue(all(x is None for x in out))
    self.assertTrue(os.path.exists(self.trig + '.skipped'))
    self.assertIn('car moving', open(self.log).read())
    for kw in ({'hibm': 1}, {'turn': 1}, {'cruise': 16}):              # any lever in use -> nothing sent
      t = ahb_stalk.StalkHighBeamTest(self.trig + 'x', self.log, clock=lambda: self.now[0])
      open(self.trig + 'x', 'w').close()
      self.assertTrue(all(t.update(CS(**kw), False) is None for _ in range(40)))
    t = ahb_stalk.StalkHighBeamTest(self.trig + 'y', self.log, clock=lambda: self.now[0])
    open(self.trig + 'y', 'w').close()
    self.assertTrue(all(t.update(CS(), True) is None for _ in range(40)))   # openpilot engaged -> nothing

  def test_driver_touching_a_lever_aborts(self):
    open(self.trig, 'w').close()
    out = self.run_for(0.2, CS())
    self.assertIn(1, out)
    out = self.run_for(1, CS(turn=2))                                   # you move the turn-signal lever mid-press
    self.assertTrue(all(x is None for x in out))
    self.assertTrue(os.path.exists(self.trig + '.aborted'))

  def test_moving_aborts(self):
    open(self.trig, 'w').close()
    self.run_for(0.2, CS())
    self.assertTrue(all(x is None for x in self.run_for(1, CS(v=0.5))))
    self.assertTrue(os.path.exists(self.trig + '.aborted'))

  def test_a_cruise_press_wins_the_frame(self):
    open(self.trig, 'w').close()
    out = self.run_for(8, CS(), busy=True)
    self.assertTrue(all(x is None for x in out))                        # skipped while the ACC owns the message
    self.assertTrue(os.path.exists(self.trig + '.done'))


if __name__ == '__main__':
  unittest.main()
