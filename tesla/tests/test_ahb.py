#!/usr/bin/env python3
"""Tests for AHB_module.py auto mode (no openpilot needed):  python3 tesla/tests/test_ahb.py"""
import os
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
for name in ('selfdrive', 'selfdrive.config', 'cereal', 'cereal.messaging'):
  sys.modules.setdefault(name, types.ModuleType(name))
sys.modules['selfdrive.config'].Conversions = object
sys.modules['cereal'].tesla = types.SimpleNamespace(AHBinfo=None)
sys.modules['cereal'].log = None
_sock = types.SimpleNamespace(receive=lambda non_blocking=True: None)
sys.modules['cereal.messaging'].sub_sock = lambda *a, **k: _sock
sys.modules['cereal.messaging'].recv_one_or_none = lambda s: None
import AHB_module as A  # noqa: E402

UNDECIDED, OFF = A.AHBDecision.DAS_HIGH_BEAM_UNDECIDED, A.AHBDecision.DAS_HIGH_BEAM_OFF
ON, SNA = A.AHBReason.HIGH_BEAM_ON, A.AHBReason.HIGH_BEAM_OFF_REASON_SNA


class CS(object):
  def __init__(self, kmh=100.0, stalk=1, enabled=0):
    self.v_ego = kmh / 3.6; self.ahbHighBeamStalkPosition = stalk; self.ahbEnabled = enabled
    self.ahbHiBeamOn = 0; self.ahbLoBeamOn = 1; self.ahbOffDuration = 5


class TestAHB(unittest.TestCase):
  def setUp(self):
    d = tempfile.mkdtemp()
    A.AHB_AUTO_FILE = os.path.join(d, 'ahb_auto'); A.AHB_TEST_FILE = os.path.join(d, 'ahb_test')
    self.now = [1000000]
    A._current_time_millis = lambda: self.now[0]
    self.c = A.AHBController(None)
    self.c.ahbInfoData = types.SimpleNamespace(radarCarDetected=False, cameraCarDetected=False)
    self.c.frameInfoGain = 510                       # dark scene: camera gain at maximum

  def flag(self, path, on=True):
    if on: open(path, 'w').close()
    elif os.path.exists(path): os.remove(path)
    self.c._flags_t = 0                              # re-read the flag files now

  def step(self, cs, ms=100):
    self.now[0] += ms
    st, reason, en = self.c.update(cs, 0, None)
    return st, reason, en

  def test_without_flag_mcu_setting_still_blocks(self):
    st, reason, en = self.step(CS())
    self.assertEqual((st, reason, en), (UNDECIDED, SNA, False))

  def test_auto_high_beam_on_road_clear(self):
    self.flag(A.AHB_AUTO_FILE)
    st, reason, en = self.step(CS(kmh=100))
    self.assertEqual((st, reason), (UNDECIDED, ON)); self.assertTrue(en)

  def test_auto_below_30_dips(self):
    self.flag(A.AHB_AUTO_FILE)
    self.assertEqual(self.step(CS(kmh=18))[0], OFF)

  def test_auto_country_road_speed_is_high_beam(self):
    self.flag(A.AHB_AUTO_FILE)
    self.assertEqual(self.step(CS(kmh=55))[1], ON)

  def test_auto_hysteresis(self):
    self.flag(A.AHB_AUTO_FILE)
    self.assertEqual(self.step(CS(kmh=32))[1], ON)
    self.assertEqual(self.step(CS(kmh=25))[1], ON)   # between 20 and 30: stays allowed
    self.assertEqual(self.step(CS(kmh=18))[0], OFF)
    self.assertEqual(self.step(CS(kmh=25))[0], OFF)  # between 20 and 30 from below: stays dipped

  def test_auto_still_dips_for_traffic(self):
    self.flag(A.AHB_AUTO_FILE)
    self.c.ahbInfoData.radarCarDetected = True
    self.assertEqual(self.step(CS())[0], OFF)
    self.c.ahbInfoData.radarCarDetected = False; self.c.ahbInfoData.cameraCarDetected = True
    self.assertEqual(self.step(CS())[0], OFF)
    self.c.ahbInfoData.cameraCarDetected = False
    self.assertEqual(self.step(CS(), ms=3000)[0], OFF)        # 5 s hold after the last car
    self.assertEqual(self.step(CS(), ms=2500)[1], ON)

  def test_auto_dips_for_bright_scene(self):
    self.flag(A.AHB_AUTO_FILE)
    self.c.frameInfoGain = 400                       # oncoming headlights / street lights
    self.assertEqual(self.step(CS())[0], OFF)

  def test_stalk_down_means_low_beam(self):
    self.flag(A.AHB_AUTO_FILE)
    self.assertEqual(self.step(CS(stalk=0))[:2], (UNDECIDED, SNA))

  def test_parked_test_mode_toggles(self):
    self.flag(A.AHB_TEST_FILE)
    seen = set(self.step(CS(kmh=0), ms=2000)[0] for _ in range(4))
    self.assertEqual(seen, {OFF, UNDECIDED})

  def test_auto_or_test_tells_carcontroller_to_send(self):
    self.assertFalse(self.c.auto_or_test())
    self.flag(A.AHB_AUTO_FILE); self.assertTrue(self.c.auto_or_test())

  def test_test_mode_does_nothing_while_driving(self):
    self.flag(A.AHB_TEST_FILE)
    self.assertEqual(self.step(CS(kmh=100))[:2], (UNDECIDED, SNA))   # MCU setting off, no auto flag


if __name__ == '__main__':
  unittest.main()
