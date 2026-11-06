#!/usr/bin/env python3
"""Tests for driving_logger.py's pure decode/classification logic (no cereal, no car, no messaging):
    python3 -m unittest tesla/tests/test_driving_logger.py

These lock the 0x659 bit positions against the two ground-truth sources in this repo: the packer
(teslacan.create_fake_DAS_msg) and the firmware's own parser of the same frame (safety_tesla.h
tesla_tx_hook). If either ever changes, this test (and driving_logger's decode) must change with it.
"""
import os
import struct
import sys
import unittest
from ctypes import create_string_buffer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import driving_logger as DL  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
try:
  import common.numpy_fast  # noqa: F401
except Exception:
  import types
  _m = types.ModuleType('common.numpy_fast'); _m.clip = lambda x, a, b: max(a, min(b, x))
  _c = types.ModuleType('common'); _c.numpy_fast = _m
  sys.modules['common'] = _c; sys.modules['common.numpy_fast'] = _m
import teslacan  # noqa: E402


def pack_659(speed_control_enabled=0, enable_steer_control=0, op_status=1, speed_override=0, apUnavailable=0,
             collision_warning=0, cc_state=0, pcc_available=0, alca_state=0):
  """build a real 0x659 frame with teslacan's own packer, so the decode test exercises the real wire format"""
  _, _, raw, _ = teslacan.create_fake_DAS_msg(
      speed_control_enabled, speed_override, apUnavailable, collision_warning, op_status,
      acc_speed_kph=50, turn_signal_needed=0, forward_collission_warning=0, adaptive_cruise=0,
      hands_on_state=0, cc_state=cc_state, pcc_available=pcc_available, alca_state=alca_state,
      acc_speed_limit=0, legal_speed_limit=0, apply_angle=0.0, enable_steer_control=enable_steer_control,
      park_brake_request=0)
  return bytes(raw)


class TestDecode659(unittest.TestCase):
  def test_both_off(self):
    d = DL.decode_0x659(pack_659(speed_control_enabled=0, enable_steer_control=0))
    self.assertFalse(d['long_enabled']); self.assertFalse(d['steer_enabled'])

  def test_long_only(self):
    d = DL.decode_0x659(pack_659(speed_control_enabled=1, enable_steer_control=0))
    self.assertTrue(d['long_enabled']); self.assertFalse(d['steer_enabled'])

  def test_steer_only(self):
    d = DL.decode_0x659(pack_659(speed_control_enabled=0, enable_steer_control=1))
    self.assertFalse(d['long_enabled']); self.assertTrue(d['steer_enabled'])

  def test_both_on(self):
    d = DL.decode_0x659(pack_659(speed_control_enabled=1, enable_steer_control=1))
    self.assertTrue(d['long_enabled']); self.assertTrue(d['steer_enabled'])

  def test_op_status_and_other_bits(self):
    d = DL.decode_0x659(pack_659(op_status=5, speed_override=1, apUnavailable=1, collision_warning=1,
                                  cc_state=2, pcc_available=1, alca_state=5))
    self.assertEqual(d['op_status'], 5); self.assertEqual(d['op_status_text'], 'warning')
    self.assertTrue(d['speed_override']); self.assertTrue(d['ap_unavailable']); self.assertTrue(d['collision_warning'])
    self.assertEqual(d['cc_state'], 2); self.assertTrue(d['pcc_available']); self.assertEqual(d['alca_state'], 5)

  def test_short_or_missing_data_is_all_none(self):
    for bad in (None, b'', b'\x00' * 3):
      d = DL.decode_0x659(bad)
      self.assertIsNone(d['long_enabled']); self.assertIsNone(d['steer_enabled']); self.assertIsNone(d['op_status'])

  def test_matches_firmware_bit_positions_directly(self):
    """cross-check against the raw formula in safety_tesla.h (not just the packer), spelled out here"""
    raw = struct.pack('BBBBBBBB', 0x80, 0, 0, 0, 0, 0, 0, 0x80)   # byte0 bit7 + byte7 bit7 set, rest 0
    d = DL.decode_0x659(raw)
    self.assertTrue(d['long_enabled']); self.assertTrue(d['steer_enabled']); self.assertEqual(d['op_status'], 0)


class TestClassifier(unittest.TestCase):
  def test_unknown_with_no_controls_state(self):
    c = DL.Classifier()
    self.assertEqual(c.update(0.0, None, None, None, None, None, None), 'UNKNOWN')

  def test_human_driving_when_disengaged_and_settled(self):
    c = DL.Classifier()
    c.update(0.0, False, 0.0, None, None, None, None)
    self.assertEqual(c.update(1.0, False, 0.0, None, None, None, None), 'HUMAN_DRIVING')

  def test_transition_right_after_engage(self):
    c = DL.Classifier()
    c.update(0.0, False, 0.0, False, 0.0, False, 0.0)
    self.assertEqual(c.update(0.05, True, 0.0, False, 0.0, False, 0.0), 'TRANSITION')

  def test_engaged_once_settled_with_both_axes_active(self):
    c = DL.Classifier()
    c.update(0.0, True, 0.0, True, 0.0, True, 0.0)
    self.assertEqual(c.update(1.0, True, 1.0, True, 1.0, True, 1.0), 'OPENPILOT_ENGAGED')

  def test_longitudinal_only(self):
    c = DL.Classifier()
    c.update(0.0, True, 0.0, True, 0.0, False, 0.0)
    self.assertEqual(c.update(1.0, True, 1.0, True, 1.0, False, 1.0), 'OPENPILOT_LONGITUDINAL_ONLY')

  def test_lateral_only(self):
    c = DL.Classifier()
    c.update(0.0, True, 0.0, False, 0.0, True, 0.0)
    self.assertEqual(c.update(1.0, True, 1.0, False, 1.0, True, 1.0), 'OPENPILOT_LATERAL_ONLY')

  def test_engaged_but_unknown_sub_state_when_659_stale(self):
    c = DL.Classifier()
    c.update(0.0, True, 0.0, True, 0.0, True, 0.0)
    # settled (no flip for >TRANSITION_S), but the 0x659-derived ages are now way stale
    self.assertEqual(c.update(5.0, True, 0.0, True, 5.0, True, 5.0), 'OPENPILOT_ENGAGED')

  def test_stale_controls_state_is_unknown_not_stuck_enabled(self):
    c = DL.Classifier()
    c.update(0.0, True, 0.0, True, 0.0, True, 0.0)
    self.assertEqual(c.update(5.0, True, 5.0, True, 0.0, True, 0.0), 'UNKNOWN')


class TestLeadDict(unittest.TestCase):
  class FakeLead(object):
    status = True
    dRel = 42.0
    vRel = -1.5
    yRel = 0.1
    aRel = 0.2
    vLeadK = 20.0
    aLeadK = 0.0

  def test_none_lead_is_all_none(self):
    d = DL.lead_dict(None)
    self.assertEqual(set(d.keys()), set(DL.LEAD_FIELDS))
    self.assertTrue(all(v is None for v in d.values()))

  def test_real_fields_pulled_through(self):
    d = DL.lead_dict(self.FakeLead())
    self.assertTrue(d['status']); self.assertEqual(d['dRel'], 42.0); self.assertEqual(d['vRel'], -1.5)

  def test_missing_field_on_schema_is_none_not_a_crash(self):
    class Partial(object):
      status = True
      dRel = 10.0
      # vRel, yRel, aRel, vLeadK, aLeadK intentionally absent - simulates an older/different schema
    d = DL.lead_dict(Partial())
    self.assertTrue(d['status']); self.assertEqual(d['dRel'], 10.0)
    for f in ('vRel', 'yRel', 'aRel', 'vLeadK', 'aLeadK'):
      self.assertIsNone(d[f])


if __name__ == '__main__':
  unittest.main()
