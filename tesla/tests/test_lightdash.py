#!/usr/bin/env python3
"""lightdash: the panda mirror reproduces the firmware's lane/object frames.  python3 tesla/tests/test_lightdash.py"""
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tools'))
import lightdash as L  # noqa: E402

TC = L.car_packers()          # the car's real teslacan.py (clip stubbed off-car)


class T(unittest.TestCase):
  def test_default_frame_equals_the_real_dump(self):
    """21:31 drive: the cluster got d0 a1 64 7f 7f 7f f0 (+counter) the whole drive = the firmware defaults (lWidth 29 overflow)"""
    m = L.PandaMirror(); b = m.lanes_frame()
    self.assertEqual(b[:7].hex(), 'd0a1647f7f7ff0')
    d = L.decode_lanes(b)
    self.assertAlmostEqual(d['width'], 6.0625); self.assertEqual((d['left'], d['right'], d['range']), (0, 0, 161))

  def test_lane_roundtrip_through_the_cars_packer(self):
    m = L.PandaMirror()
    raw = TC.create_fake_DAS_obj_lane_msg(40., 0.3, 0, 3, 3, 0.35, 0.0, -0.0004, 0.0, 30, 3.7)[2]
    m.feed(0x557, raw, 1.0); m.feed(0x659, bytes([5, 90, 0, 0xC8, 0, 0, 0, 0]), 1.0)
    d = L.decode_lanes(m.lanes_frame())
    self.assertEqual((d['left'], d['right']), (1, 1))
    self.assertAlmostEqual(d['width'], 3.6875 - 0.125, delta=0.32)          # 3.7 m quantised in 0.3125 steps
    self.assertAlmostEqual(d['c'][0], 0.35, delta=0.036)
    self.assertEqual(d['range'], 28)                                          # 30 m -> 7 x 4
    self.assertFalse(m.snapshot(1.0)['default'])

  def test_low_confidence_line_not_drawn(self):
    m = L.PandaMirror()
    m.feed(0x557, TC.create_fake_DAS_obj_lane_msg(0., 0., 0, 3, 1, 0., 0., 0., 0., 30, 3.7)[2], 1.0)   # right q3, left q1 (<=1)
    d = L.decode_lanes(m.lanes_frame()); self.assertEqual((d['left'], d['right']), (0, 1))

  def test_lead_vehicle_distance_is_restored_to_real_metres(self):
    m = L.PandaMirror()
    m.feed(0x559, TC.create_DAS_LR_object_msg(0, 0, 7, 40., 0.7, -2., 0, 0, 0, 0, 0)[2], 1.0)
    objs = [o for o in m.snapshot(1.0)['obj'] if o['slot'] == 'lead']
    self.assertEqual(len(objs), 1)
    self.assertAlmostEqual(objs[0]['dx'], 40., delta=1.01); self.assertAlmostEqual(objs[0]['dy'], 0.7, delta=0.36)
    self.assertEqual(objs[0]['relevant'], 1)

  def test_no_object_slots_are_empty(self):
    self.assertEqual(L.PandaMirror().snapshot(0.0)['obj'], [])
    m = L.PandaMirror(); m.feed(0x559, TC.create_DAS_LR_object_msg(0, 0, 0, 0., 0., 0., 0, 0, 0, 0, 0)[2], 1.0)
    self.assertEqual(m.snapshot(1.0)['obj'], [])

  def test_status_and_stale_reset(self):
    m = L.PandaMirror()
    m.feed(0x557, TC.create_fake_DAS_obj_lane_msg(0., 0., 0, 3, 3, 0.1, 0., 0., 0., 30, 3.7)[2], 1.0)
    m.feed(0x659, bytes([5, 90, 0, 0xC8, 0, 0, 0, 0]), 1.0)
    s = m.snapshot(1.5); self.assertTrue(s['present']); self.assertEqual(s['status']['op'], 5)
    self.assertFalse(m.stale(2.9)); self.assertTrue(m.stale(3.2))
    m.feed(0x659, bytes([2, 0, 0, 0, 0, 0, 0, 0]), 4.0)                        # >2 s gap: firmware reset_DAS_data()
    self.assertEqual(m.status['op'], 2); self.assertFalse(m.have_557 and m.lWidth != 29 and False)
    self.assertEqual(m.lWidth, 29)

  def test_switch_flag(self):
    m = L.PandaMirror(); self.assertIsNone(m.emu)
    m.feed(0x554, bytes([0x20, 0, 0]), 1.0); self.assertTrue(m.emu)          # 3 bytes: the length teslacan.py really sends
    m.feed(0x554, bytes([0x00, 0, 0]), 1.0); self.assertFalse(m.emu)
    m.feed(0x554, bytes([0x20, 0, 0, 0, 0, 0, 0, 0]), 1.0); self.assertTrue(m.emu)   # longer payloads still work

  def test_never_opens_the_panda(self):
    src = open(L.__file__).read()
    for bad in ('import usb', 'from panda', 'import panda', 'libusb', 'Panda('):
      self.assertNotIn(bad, src)


class WireLengths(unittest.TestCase):
  def test_0x554_is_three_bytes_in_the_car_code(self):
    """the bug the dashboard had: it ignored payloads under 8 bytes, but teslacan.py packs 0x554 as 3. Guards the shared assumption."""
    import re
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'teslacan.py')).read()
    i = src.index('msg_id = 0x554')
    self.assertEqual(re.search(r'msg_len = (\d+)', src[i:i + 80]).group(1), '3')


if __name__ == '__main__':
  unittest.main()
