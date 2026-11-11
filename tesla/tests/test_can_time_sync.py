#!/usr/bin/env python3
"""Tests for can_time_sync.decode_gtw_carstate - the bit decode of GTW_carState (0x318) date+time.
    python3 -m unittest tesla/tests/test_can_time_sync.py
"""
import datetime
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import can_time_sync as C  # noqa: E402


class TestDecode(unittest.TestCase):
  def test_real_sample_from_the_scan(self):
    # the actual 0x318 payload captured on the user's car: '9a 0b 09 11 01 35 40 01'
    dat = bytes.fromhex("9a0b091101354001")
    self.assertEqual(C.decode_gtw_carstate(dat), datetime.datetime(2026, 11, 1, 17, 53, 9))

  def test_all_zero_is_invalid(self):
    # month 0 / day 0 are not a valid date -> None, not a crash
    self.assertIsNone(C.decode_gtw_carstate(bytes(8)))

  def test_short_payload_returns_none(self):
    self.assertIsNone(C.decode_gtw_carstate(b"\x00\x00"))
    self.assertIsNone(C.decode_gtw_carstate(None))

  def test_round_trip_a_known_datetime(self):
    # build a payload for 2025-03-14 09:26:53 and confirm it decodes back
    year, month, day, hour, minute, second = 2025, 3, 14, 9, 26, 53
    v = ((year - 2000) & 0x7F)
    v |= (month & 0x0F) << 8
    v |= (second & 0x3F) << 16
    v |= (hour & 0x1F) << 24
    v |= (day & 0x1F) << 32
    v |= (minute & 0x3F) << 40
    dat = v.to_bytes(8, "little")
    self.assertEqual(C.decode_gtw_carstate(dat), datetime.datetime(2025, 3, 14, 9, 26, 53))

  def test_invalid_month_13_is_rejected(self):
    v = (25) | (13 << 8) | (1 << 32)  # month 13
    self.assertIsNone(C.decode_gtw_carstate(v.to_bytes(8, "little")))


if __name__ == "__main__":
  unittest.main()
