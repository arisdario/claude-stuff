#!/usr/bin/env python3
"""Tests for uds_relay.py (no openpilot needed):  python3 tesla/tests/test_uds_relay.py"""
import os
import socket
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import uds_relay as R  # noqa: E402


class TestRelay(unittest.TestCase):
  def setUp(self):
    self.flag = os.path.join(tempfile.mkdtemp(), 'uds_read'); open(self.flag, 'w').close()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.bind(('127.0.0.1', 0)); self.port = s.getsockname()[1]; s.close()
    self.r = R.UdsRelay(port=self.port, flag_file=self.flag)
    self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); self.t = 100.0
    self.r.update(False, self.cs(), self.t)                               # opens the port

  def cs(self, gear='park', v=0.0):
    return types.SimpleNamespace(gear_shifter=gear, v_ego=v)

  def put(self, addr, bus, data):
    self.out.sendto(R.pack(addr, bus, data), ('127.0.0.1', self.port))

  def step(self, enabled=False, cs=None):
    import time; time.sleep(0.01); self.t += 0.01
    return self.r.update(enabled, cs or self.cs(), self.t)

  def test_read_passes(self):
    self.put(0x645, 0, b'\x03\x22\xf1\x90\x55\x55\x55\x55')
    self.assertEqual(self.step(), [[0x645, 0, b'\x03\x22\xf1\x90\x55\x55\x55\x55', 0]])

  def test_writes_and_security_dropped(self):
    for d in (b'\x02\x27\x01\x55\x55\x55\x55\x55', b'\x04\x2e\xf1\x90\x00\x55\x55\x55', b'\x02\x11\x01\x55\x55\x55\x55\x55',
              b'\x04\x14\xff\xff\xff\x55\x55\x55', b'\x02\x10\x03\x55\x55\x55\x55\x55', b'\x10\x0a\x2e\xf1\x90\x00\x00\x00'):
      self.put(0x645, 0, d)
    self.assertEqual(self.step() + self.step() + self.step() + self.step(), [])

  def test_other_ids_and_buses_dropped(self):
    self.put(0x2b9, 0, b'\x03\x22\xf1\x90\x55\x55\x55\x55'); self.put(0x645, 1, b'\x03\x22\xf1\x90\x55\x55\x55\x55')
    self.assertEqual(self.step() + self.step(), [])

  def test_only_parked(self):
    f = b'\x03\x22\xf1\x90\x55\x55\x55\x55'
    self.put(0x645, 0, f); self.assertEqual(self.step(cs=self.cs('drive', 0.0)), [])
    self.put(0x645, 0, f); self.assertEqual(self.step(cs=self.cs('park', 3.0)), [])
    self.put(0x645, 0, f); self.assertEqual(self.step(enabled=True), [])

  def test_off_without_flag(self):
    os.remove(self.flag); self.r._flag_t = 0
    self.put(0x645, 0, b'\x03\x22\xf1\x90\x55\x55\x55\x55'); self.assertEqual(self.step(), [])

  def test_rate_limited_not_lost(self):
    for i in range(5): self.put(0x645, 0, b'\x03\x22\xf1\x90\x55\x55\x55\x55')
    a = self.step(); b = self.step(); c = self.step()
    self.assertEqual((len(a), len(b), len(c)), (R.PER_UPDATE, R.PER_UPDATE, 1))


if __name__ == '__main__':
  unittest.main()
