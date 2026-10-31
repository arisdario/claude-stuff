#!/usr/bin/env python3
"""Tests for radaruds.py (no openpilot needed):  python3 tests/test_radaruds.py"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import radaruds as U  # noqa: E402

VIN = b'5YJSA7E21GF000000'


def isotp_frames(payload):
  """ECU side: payload -> CAN frames (single or first + consecutive)"""
  if len(payload) <= 7: return [bytes([len(payload)]) + payload + b'\xaa' * (7 - len(payload))]
  out = [bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6]]
  rest = payload[6:]; sn = 1
  while rest:
    out.append(bytes([0x20 | (sn & 0xF)]) + rest[:7] + b'\xaa' * max(0, 7 - len(rest[:7]))); rest = rest[7:]; sn += 1
  return out


class Ecu(object):
  """answers on req+0x10; multi-frame answers wait for flow control"""
  def __init__(self, addr, bus, answers):
    self.addr, self.bus, self.answers = addr, bus, answers; self.pending = []; self.inbox = []

  def on_frame(self, addr, bus, data):
    if addr != self.addr or bus != self.bus: return []
    self.inbox.append(bytes(data))
    if data[0] >> 4 == 3:
      out, self.pending = self.pending, []; return out
    req = bytes(data[1:1 + (data[0] & 0xF)])
    fr = isotp_frames(self.answers.get(req, bytes([0x7F, req[0], 0x31])))
    self.pending = fr[1:]; return fr[:1]


def run(client, ecus, secs=3.0, cross=None):
  bus = []
  client.send = lambda a, b, d: bus.append((a, b, d))
  t = 0.0
  while t < secs:
    client.poll(t)
    while bus:
      a, b, d = bus.pop(0)
      for e in ecus:
        for f in e.on_frame(a, b, d): client.feed(e.addr + 0x10, e.bus, f, t + 0.005)
    if cross: cross(client, t)
    t += 0.01
  return client.results


class TestReadOnly(unittest.TestCase):
  def test_only_reads_pass(self):
    self.assertTrue(U.read_only(b'\x22\xf1\x90')); self.assertTrue(U.read_only(b'\x19\x02\xff'))
    self.assertTrue(U.read_only(b'\x3e\x80'))
    for bad in (b'\x27\x01', b'\x2e\xf1\x90\x00', b'\x31\x01\x02\x03', b'\x11\x01', b'\x14\xff\xff\xff', b'\x10\x03',
                b'\x28\x00\x01', b'\x85\x02', b'\x2f\x01\x02\x03', b'\x19\x04\x00\x00\x00\x01', b'\x22\xf1', b'\x34', b''):
      self.assertFalse(U.read_only(bad), bad.hex())

  def test_single_frame_refuses_writes(self):
    self.assertEqual(U.single_frame(b'\x22\xf1\x90'), b'\x03\x22\xf1\x90\x55\x55\x55\x55')
    with self.assertRaises(ValueError): U.single_frame(b'\x27\x01')

  def test_frame_ok(self):
    self.assertTrue(U.frame_ok(U.single_frame(b'\x19\x02\xff'))); self.assertTrue(U.frame_ok(U.FLOW_CONTROL))
    self.assertFalse(U.frame_ok(b'\x02\x27\x01\x00\x00\x00\x00\x00'))
    self.assertFalse(U.frame_ok(b'\x10\x0a\x2e\xf1\x90\x00\x00\x00'))     # first frame = long write
    self.assertFalse(U.frame_ok(b'\x31\x00\x00\x00\x00\x00\x00\x00'))     # flow control 'wait'/'overflow'

  def test_unknown_module_refused(self):
    c = U.UdsClient(lambda *a: None)
    with self.assertRaises(ValueError): c.request(0x123, 0, b'\x22\xf1\x90')
    with self.assertRaises(ValueError): c.request(0x645, 1, b'\x22\xf1\x90')       # right id, wrong bus
    with self.assertRaises(ValueError): c.request(0x645, 0, b'\x2e\xf1\x90\x00')


class TestClient(unittest.TestCase):
  def test_vin_multi_frame(self):
    esp = Ecu(0x645, 0, {b'\x22\xf1\x90': b'\x62\xf1\x90' + VIN})
    c = U.UdsClient(None); c.request(0x645, 0, b'\x22\xf1\x90')
    r = run(c, [esp])
    self.assertEqual(len(r), 1); self.assertTrue(r[0]['ok']); self.assertEqual(r[0]['value'], VIN.decode())
    self.assertIn(U.FLOW_CONTROL, esp.inbox)

  def test_dtcs_decoded(self):
    ans = b'\x59\x02\xff' + b'\x52\x34\x16\x09' + b'\xc1\x00\x87\x08' + b'\x00\x00\x00\x00'
    c = U.UdsClient(None); U.plan_dtcs(c, 0x730, 0)
    r = run(c, [Ecu(0x730, 0, {b'\x19\x02\xff': ans})])[0]
    self.assertEqual([d['code'] for d in r['dtcs']], ['C1234-16', 'U0100-87'])
    self.assertTrue(r['dtcs'][0]['active']); self.assertFalse(r['dtcs'][1]['active']); self.assertTrue(r['dtcs'][1]['confirmed'])

  def test_refusal_and_silence(self):
    c = U.UdsClient(None); c.request(0x641, 0, b'\x22\xf1\x90'); c.request(0x64F, 0, b'\x22\xf1\x90')
    r = run(c, [Ecu(0x641, 0, {b'\x22\xf1\x90': b'\x7f\x22\x33'})])
    self.assertEqual(r[0]['error'], 'security access denied'); self.assertEqual(r[1]['error'], 'no answer')
    s = {(m['addr'], m['bus']): m for m in U.summarize(r)}
    self.assertTrue(s[(0x641, 0)]['answered']); self.assertFalse(s[(0x64F, 0)]['answered'])
    self.assertFalse(s[(0x641, 1)]['answered'])                 # the radar (other bus) is a different module

  def test_response_pending_waits(self):
    class Slow(Ecu):
      def on_frame(self, a, b, d):
        if a != self.addr: return []
        return [b'\x03\x7f\x22\x78\xaa\xaa\xaa\xaa', b'\x06\x62\xf1\x95\x31\x2e\x30\xaa']
    c = U.UdsClient(None); c.request(0x645, 0, b'\x22\xf1\x95')
    r = run(c, [Slow(0x645, 0, {})])[0]
    self.assertTrue(r['ok']); self.assertEqual(r['value'], '1.0')

  def test_gateway_crosstalk_ignored(self):
    """the gateway's own answers on the same id (other DID, other service) must not be taken as ours"""
    esp = Ecu(0x645, 0, {b'\x22\xf1\x90': b'\x62\xf1\x90' + VIN})
    def cross(c, t):
      if abs(t - 0.02) < 0.004:
        c.feed(0x655, 0, b'\x05\x62\xf1\x87\x41\x42\xaa\xaa', t)       # gateway asked for the part number
        c.feed(0x655, 0, b'\x03\x7f\x19\x31\xaa\xaa\xaa\xaa', t)       # refusal of a gateway DTC read
        c.feed(0x655, 0, b'\x10\x14\x62\xf1\x87\x30\x31\x32', t)       # gateway multi-frame: no flow control from us
    sent = []
    c = U.UdsClient(None); c.request(0x645, 0, b'\x22\xf1\x90')
    r = run(c, [esp], cross=cross)
    self.assertEqual(len(r), 1); self.assertEqual(r[0]['value'], VIN.decode())
    self.assertEqual(sum(1 for f in esp.inbox if f == U.FLOW_CONTROL), 1)

  def test_scan_covers_every_module(self):
    c = U.UdsClient(lambda *a: None); U.plan_scan(c)
    self.assertEqual(len(c.queue), 3 * len(U.target_list()))
    self.assertTrue(all(U.read_only(q['req']) for q in c.queue))


if __name__ == '__main__':
  unittest.main()
