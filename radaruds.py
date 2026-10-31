#!/usr/bin/env python3
"""
radaruds.py - read-only UDS (ISO 14229) over ISO-TP for radarweb. Plain Python, no openpilot imports.
Tests: tests/test_radaruds.py

Only services that need NO security access, in the ECU's default session, and that only READ:
  0x22 ReadDataByIdentifier       identification DIDs (VIN, part number, software/hardware version, serial)
  0x19 ReadDTCInformation         sub-functions 0x01 (count), 0x02 (by status mask), 0x0A (supported) - fault codes
  0x3E TesterPresent              sub-function 0x00 / 0x80
plus ISO-TP flow-control frames (0x30) so multi-frame answers can be received.
Everything else is refused here, again in the carcontroller relay (tesla/uds_relay.py) and, with the panda patch, in
the panda firmware: no session change, no SecurityAccess (0x27), no writes (0x2E/0x3D), no routines (0x31), no
resets (0x11), no clearing of fault codes (0x14), no communication or DTC-setting control (0x28/0x85), no I/O
control (0x2F), no downloads/uploads. A refusal from the ECU (negative response) is shown and never retried with
anything stronger.

Requests go out only while the car is parked (enforced in the relay).
"""
import time

# ---------------- what may be sent ----------------
READ_SIDS = {0x22, 0x19, 0x3E}
DTC_SUBS = {0x01, 0x02, 0x0A}
TP_SUBS = {0x00, 0x80}

# request id -> (name, bus). Response = request + 0x10 (Tesla convention, e.g. radar 0x641 -> 0x651).
# Not included: DAS 0x649 (the panda is the DAS - its answer id 0x659 is a DAS message we send ourselves).
TARGETS = {
  0x641: ('Radar (Bosch)', 1),
  0x645: ('ESP / ABS', 0), 0x644: ('Parking brake (EPB)', 0), 0x647: ('Parking brake module', 0),
  0x643: ('Occupant classification', 0), 0x64B: ('Air suspension', 0), 0x64C: ('Instrument cluster', 0),
  0x64E: ('Parking sensors', 0), 0x64F: ('TPMS', 0), 0x730: ('Power steering (EPAS)', 0),
  0x630: ('Power steering 3P', 0), 0x6E0: ('Driver seat', 0), 0x6E2: ('Passenger seat', 0),
}
# 0x641 on the CAR bus is the airbag module (RCM); listed separately so the radar entry keeps the radar bus
TARGETS_CAR_RCM = (0x641, 'Airbag module (RCM)', 0)


def target_list():
  out = [(a, n, b) for a, (n, b) in sorted(TARGETS.items())]
  out.append(TARGETS_CAR_RCM)
  return out


def allowed_target(addr, bus):
  return any(a == addr and b == bus for a, n, b in target_list())


def read_only(payload):
  """True when this UDS request payload (no ISO-TP header) only reads"""
  if not payload: return False
  sid = payload[0]
  if sid not in READ_SIDS: return False
  if sid == 0x22: return len(payload) >= 3 and (len(payload) - 1) % 2 == 0
  if sid == 0x19: return len(payload) >= 2 and payload[1] in DTC_SUBS
  if sid == 0x3E: return len(payload) == 2 and payload[1] in TP_SUBS
  return False


def frame_ok(data):
  """True when this raw 8-byte CAN frame is a read-only ISO-TP single frame or a flow-control frame.
  The same rule is in tesla/uds_relay.py and the panda patch."""
  if not data: return False
  pci = data[0] >> 4
  if pci == 0:
    n = data[0] & 0x0F
    return 1 <= n <= 7 and len(data) >= 1 + n and read_only(bytes(data[1:1 + n]))
  if pci == 3:
    return (data[0] & 0x0F) == 0      # flow control "continue to send" only
  return False


def single_frame(payload):
  if not read_only(payload) or len(payload) > 7: raise ValueError('not a read-only single-frame request')
  return bytes([len(payload)]) + bytes(payload) + b'\x55' * (7 - len(payload))


FLOW_CONTROL = bytes([0x30, 0x00, 0x00]) + b'\x55' * 5     # send everything, no gap

# ---------------- identification DIDs (ISO 14229 standard ones) ----------------
DIDS = {
  0xF190: 'VIN', 0xF18C: 'ECU serial number', 0xF187: 'Part number', 0xF18A: 'Supplier',
  0xF191: 'Hardware number', 0xF193: 'Hardware version', 0xF195: 'Software version', 0xF197: 'System name',
  0xF180: 'Boot software', 0xF181: 'Application software',
}
IDENT_DIDS = (0xF190, 0xF187, 0xF18C, 0xF195, 0xF193, 0xF18A, 0xF197)

NRC = {0x10: 'general reject', 0x11: 'service not supported', 0x12: 'sub-function not supported',
       0x13: 'wrong length', 0x14: 'response too long', 0x21: 'busy, repeat', 0x22: 'conditions not correct',
       0x24: 'request sequence error', 0x31: 'not available (out of range)', 0x33: 'security access denied',
       0x7E: 'sub-function not supported in this session', 0x7F: 'service not supported in this session',
       0x78: 'response pending'}

DTC_STATUS_BITS = ('test failed', 'failed this cycle', 'pending', 'confirmed', 'not completed since clear',
                   'failed since clear', 'not completed this cycle', 'warning lamp')


def dtc_code(b0, b1, b2):
  """3-byte DTC -> 'C1234-56' style (SAE J2012 letter from the top two bits)"""
  letter = 'PCBU'[b0 >> 6]
  return '%s%01X%03X-%02X' % (letter, (b0 >> 4) & 0x3, ((b0 & 0x0F) << 8) | b1, b2)


def text(b):
  s = bytes(b).rstrip(b'\x00\xff ').decode('ascii', 'replace')
  return s if s and all(32 <= ord(c) < 127 for c in s) else bytes(b).hex()


def decode_response(req, resp):
  """req/resp: full UDS payloads. Returns a dict for display."""
  if not resp: return {'ok': False, 'error': 'no answer'}
  if resp[0] == 0x7F:
    code = resp[2] if len(resp) > 2 else 0
    return {'ok': False, 'nrc': code, 'error': NRC.get(code, 'NRC 0x%02X' % code)}
  if resp[0] != (req[0] + 0x40) & 0xFF: return {'ok': False, 'error': 'unexpected answer ' + bytes(resp[:4]).hex()}
  if req[0] == 0x22:
    did = (resp[1] << 8) | resp[2] if len(resp) >= 3 else None
    return {'ok': True, 'did': did, 'name': DIDS.get(did, 'DID 0x%04X' % (did or 0)), 'value': text(resp[3:]),
            'hex': bytes(resp[3:]).hex()}
  if req[0] == 0x19:
    sub = resp[1] if len(resp) > 1 else None
    if sub == 0x01 and len(resp) >= 6:
      return {'ok': True, 'count': (resp[4] << 8) | resp[5]}
    if sub in (0x02, 0x0A) and len(resp) >= 3:
      dtcs = []
      for i in range(3, len(resp) - 3, 4):
        b0, b1, b2, st = resp[i:i + 4]
        if b0 == b1 == b2 == 0: continue
        dtcs.append({'code': dtc_code(b0, b1, b2), 'raw': '%02X%02X%02X' % (b0, b1, b2), 'status': st,
                     'active': bool(st & 0x01), 'confirmed': bool(st & 0x08),
                     'flags': [n for k, n in enumerate(DTC_STATUS_BITS) if st & (1 << k)]})
      return {'ok': True, 'dtcs': dtcs, 'avail_mask': resp[2]}
  return {'ok': True, 'hex': bytes(resp).hex()}


def matches(req, resp):
  """is this answer (or the start of it) for our request? The gateway also talks to these modules on the same ids."""
  if not resp: return False
  if resp[0] == 0x7F: return len(resp) >= 2 and resp[1] == req[0]
  if resp[0] != (req[0] + 0x40) & 0xFF: return False
  if req[0] == 0x22: return len(resp) >= 3 and resp[1] == req[1] and resp[2] == req[2]
  if req[0] == 0x19: return len(resp) >= 2 and resp[1] == req[1]
  return True


# ---------------- ISO-TP receive ----------------
class IsoTpRx:
  """reassembles one ISO-TP answer; feed() returns 'fc' when a flow-control frame must be sent, 'done' when complete"""
  def __init__(self):
    self.reset()

  def reset(self):
    self.buf = b''; self.total = 0; self.next_sn = 1; self.payload = None

  def feed(self, data):
    data = bytes(data)
    if not data: return None
    pci = data[0] >> 4
    if pci == 0:
      n = data[0] & 0x0F
      self.payload = data[1:1 + n]; return 'done'
    if pci == 1:
      self.total = ((data[0] & 0x0F) << 8) | data[1]; self.buf = data[2:8]; self.next_sn = 1
      return 'fc'
    if pci == 2 and self.total:
      if (data[0] & 0x0F) != self.next_sn & 0x0F: self.reset(); return 'error'
      self.next_sn += 1; self.buf += data[1:8]
      if len(self.buf) >= self.total:
        self.payload = self.buf[:self.total]; self.total = 0; return 'done'
    return None


# ---------------- request queue ----------------
TIMEOUT_S = 0.5
PENDING_MAX_S = 5.0           # an ECU answering 0x78 'response pending' gets this long in total


class UdsClient:
  """runs one request at a time. send(addr, bus, data) is called to put a frame on the bus (via the relay).
  feed(addr, bus, data, now) with every received frame. poll(now) at 10 Hz or more."""
  def __init__(self, send):
    self.send = send; self.queue = []; self.cur = None; self.results = []; self.rx = IsoTpRx()

  def request(self, addr, bus, payload, tag=None):
    if not allowed_target(addr, bus): raise ValueError('unknown module 0x%X bus %d' % (addr, bus))
    single_frame(payload)                                       # raises if not read-only
    self.queue.append({'addr': addr, 'bus': bus, 'req': bytes(payload), 'tag': tag})

  def busy(self):
    return self.cur is not None or bool(self.queue)

  def _start(self, now):
    self.cur = self.queue.pop(0); self.cur['t0'] = now; self.cur['t'] = now; self.rx.reset()
    self.send(self.cur['addr'], self.cur['bus'], single_frame(self.cur['req']))

  def _finish(self, resp, now):
    c = self.cur; self.cur = None
    r = decode_response(c['req'], resp)
    r.update({'addr': c['addr'], 'bus': c['bus'], 'req': c['req'].hex(), 'tag': c['tag'], 't': round(now, 2),
              'ms': int((now - c['t0']) * 1000)})
    self.results.append(r); self.results = self.results[-500:]
    return r

  def feed(self, addr, bus, data, now):
    c = self.cur
    if c is None or bus != c['bus'] or addr != c['addr'] + 0x10: return None
    data = bytes(data)
    if data and data[0] >> 4 == 1 and not matches(c['req'], data[2:5]):   # someone else's answer (the gateway polls too)
      return None
    st = self.rx.feed(data)
    if st == 'fc':
      c['t'] = now; self.send(c['addr'], c['bus'], FLOW_CONTROL)
    elif st == 'done':
      p = self.rx.payload; self.rx.reset()
      if not matches(c['req'], p): return None
      if len(p) >= 3 and p[0] == 0x7F and p[2] == 0x78:          # response pending: wait longer
        c['t'] = now; c['pending'] = True; return None
      return self._finish(p, now)
    return None

  def poll(self, now):
    c = self.cur
    if c is not None:
      lim = PENDING_MAX_S if c.get('pending') else TIMEOUT_S
      if now - c['t'] > lim or now - c['t0'] > PENDING_MAX_S: return self._finish(b'', now)
      return None
    if self.queue: self._start(now)
    return None


def plan_scan(client, tag='scan'):
    """identification of every module: VIN + part + software version"""
    for a, n, b in target_list():
      for did in (0xF190, 0xF187, 0xF195):
        client.request(a, b, bytes([0x22, did >> 8, did & 0xFF]), tag=tag)


def plan_ident(client, addr, bus, tag='ident'):
  for did in IDENT_DIDS: client.request(addr, bus, bytes([0x22, did >> 8, did & 0xFF]), tag=tag)


def plan_dtcs(client, addr, bus, tag='dtc'):
  client.request(addr, bus, bytes([0x19, 0x02, 0xFF]), tag=tag)


def summarize(results):
  """per module: name, answered?, identification, fault codes"""
  mods = {}
  for a, n, b in target_list():
    mods[(a, b)] = {'addr': a, 'bus': b, 'name': n, 'answered': False, 'ident': {}, 'dtcs': None, 'errors': []}
  for r in results:
    m = mods.get((r['addr'], r['bus']))
    if m is None: continue
    if r.get('ok') or r.get('nrc') is not None: m['answered'] = True
    if r.get('ok') and 'did' in r: m['ident'][r['name']] = r['value']
    elif r.get('ok') and 'dtcs' in r: m['dtcs'] = r['dtcs']
    elif not r.get('ok'): m['errors'].append({'req': r['req'], 'error': r['error']})
    m['errors'] = m['errors'][-5:]; m['t'] = r['t']
  return [m for m in mods.values()]


def now():
  return time.monotonic()
