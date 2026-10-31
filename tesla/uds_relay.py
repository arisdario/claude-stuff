"""
uds_relay.py - lets radarweb read diagnostic information (UDS, no security access) from the car's modules.

radarweb cannot write to the 'sendcan' socket itself (carcontroller owns it), so it sends each 8-byte frame as a
UDP packet to 127.0.0.1 and carcontroller appends it to its CAN messages. The answers come back on the normal 'can'
socket that radarweb already reads.

Every frame is checked here again, whatever radarweb says:
  - only the listed diagnostic request ids, each on its own bus
  - only read-only ISO-TP single frames (0x22 ReadDataByIdentifier, 0x19 ReadDTCInformation 0x01/0x02/0x0A,
    0x3E TesterPresent) and flow-control frames - never SecurityAccess, writes, routines, resets, session changes,
    clearing fault codes or communication control
  - only with the car in Park, standing still, and openpilot not engaged
  - at most RATE_MAX frames per second
Anything else is dropped silently. Off unless /data/uds_read exists (touch /data/uds_read to enable).
Tests: tesla/tests/test_uds_relay.py
"""
import os
import socket
import struct
import time

PORT = 47710
FLAG_FILE = '/data/uds_read'
RATE_MAX = 20                  # frames per second
PER_UPDATE = 2                 # frames appended per carcontroller cycle

# request id -> bus (same list as radaruds.TARGETS; 0x641 is the radar on bus 1 and the airbag module on bus 0)
ALLOWED = {(0x641, 1), (0x641, 0), (0x645, 0), (0x644, 0), (0x647, 0), (0x643, 0), (0x64B, 0), (0x64C, 0),
           (0x64E, 0), (0x64F, 0), (0x730, 0), (0x630, 0), (0x6E0, 0), (0x6E2, 0)}


def frame_ok(d):
  """read-only ISO-TP single frame, or flow control 'continue'"""
  if len(d) != 8: return False
  pci = d[0] >> 4
  if pci == 3: return (d[0] & 0x0F) == 0
  if pci != 0: return False
  n = d[0] & 0x0F
  if n < 1 or n > 7: return False
  sid = d[1]
  if sid == 0x22: return n >= 3 and (n - 1) % 2 == 0
  if sid == 0x19: return n >= 2 and d[2] in (0x01, 0x02, 0x0A)
  if sid == 0x3E: return n == 2 and d[2] in (0x00, 0x80)
  return False


class UdsRelay:
  def __init__(self, port=PORT, flag_file=FLAG_FILE):
    self.flag_file = flag_file; self.sock = None; self.port = port
    self.sent = []; self._flag_t = 0; self._flag = False

  def _enabled(self, now):
    if now - self._flag_t > 2.0:
      self._flag_t = now; self._flag = os.path.exists(self.flag_file)
    return self._flag

  def _open(self):
    try:
      s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
      s.bind(('127.0.0.1', self.port)); s.setblocking(False); self.sock = s
    except Exception as e:
      print('uds_relay: cannot open port', self.port, e); self.sock = False

  def update(self, enabled, CS, now=None):
    """returns CAN messages [addr, 0, data, bus] to add to this cycle's sends"""
    now = time.monotonic() if now is None else now
    if not self._enabled(now): return []
    if self.sock is None: self._open()
    if not self.sock: return []
    out = []
    parked = (not enabled) and getattr(CS, 'gear_shifter', None) == 'park' and abs(getattr(CS, 'v_ego', 1.0)) < 0.1
    self.sent = [t for t in self.sent if now - t < 1.0]
    while len(out) < PER_UPDATE and len(self.sent) < RATE_MAX:     # the rest waits in the socket for the next cycle
      try:
        pkt = self.sock.recv(64)
      except (BlockingIOError, socket.error):
        break
      if len(pkt) != 11: continue
      addr, bus = struct.unpack('<HB', pkt[:3]); data = pkt[3:]
      if not parked or (addr, bus) not in ALLOWED or not frame_ok(data): continue
      out.append([addr, 0, data, bus]); self.sent.append(now)
    return out


def pack(addr, bus, data):
  """radarweb side: one UDP packet"""
  return struct.pack('<HB', addr, bus) + bytes(data)
