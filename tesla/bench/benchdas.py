#!/usr/bin/env python3
# benchdas.py - BENCH ONLY. Sends the instrument-cluster DAS messages exactly as the panda's do_fake_DAS (safety_tesla.h)
# builds them in the car, plus optionally a spoofed GTW_carConfig (0x398), so you can see on the bench whether the
# MCU2 / cluster draws the Autopilot picture (status, lane lines, car ahead, set speed).
#
# !!! NEVER run this in the car: it uses SAFETY_ALLOUTPUT and fakes the Autopilot computer and the gateway config. !!!
#
# Why it's needed: in the car do_fake_DAS runs on every 0x108 (DI_torque1, 100 Hz) the panda receives. On a bench there
# is no drive inverter, so the panda sends no DAS messages at all and the cluster has nothing to draw.
#
# The byte packing is a line-by-line port of do_fake_DAS / create_DAS_LR_object_msg. Self-check: `--selftest` packs the
# panda's start-up defaults and compares them with frames captured from the car (21:31 drive, DAS emulation on).
#
#   python3 benchdas.py --selftest                                   # no panda needed
#   python3 benchdas.py --bench --secs 60                            # active AP picture, car's own 0x398 untouched
#   python3 benchdas.py --bench --secs 60 --raw398 41154e4f00000010  # + config: dasHw 1, forwardRadarHw 1, autopilot 1
#   python3 benchdas.py --bench --op-status 2                        # "available" instead of "active"
# Python 3.7 compatible (EON).
import argparse, sys, time

IC_LANE_SCALE = 2.0   # teslacan.py


def cksm(data_lo, data_hi, msg_id, msg_len):
  """add_tesla_cksm2: id low + id high + the first msg_len bytes, mod 256"""
  c = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
  for x in range(msg_len):
    v = (data_lo >> (x * 8)) & 0xFF if x <= 3 else (data_hi >> ((x - 4) * 8)) & 0xFF
    c = (c + v) & 0xFF
  return c


def frame(lo, hi, n=8):
  return ((lo & 0xFFFFFFFF) | ((hi & 0xFFFFFFFF) << 32)).to_bytes(8, 'little')[:n]


def clip(x, lo, hi): return max(lo, min(hi, x))


def lr_object(lane, v1Class, v1Id, v1Dx, v1Dy, v1V, v2Class=0, v2Id=0, v2Dx=0, v2Dy=0, v2V=0):
  """teslacan.create_DAS_LR_object_msg (0x559); the panda copies these 8 bytes into DAS_object 0x309 unchanged"""
  important1 = 0
  if v1Dx > 0 and v1Id >= 0:
    if lane == 0: important1 = 1
    v1Class += 1
    if v1Class == 4: v1Class = 5
  if v2Dx > 0 and v2Id >= 0:
    v2Class += 1
    if v2Class == 4: v2Class = 5
  if v1Dx > 0:
    v1x = int(clip(v1Dx, 0, 127) / 0.5 / IC_LANE_SCALE) & 0xFF; v1y = int((clip(v1Dy, -22., 22.) + 22.05) / 0.35) & 0x7F
    v1v = int((clip(v1V, -30, 26) / IC_LANE_SCALE + 30) / 4) & 0x0F
  else:
    v1x, v1y, v1v, important1, v1Class = 0xFF, 0x7F, 0x0F, 0, 0
  if v2Dx > 0:
    v2x = int(clip(v2Dx, 0, 127) / 0.5 / IC_LANE_SCALE) & 0xFF; v2y = int((clip(v2Dy, -22., 22.) + 22.05) / 0.35) & 0x7F
    v2v = int((clip(v2V, -30, 26) / IC_LANE_SCALE + 30) / 4) & 0x0F
  else:
    v2x, v2y, v2v, v2Class = 0xFF, 0x7F, 0x0F, 0
  b = [lane + (v1Class << 3) + (important1 << 7), v1x, v1v + ((v1y << 4) & 0xF0),
       ((v1y >> 4) & 0x07) + ((v1Id << 3) & 0xF8), ((v1Id >> 5) & 0x03) + (v2Class << 2) + ((v2x << 7) & 0x80),
       ((v2x >> 1) & 0x7F) + ((v2v << 7) & 0x80), ((v2v >> 1) & 0x07) + ((v2y << 3) & 0xF8), ((v2y >> 5) & 0x03) + ((v2Id << 2) & 0xFC)]
  b = bytes(x & 0xFF for x in b)
  lo = int.from_bytes(b[:4], 'little'); hi = int.from_bytes(b[4:], 'little')
  if ((lo >> 8) & 0xFF) == 0:              # panda: Dx byte 0 -> "no object"
    lo, hi = 0xFFFFFF00 + lane, 0x03FFFF83
  return lo, hi


class FakeDAS:
  """state + packing of do_fake_DAS. Defaults = the panda's start-up values (reset_DAS_data + initialisers)."""
  def __init__(s):
    s.counter = 0; s.idx = dict((k, 0) for k in ('obj', 'control', 'psc', 'lanes', 'status', 'status2', 'body', 'info', 'carlog'))
    s.boot_sent = False
    s.op_status = 1; s.alca_state = 0x05; s.hands_on = 0; s.fcw = 0; s.collision_warning = 0; s.speed_limit = 0
    s.cc_state = 0; s.longC = 0; s.acc_speed_kph = 0; s.acc_speed_limit = 0; s.units_included = 0; s.uom = 0
    s.in_drive = 0; s.fleet = 0; s.ldw = 0; s.turn = 0; s.hl_req = 0; s.hl_reason = 0
    s.lWidth = 29; s.lLine = 0; s.rLine = 0; s.lQual = 0; s.rQual = 0; s.laneRange = 160
    s.c0, s.c1, s.c2, s.c3 = 0x64, 0x7F, 0x7F, 0x7F
    s.lead = (0xFFFFFF00, 0x03FFFF83); s.left = (0xFFFFFF01, 0x03FFFF83); s.right = (0xFFFFFF02, 0x03FFFF83)

  def tick(s):
    """one do_fake_DAS call (the panda runs it on every 0x108 = 100 Hz); returns [(addr, bytes)]"""
    out = []; c = s.counter = (s.counter + 1) % 300
    if not s.boot_sent:
      out.append((0x639, frame(0x00E30055, 0x002003BD))); s.boot_sent = True
    if c == 299:
      lo = (0x00041603, 0x00010002, 0x00000001)[s.idx['carlog']]; out.append((0x5D9, frame(lo, 0))); s.idx['carlog'] = (s.idx['carlog'] + 1) % 3
    if c % 3 == 0:                                                        # DAS_object 0x309
      i = s.idx['obj']
      if i == 4: i = 5                                                    # no street sign
      lo, hi = {0: s.lead, 1: s.left, 2: s.right, 3: (0xFFFFFF03, 0x00000003), 5: (0xFFFFFF05, 0xFFFFFFFF)}[i]
      out.append((0x309, frame(lo, hi))); s.idx['obj'] = (i + 1) % 6
    if c % 4 == 0:                                                        # DAS_control 0x2B9
      if s.in_drive:
        acc_state = 0x04; jerk_min, jerk_max, accel_min, accel_max, acc_speed = 0x1FF, 0xFF, 0x1FF, 0x1FF, 0xFFF
        aeb = 0
        if s.longC:
          jerk_min, jerk_max = 0x000, 0x0F
          uom = 1.609 if (s.uom == 0 and s.units_included == 0) else 1.
          acc_speed = int(s.acc_speed_limit * uom * 10.0); accel_max, accel_min = 0x1FE, 0x001
        lo = (0xFF & acc_speed) + (((acc_state << 4) + ((acc_speed & 0xF00) >> 8)) << 8) + \
             ((((jerk_min << 2) & 0xFF) + aeb) << 16) + ((((jerk_max << 3) & 0xFF) + ((jerk_min >> 6) & 0x07)) << 24)
        hi = ((((accel_min << 3) & 0xFF) + ((jerk_max >> 5) & 0x07))) + ((((accel_max << 4) & 0xFF) + ((accel_min >> 5) & 0x0F)) << 8) + \
             (((s.idx['control'] << 5) + ((accel_max >> 4) & 0x1F)) << 16)
      else:
        lo, hi = 0xFFFE0FFF, 0x001FFFFF + (s.idx['control'] << 21)
      hi += cksm(lo, hi, 0x2B9, 7) << 24; out.append((0x2B9, frame(lo, hi))); s.idx['control'] = (s.idx['control'] + 1) % 8
    if c % 4 == 2:                                                        # DAS_pscControl 0x219
      lo = 0x90 + s.idx['psc']; lo += cksm(lo, 0, 0x219, 2) << 16; out.append((0x219, frame(lo, 0, 3))); s.idx['psc'] = (s.idx['psc'] + 1) % 16
    if c % 10 == 3:                                                       # DAS_lanes 0x239
      fuse = 2
      lo = ((s.lWidth & 0x0F) << 4 if s.lWidth <= 15 else s.lWidth << 4) + (s.rLine << 1) + s.lLine + (s.laneRange << 8) + (s.c0 << 16) + (s.c1 << 24)
      hi = 0x00F00000 + (s.idx['lanes'] << 28) + s.c2 + (s.c3 << 8) + ((((s.rLine * fuse) << 2) + (s.lLine * fuse)) << 16)
      out.append((0x239, frame(lo, hi))); s.idx['lanes'] = (s.idx['lanes'] + 1) % 16
    if c % 10 == 4:                                                       # DAS_telemetry 0x3A9
      rType = 3 if s.rLine == 1 else 7; lType = 3 if s.lLine == 1 else 7
      lc = 1 if s.alca_state == 0x09 else 0; rc = 1 if s.alca_state == 0x0a else 0
      lo = ((lType + (rType << 3) + (s.lQual << 6)) << 8) + ((s.rQual + (1 << 2) + (1 << 4) + (lc << 6) + (rc << 7)) << 16)
      out.append((0x3A9, frame(lo, 0)))
    if c % 10 == 5:                                                       # DAS_chNm 0x409, DAS_visualDebug 0x249
      out.append((0x409, frame(0, 0, 1)))
      acc = 0x05 if s.cc_state >= 2 else 0x01
      out.append((0x249, frame(0x10210480, 0x01000040 + (acc << 16))))
    if c % 50 == 0:                                                       # DAS_status 0x399, DAS_status2 0x389
      lo = s.op_status + 0xF0 + (s.speed_limit << 8) + (((s.collision_warning << 6) + s.speed_limit) << 16)
      hi = ((s.fleet & 0x03) << 3) + (s.ldw << 5) + (((s.hands_on << 2) + ((s.alca_state & 0x03) << 6) + s.fleet) << 8) + \
           (((s.idx['status'] << 4) + (s.alca_state >> 2)) << 16)
      hi += cksm(lo, hi, 0x399, 7) << 24; out.append((0x399, frame(lo, hi))); s.idx['status'] = (s.idx['status'] + 1) % 16
      uom = 1. / 1.609 if (s.uom == 1 and s.units_included == 1) else 1.
      sl2 = int(s.acc_speed_limit * uom / 0.2) & 0x3FF
      if sl2 == 0: sl2 = 0x3FF
      lcw = 0x01 if s.fcw > 0 else 0x0F; b4, b5 = 0x80, 0x13
      if s.cc_state > 1:
        acc_report = 0x12 if s.fleet == 2 else 0x01; lss, radar_tel, rob = 0x03, 0x01, 0x02
        b4 = (acc_report << 2) + ((lss & 0x01) << 7); b5 = ((lss >> 1) & 0x01) + (radar_tel << 2) + (rob << 4)
      lo = sl2 + (b4 << 24); hi = 0x8000 + b5 + (s.idx['status2'] << 20) + (lcw << 16)
      hi += cksm(lo, hi, 0x389, 7) << 24; out.append((0x389, frame(lo, hi))); s.idx['status2'] = (s.idx['status2'] + 1) % 16
    if c % 50 == 9:                                                       # DAS_bodyControls 0x3E9
      lo = 0xF1 + (s.turn << 8) + (s.hl_req << 10) + (s.hl_reason << 12); hi = s.idx['body'] << 20
      hi += cksm(lo, hi, 0x3E9, 7) << 24; out.append((0x3E9, frame(lo, hi))); s.idx['body'] = (s.idx['body'] + 1) % 16
    if c % 100 == 0:                                                      # DAS_info 0x539
      tbl = [(0x0003010a, 0x004e0000), (0x0102000b, 0x00000001), (0x0000000d, 0x3eac8e5b), (0xc9060010, 0x000021b9), (0x00000011, 0),
             (0x34577f12, 0x0b89706f), (0xffff0113, 0x0000fcff), (0x00000514, 0xc309fce5), (0xe7700017, 0x000091c4), (0xb2d50018, 0x0000786c)]
      out.append((0x539, frame(*tbl[s.idx['info']]))); s.idx['info'] = (s.idx['info'] + 1) % 10
    if c % 100 == 45:                                                     # DAS_warningMatrix0/1/3 (all clear)
      out += [(0x329, frame(0, 0)), (0x369, frame(0, 0)), (0x349, frame(0, 0))]
    return out

  def active_picture(s, op_status=5, kph=True, set_speed=80, lead_m=40.):
    """what openpilot sends in the car while engaged on a road with both lane lines and a car ahead"""
    s.op_status = op_status; s.cc_state = 2 if op_status == 5 else 1; s.longC = 1 if op_status == 5 else 0
    s.alca_state = 0x08; s.in_drive = 1; s.units_included = 1; s.uom = 1 if kph else 0
    s.acc_speed_kph = set_speed; s.acc_speed_limit = set_speed
    s.lWidth = 5; s.lLine = s.rLine = 1; s.lQual = s.rQual = 3; s.laneRange = 160
    s.c0 = int(3.5 / 0.035); s.c1 = int(0.2 / 0.0016); s.c2 = int(0.0025 / 0.00002); s.c3 = int(0.00003 / 0.00000024)  # straight road
    s.lead = lr_object(0, 0, 1, lead_m, 0., 0.) if lead_m > 0 else (0xFFFFFF00, 0x03FFFF83)


def selftest():
  """pack the panda's defaults and compare with frames captured from the car (2026-10-25 21:31, DAS emulation on)"""
  das = FakeDAS(); got = {}
  for _ in range(600):
    for a, d in das.tick(): got.setdefault(a, set()).add(d.hex())
  captured = {0x239: ['d0a1647f7f7ff0b0', 'd0a1647f7f7ff0d0'], 0x399: ['f1000000004071' + '3e', 'f1000000004011de'],
              0x3E9: ['f1000000000080' + '5d', 'f1000000000020fd']}
  ok = True
  for a, frames in captured.items():
    for f in frames:
      hit = f in got.get(a, set()); ok &= hit
      print('%s %s %s' % (hex(a), f, 'reproduced' if hit else 'NOT reproduced'))
  print('selftest', 'OK' if ok else 'FAILED')
  return ok


ap = argparse.ArgumentParser()
ap.add_argument('--bench', action='store_true', help='REQUIRED: confirms you are on the bench, not in the car')
ap.add_argument('--selftest', action='store_true', help='check the packing against captured car frames (no panda needed)')
ap.add_argument('--secs', type=int, default=60)
ap.add_argument('--bus', type=int, default=0, choices=[0, 1, 2])
ap.add_argument('--op-status', type=int, default=5, dest='op_status', help='5 = active (default), 2 = available, 1 = unavailable')
ap.add_argument('--mph', action='store_true', help='car shows mph (default km/h)')
ap.add_argument('--set-speed', type=int, default=80, dest='set_speed', help='cruise set speed in car units')
ap.add_argument('--lead', type=float, default=40., help='car ahead at this distance in m (0 = none)')
ap.add_argument('--raw398', type=bytes.fromhex, default=None, help='also send this exact 0x398 frame at 1 Hz, e.g. 41154e4f00000010')
ap.add_argument('--defaults', action='store_true', help="send the panda's start-up defaults instead of the active picture")
a = ap.parse_args()
if a.selftest:
  sys.exit(0 if selftest() else 1)
if not a.bench:
  print('refusing: bench only. Add --bench if (and only if) the car is NOT connected.'); sys.exit(2)

das = FakeDAS()
if not a.defaults:
  das.active_picture(a.op_status, not a.mph, a.set_speed, a.lead)
sys.path.insert(0, '/data/openpilot')
from panda import Panda  # noqa: E402
pl = Panda.list()
if not pl:
  print('no panda found'); sys.exit(1)
p = Panda(pl[0]); p.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
print('sending DAS picture: op_status %d, lanes %s, lead %s, set speed %d %s%s' % (
  das.op_status, 'both' if das.lLine and das.rLine else 'none', ('%.0f m' % a.lead) if a.lead > 0 and not a.defaults else 'none',
  das.acc_speed_limit, 'mph' if a.mph else 'km/h', (', 0x398 = ' + ' '.join('%02x' % x for x in a.raw398)) if a.raw398 else ''))
t0 = time.time(); nxt = t0; next398 = t0; sent = 0
try:
  while time.time() - t0 < a.secs:
    now = time.time()
    if now >= nxt:
      nxt += 0.01                                              # 100 Hz, like 0x108 in the car
      for addr, dat in das.tick():
        p.can_send(addr, dat, a.bus); sent += 1
    if a.raw398 is not None and now >= next398:
      p.can_send(0x398, a.raw398, a.bus); next398 += 1.0; sent += 1
    try: p.can_recv()                                          # keep the panda's receive buffer drained
    except Exception: pass
    time.sleep(0.002)
finally:
  p.set_safety_mode(Panda.SAFETY_SILENT)
print('done, sent %d frames. Safety mode back to silent.' % sent)
