#!/usr/bin/env python3
"""
driving_logger.py - observational logger: human driving vs openpilot driving, with emphasis on
acceleration/deceleration behaviour and how it changes with a lead vehicle present.

READ-ONLY / OBSERVATIONAL ONLY. This subscribes to openpilot's own cereal messages exactly like
lightdash.py and mapcurve_launcher.py do. It never opens the panda, never calls can_send, never
publishes anything, and never imports or touches carcontroller/carstate/safety code at runtime -
it only reads what openpilot already decided and already sent. It cannot change how the car drives.

  cd /data/openpilot && PYTHONPATH=/data/openpilot python3 /data/claude-stuff/tesla/tools/driving_logger.py \
      --out /data/drivelog_$(date +%Y%m%d_%H%M%S).jsonl [--hz 20] [--duration 1800]

--------------------------------------------------------------------------------------------------
WHAT THIS VERSION OF OPENPILOT ACTUALLY EXPOSES (checked against this repo's own code, not assumed)
--------------------------------------------------------------------------------------------------
This is an old (pre-0.8.x-mainline-style) Tesla fork. Checked directly against carstate.py,
interface.py, carcontroller.py, teslacan.py and panda/board/safety/safety_tesla.h in this repo:

1. Messaging topics that exist and are used by this stack: controlsState, carState, radarState,
   sendcan, model (NOT modelV2 - this version predates modelV2; `model` only carries path/leftLane/
   rightLane/freePath, no lead information).

2. Engagement: there is NO per-axis (longitudinal-only / lateral-only) engagement flag published
   over cereal on this fork. `carcontroller.update(enabled, CS, ...)` receives one master `enabled`
   bool (== controlsState.enabled, the normal openpilot state-machine output) and internally
   derives separate longitudinal ("speed_control_enabled", from PCC/ACC) and lateral
   ("enable_steer_control") booleans - but those live only inside the carcontroller/carstate
   objects inside the controlsd process. They are NOT copied into the carState cereal message
   (grepped interface.py: no such field is set on `ret`).
   HOWEVER: carcontroller packs both into the real CAN frame it sends to the instrument cluster,
   0x659 (teslacan.create_fake_DAS_msg / safety_tesla.h tesla_tx_hook), which openpilot transmits
   on `sendcan` every cycle regardless. Decoding that frame - the exact same technique lightdash.py
   already uses read-only for 0x554/0x557/0x659 - recovers both bits straight from what openpilot
   itself is telling the car, with no new schema and no invented signal names:
     byte0 bit7 (0x80) = DAS_longC_enabled   (longitudinal actually commanded)
     byte7 bit7 (0x80) = DAS_steeringEnabled (lateral / steering actually commanded)
     byte0 bits0-3      = DAS_op_status (0-15; informational only, meaning not fully reverse engineered)
   This is the same frame, same bit positions, verified against two independent sources in this
   repo: the packer (teslacan.create_fake_DAS_msg) and the firmware's own parser (safety_tesla.h).

3. KNOWN LANDMINES on this car - do NOT trust these two carState fields for engagement/availability,
   they are hardcoded in interface.py and do not reflect reality:
     ret.cruiseState.enabled   = True   (hardcoded; "#self.CS.pcm_acc_status != 0" is commented out)
     ret.cruiseState.available = bool(self.CS.main_on)   where self.main_on is ALSO hardcoded to 1
   Both are always-true on this car. This logger records them for completeness (clearly labelled
   as unreliable) but does NOT use them to classify samples. Engagement classification instead uses
   controlsState.enabled plus the decoded 0x659 bits above.
   ret.brakePressed is also hardcoded to False in interface.py on this car (no brake switch wired
   to this signal) - recorded as-is, flagged unreliable. brakeLights (regen/physical) is separate
   and does reflect something real (DI_torque2/brake switch), see CS.brake_switch.

4. Lead vehicle: this version has no modelV2/leadsV3. The only lead-vehicle representation is
   radarState.leadOne / radarState.leadTwo. Fields actually read by this codebase's own modules
   (PCC_module.py, ACC_module.py, carcontroller.py): status, dRel, vRel, yRel, aRel, vLeadK, aLeadK.
   There is no modelProb/radar-vs-vision flag used anywhere in this repo's code for this car - if
   the field exists in the live schema it will still be picked up (see LEAD_FIELDS / getattr below),
   but nothing is assumed about it. aRel is the direct relative-acceleration field; aLeadK/vLeadK are
   the Kalman-filtered absolute lead accel/speed estimates used by PCC for following logic.

Nothing above is assumed - every field name and bit position is one this repo's own driving code
already reads or writes. Fields are still read defensively (getattr/try-except) in case a given
live cereal build doesn't carry one: a missing field becomes None in the log, not a crash, and the
first sample logs which fields were actually present so you can see your car's real schema.
"""
import argparse
import json
import os
import sys
import time

# ---------------------------------------------------------------------------------------- 0x659 decode
# Ground truth: teslacan.create_fake_DAS_msg() (the packer) and panda/board/safety/safety_tesla.h
# tesla_tx_hook's parsing of the SAME frame (the firmware's own reading of it). Both agree.
DAS_OP_STATUS_TEXT = {0: 'disabled', 1: 'unavailable', 2: 'disabling', 3: 'unavailable2', 4: 'hold', 5: 'warning'}


def decode_0x659(d):
  """d: 8 bytes (bytes/bytearray/list[int]) of the DAS_udsRequest-reused 0x659 frame openpilot sends
  on sendcan every cycle. Returns a dict of the real, already-sent control bits. Never raises -
  returns a dict of Nones if d is too short."""
  if d is None or len(d) < 8:
    return {k: None for k in ('long_enabled', 'steer_enabled', 'op_status', 'op_status_text', 'speed_override',
                               'ap_unavailable', 'collision_warning', 'cc_state', 'pcc_available', 'alca_state')}
  b0, b3, b7 = d[0], d[3], d[7]
  op = b0 & 0x0F
  return {
    'long_enabled': bool(b0 >> 7 & 1),          # DAS_longC_enabled
    'steer_enabled': bool(b7 >> 7 & 1),         # DAS_steeringEnabled
    'op_status': op,
    'op_status_text': DAS_OP_STATUS_TEXT.get(op, 'unknown_%d' % op),
    'speed_override': bool(b0 >> 6 & 1),
    'ap_unavailable': bool(b0 >> 5 & 1),
    'collision_warning': bool(b0 >> 4 & 1),
    'cc_state': b3 >> 6 & 3,
    'pcc_available': bool(b3 >> 5 & 1),
    'alca_state': b3 & 0x1F,
  }


# ---------------------------------------------------------------------------------------- engagement classifier
STALE_S = 1.0        # 0x659 / controlsState older than this = treat as unknown, not as "still enabled"
TRANSITION_S = 0.3   # settle time after any engagement-bit flip before trusting a steady state


class Classifier(object):
  """Turns (controlsState.enabled, decoded 0x659 bits) into one state string per sample. Pure logic,
  no messaging import, so it's unit-testable without cereal or a running car."""

  STATES = ('HUMAN_DRIVING', 'OPENPILOT_ENGAGED', 'OPENPILOT_LONGITUDINAL_ONLY', 'OPENPILOT_LATERAL_ONLY',
            'TRANSITION', 'UNKNOWN')

  def __init__(self):
    self._last_ctrl = None            # last-seen controlsState.enabled (bool or None)
    self._last_long = None
    self._last_steer = None
    self._flip_t = -1e9               # time of the most recent flip in any of the three above

  def update(self, now, ctrl_enabled, ctrl_age, long_enabled, long_age, steer_enabled, steer_age):
    """All *_age are seconds since that value was last refreshed (None if never seen). Returns the
    classification string for this instant."""
    ctrl = ctrl_enabled if (ctrl_enabled is not None and (ctrl_age is None or ctrl_age <= STALE_S)) else None
    lon = long_enabled if (long_enabled is not None and (long_age is None or long_age <= STALE_S)) else None
    lat = steer_enabled if (steer_enabled is not None and (steer_age is None or steer_age <= STALE_S)) else None

    def _flipped(old, new):
      """a real value change counts as a flip; losing/regaining visibility (None <-> X) does not -
      that's staleness, not the car's state changing, and shouldn't restart the settle timer."""
      return old is not None and new is not None and old != new

    if _flipped(self._last_ctrl, ctrl) or _flipped(self._last_long, lon) or _flipped(self._last_steer, lat):
      self._flip_t = now
    self._last_ctrl, self._last_long, self._last_steer = ctrl, lon, lat

    if ctrl is None:
      return 'UNKNOWN'
    if now - self._flip_t < TRANSITION_S:
      return 'TRANSITION'
    if not ctrl:
      return 'HUMAN_DRIVING'
    # ctrl is True from here on
    if lon is None or lat is None:
      return 'OPENPILOT_ENGAGED'        # engaged overall, but this fork can't see the per-axis split right now
    if lon and lat:
      return 'OPENPILOT_ENGAGED'
    if lon and not lat:
      return 'OPENPILOT_LONGITUDINAL_ONLY'
    if lat and not lon:
      return 'OPENPILOT_LATERAL_ONLY'
    return 'TRANSITION'                 # engaged=True but neither axis active yet - spin-up/spin-down edge


# ---------------------------------------------------------------------------------------- lead vehicle fields
LEAD_FIELDS = ('status', 'dRel', 'vRel', 'yRel', 'aRel', 'vLeadK', 'aLeadK')


def lead_dict(lead):
  """getattr-defensive: a field this live schema doesn't have becomes None, never an exception."""
  if lead is None:
    return {f: None for f in LEAD_FIELDS}
  out = {}
  for f in LEAD_FIELDS:
    try:
      v = getattr(lead, f)
      out[f] = bool(v) if f == 'status' else round(float(v), 3)
    except Exception:
      out[f] = None
  return out


# ---------------------------------------------------------------------------------------- the live loop
def run(out_path, hz=20.0, duration=None, log=print):
  import cereal.messaging as messaging  # local import: keeps this file importable/testable off-car

  topics = ('controlsState', 'carState', 'radarState', 'model')
  subs = {t: messaging.sub_sock(t, conflate=True) for t in topics}
  sc = messaging.sub_sock('sendcan')    # drained in full every tick, like lightdash - a flood here never starves us

  clf = Classifier()
  last_ctrl = last_long = last_steer = None
  last_das = decode_0x659(None)
  t_ctrl = t_659 = None
  reported_fields = False

  f = open(out_path, 'a')
  f.write(json.dumps({'meta': 'driving_logger start', 't': time.time(), 'hz': hz,
                       'note': 'read-only observational log; see module docstring for exact signal provenance'}) + '\n')
  f.flush()

  period = 1.0 / hz
  t_end = (time.monotonic() + duration) if duration else None
  next_t = time.monotonic()
  n = 0
  try:
    while True:
      now_mono = time.monotonic()
      if t_end is not None and now_mono >= t_end:
        break

      for ev in messaging.drain_sock(sc, wait_for_one=False):
        for c in ev.sendcan:
          if c.address == 0x659 and c.src == 0:
            last_das = decode_0x659(bytes(c.dat))
            last_long, last_steer = last_das['long_enabled'], last_das['steer_enabled']
            t_659 = now_mono

      cs = messaging.recv_one_or_none(subs['controlsState'])
      if cs is not None:
        try:
          last_ctrl = bool(cs.controlsState.enabled)
        except Exception:
          last_ctrl = None
        t_ctrl = now_mono

      car = messaging.recv_one_or_none(subs['carState'])
      rad = messaging.recv_one_or_none(subs['radarState'])

      state = clf.update(now_mono, last_ctrl, None if t_ctrl is None else now_mono - t_ctrl,
                          last_long, None if t_659 is None else now_mono - t_659,
                          last_steer, None if t_659 is None else now_mono - t_659)

      sample = {'t': round(time.time(), 3), 'state': state}
      sample['ctrl_enabled'] = last_ctrl
      sample.update({'das_%s' % k: v for k, v in last_das.items()})

      if car is not None:
        cs_ = car.carState
        def g(name, default=None):
          try:
            return getattr(cs_, name)
          except Exception:
            return default
        sample.update({
          'v_ego': round(float(g('vEgo', 0.)), 3),
          'a_ego': round(float(g('aEgo', 0.)), 3),
          'standstill': bool(g('standstill', False)),
          'gas': round(float(g('gas', 0.)), 3),
          'gas_pressed': bool(g('gasPressed', False)),
          'brake_pressed_UNRELIABLE': bool(g('brakePressed', False)),   # hardcoded False on this car - see docstring
          'brake_lights': bool(g('brakeLights', False)),
          'steering_angle': round(float(g('steeringAngle', 0.)), 2),
          'steering_pressed': bool(g('steeringPressed', False)),
          'left_blinker': bool(g('leftBlinker', False)),
          'right_blinker': bool(g('rightBlinker', False)),
          'cruise_enabled_UNRELIABLE': bool(g('cruiseState', None) and cs_.cruiseState.enabled),  # hardcoded True - see docstring
          'cruise_speed': round(float(cs_.cruiseState.speed), 2) if g('cruiseState', None) else None,
        })

      if rad is not None:
        try:
          sample['lead1'] = lead_dict(rad.radarState.leadOne)
          sample['lead2'] = lead_dict(rad.radarState.leadTwo)
        except Exception:
          sample['lead1'] = sample['lead2'] = lead_dict(None)
      else:
        sample['lead1'] = sample['lead2'] = lead_dict(None)

      if not reported_fields and car is not None and rad is not None:
        f.write(json.dumps({'meta': 'fields_seen_once', 't': time.time(),
                             'carState_sample_keys': sorted(k for k in sample if not k.startswith('das_')),
                             'lead1': sample['lead1']}) + '\n')
        reported_fields = True

      f.write(json.dumps(sample) + '\n')
      n += 1
      if n % (int(hz) * 30) == 0:   # flush roughly every 30s of samples, not every line
        f.flush()
        log('driving_logger: %d samples written' % n)

      next_t += period
      sleep_s = next_t - time.monotonic()
      if sleep_s > 0:
        time.sleep(sleep_s)
      else:
        next_t = time.monotonic()
  finally:
    f.flush()
    f.close()
    log('driving_logger: stopped, %d samples in %s' % (n, out_path))


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('---')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--out', default='/data/drivelog_%d.jsonl' % int(time.time()))
  ap.add_argument('--hz', type=float, default=20.0)
  ap.add_argument('--duration', type=float, default=None, help='seconds; default: run until Ctrl-C')
  args = ap.parse_args(argv)
  run(args.out, hz=args.hz, duration=args.duration)


if __name__ == '__main__':
  main()
