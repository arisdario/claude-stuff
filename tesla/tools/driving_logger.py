#!/usr/bin/env python3
"""
driving_logger.py - observational logger: human driving vs openpilot driving, with emphasis on
acceleration/deceleration behaviour and how it changes with a lead vehicle present.

READ-ONLY / OBSERVATIONAL ONLY. This subscribes to openpilot's own cereal messages exactly like
lightdash.py and mapcurve_launcher.py do. It never opens the panda, never calls can_send, never
publishes anything, and never imports or touches carcontroller/carstate/safety code at runtime -
it only reads what openpilot already decided and already sent. It cannot change how the car drives.
The one exception, and it changes nothing either: it also imports steer_limits.py - by its own
docstring "Pure functions, no openpilot imports" - purely to compute what that OFF-by-default layer
WOULD have output at a couple of candidate strengths, side by side with what actually happened. The
result is logged, never sent, never fed back into anything. See "STEER_LIMITS HYPOTHETICALS" below.

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

--------------------------------------------------------------------------------------------------
WIGGLE DIAGNOSIS: steering torque, rate, and the actual commanded angle
--------------------------------------------------------------------------------------------------
Two carState fields needed for this were also hardcoded to 0 and useless for diagnosis:
  - steeringTorque (CS.steer_torque_driver) - EPAS_torsionBarTorque is already parsed off the EPAS
    CAN bus by carstate.py's own CANParser; it was simply never assigned (hardcoded 0 "#JCT").
  - steeringRate (CS.angle_steers_rate) - hardcoded 0 ("#JCT"), never computed at all.
Fixed in carstate.py (see "WIGGLE-DIAG" comments there) by: (1) assigning the real, already-parsed
EPAS_torsionBarTorque instead of discarding it, and (2) a plain finite difference of the same
angle_steers signal already in use. Both are read-only with respect to control: grepped this whole
repo first - no ALCA/HSO/carcontroller code reads steer_torque_driver or angle_steers_rate, only
interface.py copies them into the cereal carState message for logging/display. Steering GAINS,
steerRatio and stiffnessFactor are untouched.
This logger also decodes the actual commanded angle (not just enabled/not) from the same 0x659
frame used for long_enabled/steer_enabled - teslacan.create_fake_DAS_msg packs it as
((apply_angle*10 + 0x4000) & 0x7FFF) + (enable_steer_control << 15) across bytes 6-7, which is
decoded back to degrees here (das_commanded_angle). Comparing das_commanded_angle against
steering_angle/steering_rate/steering_torque side by side is the intended way to see whether a
wiggle correlates with a torque blip, a commanded/measured angle mismatch, or neither.
Also subscribes to carControl (standard openpilot message; interface.py/controlsd publish it every
cycle) purely to log actuators.steerAngle and its negation (actuator_steer_angle /
actuator_steer_angle_neg) as ground truth for which sign convention matches what, at a given moment
- see "STEER_LIMITS HYPOTHETICALS" note below for why das_commanded_angle's sign can legitimately
differ from steering_angle's while openpilot is actively steering.

--------------------------------------------------------------------------------------------------
MODEL PATH / GPS
--------------------------------------------------------------------------------------------------
Every sample also logs model.path/left_lane/right_lane (25 points, every 4 m to 100 m, same decode
lightdash.py's build_model() already runs against this 'model' message) plus left/right lane
probability, and the car's own GPS (latitude/longitude/altitude/accuracy/speed/bearingDeg) from
gpsLocationExternal - carcontroller.py's publish of the car's MCU_locationStatus CAN signals, NOT
the EON's own GPS: liveLocationKalman never gets a fix on this car (see mapcurved.py's notes). Both
are held at their last known value between publishes (same "sticky" pattern as das_commanded_angle)
since model/GPS don't necessarily arrive every tick of this logger's own ~20 Hz poll.

Two further signals exist on the bus but are not yet published to cereal by this fork (steer_error
from EPAS_steeringFault, steer_not_allowed from EPAS_eacStatus - both internal-only in interface.py):
not added here to keep this change minimal; flag if you want them exposed too.

--------------------------------------------------------------------------------------------------
STEER_LIMITS HYPOTHETICALS (sl20_angle / sl25_angle) - NEVER applied, logged only
--------------------------------------------------------------------------------------------------
carcontroller.py has an opt-in rate/clip layer, steer_limits.py (off by default: /data/steer_limits_on
must exist). It needs (request, last_request, measured_angle, v_ego, strength) - "request" and "last"
are carcontroller's own apply_angle this frame and the frame before, which aren't published anywhere.
This logger approximates them with das_commanded_angle (this sample) and the previous sample's
das_commanded_angle, at this logger's own ~20 Hz sampling instead of controlsd's real ~100 Hz loop -
scaled by passing the actual elapsed time between the two 0x659 frames as call_dt, so the rate limit
is at least scaled correctly for the cadence actually sampled. This is an APPROXIMATION for comparing
candidate strengths over a whole drive, not a bit-exact replay of what the layer would have output
frame by frame. Logs sl20_angle and sl25_angle (strength 0.20 and 0.25) next to the real
steering_angle/das_commanded_angle, so you can see which one would have tracked closer without
over-damping - without ever turning the layer on or changing a single frame actually sent to the car.
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
  fields = ('long_enabled', 'steer_enabled', 'op_status', 'op_status_text', 'speed_override', 'ap_unavailable',
            'collision_warning', 'cc_state', 'pcc_available', 'alca_state', 'commanded_angle')
  if d is None or len(d) < 8:
    return {k: None for k in fields}
  b0, b3, b6, b7 = d[0], d[3], d[6], d[7]
  op = b0 & 0x0F
  # byte6/7 = c_apply_steer = ((apply_angle*10 + 0x4000) & 0x7FFF) + (enable_steer_control << 15), low byte
  # first (teslacan.create_fake_DAS_msg). bit15 is steer_enabled (already read from byte7 bit7 above); the
  # low 15 bits are the commanded angle openpilot is actually asking for, decoded back to degrees.
  composite = b6 + (b7 << 8)
  commanded_angle = round(((composite & 0x7FFF) - 0x4000) / 10.0, 2)
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
    'commanded_angle': commanded_angle,          # degrees openpilot is commanding right now (not just enabled/not)
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


def model_dict(mm):
  """model path/lane points, every 4 m out to 100 m (25 points) - same decode lightdash.py's
  build_model() already uses and has been running against this exact 'model' message. Returns None
  for p/l/r if the message doesn't have the expected 100+ points (an older/different model build),
  never raises."""
  def pts(pd):
    try:
      p = list(pd.points)
      return [round(p[i], 2) for i in range(0, min(len(p), 100), 4)] if len(p) >= 100 else None
    except Exception:
      return None
  try:
    return {
      'path': pts(mm.path), 'left_lane': pts(mm.leftLane), 'right_lane': pts(mm.rightLane),
      'left_lane_prob': round(mm.leftLane.prob, 3), 'right_lane_prob': round(mm.rightLane.prob, 3),
    }
  except Exception:
    return {'path': None, 'left_lane': None, 'right_lane': None, 'left_lane_prob': None, 'right_lane_prob': None}


GPS_FIELDS = ('latitude', 'longitude', 'altitude', 'accuracy', 'speed', 'bearingDeg')


def gps_dict(g):
  """the car's own GPS (MCU_locationStatus on CAN, see carstate.py), published as gpsLocationExternal
  by carcontroller.py - NOT the EON's own GPS: liveLocationKalman never gets a fix on this car (see
  mapcurved.py's own notes). Read defensively; a missing field is None, never a crash."""
  if g is None:
    return {f: None for f in GPS_FIELDS}
  out = {}
  for f in GPS_FIELDS:
    try:
      out[f] = round(float(getattr(g, f)), 7 if f in ('latitude', 'longitude') else 2)
    except Exception:
      out[f] = None
  return out


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


def sl_hypotheticals(sl, request, prev_request, measured, v_ego, t_659=None, prev_t_659=None):
  """pure wrapper around steer_limits.limit_angle() for the two candidate strengths; see module
  docstring (STEER_LIMITS HYPOTHETICALS). None in, None out - never raises, never applied anywhere."""
  if request is None or prev_request is None:
    return {'sl20_angle': None, 'sl25_angle': None}
  call_dt = (t_659 - prev_t_659) if (t_659 is not None and prev_t_659 is not None and t_659 > prev_t_659) else 0.01
  return {
    'sl20_angle': round(sl.limit_angle(request, prev_request, measured, v_ego, 0.20, call_dt), 2),
    'sl25_angle': round(sl.limit_angle(request, prev_request, measured, v_ego, 0.25, call_dt), 2),
  }


def _load_steer_limits():
  """from selfdrive.car.tesla on-car; falls back to a path-relative load off-car (same trick as
  lightdash.py's car_packers()). steer_limits.py has no openpilot imports, so either way works."""
  try:
    from selfdrive.car.tesla import steer_limits as sl
    return sl
  except Exception:
    import importlib.util
    p = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'steer_limits.py')
    spec = importlib.util.spec_from_file_location('steer_limits_dl', p)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


# ---------------------------------------------------------------------------------------- the live loop
def run(out_path, hz=20.0, duration=None, log=print):
  import cereal.messaging as messaging  # local import: keeps this file importable/testable off-car
  sl = _load_steer_limits()

  topics = ('controlsState', 'carState', 'radarState', 'model', 'carControl', 'gpsLocationExternal')
  subs = {t: messaging.sub_sock(t, conflate=True) for t in topics}
  sc = messaging.sub_sock('sendcan')    # drained in full every tick, like lightdash - a flood here never starves us

  clf = Classifier()
  last_ctrl = last_long = last_steer = None
  last_das = decode_0x659(None)
  last_actuator_angle = None
  last_model = model_dict(None)
  last_gps = gps_dict(None)
  t_ctrl = t_659 = None
  prev_request, prev_t_659 = None, None     # for the steer_limits hypotheticals, see module docstring
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
      cc = messaging.recv_one_or_none(subs['carControl'])
      if cc is not None:
        try:
          last_actuator_angle = float(cc.carControl.actuators.steerAngle)
        except Exception:
          last_actuator_angle = None

      mm = messaging.recv_one_or_none(subs['model'])
      if mm is not None:
        try:
          last_model = model_dict(mm.model)
        except Exception:
          pass
      gp = messaging.recv_one_or_none(subs['gpsLocationExternal'])
      if gp is not None:
        try:
          last_gps = gps_dict(gp.gpsLocationExternal)
        except Exception:
          pass

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
          # WIGGLE-DIAG: carstate.py now fills these two from the real EPAS torque signal and a finite
          # difference of steeringAngle (both were hardcoded to 0 before - see carstate.py comments
          # tagged WIGGLE-DIAG). Compare commanded (das_commanded_angle, above) vs steering_angle vs
          # steering_rate vs steering_torque to see whether the wiggle correlates with a torque blip,
          # an angle-rate oscillation, or neither.
          'steering_torque': round(float(g('steeringTorque', 0.)), 3),
          'steering_rate': round(float(g('steeringRate', 0.)), 2),
          # GROUND TRUTH for the commanded-vs-measured sign question: carControl.actuators.steerAngle is
          # openpilot's own target angle BEFORE carcontroller.py's "apply_angle = -actuators.steerAngle
          # # Tesla is reversed vs OP" negation. Logging both this raw value and its negation lets you
          # see directly which convention matches steering_angle/das_commanded_angle, instead of guessing.
          'actuator_steer_angle': None if last_actuator_angle is None else round(last_actuator_angle, 2),
          'actuator_steer_angle_neg': None if last_actuator_angle is None else round(-last_actuator_angle, 2),
          'steering_pressed': bool(g('steeringPressed', False)),
          'left_blinker': bool(g('leftBlinker', False)),
          'right_blinker': bool(g('rightBlinker', False)),
          'cruise_enabled_UNRELIABLE': bool(g('cruiseState', None) and cs_.cruiseState.enabled),  # hardcoded True - see docstring
          'cruise_speed': round(float(cs_.cruiseState.speed), 2) if g('cruiseState', None) else None,
        })

        # STEER_LIMITS HYPOTHETICALS - see module docstring. Never applied; logged only.
        request = sample.get('das_commanded_angle')
        sample.update(sl_hypotheticals(sl, request, prev_request, sample['steering_angle'], sample['v_ego'],
                                        t_659, prev_t_659))
        if request is not None:
          prev_request, prev_t_659 = request, t_659

      if rad is not None:
        try:
          sample['lead1'] = lead_dict(rad.radarState.leadOne)
          sample['lead2'] = lead_dict(rad.radarState.leadTwo)
        except Exception:
          sample['lead1'] = sample['lead2'] = lead_dict(None)
      else:
        sample['lead1'] = sample['lead2'] = lead_dict(None)

      sample['model'] = last_model
      sample['gps'] = last_gps

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
