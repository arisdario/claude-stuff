#!/usr/bin/env python3
"""
ahb_state.py - auto high beam decision state machine. Pure Python, no openpilot imports, fully unit-testable
(tests/test_ahb_state.py). AHB_module.py feeds it and maps its result onto the DAS wire message.

States:
  OFF               low beam: it is not dark, too slow, or we are in the recovery hold after traffic
  ON                high beam allowed
  UNDECIDED         hand it to the driver / OEM: feature off, flash pulled, or brightness unknown
  TEMPORARY_DIPPED  high beam was wanted but a vehicle is (or just was) ahead/oncoming -> dipped, with a recovery hold

Policy only. It returns a state, whether the beam should be high, and a reason code; it does NOT build any CAN message
and never talks to the car. Hysteresis and min-duration timers stop the beam chattering. Brightness and traffic come
from the luminance estimator and the existing model/radar perception - this file just decides.

Reason codes match AHB_module.AHBReason (0 ON, 1 moving vision target, 2 moving radar target, 3 ambient light,
4 head light, 5 SNA/none)."""

OFF, ON, UNDECIDED, TEMPORARY_DIPPED = 'off', 'on', 'undecided', 'temporary_dipped'

R_ON = 0
R_VISION_TARGET = 1
R_RADAR_TARGET = 2
R_AMBIENT = 3
R_HEAD_LIGHT = 4
R_SNA = 5

# tunable, conservative defaults -------------------------------------------------------------------------------------
SPEED_ON_MS = 30 / 3.6        # high beam allowed from 30 km/h ...
SPEED_OFF_MS = 20 / 3.6       # ... dipped again below 20 km/h (hysteresis gap)
RECOVERY_S = 5.0              # keep dipped this long after the last vehicle (matches CS.ahbOffDuration default)
MIN_ON_S = 2.0               # once high, stay high at least this long before a non-safety drop (anti-flicker)
MIN_OFF_S = 1.0              # once dipped for brightness/slow, stay down at least this long before going high
DEBOUNCE_S = 0.3             # a transient ON/OFF input must persist this long to count (not applied to safety dips)


class AhbInputs(object):
  """one cycle of decision inputs. traffic_* are already-detected vehicles (radar lead, model oncoming/lead)."""
  def __init__(self, now, enabled=False, dark=False, bright=False, lum_valid=True,
               v_ego=0.0, stalk=0, headlights_on=True, driver_override=False,
               oncoming=False, lead=False, vision_target=False, low_confidence=False):
    self.now = now
    self.enabled = enabled            # feature switched on (ahb_auto / test / MCU)
    self.dark = dark                  # luminance: dark enough for high beam
    self.bright = bright              # luminance: clearly bright (urban/day)
    self.lum_valid = lum_valid        # luminance reading is usable
    self.v_ego = v_ego
    self.stalk = stalk                # 0 normal/managed, 1 high-beam held, 2 flash pulled
    self.headlights_on = headlights_on
    self.driver_override = driver_override   # driver is forcing a change (flash, manual)
    self.oncoming = oncoming          # oncoming vehicle (model/vision)
    self.lead = lead                  # preceding vehicle (radar/vision)
    self.vision_target = vision_target  # vehicle seen by camera specifically (for the reason code)
    self.low_confidence = low_confidence  # model/camera confidence too low to trust "clear"


class AhbStateMachine(object):
  def __init__(self):
    self.state = UNDECIDED
    self.reason = R_SNA
    self.high = False
    self._since = 0.0            # when the current state was entered
    self._last_vehicle_t = -1e9  # last time any vehicle was seen
    self._speed_ok = False       # speed hysteresis latch
    self._eligible_since = None  # when the road first became high-beam-eligible (dark, clear, fast)

  def _enter(self, state, reason, high, now):
    if state != self.state:
      self._since = now
    self.state = state; self.reason = reason; self.high = high

  def _settle(self, state, reason, high, now):
    """enter a decided state; any state other than ON clears the ON-eligibility timer"""
    if state != ON:
      self._eligible_since = None
    self._enter(state, reason, high, now)
    return self._out()

  def _traffic(self, i):
    return i.oncoming or i.lead or i.vision_target

  def update(self, i):
    now = i.now
    # speed hysteresis latch
    if i.v_ego >= SPEED_ON_MS:
      self._speed_ok = True
    elif i.v_ego < SPEED_OFF_MS:
      self._speed_ok = False

    if self._traffic(i):
      self._last_vehicle_t = now

    # 1. Hand back to the driver / OEM: feature off, flash pulled, headlights off, or brightness unknown.
    if (not i.enabled) or i.stalk == 2 or i.driver_override or (not i.headlights_on):
      return self._settle(UNDECIDED, R_SNA, False, now)
    if not i.lum_valid:
      return self._settle(UNDECIDED, R_SNA, False, now)

    # 2. Clearly bright -> never high beam.
    if i.bright:
      return self._settle(OFF, R_AMBIENT, False, now)

    # 3. A vehicle ahead/oncoming, or within the recovery hold since the last one -> dip. Safety: immediate, no debounce.
    if self._traffic(i) or (now - self._last_vehicle_t) < RECOVERY_S:
      reason = R_VISION_TARGET if (i.oncoming or i.vision_target) else R_RADAR_TARGET
      return self._settle(TEMPORARY_DIPPED, reason, False, now)

    # 4. Low confidence that the road is clear -> be cautious, dip.
    if i.low_confidence:
      return self._settle(TEMPORARY_DIPPED, R_VISION_TARGET, False, now)

    # 5. Too slow, or not dark (DIM / not dark enough) -> low beam.
    if not self._speed_ok:
      return self._settle(OFF, R_SNA, False, now)
    if not i.dark:
      return self._settle(OFF, R_AMBIENT, False, now)

    # 6. Dark, clear, fast enough. Stay high if already high; else lift once eligibility has held DEBOUNCE and the
    #    current low-beam state has held MIN_OFF (anti-flicker).
    if self.state == ON:
      self.reason = R_ON; self.high = True
      return self._out()
    if self._eligible_since is None:
      self._eligible_since = now
    if (now - self._eligible_since) >= DEBOUNCE_S and (now - self._since) >= MIN_OFF_S:
      self._enter(ON, R_ON, True, now)
      return self._out()
    self.high = False                    # still ramping up: hold the current low-beam state, keep the timers running
    return {'state': self.state, 'high': False, 'reason': self.reason}

  def _out(self):
    return {'state': self.state, 'high': self.high, 'reason': self.reason}

  def diagnostics(self, now=None):
    d = {'ahb_state': self.state, 'ahb_reason': self.reason, 'ahb_high': self.high}
    if now is not None:
      d['ahb_since_s'] = round(now - self._since, 1)
      d['ahb_since_vehicle_s'] = round(now - self._last_vehicle_t, 1) if self._last_vehicle_t > -1e8 else None
    return d
