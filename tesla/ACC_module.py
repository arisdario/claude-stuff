from selfdrive.car.tesla.speed_utils.fleet_speed import FleetSpeed
from selfdrive.car.tesla.values import CruiseButtons, CruiseState
from selfdrive.config import Conversions as CV
import cereal.messaging as messaging
import json
import os
import sys
import time

ACC_LOG_DIR = '/data/acclog'          # ACCLOG: one line per ACC cycle (20/s) while ACC is on - see _acc_log()
ACC_LOG_OFF = '/data/acc_log_off'
ACC_LOG_MAX_BYTES = 300 * 1024 * 1024
# ACCPLAN: openpilot's own longitudinal planner decides the speed - the 0.7.5 MPC that keeps the distance to the car
# ahead and the cruise speed smoother for your set speed (selfdrive/controls/lib/planner.py, published as 'plan').
# This module only (1) turns that plan into cruise stalk presses (SERVO, _calc_button) and (2) keeps its own safety
# check that does not depend on the plan or on comfort (SAFETY, _fast_decel_required / POSGATE).
# The gap the planner keeps is 1.8 s in 0.7.5; tools/patch_planner_gap.py makes it your gap (town 2.5 s, highway 3.6 s).
K_CC = 0.9                  # SERVO: Tesla cruise accelerates ~0.25 m/s2 per km/h of set-speed error (logs) = 0.9 1/s
SERVO_HYST_KPH = 0.7        # SERVO: press only when the wanted set speed is this far from the car's (no +1/-1 dither)
SERVO_UP_HYST_KPH = 1.3     # SERVO: speeding up waits for a little more (radar blips push the plan up for a moment)
SERVO_TAU_S = 2.0           # SERVO: the wanted set speed is smoothed (radar noise reaches the plan) ...
SERVO_FAST_DOWN_KPH = 1.5   # ... but never kept more than 1.5 km/h above the plan's own value: slowing is not delayed
SERVO_PRESS_MS = 300        # SERVO: at most one press per 0.3 s (the car shows the new set speed ~0.2 s later)
SERVO_HUMAN_MS = 1000       # SERVO: nothing for 1 s after YOUR stalk press (your press always wins)
PLAN_MAX_AGE_MS = 500       # SERVO: a plan older than this is not used (then only the safety cancel can act)
PLAN_DRIFT_MS = 1.5         # SERVO: the plan integrates its own speed; it is used at most 1.5 m/s away from the real
                            # speed (after a cancel / your accelerator the plan can be ahead of the car: no surge)
MIN_GAP_M = 6.0             # SAFETY: the distance to keep in the worst case
PRESS_DECEL_MS2 = 0.6       # SAFETY: what cruise presses deliver (measured ~0.6 m/s2): more needed -> cancel (regen ~1.5)
LEAD_HARD_BRAKE_MS2 = -1.5  # SAFETY: the lead braking this hard while we close -> cancel at once
REENGAGE_NEED_MS2 = 0.3     # REENGAGE: cruise comes back (SET, current speed) when braking needed is below this ...
REENGAGE_LEAD_ACCEL_MS2 = -0.5  # ... and the lead is not braking harder than this (filtered)
POSITIVE_BLOCK_MS = 3000    # POSGATE: no speed-up command for 3 s after the lead decelerated / was clearly slower
LEAD_FILTER_TAU_S = 0.7     # low-pass of the lead's acceleration and relative speed (radar noise +-0.3 m/s2, +-2 km/h)
LEAD_BRAKING_MS2 = -0.25    # POSGATE: lead acceleration (filtered) below this = the lead decelerates
LEAD_SLOWER_MS = 1.5        # POSGATE: lead slower than us by this much (filtered vRel; the radar reads ~0.8 m/s low)
LEAD_FILTER_KEEP_MS = 2000  # the lead filters survive a radar drop-out this long
LEAD_HOLD_MS = 500          # radarState arrives at ~20 Hz like this loop: keep the last lead this long between messages



class ACCState():
  # Possible states of the ACC system, following the DI_cruiseState naming
  # scheme.
  OFF = 0         # Disabled by UI.
  STANDBY = 1     # Ready to be enaged.
  ENABLED = 2     # Engaged.
  NOT_READY = 9   # Not ready to be engaged due to the state of the car.
  
class _Mode():
  def __init__(self, label, autoresume, state,adaptive):
    self.label = label
    self.autoresume = autoresume
    self.state = state
    self.adaptive = adaptive
    self.next = None
  
class ACCMode():
  # Possible ACC modes, controlling how ACC behaves.
  # This is separate from ACC state. For example, you could
  # have ACC in "Autoresume" mode in "Standby" state.
  FOLLOW = _Mode(label="follow",   autoresume=False, state=ACCState.STANDBY, adaptive=True)
  AUTO =   _Mode(label="auto", autoresume=True,  state=ACCState.STANDBY, adaptive=True)
  PLAINCC = _Mode(label="cc",   autoresume=False, state=ACCState.STANDBY, adaptive=False)
  
  BUTTON_NAME = 'acc'
  BUTTON_ABREVIATION = 'ACC'
  
  # Toggle order: OFF -> ON -> AUTO -> OFF
  _all_modes = [FOLLOW, AUTO,PLAINCC]
  for index, mode in enumerate(_all_modes):
    mode.next = _all_modes[(index + 1) % len(_all_modes)]
    
  # Map labels to modes for fast lookup by label.
  _label_to_mode = {mode.label: mode for mode in _all_modes}
  @ classmethod
  def from_label(cls, label):
    return cls._label_to_mode.get(label, cls.FOLLOW)
      
  @ classmethod
  def labels(cls):
    return [mode.label for mode in cls._all_modes]

def _current_time_millis():
  return int(round(time.time() * 1000))


class ACCController():
  
  # Tesla cruise only functions above 17 MPH
  MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS
    
  def __init__(self,carcontroller):
    self.CC = carcontroller
    self.human_cruise_action_time = 0
    self.automated_cruise_action_time = 0
    self.radarState = messaging.sub_sock('radarState', conflate=True)
    self.plan_sock = messaging.sub_sock('plan', conflate=True)   # ACCPLAN
    self.last_update_time = 0
    self.enable_adaptive_cruise = False
    self.prev_enable_adaptive_cruise = False
    # Whether to re-engage automatically after being paused due to low speed or
    # user-initated deceleration.
    self.autoresume = False
    self.adaptive = False
    self.last_brake_press_time = 0
    self.last_cruise_stalk_pull_time = 0
    self.prev_cruise_buttons = CruiseButtons.IDLE
    self.prev_pcm_acc_status = 0
    self.acc_speed_kph = 0.
    self.speed_limit_kph = 0.
    self.prev_speed_limit_kph = 0.
    self.user_has_braked = False
    self.has_gone_below_min_speed = False
    self.fast_decel_time = 0
    self.lead_last_seen_time_ms = 0
    # BB speed for testing
    self.new_speed = 0
    average_speed_over_x_suggestions = 20 # 1 second (20x a second)
    self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)
    self.curve_cap_kph = 999.    # curves are not slowed for by this module (the log keeps the column: always 999)
    # ACCLOG state
    self.last_follow_msg = None
    self._log_f = None
    self._log_buf = []
    self._log_check_t = 0.
    self._log_off = False
    # ACCPLAN state
    self._plan = None            # (vStart, aStart, vTarget, aTarget, hasLead, source, vCruise, aCruise, received ms)
    self.plan_v = self.plan_a = None
    self.follow_target_kph = None
    self._want_f = None          # smoothed wanted set speed (SERVO_TAU_S) and when it was updated (ms)
    self._want_ms = 0
    self.brake_need_ms2 = 0.
    self._last_lead = None       # (lead, time ms): held LEAD_HOLD_MS between radarState messages
    self.a_lead_f = 0.           # filtered lead acceleration / relative speed (LEAD_FILTER_TAU_S), reset without a lead
    self.v_rel_f = 0.
    self._lead_f_ms = 0
    self.lead_closing_time_ms = 0

  # Updates the internal state of this controller based on user input,
  # specifically the steering wheel mounted cruise control stalk, and OpenPilot
  # UI buttons.
  def update_stat(self, CS, enabled):
    # Check if the cruise stalk was double pulled, indicating that adaptive
    # cruise control should be enabled. Twice in .75 seconds counts as a double
    # pull.
    self.prev_enable_adaptive_cruise = self.enable_adaptive_cruise
    acc_string = CS.cstm_btns.get_button_label2(ACCMode.BUTTON_NAME)
    acc_mode = ACCMode.from_label(acc_string)
    CS.cstm_btns.get_button(ACCMode.BUTTON_NAME).btn_label2 = acc_mode.label
    self.autoresume = acc_mode.autoresume
    self.adaptive = acc_mode.adaptive
    curr_time_ms = _current_time_millis()
    # Handle pressing the enable button.
    if (CS.cruise_buttons == CruiseButtons.MAIN  or CS.cruise_buttons == CruiseButtons.DECEL_SET) and self.prev_cruise_buttons != CS.cruise_buttons:
      double_pull = curr_time_ms - self.last_cruise_stalk_pull_time < 750
      self.last_cruise_stalk_pull_time = curr_time_ms
      ready = (CS.cstm_btns.get_button_status(ACCMode.BUTTON_NAME) > ACCState.OFF
               and (enabled or (CS.cruise_buttons == CruiseButtons.DECEL_SET and not self.adaptive))
               and CruiseState.is_enabled_or_standby(CS.pcm_acc_status)
               and CS.v_ego > self.MIN_CRUISE_SPEED_MS)
      if ready and double_pull and ((self.adaptive and CS.cruise_buttons == CruiseButtons.MAIN) or ((not self.adaptive) and CS.cruise_buttons == CruiseButtons.DECEL_SET)):
        # A double pull enables ACC. updating the max ACC speed if necessary.
        self.enable_adaptive_cruise = True
        # Increase ACC speed to match current, if applicable.
        # You set the speed: engage at the current speed (the map speed limit only gives a warning, carcontroller).
        self.acc_speed_kph = CS.v_ego_raw * CV.MS_TO_KPH
        self.user_has_braked = False
        self.has_gone_below_min_speed = False
      else:
        # A single pull disables ACC (falling back to just steering).
        if CS.cruise_buttons == CruiseButtons.MAIN:
          self.enable_adaptive_cruise = False
    # Handle pressing the cancel button.
    if CS.cruise_buttons == CruiseButtons.CANCEL:
      self.enable_adaptive_cruise = False
      self.acc_speed_kph = 0. 
      self.last_cruise_stalk_pull_time = 0
    # Handle pressing up and down buttons.
    elif (CS.cruise_buttons != CruiseButtons.MAIN and self.enable_adaptive_cruise and
          CS.cruise_buttons != self.prev_cruise_buttons):
      self._update_max_acc_speed(CS)
      
    if CS.brake_pressed:
      self.user_has_braked = True
      self.last_brake_press_time = _current_time_millis()
      if not self.autoresume:
        self.enable_adaptive_cruise = False
        
    if CS.v_ego < self.MIN_CRUISE_SPEED_MS:
      self.has_gone_below_min_speed = True
    
    # If autoresume is not enabled and not in standard CC, disable if we hit the brakes or gone below 18mph
    if not self.autoresume:
      if (self.adaptive and not enabled) or self.user_has_braked or self.has_gone_below_min_speed:
        self.enable_adaptive_cruise = False
    
    # Notify if ACC was toggled
    if self.prev_enable_adaptive_cruise and not self.enable_adaptive_cruise:
      CS.UE.custom_alert_message(3, "%s Disabled" % ("ACC" if self.adaptive else "CC"), 150, 4)
      CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.STANDBY)
      self.fleet_speed.reset_averager()
    elif self.enable_adaptive_cruise:
      CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.ENABLED)
      if not self.prev_enable_adaptive_cruise:
        CS.UE.custom_alert_message(2, "%s Enabled" % ("ACC" if self.adaptive else "CC"), 150)

    # Update the UI to show whether the current car state allows ACC.
    if CS.cstm_btns.get_button_status(ACCMode.BUTTON_NAME) in [ACCState.STANDBY, ACCState.NOT_READY]:
      if ((enabled or not self.adaptive)
          and CruiseState.is_enabled_or_standby(CS.pcm_acc_status)
          and CS.v_ego > self.MIN_CRUISE_SPEED_MS):
        CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.STANDBY)
      else:
        CS.cstm_btns.set_button_status(ACCMode.BUTTON_NAME, ACCState.NOT_READY)
          
    # Update prev state after all other actions.
    self.prev_cruise_buttons = CS.cruise_buttons
    self.prev_pcm_acc_status = CS.pcm_acc_status
    
  def _update_max_acc_speed(self, CS):
    # Adjust the max ACC speed based on user cruise stalk actions.
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    speed_change_map = {
      CruiseButtons.RES_ACCEL:     half_press_kph,
      CruiseButtons.RES_ACCEL_2ND: full_press_kph,
      CruiseButtons.DECEL_SET:     -1 * half_press_kph,
      CruiseButtons.DECEL_2ND:     -1 * full_press_kph
    }
    self.acc_speed_kph += speed_change_map.get(CS.cruise_buttons, 0)

    # Clip ACC speed between 0 and 170 KPH.
    self.acc_speed_kph = min(self.acc_speed_kph, 170)
    self.acc_speed_kph = max(self.acc_speed_kph, 0)

  # Decide which cruise control buttons to simluate to get the car to the desired speed.
  def update_acc(self, enabled, CS, frame, actuators, pcm_speed, speed_limit_kph, set_speed_limit_active, speed_limit_offset):
    # Adaptive cruise control
    self.prev_speed_limit_kph = self.speed_limit_kph
    # The map speed limit no longer changes the set speed: the car's two map limits flip back and forth (50 <-> 60,
    # 90 <-> 50 in the logs) and every flip used to overwrite the speed you chose. You set the speed with the stalk;
    # carcontroller gives one warning when you go above the limit.
    self.speed_limit_kph = speed_limit_kph + speed_limit_offset if (set_speed_limit_active and speed_limit_kph > 0) else 0.
    current_time_ms = _current_time_millis()
    if CruiseButtons.should_be_throttled(CS.cruise_buttons):
      self.human_cruise_action_time = current_time_ms
    button_to_press = None
    
    # If ACC is disabled, disengage traditional cruise control.
    if((self.prev_enable_adaptive_cruise) and (not self.enable_adaptive_cruise)
        and (CS.pcm_acc_status == CruiseState.ENABLED)):
      button_to_press = CruiseButtons.CANCEL

    #if non addaptive and we just engaged ACC but pcm is not engaged, then engage
    if (not self.adaptive) and self.enable_adaptive_cruise and (CS.pcm_acc_status != CruiseState.ENABLED):
      button_to_press = CruiseButtons.MAIN

    #if plain cc, not adaptive, then just return None or Cancel
    if (not self.adaptive) and self.enable_adaptive_cruise:
      self.acc_speed_kph = CS.v_cruise_actual #if not CS.imperial_speed_units else CS.v_cruise_actual * CV.MPH_TO_KPH
      return button_to_press

    #disengage if cruise is canceled
    if (not self.enable_adaptive_cruise) and (CS.pcm_acc_status >= 2) and (CS.pcm_acc_status <= 4):
      button_to_press = CruiseButtons.CANCEL 
    lead_1 = None
    #if enabled:
    lead = messaging.recv_one_or_none(self.radarState)
    if lead is not None:
        lead_1 = lead.radarState.leadOne
        self._last_lead = (lead_1, current_time_ms)
        if lead_1.dRel:
          self.lead_last_seen_time_ms = current_time_ms
    elif self._last_lead is not None and current_time_ms - self._last_lead[1] <= LEAD_HOLD_MS:
        lead_1 = self._last_lead[0]      # no new message this cycle: the lead did not vanish, keep the last one
    self._update_lead_filter(CS, lead_1, current_time_ms)
    if self.enable_adaptive_cruise and enabled: # and (button_to_press == None):
      # ACCPLAN: one decision path - safety check, then the planner's speed through the servo
      button_to_press = self._calc_button(CS, lead_1, self._read_plan(current_time_ms), current_time_ms)
    # POSGATE: the ONE exit for every button. No +1 / +5 / RES while
    # braking is needed, or within 3 s of the lead decelerating or being clearly slower (whatever rule asked for it).
    # Before the press bookkeeping below: a blocked press must not count as a press (it would delay the next one).
    if CruiseButtons.is_accel(button_to_press) and self._positive_blocked(CS, lead_1, current_time_ms):
      self.last_follow_msg = "blocked %s (lead braking/slower)" % (self.last_follow_msg or "+")
      button_to_press = None
    if button_to_press:
      self.automated_cruise_action_time = current_time_ms
      # If trying to slow below the min cruise speed, just cancel cruise.
      # This prevents a SCCM crash which is triggered by repeatedly pressing
      # stalk-down when already at min cruise speed.
      if (CruiseButtons.is_decel(button_to_press) and CS.pcm_acc_status == CruiseState.ENABLED
          and CS.v_cruise_actual - 1 < self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
        button_to_press = CruiseButtons.CANCEL
      if button_to_press == CruiseButtons.CANCEL:
        self.fast_decel_time = current_time_ms
      # Debug logging (disable in production to reduce latency of commands)
      #print "***ACC command: %s***" % button_to_press
    if self.enable_adaptive_cruise:
      self._acc_log(CS, lead_1, button_to_press)
    return button_to_press

  def _positive_blocked(self, CS, lead, now_ms):
    if now_ms < self.lead_closing_time_ms + POSITIVE_BLOCK_MS:
      return True
    if lead is not None and getattr(lead, 'dRel', 0) and self._fast_decel_required(CS, lead):
      return True
    return False

  def _acc_log(self, CS, lead, button):
    """ACCLOG: what ACC saw and decided, for tuning the following behaviour offline. One JSON list per cycle:
    [t, v_ego m/s, a_ego m/s2, car set speed kph, your speed kph, lead seen 0/1, lead dist m, lead vRel m/s, lead aRel,
     curve cap kph (999 none), button (0 none), reason, cruise state, brake 0/1, gas 0/1,
     ACCPLAN: plan speed m/s, plan accel m/s2 (None = no fresh plan), wanted set kph, braking needed m/s2,
     lead accel filtered, lead vRel filtered]. Buffered, written once a second,
    new file per start in /data/acclog, oldest files deleted above 300 MB. Off: touch /data/acc_log_off. Never raises."""
    try:
      now = time.time()
      if now - self._log_check_t > 10.:                # the off switch is looked at every 10 s, not every cycle
        self._log_check_t = now
        self._log_off = os.path.exists(ACC_LOG_OFF)
        if self._log_off and self._log_f is not None:
          self._log_f.close(); self._log_f = None
      if self._log_off:
        self._log_buf = []
        return
      seen = bool(lead is not None and lead.status and lead.dRel > 0)
      self._log_buf.append([round(now, 2), round(CS.v_ego, 2), round(getattr(CS, 'a_ego', 0.), 2), round(CS.v_cruise_actual, 1),
                            round(self.acc_speed_kph, 1), int(seen), round(lead.dRel, 1) if seen else None,
                            round(lead.vRel, 2) if seen else None, round(getattr(lead, 'aRel', 0.), 2) if seen else None,
                            round(self.curve_cap_kph), button or 0, self.last_follow_msg, int(CS.pcm_acc_status),
                            int(bool(getattr(CS, 'brake_pressed', 0))), int(bool(getattr(CS, 'user_gas_pressed', 0))),
                            None if self.plan_v is None else round(self.plan_v, 2), None if self.plan_a is None else round(self.plan_a, 2),
                            None if self.follow_target_kph is None else round(self.follow_target_kph, 1),
                            round(self.brake_need_ms2, 2) if seen else None, round(self.a_lead_f, 2), round(self.v_rel_f, 2)])
      self.last_follow_msg = None
      if len(self._log_buf) >= 20:
        if self._log_f is None:
          os.makedirs(ACC_LOG_DIR, exist_ok=True)
          files = sorted(os.path.join(ACC_LOG_DIR, f) for f in os.listdir(ACC_LOG_DIR) if f.startswith('acc_'))
          total = sum(os.path.getsize(f) for f in files)
          while files and total > ACC_LOG_MAX_BYTES:
            total -= os.path.getsize(files[0]); os.remove(files.pop(0))
          self._log_f = open(os.path.join(ACC_LOG_DIR, time.strftime('acc_%Y%m%d_%H%M%S.jsonl')), 'a')
        self._log_f.write(''.join(json.dumps(r, separators=(',', ':')) + '\n' for r in self._log_buf))
        self._log_f.flush()
        self._log_buf = []
    except Exception:
      self._log_buf = []

  def _should_autoengage_cc(self, CS, lead_car=None):
    # Automatically (re)engage cruise control so long as 
    # 1) The carstate allows cruise control
    # 2) There is no imminent threat of collision
    # 3) The user did not cancel ACC by pressing the brake
    cruise_ready = (self.enable_adaptive_cruise
                    and CS.pcm_acc_status == CruiseState.STANDBY
                    and CS.v_ego >= self.MIN_CRUISE_SPEED_MS
                    and _current_time_millis() > self.fast_decel_time + 2000)
                    
    # SAFETY (REENGAGE): engage only when cruise presses alone can handle what is left - braking needed under half of
    # what presses deliver and the lead not braking. (Waiting 3 s after the lead was last slower kept regen on far too
    # long: 120 -> 37 km/h behind a lead that settled at 60 in the bench.) A radar drop-out ("no lead", dRel 0) right
    # after a cancel is not a clear road: no lead in view must last 3 s first (bench: resumes while closing on the car).
    now_ms = _current_time_millis()
    lead_seen = lead_car is not None and lead_car.dRel > 0
    slow_lead = lead_seen and (self._fast_decel_required(CS, lead_car) or self.brake_need_ms2 > REENGAGE_NEED_MS2
                               or self.a_lead_f < REENGAGE_LEAD_ACCEL_MS2)
    if not lead_seen and now_ms < self.lead_last_seen_time_ms + 3000:
      slow_lead = True
    
    # "Autoresume" mode allows cruise to engage even after brake events, but
    # shouldn't trigger DURING braking.
    autoresume_ready = (self.autoresume 
			and CS.a_ego >= 0.1 
			and not self.CC.HSO.human_control
			and _current_time_millis() > self.last_brake_press_time + 1000)
    
    braked = self.user_has_braked or self.has_gone_below_min_speed
    
    return cruise_ready and not slow_lead and (autoresume_ready or not braked)
    
  def _fast_decel_required(self, CS, lead_car):
    """ Identifies situations which call for rapid deceleration. """
    if not lead_car or not lead_car.dRel:
      return False

    collision_imminent = self._seconds_to_collision(CS, lead_car) < 4
    
    lead_absolute_speed_ms = lead_car.vRel + CS.v_ego
    lead_too_slow = lead_absolute_speed_ms < self.MIN_CRUISE_SPEED_MS

    # SAFETY: cruise presses slow the car ~0.6 m/s2 at most; cancelling gives regen ~1.5 m/s2 at once. Cancel when the
    # deceleration NEEDED exceeds what presses deliver: the lead's own braking plus what it takes to stop closing before
    # MIN_GAP_M (need = b_lead + closing^2 / (2 room)), or when the lead brakes hard while we close. Independent of the
    # planner and of every comfort setting. (bench: a 4 s time-to-collision alone came too late at 120 km/h)
    closing_ms = max(0., -lead_car.vRel)
    room_m = lead_car.dRel - MIN_GAP_M
    b_lead = max(0., -self.a_lead_f)
    need_ms2 = 99. if room_m <= 0.5 else b_lead + closing_ms ** 2 / (2. * room_m)
    self.brake_need_ms2 = need_ms2
    brake_needed = need_ms2 > PRESS_DECEL_MS2 or (self.a_lead_f < LEAD_HARD_BRAKE_MS2 and closing_ms > 1.)

    return collision_imminent or lead_too_slow or brake_needed
    
  def _seconds_to_collision(self, CS, lead_car):
    if not lead_car or not lead_car.dRel:
      return sys.maxsize
    elif lead_car.vRel >= 0:
      return sys.maxsize
    return abs(float(lead_car.dRel) / lead_car.vRel)
    
  def _get_cc_units_kph(self, is_imperial_units):
    # Cruise control buttons behave differently depending on whether the car
    # is configured for metric or imperial units.
    if is_imperial_units:
      # Imperial unit cars adjust cruise in units of 1 and 5 mph.
      half_press_kph = 1 * CV.MPH_TO_KPH
      full_press_kph = 5 * CV.MPH_TO_KPH
    else:
      # Metric cars adjust cruise in units of 1 and 5 kph.
      half_press_kph = 1
      full_press_kph = 5
    return half_press_kph, full_press_kph
    
  # ACCPLAN SERVO: the set speed that makes Tesla's cruise follow openpilot's plan. The car's cruise accelerates by about
  # K_CC x (set speed - speed), so to get the planned acceleration a at the planned speed v the set speed is v + a / K_CC.
  # Pressed in 1 / 5 steps toward it with SERVO_HYST_KPH hysteresis; never above YOUR set speed.
  def _calc_button(self, CS, lead_car, plan, now_ms):
    button = None
    msg = None
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    lead_seen = lead_car is not None and lead_car.dRel > 0
    if self._should_autoengage_cc(CS, lead_car=lead_car if lead_seen else None) and self._no_action_for(milliseconds=100):
      # ENGAGESET: SET (stalk down) engages at the CURRENT speed. RES would restore the PREVIOUS set speed on a Tesla
      # (logs: cruise engaging at 95 while driving 65 at 16:17:18, at 104 while 83 behind a slower car at 16:48:46).
      button = CruiseButtons.DECEL_SET
      msg = "engage cruise at current speed"
    elif CS.pcm_acc_status == CruiseState.ENABLED:
      if lead_seen and self._fast_decel_required(CS, lead_car) and self._no_human_action_for(milliseconds=500):
        button = CruiseButtons.CANCEL
        msg = "Off (braking needed %.1f m/s2)" % min(self.brake_need_ms2, 9.9)
        self.new_speed = 1
      elif plan is None:
        msg = "no plan"
      elif getattr(CS, 'user_gas_pressed', False):
        msg = "your accelerator"     # you drive: no presses (the set speed would follow your foot up)
      elif self._no_automated_action_for(milliseconds=SERVO_PRESS_MS) and self._no_human_action_for(milliseconds=SERVO_HUMAN_MS):
        v_plan, a_plan = plan
        v_ref = min(max(v_plan, CS.v_ego - PLAN_DRIFT_MS), CS.v_ego + PLAN_DRIFT_MS)
        raw_kph = min((v_ref + a_plan / K_CC) * CV.MS_TO_KPH, self.acc_speed_kph)
        want_kph = self._smooth_want(raw_kph, now_ms)
        self.follow_target_kph = want_kph
        diff_kph = want_kph - CS.v_cruise_actual
        if diff_kph <= -full_press_kph:
          button = CruiseButtons.DECEL_2ND
        elif diff_kph <= -SERVO_HYST_KPH * half_press_kph:
          button = CruiseButtons.DECEL_SET
        elif CS.v_ego > self.MIN_CRUISE_SPEED_MS and diff_kph >= full_press_kph:
          button = CruiseButtons.RES_ACCEL_2ND
        elif CS.v_ego > self.MIN_CRUISE_SPEED_MS and diff_kph >= SERVO_UP_HYST_KPH * half_press_kph:
          button = CruiseButtons.RES_ACCEL
        if button is not None:
          step = {CruiseButtons.DECEL_2ND: -full_press_kph, CruiseButtons.DECEL_SET: -half_press_kph,
                  CruiseButtons.RES_ACCEL_2ND: full_press_kph, CruiseButtons.RES_ACCEL: half_press_kph}[button]
          self.new_speed = CS.v_cruise_actual + step
          msg = "%+d (plan %.0f km/h %+.2f m/s2 -> set %.1f)" % (step, v_plan * CV.MS_TO_KPH, a_plan, want_kph)
    if msg is not None:
      self.last_follow_msg = msg      # ACCLOG: the reason of this cycle's decision
    return button

  def _smooth_want(self, raw_kph, now_ms):
    """SERVO: low-pass of the wanted set speed; a drop larger than SERVO_FAST_DOWN_KPH goes through at once"""
    if self._want_f is None or now_ms - self._want_ms > 1000:
      self._want_f = raw_kph
    else:
      self._want_f += (raw_kph - self._want_f) * min(1., (now_ms - self._want_ms) / 1000. / SERVO_TAU_S)
    self._want_ms = now_ms
    self._want_f = min(self._want_f, raw_kph + SERVO_FAST_DOWN_KPH)
    return self._want_f

  def _read_plan(self, now_ms):
    """ACCPLAN: (speed m/s, acceleration m/s2) openpilot's planner wants now, or None when there is no fresh plan.
    The speed is interpolated like controlsd does (plan.vStart/aStart -> aTarget over the 0.2 s planner step). The
    planner's camera-curve slowing ('model' source) is ignored when no car is ahead: its own cruise solution is used
    (curve slowing is off on this car - night path blips made 'curves' on straight roads)."""
    msg = messaging.recv_one_or_none(self.plan_sock)
    if msg is not None:
      p = msg.plan
      self._plan = (p.vStart, p.aStart, p.vTarget, p.aTarget, bool(p.hasLead), str(p.longitudinalPlanSource),
                    p.vCruise, p.aCruise, now_ms, bool(msg.valid))
    self.plan_v = self.plan_a = None
    if self._plan is None:
      return None
    v_start, a_start, v_target, a_target, has_lead, source, v_cruise, a_cruise, t_ms, valid = self._plan
    age_s = (now_ms - t_ms) / 1000.
    if not valid or age_s * 1000. > PLAN_MAX_AGE_MS:
      return None
    if source == 'model' and not has_lead:
      v, a = v_cruise, a_cruise
    else:
      dt = min(age_s, 0.2 + 0.01) + 0.01                            # as controlsd: LON_MPC_STEP + DT_CTRL
      a = a_start + (dt / 0.2) * (a_target - a_start)
      v = v_start + dt * (a + a_start) / 2.
    self.plan_v, self.plan_a = v, a
    return v, a

  def _update_lead_filter(self, CS, lead, now_ms):
    """the lead's acceleration and relative speed, low-pass LEAD_FILTER_TAU_S (radar noise); POSGATE / REENGAGE mark"""
    if lead is None or not lead.dRel:
      if now_ms - self._lead_f_ms > LEAD_FILTER_KEEP_MS:   # a short radar drop-out keeps the filters (same car)
        self.a_lead_f, self.v_rel_f, self._lead_f_ms = 0., 0., 0
      return
    a_raw = getattr(lead, 'aLeadK', None)
    if a_raw is None:
      a_raw = getattr(lead, 'aRel', 0.) + getattr(CS, 'a_ego', 0.)
    if self._lead_f_ms == 0 or now_ms - self._lead_f_ms > LEAD_FILTER_KEEP_MS:
      # new lead: acceleration starts at 0 (one raw reading has +-0.3 m/s2 noise - a 0.6 reading alone was a false
      # cancel); a real braking car shows within ~0.5 s, and the closing speed term of the safety check acts at once
      self.a_lead_f, self.v_rel_f = 0., lead.vRel
    else:
      k = min(0.1, (now_ms - self._lead_f_ms) / 1000.) / LEAD_FILTER_TAU_S   # after a drop-out: one normal step, not a jump
      self.a_lead_f += (a_raw - self.a_lead_f) * k
      self.v_rel_f += (lead.vRel - self.v_rel_f) * k
    self._lead_f_ms = now_ms
    if self.a_lead_f < LEAD_BRAKING_MS2 or self.v_rel_f < -LEAD_SLOWER_MS:
      self.lead_closing_time_ms = now_ms

  def _no_human_action_for(self, milliseconds):
    return _current_time_millis() > self.human_cruise_action_time + milliseconds
    
  def _no_automated_action_for(self, milliseconds):
    return _current_time_millis() > self.automated_cruise_action_time + milliseconds
    
  def _no_action_for(self, milliseconds):
    return self._no_human_action_for(milliseconds) and self._no_automated_action_for(milliseconds)
