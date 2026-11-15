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
FOLLOW_CLOSE_S = 12.0        # SMOOTHFOLLOW: close a gap error over ~12 s (12 m too far -> aim 3.6 km/h faster than the lead)
FOLLOW_DEADBAND_KPH = 2.0   # press only when the set speed is this far from the target (slowing: when the gap is fine)
FOLLOW_PRESS_MS = 1000      # speeding up: one +1 per second at most -> evenly spaced small steps = a smooth ramp
FOLLOW_BIG_KPH = 6.0        # speeding up: target this far above AND the lead accelerating -> +5 (keeps up with a fast lead)
FOLLOW_DECEL_PRESS_MS = 500 # slowing down: react quickly (the lead's CURRENT speed, a press every 0.5 s, -5 when far off)
LEAD_SPEED_TAU_S = 2.0      # smoothing of the lead speed used for speeding up (radar vRel is noisy)
FOLLOW_ANTICIPATE_S = 2.0   # aim at the lead's speed this many seconds ahead when it pulls away (its acceleration, capped)
FOLLOW_ANTICIPATE_MAX = 0.6 # m/s2 - the lead's acceleration used for that is capped (radar blips gave 13 km/h jumps)
FOLLOW_REVERSE_MS = 3000    # after a press in one direction, the other direction waits this long (no +1/-1 flip-flop)
ROAD_CLEAR_PRESS_MS = 800   # no lead: +1 at most every 0.8 s back to your speed (was every 0.5 s: a +0.7 m/s2 push)
LEAD_HOLD_MS = 500          # radarState arrives at ~20 Hz like this loop: keep the last lead this long between messages
ACC_CURVES_ON = '/data/acc_curves_on'   # curve slowing with the cruise buttons is OFF unless this file exists (the
                                        # focus is smooth lead following; curves come back once that is right)
CURVE_CONFIRM_MS = 1500     # a curve limit counts only after it was there CONTINUOUSLY this long (night camera path blips
                            # gave 41-63 km/h 'curves' on a straight highway for 1-2 s: -5 then +5 = sag and push)



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
    # CURVEACC: slow down for curves with the cruise buttons (no pedal). curve_cap_kph = the curve speed now (999 = none),
    # curve_lowered = we pressed down for a curve and will press back up to your speed after it.
    self.curve_cap_kph = 999.
    self.curve_lowered = False
    self.curve_warn_time_ms = 0
    # ACCLOG state
    self.last_follow_msg = None
    self._log_f = None
    self._log_buf = []
    self._log_check_t = 0.
    self._log_off = False
    # SMOOTHFOLLOW state
    self.lead_kph_f = None
    self.lead_kph_t = 0
    self.follow_target_kph = None
    self.lead_hist = []          # last 5 (lead speed kph, lead accel) - median against single radar blips
    self._last_lead = None       # (lead, time ms): held LEAD_HOLD_MS between radarState messages
    self._cap_since = None       # when the current uninterrupted curve limit started (ms)
    self._cap_hist = []          # (ms, cap kph) of that stretch, last CURVE_CONFIRM_MS
    self.follow_dir = 0          # +1 / -1: direction of the last follow press
    self.follow_dir_ms = 0

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
    if self.enable_adaptive_cruise and enabled: # and (button_to_press == None):
      if CS.cstm_btns.get_button_label2(ACCMode.BUTTON_NAME) in ["OP", "AutoOP"]:    
        button_to_press = self._calc_button(CS, pcm_speed)
        self.new_speed = pcm_speed * CV.MS_TO_KPH
      else:
        # Alternative speed decision logic that uses the lead car's distance
        # and speed more directly.
        # Bring in the lead car distance from the radarState feed
        
        self.curve_cap_kph = self._curve_cap_kph(CS)
        button_to_press = self._calc_follow_button(CS, lead_1, speed_limit_kph, set_speed_limit_active, speed_limit_offset, frame)
        # CURVEACC: the lead car logic keeps priority - its slow-down / cancel always wins; otherwise the curve may press
        # down (or, after the curve, back up to your speed)
        if not (button_to_press is not None and (CruiseButtons.is_decel(button_to_press) or button_to_press == CruiseButtons.CANCEL)):
          curve_button = self._curve_button(CS, lead_1)
          if curve_button is not None:
            button_to_press = curve_button
    if button_to_press:
      self.automated_cruise_action_time = current_time_ms
      # If trying to slow below the min cruise speed, just cancel cruise.
      # This prevents a SCCM crash which is triggered by repeatedly pressing
      # stalk-down when already at min cruise speed.
      if (CruiseButtons.is_decel(button_to_press)
          and CS.v_cruise_actual - 1 < self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH):
        button_to_press = CruiseButtons.CANCEL
      if button_to_press == CruiseButtons.CANCEL:
        self.fast_decel_time = current_time_ms
      # Debug logging (disable in production to reduce latency of commands)
      #print "***ACC command: %s***" % button_to_press
    if self.enable_adaptive_cruise:
      self._acc_log(CS, lead_1, button_to_press)
    return button_to_press

  def _acc_log(self, CS, lead, button):
    """ACCLOG: what ACC saw and decided, for tuning the following behaviour offline. One JSON list per cycle:
    [t, v_ego m/s, a_ego m/s2, car set speed kph, your speed kph, lead seen 0/1, lead dist m, lead vRel m/s, lead aRel,
     curve cap kph (999 none), button (0 none), reason, cruise state, brake 0/1, gas 0/1]. Buffered, written once a second,
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
                            int(bool(getattr(CS, 'brake_pressed', 0))), int(bool(getattr(CS, 'user_gas_pressed', 0)))])
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

  # function to calculate the cruise button based on a safe follow distance
  def _calc_follow_button(self, CS, lead_car, speed_limit_kph, set_speed_limit_active, speed_limit_offset, frame):
    if lead_car is None:
      return None
    # Desired gap (in seconds) between cars.
    follow_time_s = CS.apFollowTimeInS
    # v_ego is in m/s, so safe_dist_m is in meters.
    safe_dist_m = CS.v_ego * follow_time_s
    current_time_ms = _current_time_millis()
     # Make sure we were able to populate lead_1.
    # dRel is in meters.
    lead_dist_m = lead_car.dRel
    lead_speed_raw_kph = (lead_car.vRel + CS.v_ego) * CV.MS_TO_KPH
    a_lead_raw = getattr(lead_car, 'aLeadK', None)
    if a_lead_raw is None:
      a_lead_raw = getattr(lead_car, 'aRel', 0.) + getattr(CS, 'a_ego', 0.)
    # radar blips: one reading jumps +15..+30 km/h for a single sample (seen on 15 Nov) - use the median of the last 5
    if lead_dist_m > 0:
      self.lead_hist = (self.lead_hist + [(lead_speed_raw_kph, a_lead_raw)])[-5:]
    else:
      self.lead_hist = []
    if self.lead_hist:
      srt = sorted(self.lead_hist)
      lead_speed_kph = srt[len(srt) // 2][0]
      a_lead_med = sorted(a for _, a in self.lead_hist)[len(self.lead_hist) // 2]
    else:
      lead_speed_kph, a_lead_med = lead_speed_raw_kph, 0.
    # Relative velocity between the lead car and our set cruise speed.
    future_vrel_kph = lead_speed_kph - CS.v_cruise_actual
    # How much we can accelerate without exceeding the max allowed speed.
    max_acc_speed_kph = self.fleet_speed.adjust(CS, self.acc_speed_kph * CV.KPH_TO_MS, frame) * CV.MS_TO_KPH
    max_acc_speed_kph = min(max_acc_speed_kph, self.curve_cap_kph)   # CURVEACC: never speed up into a curve
    available_speed_kph = max_acc_speed_kph - CS.v_cruise_actual
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    # button to issue
    button = None
    # debug msg
    msg = None

    # Automatically engage traditional cruise if ACC is active.
    if self._should_autoengage_cc(CS, lead_car=lead_car) and self._no_action_for(milliseconds=100):
      button = CruiseButtons.RES_ACCEL
    # If traditional cruise is engaged, then control it.
    elif CS.pcm_acc_status == CruiseState.ENABLED:
      
      # Disengage cruise control if a slow object is seen ahead. This triggers
      # full regen braking, which is stronger than the braking that happens if
      # you just reduce cruise speed.
      if self._fast_decel_required(CS, lead_car) and self._no_human_action_for(milliseconds=500):
        msg = "Off (Slow traffic)"
        button = CruiseButtons.CANCEL
        self.new_speed = 1
        
      # if cruise is set to faster than the max speed, slow down
      elif CS.v_cruise_actual > max_acc_speed_kph and self._no_action_for(milliseconds=300):
        msg =  "Slow to max"
        button = CruiseButtons.DECEL_SET
        self.new_speed =  max_acc_speed_kph 
        
      elif (# if we have a populated lead_distance
            lead_dist_m > 0
            and self._no_action_for(milliseconds=300)
            # and we're moving
            and CS.v_cruise_actual > full_press_kph):
        ### Slowing down ###
        # Reduce speed significantly if lead_dist < safe dist
        # and if the lead car isn't already pulling away.
        if lead_dist_m < safe_dist_m * .5 and future_vrel_kph < 2:
          msg =  "-5 (Significantly too close)"
          button = CruiseButtons.DECEL_2ND
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - full_press_kph
        # Don't rush up to lead car
        elif future_vrel_kph < -15:
          msg =  "-5 (approaching too fast)"
          button = CruiseButtons.DECEL_2ND
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - full_press_kph
        elif future_vrel_kph < -8:
          msg =  "-1 (approaching too fast)"
          button = CruiseButtons.DECEL_SET
          self.new_speed = CS.v_ego * CV.MS_TO_KPH - half_press_kph
        # SMOOTHFOLLOW: instead of chasing the gap press by press (+1 up to 3x a second when the lead pulled away, then
        # "-1 near safe distance" - felt as a push and a sag), aim the set speed at ONE target: the lead's smoothed speed
        # plus a slow correction that closes / opens the gap towards safe_dist over FOLLOW_CLOSE_S. Press +1/-1 only when
        # the car's set speed is FOLLOW_DEADBAND_KPH or more away from it, at most every FOLLOW_PRESS_MS. The danger cases
        # above (much too close, closing fast, slow traffic) still act at once.
        else:
          # asymmetric on purpose: speeding up uses the smoothed lead speed and is calm; slowing down uses the lead's
          # current speed (whichever is lower) and reacts quickly - a lead that brakes must never be followed lazily
          lead_kph_f = self._lead_speed_filtered(lead_speed_kph, current_time_ms)
          gap_err_m = lead_dist_m - safe_dist_m
          gap_kph = max(-8., min(8., gap_err_m / FOLLOW_CLOSE_S * CV.MS_TO_KPH))
          a_lead = a_lead_med
          ahead_kph = max(0., min(FOLLOW_ANTICIPATE_MAX, a_lead)) * FOLLOW_ANTICIPATE_S * CV.MS_TO_KPH   # only when it pulls away, capped
          up_target_kph = min(lead_kph_f + ahead_kph + gap_kph, max_acc_speed_kph)
          down_target_kph = min(lead_kph_f, lead_speed_kph) + gap_kph
          self.follow_target_kph = up_target_kph
          down_dead_kph = half_press_kph * 1.5 if gap_err_m < 0 else FOLLOW_DEADBAND_KPH   # closer than wanted: react sooner
          urgent = gap_err_m < -0.3 * safe_dist_m or lead_speed_kph < CS.v_ego * CV.MS_TO_KPH - 8.
          since_ms = current_time_ms - self.follow_dir_ms
          may_down = urgent or self.follow_dir != 1 or since_ms > FOLLOW_REVERSE_MS
          may_up = self.follow_dir != -1 or since_ms > FOLLOW_REVERSE_MS
          if (may_down and down_target_kph <= CS.v_cruise_actual - down_dead_kph
              and self._no_action_for(milliseconds=FOLLOW_DECEL_PRESS_MS) and self._no_human_action_for(milliseconds=1000)):
            big = down_target_kph <= CS.v_cruise_actual - full_press_kph
            msg = "%s (follow: aim %.0f)" % ("-5" if big else "-1", down_target_kph)
            button = CruiseButtons.DECEL_2ND if big else CruiseButtons.DECEL_SET
            self.new_speed = CS.v_cruise_actual - (full_press_kph if big else half_press_kph)
            self.follow_dir, self.follow_dir_ms = -1, current_time_ms
          elif (may_up and up_target_kph - CS.v_cruise_actual >= FOLLOW_DEADBAND_KPH and available_speed_kph >= half_press_kph
                and self._no_action_for(milliseconds=FOLLOW_PRESS_MS) and self._no_human_action_for(milliseconds=1000)):
            big = up_target_kph - CS.v_cruise_actual >= FOLLOW_BIG_KPH and a_lead > 0.3 and available_speed_kph >= full_press_kph
            msg = "%s (follow: aim %.0f)" % ("+5" if big else "+1", up_target_kph)
            button = CruiseButtons.RES_ACCEL_2ND if big else CruiseButtons.RES_ACCEL
            self.new_speed = CS.v_cruise_actual + (full_press_kph if big else half_press_kph)
            self.follow_dir, self.follow_dir_ms = 1, current_time_ms

      # If lead_dist is reported as 0, no one is detected in front of you so you
      # can speed up. Only accel on straight-aways; vision radar often
      # loses lead car in a turn.
      elif (lead_dist_m == 0
            and CS.angle_steers < 2.0
            and half_press_kph < available_speed_kph
            and self._no_action_for(milliseconds=ROAD_CLEAR_PRESS_MS)
            and self._no_human_action_for(milliseconds=1000)
            and current_time_ms > self.lead_last_seen_time_ms + 4000):
          msg =  "+1 (road clear)"
          button = CruiseButtons.RES_ACCEL
          self.new_speed = CS.v_ego * CV.MS_TO_KPH + half_press_kph

    if (current_time_ms > self.last_update_time + 1000):
      ratio = 0
      if safe_dist_m > 0:
        ratio = (lead_dist_m / safe_dist_m) * 100
      print ("Ratio: {0:.1f}%  lead: {1:.1f}m  avail: {2:.1f}kph  vRel: {3:.1f}kph  Angle: {4:.1f}deg".format(
        ratio, lead_dist_m, available_speed_kph, lead_car.vRel * CV.MS_TO_KPH, CS.angle_steers))
      self.last_update_time = current_time_ms
      if msg is not None:
        print ("ACC: " + msg)
    if msg is not None:
      self.last_follow_msg = msg      # ACCLOG: the reason of this cycle's decision
    return button
    
  def _should_autoengage_cc(self, CS, lead_car=None):
    # Automatically (re)engage cruise control so long as 
    # 1) The carstate allows cruise control
    # 2) There is no imminent threat of collision
    # 3) The user did not cancel ACC by pressing the brake
    cruise_ready = (self.enable_adaptive_cruise
                    and CS.pcm_acc_status == CruiseState.STANDBY
                    and CS.v_ego >= self.MIN_CRUISE_SPEED_MS
                    and _current_time_millis() > self.fast_decel_time + 2000)
                    
    slow_lead = lead_car and lead_car.dRel > 0 and lead_car.vRel < 0 or self._fast_decel_required(CS, lead_car) # pylint: disable=chained-comparison
    
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
    
    return collision_imminent or lead_too_slow
    
  def _seconds_to_collision(self, CS, lead_car):
    if not lead_car or not lead_car.dRel:
      return sys.maxsize
    elif lead_car.vRel >= 0:
      return sys.maxsize
    return abs(float(lead_car.dRel) / lead_car.vRel)
    
  def _lead_speed_filtered(self, lead_kph, now_ms):
    """SMOOTHFOLLOW: lead speed smoothed with a ~1.5 s time constant; restarts if the lead was not seen for 2 s"""
    if self.lead_kph_f is None or now_ms - self.lead_kph_t > 2000:
      self.lead_kph_f = lead_kph
    else:
      k = min(1., (now_ms - self.lead_kph_t) / 1000. / LEAD_SPEED_TAU_S)
      self.lead_kph_f += (lead_kph - self.lead_kph_f) * k
    self.lead_kph_t = now_ms
    return self.lead_kph_f

  def _curve_cap_kph(self, CS):
    """CURVEACC: the speed (kph) the curve ahead allows now (999 = no curve). Ignored while signalling / changing lane
    (the camera path bends then without a curve). Warns once every 3 s when far too fast for the buttons alone."""
    if CS.turn_signal_blinking or getattr(self.CC, 'alca_enabled', False):
      return 999.
    now_ms = _current_time_millis()
    if now_ms - getattr(self, '_curves_checked_ms', -1e9) > 5000:        # look at the switch file every 5 s only
      self._curves_checked_ms = now_ms
      self._curves_on = os.path.exists(ACC_CURVES_ON)
    if not self._curves_on:
      self._cap_since, self._cap_hist = None, []
      return 999.
    v_curve = self.CC.PCC.curve_speed_for_acc(CS)
    now_ms = _current_time_millis()
    if v_curve >= 254.:
      self._cap_since, self._cap_hist = None, []
      return 999.
    # only a curve that stays: CURVE_CONFIRM_MS without interruption, then the least strict value of that stretch
    if self._cap_since is None:
      self._cap_since = now_ms
    self._cap_hist = [h for h in self._cap_hist if now_ms - h[0] <= CURVE_CONFIRM_MS] + [(now_ms, v_curve)]
    if now_ms - self._cap_since < CURVE_CONFIRM_MS:
      return 999.
    v_curve = max(v for _, v in self._cap_hist)
    if CS.v_ego > v_curve + 4.0 and now_ms - self.curve_warn_time_ms > 3000:     # ~15 km/h over: cruise regen alone is too late
      self.curve_warn_time_ms = now_ms
      CS.UE.custom_alert_message(4, "Sharp curve - brake!", 150, 4)
    # never below the lowest cruise speed (+3): lower than that the code would cancel cruise in the middle of the curve
    return max(v_curve * CV.MS_TO_KPH, self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH + 3.)

  def _curve_button(self, CS, lead_car):
    """CURVEACC: press down while the cruise speed is above the curve speed; after the curve press back up to your
    speed (only what the curve took away, and not toward a lead car that is close)."""
    if CS.pcm_acc_status != CruiseState.ENABLED:
      self.curve_lowered = False
      return None
    half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
    target_kph = min(self.acc_speed_kph, self.curve_cap_kph)
    over_kph = CS.v_cruise_actual - target_kph
    if over_kph >= 2 * full_press_kph and self._no_action_for(milliseconds=300):
      self.curve_lowered = True; self.last_follow_msg = "curve -5"
      return CruiseButtons.DECEL_2ND
    if over_kph >= half_press_kph * 1.5 and self._no_action_for(milliseconds=500):
      self.curve_lowered = True; self.last_follow_msg = "curve -1"
      return CruiseButtons.DECEL_SET
    if self.curve_lowered:
      if lead_car is not None and lead_car.status and lead_car.dRel > 0:
        self.curve_lowered = False                    # a car ahead: the follow logic sets the speed from here
        return None
      under_kph = target_kph - CS.v_cruise_actual
      if under_kph < half_press_kph:
        if self.curve_cap_kph >= self.acc_speed_kph:
          self.curve_lowered = False                  # curve over and back at your speed
      elif self._no_human_action_for(milliseconds=1000) and self._no_action_for(milliseconds=1000):
        self.last_follow_msg = "after curve +1"       # +1 per second: a smooth ramp (+5 steps were a push)
        return CruiseButtons.RES_ACCEL
    return None

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
    
  # Adjust speed based off OP's longitudinal model. As of OpenPilot 0.5.3, this
  # is inoperable because the planner crashes when given only visual radar
  # inputs. (Perhaps this can be used in the future with a radar install, or if
  # OpenPilot planner changes.)
  def _calc_button(self, CS, desired_speed_ms):
    button_to_press = None
    # Automatically engange traditional cruise if appropriate.
    if self._should_autoengage_cc(CS) and desired_speed_ms >= CS.v_ego:
      button_to_press = CruiseButtons.RES_ACCEL
    # If traditional cruise is engaged, then control it.
    elif (CS.pcm_acc_status == CruiseState.ENABLED
          # But don't make adjustments if a human has manually done so in
          # the last 3 seconds. Human intention should not be overridden.
          and self._no_human_action_for(milliseconds=3000)
          and self._no_automated_action_for(milliseconds=500)):
      # The difference between OP's target speed and the current cruise
      # control speed, in KPH.
      speed_offset_kph = (desired_speed_ms * CV.MS_TO_KPH - CS.v_cruise_actual)
    
      half_press_kph, full_press_kph = self._get_cc_units_kph(CS.imperial_speed_units)
      
      # Reduce cruise speed significantly if necessary. Multiply by a % to
      # make the car slightly more eager to slow down vs speed up.
      if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
        button_to_press = CruiseButtons.CANCEL
      if speed_offset_kph < -2 * full_press_kph and CS.v_cruise_actual > 0:
        button_to_press = CruiseButtons.CANCEL
      elif speed_offset_kph < -0.6 * full_press_kph and CS.v_cruise_actual > 0:
        # Send cruise stalk dn_2nd.
        button_to_press = CruiseButtons.DECEL_2ND
      # Reduce speed slightly if necessary.
      elif speed_offset_kph < -0.9 * half_press_kph and CS.v_cruise_actual > 0:
        # Send cruise stalk dn_1st.
        button_to_press = CruiseButtons.DECEL_SET
      # Increase cruise speed if possible.
      elif CS.v_ego > self.MIN_CRUISE_SPEED_MS:
        # How much we can accelerate without exceeding max allowed speed.
        available_speed_kph = self.acc_speed_kph - CS.v_cruise_actual
        if speed_offset_kph >= full_press_kph and full_press_kph < available_speed_kph:
          # Send cruise stalk up_2nd.
          button_to_press = CruiseButtons.RES_ACCEL_2ND
        elif speed_offset_kph >= half_press_kph and half_press_kph < available_speed_kph:
          # Send cruise stalk up_1st.
          button_to_press = CruiseButtons.RES_ACCEL
    return button_to_press
    
  def _no_human_action_for(self, milliseconds):
    return _current_time_millis() > self.human_cruise_action_time + milliseconds
    
  def _no_automated_action_for(self, milliseconds):
    return _current_time_millis() > self.automated_cruise_action_time + milliseconds
    
  def _no_action_for(self, milliseconds):
    return self._no_human_action_for(milliseconds) and self._no_automated_action_for(milliseconds)
