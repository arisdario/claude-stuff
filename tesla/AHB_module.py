from selfdrive.config import Conversions as CV
from cereal import tesla,log
import cereal.messaging as messaging
import os
import time

# camera-luminance brightness estimate + decision state machine (new; see ahb_luminance.py / ahb_state.py).
# Import works both inside openpilot (package path) and from the standalone test (which puts tesla/ on sys.path).
try:
  from selfdrive.car.tesla import ahb_luminance, ahb_state
except Exception:
  import ahb_luminance, ahb_state

DEBUG = False

# AHBAUTO: a pre-AP car with MCU2 never sends MCU_chassisControl.MCU_ahlbEnable (there is no Autopilot menu), so the
# check "AHB enabled in CID" below always fails. touch /data/ahb_auto to use your own switch instead.
#   stalk in normal position  -> the code manages the beams: DAS decision ON when the road is clear, OFF otherwise
#                                (works only if the light controller obeys DAS "on" - check with /data/ahb_test)
#   stalk in high-beam position -> Tesla's way: you hold high, the code dips (OFF) for traffic, else UNDECIDED (stalk)
#   stalk pulled (flash)      -> UNDECIDED, the driver wins
# Oncoming/preceding traffic checks (radar, camera, image brightness, hold time) stay exactly as they are.
AHB_AUTO_FILE = '/data/ahb_auto'
AHB_AUTO_ON_MS = 30.0 / 3.6      # auto mode: high beam allowed from 30 km/h (country roads at 50-60 are where it matters) ...
AHB_AUTO_OFF_MS = 20.0 / 3.6     # ... and dipped again below 20 km/h (the gap stops flicker, not a high threshold)
# AHBTEST: touch /data/ahb_test, parked with lights on, every 2 s:
#   stalk in high-beam position -> dip / release   (does the car obey a DAS dip?)
#   stalk in normal position    -> high on / off   (does the car obey DAS "on"? needed for code-managed high beam)
# rm the file when done.
AHB_TEST_FILE = '/data/ahb_test'

class AHBReason():
  HIGH_BEAM_ON = 0
  HIGH_BEAM_OFF_REASON_MOVING_VISION_TARGET = 1
  HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET = 2
  HIGH_BEAM_OFF_REASON_AMBIENT_LIGHT = 3
  HIGH_BEAM_OFF_REASON_HEAD_LIGHT = 4
  HIGH_BEAM_OFF_REASON_SNA = 5

class AHBDecision():
  DAS_HIGH_BEAM_UNDECIDED = 0
  DAS_HIGH_BEAM_OFF = 1
  DAS_HIGH_BEAM_ON = 2
  DAS_HIGH_BEAM_SNA = 3

def _current_time_millis():
  return int(round(time.time() * 1000))

def _debug(message):
  if DEBUG:
    print(message)

class AHBController():

  def __init__(self,carcontroller):
    self.time_last_car_detected = 0
    self.time_last_high_beam_cancel = 0
    self.prev_car_present = False
    self.prev_highLowBeamStatus = 0
    self.prev_highLowBeamReason = 0
    self.prev_light_stalk_position = 0
    self.prev_high_beam_on = False
    self.prev_lights_on = False
    self.ahbInfo = messaging.sub_sock('ahbInfo', conflate=True)
    self.ahbInfoData = None
    self.ahbIsEnabled = False
    self.frameInfo = messaging.sub_sock('frame', conflate=True)
    self.frameInfoData = None
    self.frameInfoGain = 0
    self.auto_allowed = False          # AHBAUTO speed hysteresis state
    self._flags = (False, False); self._flags_t = 0   # (auto, test) flag files, checked once a second
    self.lum = ahb_luminance.LuminanceEstimator()     # road-scene brightness from camera exposure (relative estimate)
    self.ahb_sm = ahb_state.AhbStateMachine()         # 4-state decision engine (runs for diagnostics; see note below)
    self.diag = {}                                    # last cycle's diagnostics (luminance, state, reason)
    self._diag_t = 0

  def auto_or_test(self):
    """AHBAUTO: carcontroller sends the high-beam decision when this is true even without has_tesla_ic_integration
    (turning that on would also take lane change and speed limit from MCU settings a pre-AP MCU2 never sends)"""
    auto, test = self._flag_files(_current_time_millis())
    return auto or test

  def _flag_files(self, tms_now):
    if tms_now - self._flags_t > 1000:
      self._flags_t = tms_now
      self._flags = (os.path.exists(AHB_AUTO_FILE), os.path.exists(AHB_TEST_FILE))
    return self._flags

  def set_and_return(self,CS,frame,highLowBeamStatus,highLowBeamReason):
    self.prev_light_stalk_position = CS.ahbHighBeamStalkPosition
    if self.ahbInfoData is not None:
      self.prev_car_present = self.ahbInfoData.cameraCarDetected or self.ahbInfoData.radarCarDetected
    self.prev_highLowBeamStatus = highLowBeamStatus
    self.prev_highLowBeamReason = highLowBeamReason
    self.prev_high_beam_on = CS.ahbHiBeamOn
    self.prev_lights_on = CS.ahbLoBeamOn
    self.ahbIsEnabled = self.ahbIsEnabled and bool(CS.ahbEnabled or self._flags[0] or self._flags[1])   # AHBTEST: test mode reports 'AHB enabled' like auto mode does
    return highLowBeamStatus,highLowBeamReason,self.ahbIsEnabled

  def _scene_is_bright(self):
    """True = too bright for high beam. Prefer the luminance estimator (needs real gain+integLines); fall back to the
    original gain>=510 rule when it has no valid reading (tests, or a frame without integLines)."""
    if self.lum.valid:
      return not self.lum.is_dark()
    return self.frameInfoGain < 510

  def _run_diag(self, CS, tms_now):
    """Run the 4-state decision engine for diagnostics/logging and record this cycle's state. It does NOT drive the
    DAS output (the proven update() logic above remains the authority, so existing behaviour and tests are preserved);
    it is the richer engine, ready to take over once the luminance thresholds are calibrated on a night drive.
    Cheap: one dict per cycle. Optionally dumped to /data/ahb_diag.json at <=2 Hz when /data/ahb_debug exists."""
    info = self.ahbInfoData
    oncoming = bool(info and getattr(info, 'cameraCarDetected', False))
    lead = bool(info and getattr(info, 'radarCarDetected', False))
    inp = ahb_state.AhbInputs(
      now=tms_now / 1000.0,
      enabled=bool(CS.ahbEnabled or self._flags[0] or self._flags[1]),
      dark=self.lum.is_dark() if self.lum.valid else (self.frameInfoGain >= 510),
      bright=(self.lum.level == ahb_luminance.BRIGHT) if self.lum.valid else False,
      lum_valid=self.lum.valid if self.lum.valid else (self.frameInfoGain > 0),
      v_ego=CS.v_ego, stalk=CS.ahbHighBeamStalkPosition, headlights_on=bool(CS.ahbLoBeamOn),
      oncoming=oncoming, lead=lead, vision_target=oncoming)
    try:
      sm = self.ahb_sm.update(inp)
    except Exception:
      sm = {'state': None, 'reason': None, 'high': None}
    self.diag = dict(self.lum.diagnostics())
    self.diag.update({'frame_gain': self.frameInfoGain, 'sm_state': sm['state'], 'sm_high': sm['high'],
                      'sm_reason': sm['reason'], 'oncoming': oncoming, 'lead': lead, 'v_ego': round(CS.v_ego, 1)})
    if tms_now - self._diag_t > 500 and os.path.exists('/data/ahb_debug'):
      self._diag_t = tms_now
      try:
        import json
        with open('/data/ahb_diag.json', 'w') as f:
          json.dump(self.diag, f)
      except Exception:
        pass

  def update(self, CS, frame,ahbLead1):
    tms_now = _current_time_millis()
    ahbInfoMsg = self.ahbInfo.receive(non_blocking=True)
    frameInfoMsg = messaging.recv_one_or_none(self.frameInfo)
    if ahbInfoMsg is not None:
      self.ahbInfoData = tesla.AHBinfo.from_bytes(ahbInfoMsg)
    if frameInfoMsg is not None:
      self.frameInfoData = frameInfoMsg.frame
      frameInfoGain = self.frameInfoData.globalGain
      exposureTime = self.frameInfoData.androidCaptureResult.exposureTime
      frameDuration = self.frameInfoData.androidCaptureResult.frameDuration
      if frameInfoGain != self.frameInfoGain:
        self.frameInfoGain = frameInfoGain
      # feed the luminance estimator: globalGain + integLines -> relative brightness (see ahb_luminance.py).
      integ_lines = getattr(self.frameInfoData, 'integLines', None)
      self.lum.update_exposure(self.frameInfoGain, integ_lines, tms_now / 1000.0)
      _debug("AHB Camera has new data [ frame - " + str(self.frameInfoData.frameId) + "]  = " + str(frameInfoGain) + ", exposure = " + str(exposureTime) + ", frame duration = " + str(frameDuration))
    self.lum.tick(tms_now / 1000.0)      # expire the estimate if the camera stalls
    self._run_diag(CS, tms_now)
    auto, test = self._flag_files(tms_now)
    #AHBTEST: parked -> stalk high: dip / release every 2 s;  stalk normal: DAS high on / off every 2 s
    if test and CS.v_ego < 0.5 and CS.ahbHighBeamStalkPosition in (0, 1):
      self.ahbIsEnabled = True
      first_half = (tms_now // 2000) % 2 == 0
      if CS.ahbHighBeamStalkPosition == 1:
        return self.set_and_return(CS, frame, AHBDecision.DAS_HIGH_BEAM_OFF if first_half else AHBDecision.DAS_HIGH_BEAM_UNDECIDED,
                                   AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET if first_half else AHBReason.HIGH_BEAM_ON)
      return self.set_and_return(CS, frame, AHBDecision.DAS_HIGH_BEAM_ON if first_half else AHBDecision.DAS_HIGH_BEAM_OFF,
                                 AHBReason.HIGH_BEAM_ON if first_half else AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET)
    #if AHB not enabled, then return OFF (AHBAUTO: /data/ahb_auto replaces the MCU setting)
    if CS.ahbEnabled != 1 and not auto:
      _debug("AHB not enabled in CID")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #AHBAUTO code-managed: stalk in normal position, the code switches high beam on and off itself
    managed = auto and CS.ahbHighBeamStalkPosition == 0
    #if stalk is not in the high beam position, return UNDECIDED (code-managed: only when you pull it to flash)
    if (CS.ahbHighBeamStalkPosition != 1) and not managed:
      _debug("High Beam not on")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      self.ahbIsEnabled = False
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason) 
    else :
      self.ahbIsEnabled = True
    #AHBAUTO: only from 30 km/h, dipped below 20 km/h
    if auto and CS.ahbEnabled != 1:
      if CS.v_ego >= AHB_AUTO_ON_MS: self.auto_allowed = True
      elif CS.v_ego < AHB_AUTO_OFF_MS: self.auto_allowed = False
      if not self.auto_allowed:
        _debug("AHB auto: below 30 km/h")
        return self.set_and_return(CS,frame,AHBDecision.DAS_HIGH_BEAM_OFF,AHBReason.HIGH_BEAM_OFF_REASON_SNA)
    #if moving below 10mph take no decision, then return undecided
    if CS.v_ego < 4.47:
      _debug("moving too slow for decision")
      highLowBeamStatus = self.prev_highLowBeamStatus
      highLowBeamReason = self.prev_highLowBeamReason
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    if self.ahbInfoData is None:
      _debug("No radar info")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    # scene bright enough that high beam is not warranted (oncoming light, street lighting, dusk). Uses the camera
    # luminance estimator when it has real exposure data (gain + integLines); else the original gain>=510 rule, so the
    # existing tests and any frame without integLines behave exactly as before.
    if self._scene_is_bright():
      _debug("scene too bright for high beam")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_AMBIENT_LIGHT
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by radarD, i.e. OP has Lead, then reset timer and return OFF
    if False and ahbLead1 is not None:
      _debug("OP detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by radar, then reset timer and return OFF
    if self.ahbInfoData.radarCarDetected:
      _debug("Radar detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if lead car detected by vision, then reset timer and return OFF
    if self.ahbInfoData.cameraCarDetected:
      _debug("Vision detected car")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_VISION_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if still waiting for the delay after car detected, send off
    if (tms_now - self.time_last_car_detected < 1000 * CS.ahbOffDuration ):
      _debug("Waiting for time delay since last car")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason) 
    #we got here, high beam should be on (code-managed: say ON; otherwise UNDECIDED lets the stalk keep high beam)
    highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_ON if managed else AHBDecision.DAS_HIGH_BEAM_UNDECIDED
    highLowBeamReason = AHBReason.HIGH_BEAM_ON
    _debug("All conditions met, turn lights ON")
    return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
