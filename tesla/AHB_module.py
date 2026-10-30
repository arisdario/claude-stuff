from selfdrive.config import Conversions as CV
from cereal import tesla,log
import cereal.messaging as messaging
import os
import time

DEBUG = False

# AHBAUTO: a pre-AP car with MCU2 never sends MCU_chassisControl.MCU_ahlbEnable (there is no Autopilot menu), so the
# check "AHB enabled in CID" below always fails. touch /data/ahb_auto to use your own switch instead. The car only
# obeys a DIP from here (OFF); "on" is UNDECIDED = your stalk decides, so leave the stalk in the high-beam position.
# Oncoming/preceding traffic checks (radar, camera, image brightness, hold time) stay exactly as they are.
AHB_AUTO_FILE = '/data/ahb_auto'
AHB_AUTO_ON_MS = 70.0 / 3.6      # auto mode: high beam allowed from 70 km/h ...
AHB_AUTO_OFF_MS = 60.0 / 3.6     # ... and dipped again below 60 km/h (no flicker around 70)
# AHBTEST: touch /data/ahb_test, parked with lights on and the stalk in high beam: dips and releases every 2 s, so you
# can see whether your light controller obeys the (fake) Autopilot computer at all. rm the file when done.
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
    self.ahbIsEnabled = self.ahbIsEnabled and (CS.ahbEnabled or self._flags[0])
    return highLowBeamStatus,highLowBeamReason,self.ahbIsEnabled

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
      _debug("AHB Camera has new data [ frame - " + str(self.frameInfoData.frameId) + "]  = " + str(frameInfoGain) + ", exposure = " + str(exposureTime) + ", frame duration = " + str(frameDuration))
    auto, test = self._flag_files(tms_now)
    #AHBTEST: parked, lights on, stalk in high beam -> dip / release every 2 s
    if test and CS.v_ego < 0.5 and CS.ahbHighBeamStalkPosition == 1:
      self.ahbIsEnabled = True
      dip = (tms_now // 2000) % 2 == 0
      return self.set_and_return(CS, frame, AHBDecision.DAS_HIGH_BEAM_OFF if dip else AHBDecision.DAS_HIGH_BEAM_UNDECIDED,
                                 AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET if dip else AHBReason.HIGH_BEAM_ON)
    #if AHB not enabled, then return OFF (AHBAUTO: /data/ahb_auto replaces the MCU setting)
    if CS.ahbEnabled != 1 and not auto:
      _debug("AHB not enabled in CID")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
    #if stalk is not in the high beam position, return UNDECIDED
    if (CS.ahbHighBeamStalkPosition != 1):
      _debug("High Beam not on")
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_SNA
      self.ahbIsEnabled = False
      return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason) 
    else :
      self.ahbIsEnabled = True
    #AHBAUTO: only from 70 km/h, dipped below 60 km/h
    if auto and CS.ahbEnabled != 1:
      if CS.v_ego >= AHB_AUTO_ON_MS: self.auto_allowed = True
      elif CS.v_ego < AHB_AUTO_OFF_MS: self.auto_allowed = False
      if not self.auto_allowed:
        _debug("AHB auto: below 70 km/h")
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
    # if gain less than max, we might see incoming car
    if self.frameInfoGain < 510:
      _debug("OP camera gain < 510")
      self.time_last_car_detected = tms_now
      highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_OFF
      highLowBeamReason = AHBReason.HIGH_BEAM_OFF_REASON_MOVING_RADAR_TARGET
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
    #we got here, high beam should be on
    highLowBeamStatus = AHBDecision.DAS_HIGH_BEAM_UNDECIDED
    highLowBeamReason = AHBReason.HIGH_BEAM_ON
    _debug("All conditions met, turn lights ON")
    return self.set_and_return(CS,frame,highLowBeamStatus,highLowBeamReason)
