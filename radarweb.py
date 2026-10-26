#!/usr/bin/env python3
# radarweb.py - live radar view in a browser (read-only). Runs a small web server on the EON; open it from your phone.
#   cd /data/openpilot && PYTHONPATH=/data/openpilot:/data nohup python3 /data/radarweb.py 8088 >/dev/null 2>&1 &
#   then on a phone/laptop on the SAME network:   http://<EON-IP>:8088/        (the script prints the IP guess to /data/radarweb.log)
#   /map = Tesla map/nav data page (speed limit, spline, curvature, raw CAN, carState fields); logs 1 line/s to /data/maplog/
#   /  = driver's-eye road view (lane lines, path, class icons)    /top = top-down view    /?demo=1 = demo without the car
# API (JSON):
#   GET /api/state    one snapshot (adds: cruise km/h, alert text, model = lane/path polynomials)
#   GET /api/stream   server-sent events, 10 snapshots per second  (what the page uses)
# Password: RADARWEB_AUTH='user:password' python3 radarweb.py ...   (needed before exposing it with ngrok: the page shows speed and GPS position)
# Needs /data/radarlog.py next to it (uses its decoders). Sends nothing to the car. No password: use only on a network you trust.
import sys, os, time, json, socket, threading
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
try:
    import cereal.messaging as messaging
except ImportError:      # not on the EON (e.g. replay on a Mac): decoders and pages still importable
    messaging = None
from radarlog import decode_a, decode_b, decode_sgu, decode_alerts, CLASS, MOVING
import radarml

def _arg(i, cast, default):
    try: return cast(sys.argv[i])
    except Exception: return default
PORT = _arg(1, int, 8088)
RADAR_SRC = _arg(2, int, 1)
AUTH = os.environ.get('RADARWEB_AUTH', '')          # 'user:password' -> every page and API needs HTTP basic auth (use this before exposing it with ngrok)
BIND = os.environ.get('RADARWEB_BIND', '0.0.0.0')   # '127.0.0.1' = only reachable through ngrok / on the EON itself
STALE = 0.3
# LABELBTN: the road/top views have buttons (Pedestrian/Vehicle/Two-wheel/Construction/Missed/False+)
# that POST here instead of you having to ssh in and `echo "..." > mark` while driving. This writes to
# the exact same file canraw.py (or rawlog.py) already polls for its 0x04 MARK record - no change needed
# on that end - so the label lands in whichever recording is currently running, timestamped, and shows
# up automatically as the "MARK: ..." alert banner when you replay that recording later.
LABEL_FILE = os.environ.get('CANRAW_MARK', '/data/canraw/mark')
# ICON_DIR: where /icons/*.webp is served from. '/data/radar_icons' is the EON convention; off the EON
# (e.g. testing on a Mac) that path often can't even be created (sealed system volume on modern macOS),
# so fall back to an 'icons' folder sitting right next to this script - no root/sudo needed there.
# Override explicitly with RADAR_ICON_DIR= if you want some other location on either platform.
ICON_DIR = os.environ.get('RADAR_ICON_DIR') or ('/data/radar_icons' if os.path.isdir('/data/radar_icons')
           else os.path.join(os.path.dirname(os.path.abspath(__file__)), 'icons'))
LAST_LABEL = {'t': 0.0, 'text': ''}
# LEARN: per-object labels from the /learn page (slot + features + your label), one JSON row per line.
OBJLABEL_FILE = os.environ.get('LEARN_LABELS', '/data/canraw/labels.jsonl')
STATE = {'acc': False, 'alc': False, 't': 0, 'v': 0.0, 'ang': 0.0, 'op': False, 'cruise': 0, 'alert': '', 'model': None, 'lead': None, 'status': {}, 'o': [], 'pw': None, 'lim': None, 'park': None, 'batt': None}
LOCK = threading.Lock()
MAP = {'car': {}, 'ctl': {}}
MAPDB = {int(k): v for k, v in json.loads(r'''{"1160":[{"n":"DAS_steeringControl","s":[["DAS_steeringControlType",null,23,2,false,1.0,0.0,"",0],["DAS_steeringAngleRequest",null,6,15,false,0.1,-1638.35,"deg",0],["DAS_steeringHapticRequest",null,7,1,false,1.0,0.0,"",0]]}],"697":[{"n":"DAS_control","s":[["DAS_setSpeed",null,0,12,false,0.1,0.0,"kph",1],["DAS_accState",null,12,4,false,1.0,0.0,"",1],["DAS_aebEvent",null,16,2,false,1.0,0.0,"",1],["DAS_jerkMin",null,18,9,false,0.03,-15.232,"m/s^3",1],["DAS_jerkMax",null,27,8,false,0.059,0.0,"m/s^3",1],["DAS_accelMin",null,35,9,false,0.04,-15.0,"m/s^2",1],["DAS_accelMax",null,44,9,false,0.04,-15.0,"m/s^2",1]]}],"521":[{"n":"DAS_longControl","s":[["DAS_locMode",null,0,2,false,1.0,0.0,"",1],["DAS_locState",null,2,3,false,1.0,0.0,"",1],["DAS_locRequest",null,5,3,false,1.0,0.0,"",1],["DAS_locJerkMin",null,8,8,false,0.034,-8.67,"m/s^3",1],["DAS_locJerkMax",null,16,8,false,0.034,0.0,"m/s^3",1],["DAS_locSpeed",null,24,11,false,0.1,0.0,"kph",1],["DAS_locAccelMin",null,35,9,false,0.04,-15.0,"m/s^2",1],["DAS_locAccelMax",null,44,9,false,0.04,-15.0,"m/s^2",1]]}],"569":[{"n":"DAS_lanes","s":[["DAS_leftLaneExists",null,0,1,false,1.0,0.0,"",1],["DAS_rightLaneExists",null,1,1,false,1.0,0.0,"",1],["DAS_virtualLaneWidth",null,4,4,false,0.3125,2.0,"m",1],["DAS_virtualLaneViewRange",null,8,8,false,1.0,0.0,"m",1],["DAS_virtualLaneC0",null,16,8,false,0.035,-3.5,"m",1],["DAS_virtualLaneC1",null,24,8,false,0.0016,-0.2,"rad",1],["DAS_virtualLaneC2",null,32,8,false,2e-05,-0.0025,"m-1",1],["DAS_virtualLaneC3",null,40,8,false,2.4e-07,-3e-05,"m-2",1],["DAS_leftLineUsage",null,48,2,false,1.0,0.0,"",1],["DAS_rightLineUsage",null,50,2,false,1.0,0.0,"",1],["DAS_leftFork",null,52,2,false,1.0,0.0,"",1],["DAS_rightFork",null,54,2,false,1.0,0.0,"",1]]}],"257":[{"n":"GTW_epasControl","s":[["GTW_epasControlType",null,15,2,false,1.0,0.0,"",0],["GTW_epasEmergencyOn",null,7,1,false,1.0,0.0,"",0],["GTW_epasLDWEnabled",null,12,1,false,1.0,0.0,"",0],["GTW_epasPowerMode",null,6,4,false,1.0,0.0,"",0],["GTW_epasTuneRequest",null,2,3,false,1.0,0.0,"",0]]}],"880":[{"n":"EPAS_sysStatus","s":[["EPAS_currentTuneMode",null,7,4,false,1.0,0.0,"",0],["EPAS_eacErrorCode",null,23,4,false,1.0,0.0,"",0],["EPAS_eacStatus",null,55,3,false,1.0,0.0,"",0],["EPAS_handsOnLevel",null,39,2,false,1.0,0.0,"",0],["EPAS_internalSAS",null,37,14,false,0.1,-819.200012,"deg",0],["EPAS_steeringFault",null,2,1,false,1.0,0.0,"",0],["EPAS_steeringRackForce",null,1,10,false,50.0,-25575.0,"N",0],["EPAS_steeringReduced",null,3,1,false,1.0,0.0,"",0],["EPAS_torsionBarTorque",null,19,12,false,0.01,-20.5,"Nm",0]]}],"305":[{"n":"EPAS3P_sysStatus","s":[["EPAS_currentTuneMode",null,7,4,false,1.0,0.0,"",0],["EPAS_eacErrorCode",null,23,4,false,1.0,0.0,"",0],["EPAS_eacStatus",null,55,3,false,1.0,0.0,"",0],["EPAS_handsOnLevel",null,39,2,false,1.0,0.0,"",0],["EPAS_internalSAS",null,37,14,false,0.1,-819.2,"deg",0],["EPAS_steeringFault",null,2,1,false,1.0,0.0,"",0],["EPAS_steeringRackForce",null,1,10,false,50.0,-25575.0,"N",0],["EPAS_steeringReduced",null,3,1,false,1.0,0.0,"",0],["EPAS_torsionBarTorque",null,19,12,false,0.01,-20.5,"Nm",0]]}],"3":[{"n":"STW_ANGL_STAT","s":[["StW_Angl",null,5,14,false,0.5,-2048.0,"deg",0],["StW_AnglSpd",null,21,14,false,0.5,-2048.0,"/s",0],["StW_AnglSens_Stat",null,33,2,false,1.0,0.0,"",0],["StW_AnglSens_Id",null,35,2,false,1.0,0.0,"",0]]}],"14":[{"n":"STW_ANGLHP_STAT","s":[["StW_AnglHP",null,5,14,false,0.1,-819.2,"deg",0],["StW_AnglHP_Spd",null,21,14,false,0.5,-4096.0,"deg/s",0],["StW_AnglHP_Sens_Stat",null,33,2,false,1.0,0.0,"",0],["StW_AnglHP_Sens_Id",null,35,2,false,1.0,0.0,"",0]]}],"264":[{"n":"DI_torque1","s":[["DI_torqueDriver",null,0,13,true,0.25,0.0,"Nm",1],["DI_torqueMotor",null,16,13,true,0.25,0.0,"Nm",1],["DI_soptState",null,29,3,false,1.0,0.0,"",1],["DI_motorRPM",null,32,16,true,1.0,0.0,"RPM",1],["DI_pedalPos",null,48,8,false,0.4,0.0,"%",1]]}],"280":[{"n":"DI_torque2","s":[["DI_torqueEstimate",null,0,12,true,0.5,0.0,"Nm",1],["DI_gear",null,12,3,false,1.0,0.0,"",1],["DI_brakePedal",null,15,1,false,1.0,0.0,"",1],["DI_vehicleSpeed",null,16,12,false,0.05,-25.0,"MPH",1],["DI_gearRequest",null,28,3,false,1.0,0.0,"",1],["DI_torqueInterfaceFailure",null,31,1,false,1.0,0.0,"",1],["DI_brakePedalState",null,36,2,false,1.0,0.0,"",1],["DI_epbParkRequest",null,38,1,false,1.0,0.0,"",1],["DI_epbInterfaceReady",null,39,1,false,1.0,0.0,"",1]]}],"309":[{"n":"ESP_135h","s":[["ESP_absBrakeEvent",null,2,1,false,1.0,0.0,"",0],["ESP_brakeDiscWipingActive",null,4,1,false,1.0,0.0,"",0],["ESP_brakeLamp",null,3,1,false,1.0,0.0,"",0],["ESP_espFaultLamp",null,6,1,false,1.0,0.0,"",0],["ESP_espLampFlash",null,7,1,false,1.0,0.0,"",0],["ESP_hillStartAssistActive",null,1,2,false,1.0,0.0,"",0],["ESP_messagePumpService",null,24,1,false,1.0,0.0,"",0],["ESP_messagePumpFailure",null,25,1,false,1.0,0.0,"",0],["ESP_messageEBDFailure",null,26,1,false,1.0,0.0,"",0],["ESP_absFaultLamp",null,27,1,false,1.0,0.0,"",0],["ESP_tcDisabledByFault",null,28,1,false,1.0,0.0,"",0],["ESP_messageDynoModeActive",null,29,1,false,1.0,0.0,"",0],["ESP_hydraulicBoostEnabled",null,30,1,false,1.0,0.0,"",0],["ESP_espOffLamp",null,31,1,false,1.0,0.0,"",0],["ESP_stabilityControlSts",null,14,3,false,1.0,0.0,"",0],["ESP_tcLampFlash",null,5,1,false,1.0,0.0,"",0],["ESP_tcOffLamp",null,15,1,false,1.0,0.0,"",0]]}],"341":[{"n":"ESP_B","s":[["ESP_vehicleSpeed",null,47,16,false,0.00999999978,0.0,"kph",0],["ESP_vehicleSpeedQF",null,57,2,false,1.0,0.0,"",0],["ESP_wheelPulseCountFrL",null,7,8,false,1.0,0.0,"",0],["ESP_wheelPulseCountFrR",null,15,8,false,1.0,0.0,"",0],["ESP_wheelPulseCountReL",null,23,8,false,1.0,0.0,"",0],["ESP_wheelPulseCountReR",null,31,8,false,1.0,0.0,"",0]]}],"513":[{"n":"SDM1","s":[["SDM_bcklPassStatus",null,3,2,false,1.0,0.0,"",0],["SDM_bcklDrivStatus",null,5,2,false,1.0,0.0,"",0]]}],"532":[{"n":"EPB_epasControl","s":[["EPB_epasEACAllow",null,2,3,false,1.0,0.0,"",0]]}],"792":[{"n":"GTW_carState","s":[["YEAR",null,0,7,false,1.0,2000.0,"Year",1],["CERRD",null,7,1,false,1.0,0.0,"",1],["MONTH",null,8,4,false,1.0,0.0,"Month",1],["DOOR_STATE_FL",null,12,2,false,1.0,0.0,"",1],["DOOR_STATE_FR",null,14,2,false,1.0,0.0,"",1],["SECOND",null,16,6,false,1.0,0.0,"s",1],["DOOR_STATE_RL",null,22,2,false,1.0,0.0,"",1],["Hour",null,24,5,false,1.0,0.0,"h",1],["DOOR_STATE_RR",null,29,2,false,1.0,0.0,"",1],["DAY",null,32,5,false,1.0,0.0,"",1],["MINUTE",null,40,6,false,1.0,0.0,"min",1],["BOOT_STATE",null,46,2,false,1.0,0.0,"",1],["GTW_updateInProgress",null,48,2,false,1.0,0.0,"",1],["DOOR_STATE_FrontTrunk",null,50,2,false,1.0,0.0,"",1],["MCU_factoryMode",null,52,1,false,1.0,0.0,"",1],["MCU_transportModeOn",null,53,1,false,1.0,0.0,"",0],["BC_headLightLStatus",null,55,2,false,1.0,0.0,"",0],["BC_headLightRStatus",null,57,2,false,1.0,0.0,"",0],["BC_indicatorLStatus",null,59,2,false,1.0,0.0,"",0],["BC_indicatorRStatus",null,61,2,false,1.0,0.0,"",0]]}],"872":[{"n":"DI_state","s":[["DI_systemState",null,0,3,false,1.0,0.0,"",1],["DI_vehicleHoldState",null,3,3,false,1.0,0.0,"",1],["DI_proximity",null,6,1,false,1.0,0.0,"",1],["DI_driveReady",null,7,1,false,1.0,0.0,"",1],["DI_regenLight",null,8,1,false,1.0,0.0,"",1],["DI_state",null,9,3,false,1.0,0.0,"",1],["DI_cruiseState",null,12,4,false,1.0,0.0,"",1],["DI_analogSpeed",null,16,12,false,0.1,0.0,"speed",1],["DI_immobilizerState",null,28,3,false,1.0,0.0,"",1],["DI_speedUnits",null,31,1,false,1.0,0.0,"",1],["DI_cruiseSet",null,32,9,false,0.5,0.0,"speed",1],["DI_aebState",null,41,3,false,1.0,0.0,"",1],["DI_digitalSpeed",null,48,8,false,1.0,0.0,"",1]]}],"109":[{"n":"SBW_RQ_SCCM","s":[["StW_Sw_Stat3",null,0,3,false,1.0,0.0,"",1],["MsgTxmtId",null,6,2,false,1.0,0.0,"",1],["TSL_RND_Posn_StW",null,8,4,false,1.0,0.0,"",1],["TSL_P_Psd_StW",null,12,2,false,1.0,0.0,"",1]]}],"69":[{"n":"STW_ACTN_RQ","s":[["SpdCtrlLvr_Stat",null,0,6,false,1.0,0.0,"",1],["VSL_Enbl_Rq",null,6,1,false,1.0,0.0,"",1],["SpdCtrlLvrStat_Inv",null,7,1,false,1.0,0.0,"",1],["DTR_Dist_Rq",null,8,8,false,1.0,0.0,"",1],["TurnIndLvr_Stat",null,16,2,false,1.0,0.0,"",1],["HiBmLvr_Stat",null,18,2,false,1.0,0.0,"",1],["WprWashSw_Psd",null,20,2,false,1.0,0.0,"",1],["WprWash_R_Sw_Posn_V2",null,22,2,false,1.0,0.0,"",1],["StW_Lvr_Stat",null,24,3,false,1.0,0.0,"",1],["StW_Cond_Flt",null,27,1,false,1.0,0.0,"",1],["StW_Cond_Psd",null,28,2,false,1.0,0.0,"",1],["HrnSw_Psd",null,30,2,false,1.0,0.0,"",1],["StW_Sw00_Psd",null,32,1,false,1.0,0.0,"",1],["StW_Sw01_Psd",null,33,1,false,1.0,0.0,"",1],["StW_Sw02_Psd",null,34,1,false,1.0,0.0,"",1],["StW_Sw03_Psd",null,35,1,false,1.0,0.0,"",1],["StW_Sw04_Psd",null,36,1,false,1.0,0.0,"",1],["StW_Sw05_Psd",null,37,1,false,1.0,0.0,"",1],["StW_Sw06_Psd",null,38,1,false,1.0,0.0,"",1],["StW_Sw07_Psd",null,39,1,false,1.0,0.0,"",1],["StW_Sw08_Psd",null,40,1,false,1.0,0.0,"",1],["StW_Sw09_Psd",null,41,1,false,1.0,0.0,"",1],["StW_Sw10_Psd",null,42,1,false,1.0,0.0,"",1],["StW_Sw11_Psd",null,43,1,false,1.0,0.0,"",1],["StW_Sw12_Psd",null,44,1,false,1.0,0.0,"",1],["StW_Sw13_Psd",null,45,1,false,1.0,0.0,"",1],["StW_Sw14_Psd",null,46,1,false,1.0,0.0,"",1],["StW_Sw15_Psd",null,47,1,false,1.0,0.0,"",1],["WprSw6Posn",null,48,3,false,1.0,0.0,"",1]]}],"643":[{"n":"BODY_R1","s":[["AirTemp_Insd",null,47,8,false,0.25,0.0,"C",0],["AirTemp_Outsd",null,63,8,false,0.5,-40.0,"C",0],["Bckl_Sw_RL_Stat_SAM_R",null,49,2,false,1.0,0.0,"",0],["Bckl_Sw_RM_Stat_SAM_R",null,53,2,false,1.0,0.0,"",0],["Bckl_Sw_RR_Stat_SAM_R",null,51,2,false,1.0,0.0,"",0],["DL_RLtch_Stat",null,9,2,false,1.0,0.0,"",0],["DrRLtch_FL_Stat",null,1,2,false,1.0,0.0,"",0],["DrRLtch_FR_Stat",null,3,2,false,1.0,0.0,"",0],["DrRLtch_RL_Stat",null,5,2,false,1.0,0.0,"",0],["DrRLtch_RR_Stat",null,7,2,false,1.0,0.0,"",0],["EngHd_Stat",null,11,2,false,1.0,0.0,"",0],["LoBm_On_Rq",null,32,1,false,1.0,0.0,"",0],["HiBm_On",null,33,1,false,1.0,0.0,"",0],["Hrn_On",null,26,1,false,1.0,0.0,"",0],["IrLmp_D_Lt_Flt",null,34,1,false,1.0,0.0,"",0],["IrLmp_P_Rt_Flt",null,35,1,false,1.0,0.0,"",0],["LgtSens_Twlgt",null,18,3,false,1.0,0.0,"Steps",0],["LgtSens_SNA",null,19,1,false,1.0,0.0,"",0],["LgtSens_Tunnel",null,20,1,false,1.0,0.0,"",0],["LgtSens_Flt",null,21,1,false,1.0,0.0,"",0],["LgtSens_Night",null,22,1,false,1.0,0.0,"",0],["ADL_LoBm_On_Rq",null,23,1,false,1.0,0.0,"",0],["LoBm_D_Lt_Flt",null,36,1,false,1.0,0.0,"",0],["LoBm_P_Rt_Flt",null,37,1,false,1.0,0.0,"",0],["MPkBrk_Stat",null,28,1,false,1.0,0.0,"",0],["RevGr_Engg",null,39,2,false,1.0,0.0,"",0],["StW_Cond_Stat",null,55,2,false,1.0,0.0,"",0],["Term54_Actv",null,27,1,false,1.0,0.0,"",0],["Trlr_Stat",null,25,2,false,1.0,0.0,"",0],["VTA_Alm_Actv",null,13,1,false,1.0,0.0,"",0],["WprOutsdPkPosn",null,29,1,false,1.0,0.0,"",0]]}],"760":[{"n":"UI_gpsVehicleSpeed","s":[["UI_gpsHDOP",null,0,8,false,0.1,0.0,"1",1],["UI_gpsVehicleHeading",null,8,16,false,0.0078125,0.0,"deg",1],["UI_gpsVehicleSpeed",null,24,16,false,0.00390625,0.0,"km/hr",1],["UI_userSpeedOffset",null,40,6,false,1.0,-30.0,"kph/mph",1],["UI_mapSpeedLimitUnits",null,46,1,false,1.0,0.0,"",1],["UI_userSpeedOffsetUnits",null,47,1,false,1.0,0.0,"",1],["UI_mppSpeedLimit",null,48,5,false,5.0,0.0,"kph/mph",1],["UI_gpsNmeaMIA",null,53,1,false,1.0,0.0,"",1]]}],"536":[{"n":"MCU_chassisControl","s":[["MCU_dasDebugEnable",null,0,1,false,1.0,0.0,"",1],["MCU_parkBrakeRequest",null,1,2,false,1.0,0.0,"",1],["MCU_trailerModeCH",null,3,1,false,1.0,0.0,"",1],["MCU_fcwSensitivity",null,4,2,false,1.0,0.0,"",1],["MCU_fcwEnable",null,6,2,false,1.0,0.0,"",1],["MCU_latControlEnable",null,8,2,false,1.0,0.0,"",1],["MCU_accOvertakeEnable",null,10,2,false,1.0,0.0,"",1],["MCU_ldwEnable",null,12,2,false,1.0,0.0,"",1],["MCU_aebEnable",null,14,2,false,1.0,0.0,"",1],["MCU_bsdEnable",null,16,2,false,1.0,0.0,"",1],["MCU_ahlbEnable",null,18,2,false,1.0,0.0,"",1],["MCU_parkSetting",null,20,2,false,1.0,0.0,"",1],["MCU_pedalSafetyEnable",null,22,2,false,1.0,0.0,"",1],["MCU_frontDefrostReq_das",null,24,2,false,1.0,0.0,"",1],["MCU_autoParkRequest",null,26,4,false,1.0,0.0,"",1],["MCU_redLightStopSignEnable",null,30,2,false,1.0,0.0,"",1],["MCU_enableCreepTorqueCH",null,32,1,false,1.0,0.0,"",1],["MCU_narrowGarages",null,33,1,false,1.0,0.0,"",1],["MCU_rebootAutopilot",null,34,1,false,1.0,0.0,"",1],["MCU_enableAutowipers",null,35,1,false,1.0,0.0,"",1],["MCU_overPaintedUSS",null,38,2,false,1.0,0.0,"",1],["MCU_selfParkTune",null,40,4,false,1.0,0.0,"",1],["MCU_towModeEnable",null,44,2,false,1.0,0.0,"",1],["MCU_zeroSpeedConfirmed",null,46,2,false,1.0,0.0,"",1],["MCU_aesEnable",null,48,2,false,1.0,0.0,"",1],["MCU_autoLaneChangeEnable",null,50,2,false,1.0,0.0,"",1]]}],"904":[{"n":"MCU_clusterBacklightRequest","s":[["MCU_clusterBacklightOn",null,7,1,false,1.0,0.0,"",1],["MCU_clusterBrightnessLevel",null,8,8,false,0.5,0.0,"%",1],["MCU_clusterReadyForDrive",null,6,1,false,1.0,0.0,"",1],["MCU_clusterReadyForPowerOff",null,5,1,false,1.0,0.0,"",1]]}],"984":[{"n":"MCU_locationStatus","s":[["MCU_gpsAccuracy",null,57,7,false,0.2,0.0,"m",1],["MCU_latitude",null,0,28,true,1e-06,0.0,"deg",1],["MCU_longitude",null,28,29,true,1e-06,0.0,"deg",1]]}],"104":[{"n":"MCU_locationStatus2","s":[["MCU_elevation",null,0,32,true,0.1,0.0,"m",1],["MCU_navigonExpectedSpeed",null,32,7,false,1.0,0.0,"mph",1]]}],"840":[{"n":"GTW_status","s":[["GTW_accGoingDown",null,6,1,false,1.0,0.0,"",0],["GTW_accRailReq",null,8,1,false,1.0,0.0,"",0],["GTW_brakePressed",null,1,1,false,1.0,0.0,"",0],["GTW_driveGoingDown",null,7,1,false,1.0,0.0,"",0],["GTW_driveRailReq",null,0,1,false,1.0,0.0,"",0],["GTW_driverIsLeaving",null,5,1,false,1.0,0.0,"",0],["GTW_driverPresent",null,2,1,false,1.0,0.0,"",0],["GTW_hvacGoingDown",null,11,1,false,1.0,0.0,"",0],["GTW_hvacRailReq",null,9,1,false,1.0,0.0,"",0],["GTW_icPowerOff",null,4,1,false,1.0,0.0,"",0],["GTW_notEnough12VForDrive",null,3,1,false,1.0,0.0,"",0],["GTW_preconditionRequest",null,10,1,false,1.0,0.0,"",0]]}],"920":[{"n":"GTW_carConfig","s":[["GTW_performanceConfig",null,2,3,false,1.0,0.0,"",0],["GTW_fourWheelDrive",null,4,2,false,1.0,0.0,"",0],["GTW_unknown1",null,5,1,false,1.0,0.0,"",0],["GTW_dasHw",null,7,2,false,1.0,0.0,"",0],["GTW_parkAssistInstalled",null,9,2,false,1.0,0.0,"",0],["GTW_forwardRadarHw",null,11,2,false,1.0,0.0,"",0],["GTW_airSuspensionInstalled",null,14,3,false,1.0,0.0,"",0],["GTW_unknown2",null,15,1,false,1.0,0.0,"",0],["GTW_country",null,23,16,false,1.0,0.0,"",0],["GTW_parkSensorGeometryType",null,33,2,false,1.0,0.0,"",0],["GTW_rhd",null,34,1,false,1.0,0.0,"",0],["GTW_bodyControlsType",null,35,1,false,1.0,0.0,"",0],["GTW_radarPosition",null,39,4,false,1.0,0.0,"",0],["GTW_rearCornerRadarHw",null,41,2,false,1.0,0.0,"",0],["GTW_frontCornerRadarHw",null,43,2,false,1.0,0.0,"",0],["GTW_epasType",null,45,2,false,1.0,0.0,"",0],["GTW_chassisType",null,47,2,false,1.0,0.0,"",0],["GTW_wheelType",null,52,5,false,1.0,0.0,"",0],["GTW_rearSeatControllerMask",null,55,3,false,1.0,0.0,"",0],["GTW_euVehicle",null,56,1,false,1.0,0.0,"",0],["GTW_foldingMirrorsInstalled",null,57,1,false,1.0,0.0,"",0],["GTW_brakeHwType",null,59,2,false,1.0,0.0,"",0],["GTW_autopilot",null,61,2,false,1.0,0.0,"",0],["GTW_unknown3",null,63,2,false,1.0,0.0,"",0]]}],"1006":[{"n":"UI_autopilotControl","s":[["UI_autopilotControlIndex",-1,0,3,false,1.0,0.0,"",1],["UI_hovEnabled",0,3,1,false,1.0,0.0,"",1],["UI_donDisableAutoWiperDuration",0,4,3,false,1.0,0.0,"",1],["UI_donDisableOnAutoWiperSpeed",0,7,4,false,1.0,0.0,"",1],["UI_blindspotMinSpeed",0,11,4,false,1.0,0.0,"",1],["UI_blindspotDistance",0,15,3,false,1.0,0.0,"",1],["UI_blindspotTTC",0,18,3,false,1.0,0.0,"",1],["UI_donStopEndOfRampBuffer",0,21,3,false,1.0,0.0,"",1],["UI_donDisableCutin",0,24,1,false,1.0,0.0,"",1],["UI_donMinGoreWidthForAbortMap",0,25,4,false,1.0,0.0,"",1],["UI_donAlcProgGoreAbortThres",0,29,4,false,1.0,0.0,"",1],["UI_donMinGoreWidthForAbortNotMap",0,33,4,false,1.0,0.0,"",1],["UI_alcDisableUltrasonicCheck",0,37,1,false,1.0,0.0,"",1],["UI_alcUltrasonicDistance",0,38,4,false,1.0,0.0,"",1],["UI_alcUltrasonicWaitTime",0,42,3,false,1.0,0.0,"",1],["UI_alcEgoLeadingReactionAccel",0,48,2,false,1.0,0.0,"",1],["UI_alcMergIntervalRearDHyst",0,50,2,false,1.0,0.0,"",1],["UI_alcMergingIntervalHeadwayHyst",0,52,2,false,1.0,0.0,"",1],["UI_alcAssertivenessRate",0,54,2,false,1.0,0.0,"",1],["UI_alcViewRangeSensitivity",0,56,2,false,1.0,0.0,"",1],["UI_camBlockLaneCheckDisable",1,3,1,false,1.0,0.0,"",1],["UI_camBlockLaneCheckThreshold",1,4,6,false,0.01587,0.0,"%",1],["UI_camBlockBlurDisable",1,10,1,false,1.0,0.0,"",1],["UI_camBlockBlurThreshold",1,11,6,false,0.01587,0.0,"%",1]]}],"728":[{"n":"UI_csaOfframpCurvature","s":[["UI_csaOfframpCurvC2",null,0,16,true,1e-06,0.0,"1/m",1],["UI_csaOfframpCurvC3",null,16,16,true,4e-09,0.0,"1/m2",1],["UI_csaOfframpCurvRange",null,32,8,false,2.0,0.0,"m",1],["UI_csaOfframpCurvUsingTspline",null,48,1,false,1.0,0.0,"",1],["UI_csaOfframpCurvReserved",null,49,7,false,1.0,0.0,"",1]]}],"744":[{"n":"UI_csaRoadCurvature","s":[["UI_csaRoadCurvC2",null,0,16,true,1e-06,0.0,"1/m",1],["UI_csaRoadCurvC3",null,16,16,true,4e-09,0.0,"1/m2",1],["UI_csaRoadCurvRange",null,32,8,false,2.0,0.0,"m",1],["UI_csaRoadCurvUsingTspline",null,48,1,false,1.0,0.0,"",1],["UI_csaRoadCurvReserved",null,49,7,false,1.0,0.0,"",1]]}],"1080":[{"n":"UI_driverAssistAnonDebugParams","s":[["UI_anonDebugParam1",null,0,7,false,1.0,0.0,"",1],["UI_anonDebugFlag1",null,7,1,false,1.0,0.0,"",1],["UI_anonDebugParam2",null,8,7,false,1.0,0.0,"",1],["UI_anonDebugFlag2",null,15,1,false,1.0,0.0,"",1],["UI_anonDebugParam3",null,16,7,false,1.0,0.0,"",1],["UI_anonDebugFlag3",null,23,1,false,1.0,0.0,"",1],["UI_anonDebugParam4",null,24,7,false,1.0,0.0,"",1],["UI_anonDebugFlag4",null,31,1,false,1.0,0.0,"",1],["UI_anonDebugParam5",null,32,7,false,1.0,0.0,"",1],["UI_anonDebugParam6",null,40,7,false,1.0,0.0,"",1],["UI_anonDebugParam7",null,48,7,false,1.0,0.0,"",1],["UI_visionSpeedSlider",null,56,7,false,1.0,0.0,"",1]]}],"1000":[{"n":"UI_driverAssistControl","s":[["UI_autopilotControlRequest",null,0,1,false,1.0,0.0,"",1],["UI_ulcStalkConfirm",null,1,1,false,1.0,0.0,"",1],["UI_summonHeartbeat",null,2,2,false,1.0,0.0,"",1],["UI_curvSpeedAdaptDisable",null,4,1,false,1.0,0.0,"",1],["UI_dasDeveloper",null,5,1,false,1.0,0.0,"",1],["UI_enableVinAssociation",null,6,1,false,1.0,0.0,"",1],["UI_lssLkaEnabled",null,7,1,false,1.0,0.0,"",1],["UI_lssLdwEnabled",null,8,1,false,1.0,0.0,"",1],["UI_autoSummonEnable",null,10,1,false,1.0,0.0,"",1],["UI_exceptionListEnable",null,11,1,false,1.0,0.0,"",1],["UI_roadCheckDisable",null,12,1,false,1.0,0.0,"",1],["UI_driveOnMapsEnable",null,13,1,false,1.0,0.0,"",1],["UI_handsOnRequirementDisable",null,14,1,false,1.0,0.0,"",1],["UI_forksEnable",null,15,1,false,1.0,0.0,"",1],["UI_fuseLanesDisable",null,16,1,false,1.0,0.0,"",1],["UI_fuseHPPDisable",null,17,1,false,1.0,0.0,"",1],["UI_fuseVehiclesDisable",null,18,1,false,1.0,0.0,"",1],["UI_enableNextGenACC",null,19,1,false,1.0,0.0,"",1],["UI_visionSpeedType",null,20,2,false,1.0,0.0,"",1],["UI_curvatureDatabaseOnly",null,22,1,false,1.0,0.0,"",1],["UI_lssElkEnabled",null,23,1,false,1.0,0.0,"",1],["UI_summonExitType",null,24,2,false,1.0,0.0,"",1],["UI_summonEntryType",null,26,2,false,1.0,0.0,"",1],["UI_selfParkRequest",null,28,4,false,1.0,0.0,"",1],["UI_summonReverseDist",null,32,6,false,1.0,0.0,"",1],["UI_undertakeAssistEnable",null,38,1,false,1.0,0.0,"",1],["UI_adaptiveSetSpeedEnable",null,39,1,false,1.0,0.0,"",1],["UI_drivingSide",null,40,2,false,1.0,0.0,"",1],["UI_enableClipTelemetry",null,42,1,false,1.0,0.0,"",1],["UI_enableTripTelemetry",null,43,1,false,1.0,0.0,"",1],["UI_enableRoadSegmentTelemetry",null,44,1,false,1.0,0.0,"",1],["UI_followNavRouteEnable",null,46,1,false,1.0,0.0,"",1],["UI_ulcSpeedConfig",null,48,2,false,1.0,0.0,"",1],["UI_ulcBlindSpotConfig",null,50,2,false,1.0,0.0,"",1],["UI_autopilotAlwaysOn",null,52,1,false,1.0,0.0,"",1],["UI_accFromZero",null,53,1,false,1.0,0.0,"",1],["UI_alcOffHighwayEnable",null,54,1,false,1.0,0.0,"",1],["UI_validationLoop",null,55,1,false,1.0,0.0,"",1],["UI_ulcOffHighway",null,56,1,false,1.0,0.0,"",1],["UI_enableNavRouteCSA",null,57,1,false,1.0,0.0,"",1],["UI_enableCutinExperiments",null,58,1,false,1.0,0.0,"",1],["UI_source3D",null,60,3,false,1.0,0.0,"",1],["UI_enableVisionOnlyStops",null,63,1,false,1.0,0.0,"",1]]}],"968":[{"n":"UI_driverAssistMapData","s":[["UI_mapSpeedLimitDependency",null,0,3,false,1.0,0.0,"",1],["UI_roadClass",null,3,3,false,1.0,0.0,"",1],["UI_inSuperchargerGeofence",null,6,1,false,1.0,0.0,"",1],["UI_mapSpeedUnits",null,7,1,false,1.0,0.0,"",1],["UI_mapSpeedLimit",null,8,5,false,1.0,0.0,"",1],["UI_mapSpeedLimitType",null,13,3,false,1.0,0.0,"",1],["UI_countryCode",null,16,10,false,1.0,0.0,"",1],["UI_streetCount",null,26,2,false,1.0,0.0,"",1],["UI_gpsRoadMatch",null,28,1,false,1.0,0.0,"",1],["UI_navRouteActive",null,29,1,false,1.0,0.0,"",1],["UI_parallelAutoparkEnabled",null,30,1,false,1.0,0.0,"",1],["UI_perpendicularAutoparkEnabled",null,31,1,false,1.0,0.0,"",1],["UI_nextBranchDist",null,32,5,false,10.0,0.0,"m",1],["UI_controlledAccess",null,37,1,false,1.0,0.0,"",1],["UI_nextBranchLeftOffRamp",null,38,1,false,1.0,0.0,"",1],["UI_nextBranchRightOffRamp",null,39,1,false,1.0,0.0,"",1],["UI_rejectLeftLane",null,40,1,false,1.0,0.0,"",1],["UI_rejectRightLane",null,41,1,false,1.0,0.0,"",1],["UI_rejectHPP",null,42,1,false,1.0,0.0,"",1],["UI_rejectNav",null,43,1,false,1.0,0.0,"",1],["UI_rejectLeftFreeSpace",null,44,1,false,1.0,0.0,"",1],["UI_rejectRightFreeSpace",null,45,1,false,1.0,0.0,"",1],["UI_rejectAutosteer",null,46,1,false,1.0,0.0,"",1],["UI_rejectHandsOn",null,47,1,false,1.0,0.0,"",1],["UI_acceptBottsDots",null,48,1,false,1.0,0.0,"",1],["UI_autosteerRestricted",null,49,1,false,1.0,0.0,"",1],["UI_pmmEnabled",null,50,1,false,1.0,0.0,"",1],["UI_scaEnabled",null,51,1,false,1.0,0.0,"",1]]}],"568":[{"n":"UI_driverAssistRoadSign","s":[["UI_roadSign",-1,0,8,false,1.0,0.0,"",1],["UI_splineLocConfidence",null,40,7,false,1.0,0.0,"",1],["UI_splineID",null,48,4,false,1.0,0.0,"",1],["UI_dummyData",0,8,1,false,1.0,0.0,"",1],["UI_stopSignStopLineDist",1,8,10,false,0.25,-8.0,"m",1],["UI_stopSignStopLineConf",1,18,7,false,1.0,0.0,"",1],["UI_trafficLightStopLineDist",2,8,10,false,0.25,-8.0,"m",1],["UI_trafficLightStopLineConf",2,18,7,false,1.0,0.0,"",1],["UI_baseMapSpeedLimitMPS",3,8,8,false,0.25,0.0,"m/s",1],["UI_bottomQrtlFleetSpeedMPS",3,16,8,false,0.25,0.0,"m/s",1],["UI_topQrtlFleetSpeedMPS",3,24,8,false,0.25,0.0,"m/s",1],["UI_meanFleetSplineSpeedMPS",4,8,8,false,0.25,0.0,"m/s",1],["UI_medianFleetSpeedMPS",4,16,8,false,0.25,0.0,"m/s",1],["UI_meanFleetSplineAccelMPS2",4,24,8,false,0.05,-6.35,"m/s^2",1],["UI_rampType",4,32,3,false,1.0,0.0,"",1],["UI_currSplineIdFull",5,8,32,false,1.0,0.0,"",1]]}],"696":[{"n":"UI_radarMapData","s":[["UI_radarTargetDx",null,0,8,false,1.0,-95.0,"m",1],["UI_radarTargetDxEnd",null,8,8,false,1.0,0.0,"m",1],["UI_radarTargetTrustMap",null,16,1,false,1.0,0.0,"",1],["UI_radarEnableBraking",null,17,1,false,1.0,0.0,"",1]]}],"712":[{"n":"UI_roadCurvature","s":[["UI_roadCurvC0",null,0,11,true,0.02,0.0,"m",1],["UI_roadCurvC1",null,11,10,true,0.00075,0.0,"1",1],["UI_roadCurvC2",null,21,14,true,7.5e-06,0.0,"1/m",1],["UI_roadCurvC3",null,35,13,true,3e-08,0.0,"1/m2",1],["UI_roadCurvRange",null,48,6,false,4.0,0.0,"m",1],["UI_roadCurvHealth",null,54,2,false,1.0,0.0,"",1]]}],"582":[{"n":"UI_solarData","s":[["UI_solarAzimuthAngle",null,0,16,true,1.0,0.0,"deg",1],["UI_solarAzimuthAngleCarRef",null,16,9,true,1.0,0.0,"deg",1],["UI_isSunUp",null,25,1,false,1.0,0.0,"",1],["UI_solarElevationAngle",null,32,8,true,1.0,0.0,"deg",1]]}],"824":[{"n":"UI_status","s":[["UI_touchActive",null,0,1,false,1.0,0.0,"",1],["UI_audioActive",null,1,1,false,1.0,0.0,"",1],["UI_bluetoothActive",null,2,1,false,1.0,0.0,"",1],["UI_cellActive",null,3,1,false,1.0,0.0,"",1],["UI_displayReady",null,4,1,false,1.0,0.0,"",1],["UI_gpsActive",null,5,1,false,1.0,0.0,"",1],["UI_wifiConnected",null,6,1,false,1.0,0.0,"",1],["UI_systemActive",null,7,1,false,1.0,0.0,"",1],["UI_xmActive",null,8,1,false,1.0,0.0,"",1],["UI_displayOn",null,9,1,false,1.0,0.0,"",1],["UI_readyForDrive",null,10,1,false,1.0,0.0,"",1],["UI_cellConnected",null,11,1,false,1.0,0.0,"",1],["UI_vpnActive",null,12,1,false,1.0,0.0,"",1],["UI_wifiActive",null,13,1,false,1.0,0.0,"",1],["UI_cameraActive",null,14,1,false,1.0,0.0,"",1],["UI_usbActive",null,15,1,false,1.0,0.0,"",1],["UI_screenshotActive",null,16,1,false,1.0,0.0,"",1],["UI_monitorModemPower",null,17,1,false,1.0,0.0,"",1],["UI_factoryReset",null,18,2,false,1.0,0.0,"",1],["UI_cellNetworkTechnology",null,20,4,false,1.0,0.0,"",1],["UI_tegraCoreTemperature",null,24,8,false,1.0,-64.0,"deg C",1],["UI_tegraAmbientTemperature",null,32,8,false,1.0,-64.0,"deg C",1],["UI_googleWifiUsages",null,40,8,false,1.0,0.0,"",1],["UI_autopilotTrial",null,48,2,false,1.0,0.0,"",1],["UI_cellSignalBars",null,50,3,false,1.0,0.0,"",1],["UI_hardwareType",null,53,2,false,1.0,0.0,"",1],["UI_developmentCar",null,55,1,false,1.0,0.0,"",1],["UI_cellReceiverPower",null,56,8,false,1.0,-128.0,"dB",1]]}],"1064":[{"n":"UI_telemetryControl","s":[["UI_TCR_enable",null,0,1,false,1.0,0.0,"",1],["UI_TCR_moveStateStanding",null,1,1,false,1.0,0.0,"",1],["UI_TCR_moveStateStopped",null,2,1,false,1.0,0.0,"",1],["UI_TCR_moveStateMoving",null,3,1,false,1.0,0.0,"",1],["UI_TCR_moveStateIndeterm",null,4,1,false,1.0,0.0,"",1],["UI_TCR_classConstElem",null,5,1,false,1.0,0.0,"",1],["UI_TCR_classMovingPed",null,6,1,false,1.0,0.0,"",1],["UI_TCR_classMovingTwoWheel",null,7,1,false,1.0,0.0,"",1],["UI_TCR_classMovingFourWheel",null,8,1,false,1.0,0.0,"",1],["UI_TCR_classUnknown",null,9,1,false,1.0,0.0,"",1],["UI_TCR_downSampleFactor",null,16,4,false,1.0,0.0,"",1],["UI_TCR_wExist",null,24,5,false,1.0,0.0,"",1],["UI_TCR_vehSpeed",null,32,8,false,1.0,0.0,"",1],["UI_TCR_minRCS",null,40,8,false,0.25,-14.0,"dB",1],["UI_TCR_maxDy",null,48,5,false,0.5,0.0,"m",1],["UI_TCR_maxObjects",null,56,5,false,1.0,0.0,"",1],["UI_TCR_maxRoadClass",null,61,3,false,1.0,0.0,"",1]]}],"522":[{"n":"BrakeMessage","s":[["driverBrakeStatus",null,2,2,false,1.0,0.0,"",1]]}],"921":[{"n":"AutopilotStatus","s":[["autopilotStatus",null,0,4,false,1.0,0.0,"",1],["DAS_blindSpotRearLeft",null,4,2,false,1.0,0.0,"",1],["DAS_blindSpotRearRight",null,6,2,false,1.0,0.0,"",1],["DAS_fusedSpeedLimit",null,8,5,false,5.0,0.0,"kph/mph",1],["DAS_suppressSpeedWarning",null,13,1,false,1.0,0.0,"",1],["DAS_summonObstacle",null,14,1,false,1.0,0.0,"",1],["DAS_summonClearedGate",null,15,1,false,1.0,0.0,"",1],["DAS_visionOnlySpeedLimit",null,16,5,false,5.0,0.0,"kph/mph",1],["DAS_heaterState",null,21,1,false,1.0,0.0,"",1],["DAS_forwardCollisionWarning",null,22,2,false,1.0,0.0,"",1],["DAS_autoparkReady",null,24,1,false,1.0,0.0,"",1],["DAS_autoParked",null,25,1,false,1.0,0.0,"",1],["DAS_autoparkWaitingForBrake",null,26,1,false,1.0,0.0,"",1],["DAS_summonFwdLeashReached",null,28,1,false,1.0,0.0,"",1],["DAS_summonRvsLeashReached",null,29,1,false,1.0,0.0,"",1],["DAS_sideCollisionAvoid",null,30,2,false,1.0,0.0,"",1],["DAS_sideCollisionWarning",null,32,2,false,1.0,0.0,"",1],["DAS_sideCollisionInhibit",null,34,1,false,1.0,0.0,"",1],["DAS_csaState",null,35,2,false,1.0,0.0,"",1],["DAS_laneDepartureWarning",null,37,3,false,1.0,0.0,"",1],["DAS_fleetSpeedState",null,40,2,false,1.0,0.0,"",1],["DAS_autopilotHandsOnState",null,42,4,false,1.0,0.0,"",1],["DAS_autoLaneChangeState",null,46,5,false,1.0,0.0,"",1],["DAS_summonAvailable",null,51,1,false,1.0,0.0,"",1]]},{"n":"M3_DAS_status","s":[["DAS_autoLaneChangeState",null,46,5,false,1.0,0.0,"",1],["DAS_autoParked",null,25,1,false,1.0,0.0,"",1],["DAS_autoparkReady",null,24,1,false,1.0,0.0,"",1],["DAS_autoparkWaitingForBrake",null,26,1,false,1.0,0.0,"",1],["DAS_autopilotHandsOnState",null,42,4,false,1.0,0.0,"",1],["DAS_autopilotState",null,0,4,false,1.0,0.0,"",1],["DAS_blindSpotRearLeft",null,4,2,false,1.0,0.0,"",1],["DAS_blindSpotRearRight",null,6,2,false,1.0,0.0,"",1],["DAS_fleetSpeedState",null,40,2,false,1.0,0.0,"",1],["DAS_forwardCollisionWarning",null,22,2,false,1.0,0.0,"",1],["DAS_fusedSpeedLimit",null,8,5,false,5.0,0.0,"kph/mph",1],["DAS_heaterState",null,21,1,false,1.0,0.0,"",1],["DAS_laneDepartureWarning",null,37,3,false,1.0,0.0,"",1],["DAS_lssState",null,29,3,false,1.0,0.0,"",1],["DAS_sideCollisionAvoid",null,32,2,false,1.0,0.0,"",1],["DAS_sideCollisionInhibit",null,36,1,false,1.0,0.0,"",1],["DAS_sideCollisionWarning",null,34,2,false,1.0,0.0,"",1],["DAS_summonAvailable",null,51,1,false,1.0,0.0,"",1],["DAS_summonClearedGate",null,15,1,false,1.0,0.0,"",1],["DAS_summonFwdLeashReached",null,27,1,false,1.0,0.0,"",1],["DAS_summonObstacle",null,14,1,false,1.0,0.0,"",1],["DAS_summonRvsLeashReached",null,28,1,false,1.0,0.0,"",1],["DAS_suppressSpeedWarning",null,13,1,false,1.0,0.0,"",1],["DAS_visionOnlySpeedLimit",null,16,5,false,5.0,0.0,"kph/mph",1]]}],"905":[{"n":"DAS_status2","s":[["DAS_accSpeedLimit",null,0,10,false,0.2,0.0,"mph",1],["DAS_pmmObstacleSeverity",null,10,3,false,1.0,0.0,"",1],["DAS_pmmLoggingRequest",null,13,1,false,1.0,0.0,"",1],["DAS_activationFailureStatus",null,14,2,false,1.0,0.0,"",1],["DAS_pmmUltrasonicsFaultReason",null,16,3,false,1.0,0.0,"",1],["DAS_pmmRadarFaultReason",null,19,2,false,1.0,0.0,"",1],["DAS_pmmSysFaultReason",null,21,3,false,1.0,0.0,"",1],["DAS_pmmCameraFaultReason",null,24,2,false,1.0,0.0,"",1],["DAS_ACC_report",null,26,5,false,1.0,0.0,"",1],["DAS_lssState",null,31,3,false,1.0,0.0,"",1],["DAS_radarTelemetry",null,34,2,false,1.0,0.0,"",1],["DAS_robState",null,36,2,false,1.0,0.0,"",1],["DAS_driverInteractionLevel",null,38,2,false,1.0,0.0,"",1],["DAS_ppOffsetDesiredRamp",null,40,8,false,0.01,-1.28,"m",1],["DAS_longCollisionWarning",null,48,4,false,1.0,0.0,"",1]]}],"1001":[{"n":"DAS_bodyControls","s":[["DAS_headlightRequest",null,0,2,false,1.0,0.0,"",1],["DAS_hazardLightRequest",null,2,2,false,1.0,0.0,"",1],["DAS_wiperSpeed",null,4,4,false,1.0,0.0,"",1],["DAS_turnIndicatorRequest",null,8,2,false,1.0,0.0,"",1],["DAS_highLowBeamDecision",null,10,2,false,1.0,0.0,"",1],["DAS_highLowBeamOffReason",null,12,3,false,1.0,0.0,"",1],["DAS_turnIndicatorRequestReason",null,16,4,false,1.0,0.0,"",1]]}],"780":[{"n":"DriverSeat","s":[["occupancyStatus",null,16,3,false,1.0,0.0,"",1],["buckleStatus",null,19,2,false,1.0,0.0,"",1]]}],"262":[{"n":"PT_DI_torque1","s":[["DI_torqueDriver",null,0,13,true,0.25,0.0,"Nm",1],["DI_torqueMotor",null,16,13,true,0.25,0.0,"Nm",1],["DI_soptState",null,29,3,false,1.0,0.0,"",1],["DI_motorRPM",null,32,16,true,1.0,0.0,"RPM",1],["DI_pedalPos",null,48,8,false,0.4,0.0,"%",1]]}],"278":[{"n":"PT_DI_torque2","s":[["DI_torqueEstimate",null,0,12,true,0.5,0.0,"Nm",1],["DI_gear",null,12,3,false,1.0,0.0,"",1],["DI_brakePedal",null,15,1,false,1.0,0.0,"",1],["DI_vehicleSpeed",null,16,12,false,0.05,-25.0,"MPH",1],["DI_gearRequest",null,28,3,false,1.0,0.0,"",1],["DI_torqueInterfaceFailure",null,31,1,false,1.0,0.0,"",1],["DI_brakePedalState",null,36,2,false,1.0,0.0,"",1],["DI_epbParkRequest",null,38,1,false,1.0,0.0,"",1],["DI_epbInterfaceReady",null,39,1,false,1.0,0.0,"",1]]}],"504":[{"n":"PT_BrakeMessage","s":[["driverBrakeStatus",null,2,2,false,1.0,0.0,"",1]]}],"598":[{"n":"PT_DI_state","s":[["DI_systemState",null,0,3,false,1.0,0.0,"",1],["DI_vehicleHoldState",null,3,3,false,1.0,0.0,"",1],["DI_proximity",null,6,1,false,1.0,0.0,"",1],["DI_driveReady",null,7,1,false,1.0,0.0,"",1],["DI_regenLight",null,8,1,false,1.0,0.0,"",1],["DI_state",null,9,3,false,1.0,0.0,"",1],["DI_cruiseState",null,12,4,false,1.0,0.0,"",1],["DI_analogSpeed",null,16,12,false,0.1,0.0,"speed",1],["DI_immobilizerState",null,28,3,false,1.0,0.0,"",1],["DI_speedUnits",null,31,1,false,1.0,0.0,"",1],["DI_cruiseSet",null,32,9,false,0.5,0.0,"speed",1],["DI_aebState",null,41,3,false,1.0,0.0,"",1],["DI_digitalSpeed",null,48,8,false,1.0,0.0,"",1]]}],"703":[{"n":"PT_DAS_control","s":[["DAS_setSpeed",null,0,12,false,0.1,0.0,"kph",1],["DAS_accState",null,12,4,false,1.0,0.0,"",1],["DAS_aebEvent",null,16,2,false,1.0,0.0,"",1],["DAS_jerkMin",null,18,9,false,0.03,-15.232,"m/s^3",1],["DAS_jerkMax",null,27,8,false,0.059,0.0,"m/s^3",1],["DAS_accelMin",null,35,9,false,0.04,-15.0,"m/s^2",1],["DAS_accelMax",null,44,9,false,0.04,-15.0,"m/s^2",1]]}],"777":[{"n":"M3_DAS_object","s":[["DAS_objectId",-1,0,3,false,1.0,0.0,"",1],["DAS_cutinVehDx",3,8,8,false,0.5,0.0,"m",1],["DAS_cutinVehDy",3,20,7,false,0.35,-22.05,"m",1],["DAS_cutinVehHeading",5,56,8,false,0.0245437,-3.14159,"rad",1],["DAS_cutinVehId",3,27,7,false,1.0,0.0,"",1],["DAS_cutinVehRelevantForControl",3,7,1,false,1.0,0.0,"",1],["DAS_cutinVehType",3,3,3,false,1.0,0.0,"",1],["DAS_cutinVehVxRel",3,16,4,false,4.0,-30.0,"m/s",1],["DAS_leadVeh2Dx",0,39,8,false,0.5,0.0,"m",1],["DAS_leadVeh2Dy",0,51,7,false,0.35,-22.05,"m",1],["DAS_leadVeh2Heading",5,16,8,false,0.0245437,-3.14159,"rad",1],["DAS_leadVeh2Id",0,58,6,false,1.0,0.0,"",1],["DAS_leadVeh2RelevantForControl",0,38,1,false,1.0,0.0,"",1],["DAS_leadVeh2Type",0,34,3,false,1.0,0.0,"",1],["DAS_leadVeh2VxRel",0,47,4,false,4.0,-30.0,"m/s",1],["DAS_leadVehDx",0,8,8,false,0.5,0.0,"m",1],["DAS_leadVehDy",0,20,7,false,0.35,-22.05,"m",1],["DAS_leadVehHeading",5,8,8,false,0.0245437,-3.14159,"rad",1],["DAS_leadVehId",0,27,7,false,1.0,0.0,"",1],["DAS_leadVehRelevantForControl",0,7,1,false,1.0,0.0,"",1],["DAS_leadVehType",0,3,3,false,1.0,0.0,"",1],["DAS_leadVehVxRel",0,16,4,false,4.0,-30.0,"m/s",1],["DAS_leftVeh2Dx",1,39,8,false,0.5,0.0,"m",1],["DAS_leftVeh2Dy",1,51,7,false,0.35,-22.05,"m",1],["DAS_leftVeh2Heading",5,32,8,false,0.0245437,-3.14159,"rad",1],["DAS_leftVeh2Id",1,58,6,false,1.0,0.0,"",1],["DAS_leftVeh2RelevantForControl",1,38,1,false,1.0,0.0,"",1],["DAS_leftVeh2Type",1,34,3,false,1.0,0.0,"",1],["DAS_leftVeh2VxRel",1,47,4,false,4.0,-30.0,"m/s",1],["DAS_leftVehDx",1,8,8,false,0.5,0.0,"m",1],["DAS_leftVehDy",1,20,7,false,0.35,-22.05,"m",1],["DAS_leftVehHeading",5,24,8,false,0.0245437,-3.14159,"rad",1],["DAS_leftVehId",1,27,7,false,1.0,0.0,"",1],["DAS_leftVehRelevantForControl",1,7,1,false,1.0,0.0,"",1],["DAS_leftVehType",1,3,3,false,1.0,0.0,"",1],["DAS_leftVehVxRel",1,16,4,false,4.0,-30.0,"m/s",1],["DAS_rightVeh2Dx",2,39,8,false,0.5,0.0,"m",1],["DAS_rightVeh2Dy",2,51,7,false,0.35,-22.05,"m",1],["DAS_rightVeh2Heading",5,48,8,false,0.0245437,-3.14159,"rad",1],["DAS_rightVeh2Id",2,58,6,false,1.0,0.0,"",1],["DAS_rightVeh2RelevantForControl",2,38,1,false,1.0,0.0,"",1],["DAS_rightVeh2Type",2,34,3,false,1.0,0.0,"",1],["DAS_rightVeh2VxRel",2,47,4,false,4.0,-30.0,"m/s",1],["DAS_rightVehDx",2,8,8,false,0.5,0.0,"m",1],["DAS_rightVehDy",2,20,7,false,0.35,-22.05,"m",1],["DAS_rightVehHeading",5,40,8,false,0.0245437,-3.14159,"rad",1],["DAS_rightVehId",2,27,7,false,1.0,0.0,"",1],["DAS_rightVehRelevantForControl",2,7,1,false,1.0,0.0,"",1],["DAS_rightVehType",2,3,3,false,1.0,0.0,"",1],["DAS_rightVehVxRel",2,16,4,false,4.0,-30.0,"m/s",1],["DAS_roadSignArrow",4,27,3,false,1.0,0.0,"",1],["DAS_roadSignColor",4,3,3,false,1.0,0.0,"",1],["DAS_roadSignControlActive",4,24,1,false,1.0,0.0,"",1],["DAS_roadSignId",4,6,8,false,1.0,0.0,"",1],["DAS_roadSignOrientation",4,30,2,false,1.0,0.0,"",1],["DAS_roadSignSource",4,25,2,false,1.0,0.0,"",1],["DAS_roadSignStopLineDist",4,14,10,false,0.2,-20.0,"m",1]]}],"1016":[{"n":"M3_UI_driverAssistControl","s":[["UI_accFollowDistanceSetting",null,45,3,false,1.0,0.0,"",1],["UI_adaptiveSetSpeedEnable",null,39,1,false,1.0,0.0,"",1],["UI_alcOffHighwayEnable",null,56,1,false,1.0,0.0,"",1],["UI_autoSummonEnable",null,10,1,false,1.0,0.0,"",1],["UI_autopilotControlRequest",null,0,1,false,1.0,0.0,"",1],["UI_coastToCoast",null,9,1,false,1.0,0.0,"",1],["UI_curvSpeedAdaptDisable",null,4,1,false,1.0,0.0,"",1],["UI_curvatureDatabaseOnly",null,22,1,false,1.0,0.0,"",1],["UI_dasDeveloper",null,5,1,false,1.0,0.0,"",1],["UI_driveOnMapsEnable",null,13,1,false,1.0,0.0,"",1],["UI_drivingSide",null,40,2,false,1.0,0.0,"",1],["UI_enableBrakeLightPulse",null,63,1,false,1.0,0.0,"",1],["UI_enableClipTelemetry",null,42,1,false,1.0,0.0,"",1],["UI_enableRoadSegmentTelemetry",null,44,1,false,1.0,0.0,"",1],["UI_enableTripTelemetry",null,43,1,false,1.0,0.0,"",1],["UI_enableVinAssociation",null,6,1,false,1.0,0.0,"",1],["UI_enableVisionOnlyStops",null,60,1,false,1.0,0.0,"",1],["UI_exceptionListEnable",null,11,1,false,1.0,0.0,"",1],["UI_followNavRouteEnable",null,49,1,false,1.0,0.0,"",1],["UI_fuseHPPDisable",null,17,1,false,1.0,0.0,"",1],["UI_fuseLanesDisable",null,16,1,false,1.0,0.0,"",1],["UI_fuseVehiclesDisable",null,18,1,false,1.0,0.0,"",1],["UI_handsOnRequirementDisable",null,14,1,false,1.0,0.0,"",1],["UI_hasDriveOnNav",null,48,1,false,1.0,0.0,"",1],["UI_lssElkEnabled",null,23,1,false,1.0,0.0,"",1],["UI_lssLdwEnabled",null,8,1,false,1.0,0.0,"",1],["UI_lssLkaEnabled",null,7,1,false,1.0,0.0,"",1],["UI_roadCheckDisable",null,12,1,false,1.0,0.0,"",1],["UI_selfParkRequest",null,28,4,false,1.0,0.0,"",1],["UI_smartSummonType",null,58,2,false,1.0,0.0,"",1],["UI_source3D",null,61,2,false,1.0,0.0,"",1],["UI_summonEntryType",null,26,2,false,1.0,0.0,"",1],["UI_summonExitType",null,24,2,false,1.0,0.0,"",1],["UI_summonHeartbeat",null,2,2,false,1.0,0.0,"",1],["UI_summonReverseDist",null,32,6,false,1.0,0.0,"",1],["UI_ulcBlindSpotConfig",null,52,2,false,1.0,0.0,"",1],["UI_ulcOffHighway",null,15,1,false,1.0,0.0,"",1],["UI_ulcSpeedConfig",null,50,2,false,1.0,0.0,"",1],["UI_ulcStalkConfirm",null,1,1,false,1.0,0.0,"",1],["UI_undertakeAssistEnable",null,38,1,false,1.0,0.0,"",1],["UI_validationLoop",null,57,1,false,1.0,0.0,"",1],["UI_visionSpeedType",null,20,2,false,1.0,0.0,"",1]]}],"1021":[{"n":"M3_UI_autopilotControl","s":[["UI_autopilotControlIndex",-1,0,3,false,1.0,0.0,"",1],["UI_apmv3Branch",1,40,3,false,1.0,0.0,"",1],["UI_applyEceR79",1,19,1,false,1.0,0.0,"",1],["UI_blindspotDistance",0,15,3,false,1.0,0.0,"",1],["UI_blindspotMinSpeed",0,11,4,false,1.0,0.0,"",1],["UI_blindspotTTC",0,18,3,false,1.0,0.0,"",1],["UI_disableBackup",1,30,1,false,1.0,0.0,"",1],["UI_disableFisheye",1,25,1,false,1.0,0.0,"",1],["UI_disableLeftPillar",1,26,1,false,1.0,0.0,"",1],["UI_disableLeftRepeater",1,28,1,false,1.0,0.0,"",1],["UI_disableMain",1,23,1,false,1.0,0.0,"",1],["UI_disableNarrow",1,24,1,false,1.0,0.0,"",1],["UI_disableRadar",1,31,1,false,1.0,0.0,"",1],["UI_disableRightPillar",1,27,1,false,1.0,0.0,"",1],["UI_disableRightRepeater",1,29,1,false,1.0,0.0,"",1],["UI_donAlcProgGoreAbortThres",0,29,4,false,1.0,0.0,"",1],["UI_donDisableAutoWiperDuration",0,4,3,false,1.0,0.0,"",1],["UI_donDisableCutin",0,24,1,false,1.0,0.0,"",1],["UI_donDisableOnAutoWiperSpeed",0,7,4,false,1.0,0.0,"",1],["UI_donMinGoreWidthForAbortMap",0,25,4,false,1.0,0.0,"",1],["UI_donMinGoreWidthForAbortNotMap",0,33,4,false,1.0,0.0,"",1],["UI_donStopEndOfRampBuffer",0,21,3,false,1.0,0.0,"",1],["UI_driverMonitorConfirmation",1,17,1,false,1.0,0.0,"",1],["UI_enableAutopilotStopWarning",1,44,1,false,1.0,0.0,"",1],["UI_enableCabinCamera",1,43,1,false,1.0,0.0,"",1],["UI_enableCabinCameraTelemetry",1,48,1,false,1.0,0.0,"",1],["UI_enableMapStops",1,20,1,false,1.0,0.0,"",1],["UI_factorySummonEnable",1,39,1,false,1.0,0.0,"",1],["UI_fsdStopsControlEnabled",0,38,1,false,1.0,0.0,"",1],["UI_fsdVisualizationEnabled",0,37,1,false,1.0,0.0,"",1],["UI_hardCoreSummon",1,47,1,false,1.0,0.0,"",1],["UI_homelinkNearby",0,45,1,false,1.0,0.0,"",1],["UI_hovEnabled",0,3,1,false,1.0,0.0,"",1],["UI_noStalkConfirmAlertChime",1,37,1,false,1.0,0.0,"",1],["UI_noStalkConfirmAlertHaptic",1,32,1,false,1.0,0.0,"",1],["UI_showLaneGraph",1,45,1,false,1.0,0.0,"",1],["UI_showTrackLabels",1,46,1,false,1.0,0.0,"",1],["UI_ulcSnooze",1,36,1,false,1.0,0.0,"",1]]}]}''').items()}   # message tables from commaai/opendbc (tesla_can, tesla_powertrain) and joshwardell/model3dbc (M3_ prefix)

RAW = {}        # CAN address -> [bus, count, last bytes, last time, count at last tick, Hz]   (the worker only stores; decoding happens when the page asks)
MUXLAST = {}    # (address, mux value) -> last bytes, for multiplexed messages
MUXINFO = {}    # address -> (start, length, byteorder) of the mux selector
for _a, _tabs in MAPDB.items():
    for _t in _tabs:
        for _s in _t['s']:
            if _s[1] == -1: MUXINFO[_a] = (_s[2], _s[3], _s[8])
def _dec(v, dat_len, st, ln, sg, sc, of, bo):
    if bo == 1: x = (v >> st) & ((1 << ln) - 1)
    else:                                   # Motorola / big-endian signal: st = DBC start bit (MSB)
        x = 0; i = st
        for _ in range(ln):
            x = (x << 1) | ((v >> ((i // 8) * 8 + i % 8)) & 1)
            i = i + 15 if i % 8 == 0 else i - 1
    if sg and x >= 1 << (ln - 1): x -= 1 << ln
    return x * sc + of
def decode_one(a, d):
    sigs = d['s']; out = {}; cur = None
    if a in MUXINFO:
        st, ln, bo = MUXINFO[a]
        items = sorted((k[1], b) for k, b in list(MUXLAST.items()) if k[0] == a)
        cur = (int.from_bytes(RAW[a][2], 'little') >> st) & ((1 << ln) - 1)
    else:
        items = [(None, RAW[a][2])]
    for mx, dat in items:
        v = int.from_bytes(dat, 'little')
        for n, m, st, ln, sg, sc, of, un, bo in sigs:
            if m is not None and m != -1 and m != mx: continue
            x = _dec(v, len(dat), st, ln, sg, sc, of, bo)
            x = round(x, 6) if (sc != 1 or of != 0) else int(x)
            out[n] = (str(x) + ' ' + un) if un else x
    return {'name': d['n'], 'addr': a, 'mux': cur, 'sig': out}
def build_map(now):
    msgs = {}; raw = {}
    for a, r in list(RAW.items()):
        tabs = [] if r[0] == RADAR_SRC else MAPDB.get(a, [])     # the car DBC does not apply to the radar bus (its ids overlap)
        raw[a] = {'src': r[0], 'hz': r[5], 'age': round(now - r[3], 1), 'hex': ' '.join('%02x' % x for x in r[2]), 'name': ' / '.join(t['n'] for t in tabs)}
        for d in tabs:
            try: msgs['%d:%s' % (a, d['n'])] = decode_one(a, d)
            except Exception: pass
    return msgs, raw
def _flat(d, pre=''):
    o = {}
    for k, x in d.items():
        if isinstance(x, dict): o.update(_flat(x, pre + k + '.'))
        elif isinstance(x, (int, float, bool, str)) or x is None: o[pre + k] = x
        elif isinstance(x, list) and len(x) <= 8 and all(isinstance(i, (int, float, bool, str)) for i in x): o[pre + k] = x
    return o
LOGDIR = '/data/maplog'
# PERF: STATE['o'] (streamed to every page 10x/s) only carries what the pages use; the full objects (extra radar fields
# for /learn labels) stay in OBJFULL. SNAP = STATE as JSON, built once per tick and shared by all browser streams.
PUB_KEYS = ('s', 'd', 'y', 'vr', 'gs', 'pe', 'c', 'inf', 'pc', 'len', 'ms', 'stationary', 'rc', 'ml', 'vy', 'cross')
OBJFULL = {}; SNAP = [None, 0.0]; TICK = [0]
# PERF (from a RADARWEB_PROFILE run on the EON: half the busy time was the per-CAN-frame loop): only these IDs are handled
# on every frame - radar objects/status, the IDs decoded in the loop, multiplexed messages. All others just get counted,
# and their latest bytes (for /map and the log) refreshed twice a second.
HOT = set([0x108, 0x2F8, 0x32E, 0x33E, 0x102, 0x302, 0x382, 0x212, 0x232, 0x3D2, 0x754, 0x51E, 0x6F2, 0x300, 0x301, 0x501]) | set(range(0x310, 0x36F)) | set(MUXINFO)
RAWDUE = [0.0]; SUBDUE = [0.0]
# radar_decode: radarml.decode_pair decodes each object frame pair once (radarlog's decode_a/decode_b + decode_ab did it
# twice, bit by bit). It is only trusted after it gave exactly radarlog's values for the first DEC_CHECK object frames
# from your radar; on any difference radarweb keeps using radarlog's decoders and prints what differed.
DEC_CHECK = 200; DEC = {'n': 0, 'fast': None}
def radar_decode(ba, bb):
    if DEC['fast'] is not True:
        a, b = decode_a(ba), decode_b(bb)
        if DEC['fast'] is None:
            fa, fb, _ = radarml.decode_pair(ba, bb)
            bad = [k for k in fa if abs(float(fa[k]) - float(a.get(k, 1e9))) > 1e-6] + [k for k in fb if abs(float(fb[k]) - float(b.get(k, 1e9))) > 1e-6]
            if bad:
                DEC['fast'] = False; print('fast radar decoder differs from radarlog on %s - keeping radarlog' % ', '.join(bad))
            else:
                DEC['n'] += 1
                if DEC['n'] >= DEC_CHECK: DEC['fast'] = True; print('fast radar decoder matches radarlog on %d frames - using it' % DEC_CHECK)
        return a, b, radarml.decode_ab(ba, bb)
    return radarml.decode_pair(ba, bb)
XS = []         # crossing detections since the last /data/maplog line (logged as 'xs' so radartrip.py can map them)
# --- class cleanup (the radar only classifies ~5% of its tracks, flips between classes, and its pedestrian guesses are mostly low confidence) ---
PED_MIN_PC = 50      # pedestrian / two-wheeler only shown if the radar's class probability is at least this (%); below that it is drawn as an unclassified marker
HOLD_S = 8.0         # a class the radar gave a track is kept this long (while the track stays within 8 m / 3 m of its last position)
MOVER_MS = 4.0       # unclassified object moving at least this fast over the ground (m/s, 14 km/h) and within 6 m sideways is drawn as a vehicle (marked inferred)
CLS_MEM = {}
LIMLAST = [0, None]
PARKT = [0.0]
PARKV = {'f': None, 'r': None}
# BATT: battery/BMS fields accumulated across several CAN IDs, pack-level only. No source checked
# (leaked internal Tesla DB, WK057/Skie.net, Scan My Tesla's own app source, Jack Rickard/evtv.me)
# has an actual bit-level decode for per-cell/brick voltage or temp - see the raw-capture block below
# for the two candidate IDs that might get you there with real data. Keeping every SOC candidate under
# its own key (not merged into one number) so they can be watched side by side against the dash % -
# that's how you tell which one, if any, is right for this car.
BATT = {'soc_302': None, 'energy': None, 'pack': None, 'bms_state': None, 'limits': None, 'totals': None,
        'cells_raw': {}}
def refine_class(k, now, cname, pc, d, y, gs):
    inf = False
    if cname in ('pedestrian', 'two_wheel') and pc < PED_MIN_PC: cname = 'unknown'
    mem = CLS_MEM.get(k)
    if cname != 'unknown': CLS_MEM[k] = [cname, now, d, y]
    elif mem and now - mem[1] < HOLD_S and abs(d - mem[2]) < 8 and abs(y - mem[3]) < 3:
        cname = mem[0]; mem[2] = d; mem[3] = y
    if cname == 'unknown' and abs(gs) >= MOVER_MS and abs(y) < 6: cname = 'four_wheel'; inf = True
    elif cname == 'pedestrian' and abs(gs) >= MOVER_MS: cname = 'four_wheel'; inf = True      # nobody walks at 14 km/h: the radar called a car a pedestrian
    elif cname == 'two_wheel' and abs(gs) >= 10.0: cname = 'four_wheel'; inf = True           # faster than 36 km/h is not a bicycle
    return cname, inf

# ML: classifier trained from your /learn labels (radarlearn_train.py on a laptop -> radar_model.json). Every track
# keeps HIST_S of history; the model gets the current frame + that history, its probabilities are smoothed per
# track, and it only replaces the radar/rule class above when it is at least min_p sure. The physical limits
# (nobody walks at 14 km/h, no bicycle at 36 km/h) are applied again after it. No model file = old behaviour.
MODEL_FILE = os.environ.get('RADAR_MODEL', '/data/radar_model.json')
TRK = radarml.TrackHist()
MODEL_STALE = [False]   # installed model predates the current features: retrain at the next stop instead of waiting for new labels
try: MODEL = radarml.Model(MODEL_FILE) if os.path.exists(MODEL_FILE) else None
except Exception as e:
    MODEL = None; MODEL_STALE[0] = 'other features' in str(e); print('radar model not loaded:', e)
MLLAST = {}     # slot -> last (label, probability): the model runs once per second per object (staggered), not 10x
def ml_class(k, now, o, cname, inf, hist=None, run=True):
    if MODEL is None: return cname, inf, None
    if run or k not in MLLAST: MLLAST[k] = MODEL.track_class(k, radarml.features(o, hist, STATE['v']), alpha=0.6)
    ml, p = MLLAST[k]
    if p >= MODEL.min_p and radarml.DISPLAY.get(ml, 'unknown') != cname:
        cname = radarml.DISPLAY.get(ml, 'unknown'); inf = True
        gs = abs(o['gs'])
        if cname == 'pedestrian' and gs >= MOVER_MS or cname == 'two_wheel' and gs >= 10.0: cname = 'four_wheel'
    return cname, inf, {'c': ml, 'p': round(p, 2)}

# AUTOTRAIN: retrain by itself once AUTOTRAIN_NEW new /learn labels have piled up (and at least radarml.AUTO_MIN_ROWS
# in total). Only while parked (under 1 km/h, openpilot not engaged, for a minute), as a separate `nice -n 19`
# process so it never competes with openpilot for CPU, killed the moment the car moves; it starts again on the next
# stop. radarml.autotrain only installs the new model if it scores better (cross-validated) than no model and than the
# installed one. The worker reloads the model whenever the file changes - no restart needed. AUTOTRAIN_NEW=0 disables.
AUTO_NEW = int(os.environ.get('AUTOTRAIN_NEW', '30'))
# AUTOLABEL: objects label themselves (radarml.AutoLabeler: moved fast = vehicle, never moved while you drove past =
# static, flickering/inconsistent = not real, radar sure for the whole track = that class). Written to their own file
# next to your /learn labels; both are used for training (yours count MANUAL_WEIGHT times more). AUTOLABEL=0 disables.
AUTOLABEL_FILE = os.environ.get('AUTOLABELS', os.path.join(os.path.dirname(OBJLABEL_FILE), 'autolabels.jsonl'))
AUTOLABEL_MAX = 50 * 1024 * 1024       # stop adding automatic labels once the file is this big
AL = radarml.AutoLabeler() if os.environ.get('AUTOLABEL', '1') != '0' else None
def autolabel_write(rows):
    if not rows: return
    try:
        if os.path.exists(AUTOLABEL_FILE) and os.path.getsize(AUTOLABEL_FILE) > AUTOLABEL_MAX: return
        os.makedirs(os.path.dirname(AUTOLABEL_FILE), exist_ok=True)
        with open(AUTOLABEL_FILE, 'a') as f:
            for r in rows: f.write(json.dumps(r, separators=(',', ':')) + '\n')
    except Exception as e: print('autolabel:', e)
# /ml page data: label counts per class are read incrementally (only bytes added since the last call), so a big
# autolabels.jsonl is never re-parsed; classes are pulled out with a regex instead of decoding every JSON row.
import re as _re_ml
_CRE = _re_ml.compile(r'[{,]\s*"c":\s*"([a-z_]+)"')
MLSUM = {}     # file -> {'off': bytes read, 'n': {target class: count}, 'recent': [last rows]}
def _scan_labels(fn):
    st = MLSUM.setdefault(fn, {'off': 0, 'n': {}, 'recent': []})
    try: sz = os.path.getsize(fn)
    except OSError: return st
    if sz < st['off']: st.update(off=0, n={}, recent=[])            # file was deleted/reset
    if sz == st['off']: return st
    with open(fn, 'rb') as f:
        f.seek(st['off']); chunk = f.read(); st['off'] += len(chunk)
    lines = chunk.decode('utf8', 'replace').splitlines()
    for ln in lines:
        m = _CRE.search(ln)
        if m: c = radarml.TARGET.get(m.group(1), m.group(1)); st['n'][c] = st['n'].get(c, 0) + 1
    for ln in lines[-12:]:
        try:
            r = json.loads(ln)
            st['recent'].append({'c': r.get('c'), 'd': r.get('d'), 'y': r.get('y'), 'gs': r.get('gs'), 'radar_c': r.get('radar_c'), 't': r.get('t'), 'auto': bool(r.get('auto'))})
        except ValueError: pass
    st['recent'] = st['recent'][-12:]
    return st
def ml_summary():
    man = _scan_labels(OBJLABEL_FILE); aut = _scan_labels(AUTOLABEL_FILE)
    m = MODEL.m if MODEL is not None else None
    model = None
    if m:
        model = {k: m.get(k) for k in ('algo', 'rows', 'cv_macro_f1', 'baseline_f1', 'trained', 'min_p', 'classes', 'metrics', 'candidates', 'baseline', 'feats_version', 'kind')}
        model['trees'] = len(m.get('trees', [])); model['prev'] = os.path.exists(MODEL_FILE + '.prev')
        if m.get('kind') == 'mlp': model['layers'] = [len(b_) for _, b_ in m['layers']]
    try: last = json.load(open(MODEL_FILE + '.status.json'))
    except Exception: last = None
    with LOCK: objs = [{k: o.get(k) for k in ('s', 'd', 'y', 'gs', 'c', 'rc', 'pc', 'ml', 'inf')} for o in STATE['o']]
    return {'model': model, 'last_training': last, 'status': STATE.get('ml'), 'manual': man['n'], 'auto': aut['n'],
            'recent': sorted(man['recent'] + aut['recent'], key=lambda r: r.get('t') or 0)[-12:], 'objects': objs,
            'files': {'manual': OBJLABEL_FILE, 'auto': AUTOLABEL_FILE, 'model': MODEL_FILE}, 'autolabel_on': AL is not None,
            'next_at': max(radarml.AUTO_MIN_ROWS, AUTO['trained_rows'] + AUTO_NEW), 'total': AUTO['rows']}

# TRIPS: /trips shows a /data/maplog drive on an OpenStreetMap map (radartrip.py). Turning a 50 MB log into a trip takes
# a while on the EON, so it runs as a separate `nice -n 19` process and the result is cached next to the log
# (map_*.trip.json); the drive being recorded right now is re-processed at most once a minute.
import re as _re_trip
TRIPJOBS = {}
def trips_list():
    out = []
    try: names = sorted((f for f in os.listdir(LOGDIR) if _re_trip.match(r'^map_\d{8}_\d{6}\.jsonl$', f)), reverse=True)
    except OSError: names = []
    for f in names:
        p_ = os.path.join(LOGDIR, f); c_ = p_[:-6] + '.trip.json'
        out.append({'name': f, 'size': os.path.getsize(p_), 'ready': os.path.exists(c_) and os.path.getmtime(c_) >= os.path.getmtime(p_) - 60})
    return {'files': out}
def trip_get(f):
    if not _re_trip.match(r'^map_\d{8}_\d{6}\.jsonl$', f or ''): return {'error': 'bad file name'}
    src = os.path.join(LOGDIR, f); cache = src[:-6] + '.trip.json'
    if not os.path.exists(src): return {'error': 'no such log'}
    job = TRIPJOBS.get(f)
    if job is not None and job.poll() is None: return {'pending': True}
    TRIPJOBS.pop(f, None)
    fresh = os.path.exists(cache) and (os.path.getmtime(cache) >= os.path.getmtime(src) or
                                       (src == MAP.get('logfile') and time.time() - os.path.getmtime(cache) < 60))
    if fresh or (job is not None and os.path.exists(cache)):
        try: return json.load(open(cache))
        except Exception: pass
    if job is not None and job.returncode: return {'error': 'processing failed (exit %s), see /tmp/radar_trip.log' % job.returncode}
    import subprocess
    TRIPJOBS[f] = subprocess.Popen(['nice', '-n', '19', sys.executable, os.path.join(os.path.dirname(os.path.abspath(radarml.__file__)), 'radartrip.py'), '--json', src, cache],
                                   stdout=subprocess.DEVNULL, stderr=open('/tmp/radar_trip.log', 'w'))
    return {'pending': True}

def _label_lines():
    n = 0
    for fn in (OBJLABEL_FILE, AUTOLABEL_FILE):
        try: n += sum(1 for _ in open(fn))
        except IOError: pass
    return n
AUTO = {'proc': None, 'park': 0.0, 'next': 0.0, 'size': -1, 'rows': 0, 'trained_rows': 0, 'prev_rows': 0, 'mtime': 0.0, 'msg': ''}
try:
    _st = json.load(open(MODEL_FILE + '.status.json')); AUTO['trained_rows'] = _st.get('labels', 0); AUTO['msg'] = _st.get('msg', '')
except Exception: pass
if MODEL_STALE[0]: AUTO['trained_rows'] = -10 ** 6; AUTO['msg'] = 'model v2: the installed model uses older features - retraining at the next stop'
try: AUTO['mtime'] = os.path.getmtime(MODEL_FILE)
except OSError: pass
def start_training(manual):
    """run radarml --autotrain in the background at the lowest CPU priority (automatic when parked, or "Train now" on /ml)"""
    if AUTO['proc'] is not None: return False
    import subprocess
    AUTO['prev_rows'] = AUTO['trained_rows']; AUTO['trained_rows'] = max(AUTO['rows'], 0); AUTO['manual'] = manual
    AUTO['msg'] = 'training on %d labels%s…' % (AUTO['rows'], ' (started by you)' if manual else '')
    AUTO['proc'] = subprocess.Popen(['nice', '-n', '19', sys.executable, radarml.__file__, '--autotrain', OBJLABEL_FILE + ',' + AUTOLABEL_FILE, MODEL_FILE],
                                    stdout=subprocess.DEVNULL, stderr=open('/tmp/radar_autotrain.log', 'w'))
    STATE['ml'] = {'msg': AUTO['msg'], 'labels': AUTO['rows'], 'training': True, 'auto': dict(AL.counts) if AL else None}   # show it right away
    return True

def autotrain_tick(now):
    global MODEL
    try:                                   # pick up a new model file (autotrain, or one copied over from the laptop)
        mt = os.path.getmtime(MODEL_FILE)
        if mt != AUTO['mtime']:
            AUTO['mtime'] = mt; MODEL = radarml.Model(MODEL_FILE); print('radar model loaded:', MODEL_FILE)
    except OSError: pass
    except Exception as e:
        print('radar model not loaded:', e)
        if 'other features' in str(e): MODEL = None; AUTO['trained_rows'] = -10 ** 6
    parked = STATE['v'] < 0.3 and not STATE['op']
    in_p = 'park' in str(MAP.get('car', {}).get('gearShifter', '')).lower()
    p = AUTO['proc']
    if p is not None:
        if p.poll() is None:
            if not parked and not AUTO.get('manual'):      # a training you started with "Train now" is left to finish
                p.kill(); p.wait(); AUTO['proc'] = None; AUTO['msg'] = 'training stopped (car moving), retries when parked'
                AUTO['trained_rows'] = AUTO['prev_rows']; AUTO['park'] = 0.0
        else:
            AUTO['proc'] = None
            try: AUTO['msg'] = json.load(open(MODEL_FILE + '.status.json')).get('msg', '')
            except Exception: AUTO['msg'] = 'training failed (exit %s)' % p.returncode
        STATE['ml'] = {'msg': AUTO['msg'], 'labels': AUTO['rows'], 'training': AUTO['proc'] is not None, 'auto': dict(AL.counts) if AL else None}
        return
    if not parked: AUTO['park'] = 0.0
    elif not AUTO['park']: AUTO['park'] = now
    if now >= AUTO['next']:                # count labels at most once a minute, and only when the file grew
        AUTO['next'] = now + 60
        sz = sum(os.path.getsize(fn) for fn in (OBJLABEL_FILE, AUTOLABEL_FILE) if os.path.exists(fn))
        if sz != AUTO['size']: AUTO['size'] = sz; AUTO['rows'] = _label_lines()
    due = AUTO_NEW > 0 and AUTO['rows'] >= radarml.AUTO_MIN_ROWS and AUTO['rows'] - AUTO['trained_rows'] >= AUTO_NEW
    # really parked: gear in P for 30 s, or stopped 2 min (model v2 training takes longer than a red light)
    if due and AUTO['park'] and (in_p and now - AUTO['park'] > 30 or now - AUTO['park'] > 120):
        start_training(False)
    elif not AUTO['msg'] or AUTO['msg'].startswith('waiting'):
        AUTO['msg'] = 'waiting: %d labels, next training at %d%s' % (AUTO['rows'], max(radarml.AUTO_MIN_ROWS, AUTO['trained_rows'] + AUTO_NEW),
                                                                     ' (when parked)' if due else '')
    STATE['ml'] = {'msg': AUTO['msg'], 'labels': AUTO['rows'], 'training': AUTO['proc'] is not None, 'auto': dict(AL.counts) if AL else None}

def worker():
    # PROFILE: RADARWEB_PROFILE=60 profiles the data loop (cProfile) for its first 60 s, writes where the time went (top
    # functions by total and by own time) to /tmp/radarweb_profile.txt, then carries on unprofiled in the same loop.
    secs = float(os.environ.get('RADARWEB_PROFILE', '0') or 0)
    prof = None
    if secs > 0:
        import cProfile; prof = [cProfile.Profile(), time.time() + secs, secs]; prof[0].enable()
    _worker(prof)

def _profile_done(prof):
    import pstats, io
    prof[0].disable(); out = io.StringIO()
    st = pstats.Stats(prof[0], stream=out); st.sort_stats('cumulative').print_stats(25); st.sort_stats('tottime').print_stats(25)
    open('/tmp/radarweb_profile.txt', 'w').write('profiled %.0f s of the radarweb data loop\n' % prof[2] + out.getvalue())
    print('profile written to /tmp/radarweb_profile.txt')

def _worker(prof=None):
    if messaging is None:
        print('no openpilot messaging here (not on the EON): live data is off. Open /?demo=1 for the demo scene, or use radarreplay.py to replay a recording.'); return
    can = messaging.sub_sock('can'); subs = {}
    for svc in ('carState', 'controlsState', 'radarState', 'model'):
        # conflate: keep only the newest message - only the latest state is used, and a reader that falls behind
        # (busy page threads, camera encoding) otherwise gets "Reader was evicted, reconnecting". Not for 'can'.
        try: subs[svc] = messaging.sub_sock(svc, conflate=True)
        except TypeError:
            try: subs[svc] = messaging.sub_sock(svc)
            except Exception: pass
        except Exception: pass
    A = {}; B = {}; status = {}; nxt = 0; tick = 0; logf = None; last_tick = time.time(); td = {'car': 0.0, 'ctl': 0.0}
    while True:
        now = time.time()
        raw_due = now >= RAWDUE[0]
        if raw_due: RAWDUE[0] = now + 0.5
        for ev in messaging.drain_sock(can, wait_for_one=False):
            for m in ev.can:
                a = m.address; r = RAW.get(a)
                if r is not None and not raw_due and a not in HOT:
                    r[1] += 1; continue      # PERF: an ID nothing decodes - count it (Hz on /map), refresh its bytes twice a second
                d = m.dat; ld = len(d)
                if ld > 8: continue
                if r is None:
                    if len(RAW) >= 600: continue
                    r = RAW[a] = [m.src, 0, b'', now, 0, 0.0]
                r[0] = m.src; r[1] += 1; r[2] = bytes(d); r[3] = now
                if a == 0x108 and ld == 8 and m.src != RADAR_SRC:      # DI_torque1: motor torque Nm (16|13, 0.25), rpm (32|16), pedal % (48|8, 0.4)
                    try:
                        _v = int.from_bytes(r[2], 'little'); _t = (_v >> 16) & 0x1FFF; _w = (_v >> 32) & 0xFFFF
                        _t = (_t - 8192 if _t >= 4096 else _t) * 0.25; _w = _w - 65536 if _w >= 32768 else _w
                        STATE['pw'] = {'kw': round(_t * _w * 0.10471975512 / 1000.0, 1), 'tq': round(_t), 'rpm': _w, 'ped': round(((_v >> 48) & 0xFF) * 0.4)}
                    except Exception: pass
                if a == 0x2F8 and ld == 8 and m.src != RADAR_SRC:      # UI_gpsVehicleSpeed: UI_mppSpeedLimit (48|5, x5) + UI_mapSpeedLimitUnits (46|1; 1 = km/h): the map speed limit (0 = unknown, kept 60 s)
                    try:
                        _v = int.from_bytes(r[2], 'little'); _l = ((_v >> 48) & 31) * 5
                        if _l > 0: LIMLAST[0] = now; LIMLAST[1] = {'v': _l, 'kmh': bool((_v >> 46) & 1)}
                        STATE['lim'] = LIMLAST[1] if LIMLAST[1] and now - LIMLAST[0] < 60 else None
                    except Exception: pass
                if a in (0x32E, 0x33E) and ld == 5 and m.src == 0:    # PARK_sensorStatusFront / Rear on the chassis bus: corner distances in cm (255 = nothing in range): left (8|8), right (16|8), system status (24|2)
                    try:
                        _v = int.from_bytes(r[2], 'little'); PARKT[0] = now
                        PARKV['f' if a == 0x32E else 'r'] = {'l': (_v >> 8) & 255, 'r': (_v >> 16) & 255, 'ss': (_v >> 24) & 3}
                        STATE['park'] = dict(PARKV)
                    except Exception: pass
                if a == 0x102 and ld >= 4 and m.src == 0:    # BMS pack V/I, WK057/Skie.net: voltage (0|16,u,x0.01)V, current (16|15,signed,x0.1,-1000A offset). The -1000A offset on TOP of a signed field is unusual but is literally what the source gives - sanity check against a clamp meter once before trusting the sign/magnitude.
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        _i = (_v >> 16) & 0x7FFF
                        if _i >= 1 << 14: _i -= 1 << 15
                        BATT['pack'] = {'v': round(((_v >> 0) & 0xFFFF) * 0.01, 2), 'a': round(_i * 0.1 - 1000, 1)}
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                if a == 0x302 and ld >= 3 and m.src == 0:    # SOC Min/UI, WK057/Skie.net - sheet flags BOTH fields "off compared to in-car display" (accuracy 3/10): two 10-bit fields at (0|10) and (10|10), x0.1
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        BATT['soc_302'] = {'min': round(((_v >> 0) & 0x3FF) * 0.1, 1), 'ui': round(((_v >> 10) & 0x3FF) * 0.1, 1)}
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                if a == 0x382 and ld == 8 and m.src == 0:    # pack energy, WK057/Skie.net raw fields (accuracy 10/10): nominalFullPack(0|10,x0.1) nominalRemaining(10|10,x0.1) expectedRemaining(20|10,x0.1) idealRemaining(30|10,x0.1) energyBuffer(50|8,x0.1) kWh. soc_calc is scan-my-tesla's formula (accuracy 7/10) - the 76.0 constant is specific to the 85/90kWh classic pack, wrong for other pack sizes.
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        _full = ((_v >> 0) & 0x3FF) * 0.1; _nom = ((_v >> 10) & 0x3FF) * 0.1
                        _exp = ((_v >> 20) & 0x3FF) * 0.1; _ideal = ((_v >> 30) & 0x3FF) * 0.1
                        _buf = ((_v >> 50) & 0xFF) * 0.1
                        BATT['energy'] = {'full_kwh': round(_full, 1), 'nominal_kwh': round(_nom, 1), 'expected_kwh': round(_exp, 1),
                                          'ideal_kwh': round(_ideal, 1), 'buffer_kwh': round(_buf, 1),
                                          'soc_calc': round(((_nom - _buf) / 76.0) * 100, 1) if _full else None}
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                if a == 0x212 and ld >= 3 and m.src == 0:    # BMS_state, leaked internal DB enum at bit (20|4) - cross-confirmed against an independent second source (accuracy 6/10 on the source, but the two agree exactly on bit position): 0 STANDBY 1 DRIVE 2 SUPPORT 3 CHARGER 4 FASTCHARGE 5 CHARGERVOLTAGE 6 CLEARFAULT 7 FAULT 8 WELD 15 SNA
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        BATT['bms_state'] = (_v >> 20) & 0xF
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                if a == 0x232 and ld >= 4 and m.src == 0:    # max discharge/regen power, WK057/Skie.net (accuracy 10/10): regen(0|16,x0.01) discharge(16|16,x0.01) kW
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        BATT['limits'] = {'max_regen_kw': round(((_v >> 0) & 0xFFFF) * 0.01, 2), 'max_discharge_kw': round(((_v >> 16) & 0xFFFF) * 0.01, 2)}
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                if a == 0x3D2 and ld == 8 and m.src == 0:    # lifetime charge/discharge totals, WK057/Skie.net (accuracy 10/10): discharge(0|32,x1) charge(32|32,x1), raw Wh - divide by 1000 for kWh (done here)
                    try:
                        _v = int.from_bytes(r[2], 'little')
                        BATT['totals'] = {'discharge_kwh': round((_v & 0xFFFFFFFF) / 1000.0, 1), 'charge_kwh': round(((_v >> 32) & 0xFFFFFFFF) / 1000.0, 1)}
                        STATE['batt'] = dict(BATT)
                    except Exception: pass
                # NOT decoded - no verified bit layout exists in any source checked for actual per-cell/brick
                # voltage or temp. 0x6F2 is named "Cell 1..96 voltage" / "Cell 1..32 temp" (Jack Rickard/evtv.me,
                # multiplexed) but with no startBit/length given at all. 0x754 is Scan My Tesla's "cell voltage
                # block selector" - a real lead (meant to be paired with 0x51E data frames, multiplexed by that
                # selector) but still unresolved here. Storing raw hex + timestamp only, so you can watch these
                # change in the browser (try it while charging, when cell spread is most visible) and
                # reverse-engineer the per-cell mapping from an actual capture on your car.
                if a in (0x754, 0x51E, 0x6F2) and m.src == 0:
                    BATT['cells_raw'][hex(a)] = {'hex': r[2].hex(), 't': round(now, 2)}
                    STATE['batt'] = dict(BATT)
                mi = MUXINFO.get(a)
                if mi is not None and ld == 8: MUXLAST[(a, (int.from_bytes(r[2], 'little') >> mi[0]) & ((1 << mi[1]) - 1))] = r[2]
                if m.src == RADAR_SRC and ld == 8 and 0x310 <= a < 0x36F:
                    k = (a - 0x310) // 3; q = (a - 0x310) % 3
                    if q == 0: A[k] = (r[2], now)
                    elif q == 1: B[k] = (r[2], now)
        if STATE.get('park') and now - PARKT[0] > 2.0: STATE['park'] = None
        _subs = subs.items() if now >= SUBDUE[0] else ()      # PERF: state streams are conflated (newest only): read them 10x/s
        if _subs: SUBDUE[0] = now + 0.1
        for svc, s in _subs:
            for ev in messaging.drain_sock(s, wait_for_one=False):
                try:
                    if svc == 'carState':
                        STATE['v'] = round(ev.carState.vEgo, 2); STATE['ang'] = round(ev.carState.steeringAngle, 1)
                        try: STATE['a'] = float(ev.carState.aEgo)
                        except Exception: pass
                        try: STATE['acc'] = bool(ev.carState.cruiseState.enabled and ev.carState.cruiseState.speed > 0.5)   # 'enabled' alone is true while parked, speed is 0 until a set speed exists
                        except Exception: pass
                        if now >= td['car']:
                            td['car'] = now + 0.5
                            try: MAP['car'] = _flat(ev.carState.to_dict())
                            except Exception: pass
                    elif svc == 'controlsState':
                        c = ev.controlsState; STATE['op'] = bool(c.enabled); STATE['cruise'] = round(c.vCruise, 1)
                        STATE['alert'] = c.alertText1 if c.alertText1 else ''; STATE['alc'] = bool(c.enabled)
                        if now >= td['ctl']:
                            td['ctl'] = now + 0.5
                            try: MAP['ctl'] = _flat(ev.controlsState.to_dict())
                            except Exception: pass
                    elif svc == 'model':
                        mm = ev.model
                        def poly(pd):
                            p = list(pd.poly)
                            if len(p) == 4: return [float(x) for x in p]
                            import numpy as np
                            pts = list(pd.points)[:50]
                            return [float(x) for x in np.polyfit(range(len(pts)), pts, 3)] if len(pts) > 10 else None
                        STATE['model'] = {'p': poly(mm.path), 'l': poly(mm.leftLane), 'r': poly(mm.rightLane),
                                          'pl': round(mm.leftLane.prob, 2), 'pr': round(mm.rightLane.prob, 2)}
                    elif svc == 'radarState':
                        l = ev.radarState.leadOne
                        STATE['lead'] = {'d': round(l.dRel, 1), 'y': round(l.yRel, 2), 'vr': round(l.vRel, 1)} if l.status else None
                except Exception: pass
        if now >= nxt:
            nxt = now + 0.1; objs = []
            for ad, fn, key in ((0x301, decode_sgu, None), (0x501, decode_alerts, 'alerts')):
                rr = RAW.get(ad)
                if rr and rr[0] == RADAR_SRC and len(rr[2]) == 8:
                    if key: status[key] = fn(rr[2])
                    else: status.update(fn(rr[2]))
            for k in range(32):
                if k not in A or k not in B: continue
                (ba, ta), (bb, tb) = A[k], B[k]
                if now - ta > STALE or now - tb > STALE: continue
                a, b, _x = radar_decode(ba, bb)
                if not (a['valid'] or a['tracked']) or a['long_dist'] <= 0 or a['long_dist'] >= 250 or a['index'] != b['index2']: continue
                gs = a['long_speed'] + STATE['v']
                cname, inf = refine_class(k, now, CLASS.get(b['cls'], 'code%d' % b['cls']), b['prob_class'], a['long_dist'], a['lat_dist'], gs)
                o = {'s': k, 'd': round(a['long_dist'], 1), 'y': round(a['lat_dist'], 2), 'vr': round(a['long_speed'], 1),
                     'gs': round(gs, 1), 'pe': round(a['prob_exist']), 'c': cname, 'inf': inf,
                     'pc': round(b['prob_class']), 'len': round(b['length'], 1), 'ms': MOVING.get(b['moving'], b['moving']),
                     'stationary': abs(gs) < 1.5, 'rc': cname}     # rc = class before the model: what /learn stores as radar_c
                o.update(_x)           # lat speed, long accel, dZ, obstacle probs, sigmas (tesla_radar.dbc)
                o['ga'] = o.get('la', 0.0) + STATE.get('a', 0.0)   # ground acceleration = radar's relative + your own
                TRK.add(k, now, o); o['_h'] = TRK.get(k, now, rnd=False)      # one history copy per object per tick
                o['vy_raw'] = radarml.lateral_speed(o['_h'], o.get('ls'))
                objs.append(o)
            TICK[0] += 1
            _yaw = radarml.scene_yaw(objs)          # your rotation, measured from the stationary tracks (see radarml CROSSING)
            for o in objs:
                _vy = radarml.ground_vy(o.pop('vy_raw', None), _yaw, o['d'], STATE['v'], STATE['ang'])
                o['vy'] = round(_vy, 2) if _vy is not None else None
                TRK.set_last(o['s'], 'vy', o['vy'])                 # model v2 feature: sideways ground speed history
                if o['_h']: o['_h'][-1][radarml.HIST_FIELDS.index('vy')] = o['vy'] or 0.0
                o['c'], o['inf'], o['ml'] = ml_class(o['s'], now, o, o['c'], o['inf'], o['_h'], (TICK[0] + o['s']) % 10 == 0)
                if o['c'] == 'unknown':          # crossing in front of you at walking / cycling / car pace (the radar can't see that)
                    _xc = radarml.cross_class(o['vy'], o['gs'], o['d'])
                    if _xc:
                        o['c'] = _xc; o['inf'] = True; o['cross'] = True
                        if not any(x[0] == o['s'] for x in XS) and len(XS) < 64: XS.append([o['s'], o['d'], o['y'], o['gs'], o['vy'], _xc])
                if AL: autolabel_write(AL.update(o['s'], now, o, STATE['v'], o['_h']))
                del o['_h']
            alive = set(x['s'] for x in objs); TRK.drop_missing(alive)
            for k in [k for k in MLLAST if k not in alive]: del MLLAST[k]
            if AL: autolabel_write(AL.end_missing(alive))
            if MODEL is not None:
                for k in [k for k in MODEL.smooth if k not in alive]: MODEL.forget(k)
            OBJFULL.clear(); OBJFULL.update((o['s'], o) for o in objs)
            with LOCK:
                STATE['t'] = round(now, 2); STATE['o'] = [dict((key, o[key]) for key in PUB_KEYS if key in o) for o in objs]
                STATE['status'] = {k: v for k, v in status.items()}
                SNAP[0] = json.dumps(STATE); SNAP[1] = now
        if now >= tick:
            dt = max(0.001, now - last_tick); last_tick = now; tick = now + 1.0
            try: autotrain_tick(now)
            except Exception as e: print('autotrain:', e)
            for r in RAW.values():
                r[5] = round((r[1] - r[4]) / dt, 1); r[4] = r[1]
            if RAW:
                try:
                    msgs, rawv = build_map(now)
                    if logf is None:
                        os.makedirs(LOGDIR, exist_ok=True); MAP['logfile'] = LOGDIR + '/map_%s.jsonl' % time.strftime('%Y%m%d_%H%M%S'); logf = open(MAP['logfile'], 'a')
                    with LOCK: _o = [[o['s'], o['d'], o['y'], o['gs'], o.get('vy'), o['c'], o.get('rc'), (o.get('ml') or {}).get('c')] for o in STATE['o']]
                    _xs = XS[:]; del XS[:]
                    logf.write(json.dumps({'t': round(now, 2), 'v': STATE['v'], 'acc': STATE.get('acc'), 'alc': STATE.get('alc'), 'o': _o, 'xs': _xs, 'msgs': {m['name']: m['sig'] for m in msgs.values()},
                                           'raw': {hex(a): r['hex'] for a, r in rawv.items()}, 'car': MAP['car']}) + '\n'); logf.flush()
                except Exception: pass
        time.sleep(0.01)
        if prof and now > prof[1]: _profile_done(prof); prof = None

ROAD = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Radar road view</title><style>
html,body{margin:0;height:100%;background:#0d0f12;overflow:hidden;font-family:system-ui,Segoe UI,sans-serif}
canvas{display:block;width:100vw;height:100vh}
#nav{position:fixed;right:10px;bottom:8px;z-index:50}#nav summary{list-style:none;cursor:pointer;width:36px;height:36px;display:flex;align-items:center;justify-content:center;background:#1c2128;border:1px solid #30363d;border-radius:8px;font-size:18px;color:#cfd6dd}#nav summary::-webkit-details-marker{display:none}#nav[open] summary{background:#262c35}#nav>div{position:absolute;right:0;bottom:44px;background:#1c2128;border:1px solid #30363d;border-radius:8px;padding:6px 0;min-width:190px;box-shadow:0 8px 24px rgba(0,0,0,.45)}#nav a,#nav>div span{display:block;padding:9px 14px;color:#cfd6dd;text-decoration:none;font-size:13px;white-space:nowrap}#nav a:hover{background:#262c35}
#labelbar{position:fixed;left:0;right:0;bottom:34px;display:flex;gap:6px;justify-content:center;flex-wrap:wrap;padding:0 8px;pointer-events:none}
#labelbar button{pointer-events:auto;border:0;border-radius:16px;padding:9px 13px;font:600 13px system-ui,sans-serif;color:#fff;opacity:.92;box-shadow:0 1px 4px rgba(0,0,0,.4)}
#labeltoast{position:fixed;left:0;right:0;bottom:66px;text-align:center;font:600 13px system-ui,sans-serif;color:#fff;text-shadow:0 1px 3px rgba(0,0,0,.8);pointer-events:none;opacity:0;transition:opacity .25s}
#labeltoast.show{opacity:1}
</style></head><body><canvas id=c></canvas><details id=nav><summary>☰</summary><div><span id=conn style="color:#9aa0a6">connecting…</span><a href="#" id=snd>lead-car chime: on</a><a href="#" id=nearsw onclick="return nearCycle()"></a><a href="#" id=themesw onclick="return themeCycle()"></a><a href="#" onclick="return tripReset()">reset trip</a><a href="/map">map data</a><a href="/top">top-down view</a><a href="/near">near view</a><a href="/calib">calibrate radar</a><a href="/video">video</a><a href="/learn">label objects</a><a href="?demo=1">demo</a></div></details>
<div id=labeltoast></div>
<div id=labelbar>
 <button style="background:#1a73e8" onclick="return mark('LABEL pedestrian')">Pedestrian</button>
 <button style="background:#e5484d" onclick="return mark('LABEL vehicle')">Vehicle</button>
 <button style="background:#8e44ad" onclick="return mark('LABEL two_wheel')">Two-wheel</button>
 <button style="background:#f08a00" onclick="return mark('LABEL obstacle')">Obstacle</button>
 <button style="background:#2fbf71" onclick="return mark('LABEL animal')">Animal</button>
 <button style="background:#7d8691" onclick="return mark('LABEL missed')">Missed!</button>
 <button style="background:#444b54" onclick="return mark('LABEL false_positive')">False+</button>
</div>
<script>
var COL={four_wheel:'#e5484d',two_wheel:'#8e44ad',pedestrian:'#1a73e8',construction:'#f08a00',unknown:'#7d8691'};
var NAME={four_wheel:'Vehicle',two_wheel:'Two-wheeler',pedestrian:'Pedestrian',construction:'Construction object',unknown:'Unknown object'};
var HGT={four_wheel:1.5,two_wheel:1.3,pedestrian:1.75,construction:.75,unknown:1.0},WID={four_wheel:1.9,two_wheel:.8,pedestrian:.6,construction:.5,unknown:.9};
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,demo=/demo/.test(location.search);
function pv(p,x){return ((p[0]*x+p[1])*x+p[2])*x+p[3]}
var YS=0;
/* farthest distance where the cubic stays sane (|offset|<=45 m); beyond that a sharp bend makes it explode */
function lim(a,mx){var x;for(x=3;x<=mx;x+=3){if(Math.abs(pv(a,x))>45)return Math.max(6,x-3)}return mx}
/* the radar reports class 'unknown' for ~95% of tracks; infer a vehicle for the lead and for moving car-length objects (marked ~) */
function eclass(o,isl){if(o.c!='unknown')return [o.c,!!o.inf];if(isl)return ['four_wheel',true];if(!o.stationary&&o.len>=2.5)return ['four_wheel',true];return ['unknown',false]}
function lerp(a,b,t){return a+(b-a)*t}
function rr(x,y,w,h,r){g.beginPath();g.moveTo(x+r,y);g.arcTo(x+w,y,x+w,y+h,r);g.arcTo(x+w,y+h,x,y+h,r);g.arcTo(x,y+h,x,y,r);g.arcTo(x,y,x+w,y,r);g.closePath()}
/* ---------- icons (drawn in a box x,y,w,h) ---------- */
var SEDANF=new Image();SEDANF.src='/icons/sedanf.webp';var ONC=false;
var SEDAN=new Image();SEDAN.src='/icons/sedan.webp';
function iconCar(x,y,w,h,c){var I=ONC&&SEDANF.complete&&SEDANF.naturalWidth?SEDANF:SEDAN;if(I.complete&&I.naturalWidth){var sh=w*I.naturalHeight/I.naturalWidth,sy=y+h-sh;g.drawImage(I,x,sy,w,sh);
  if(I===SEDANF){var hh=Math.max(2,sh*.07),hy=sy+sh*.4;g.fillStyle='rgba(255,246,200,.95)';rr(x+w*.14,hy,w*.2,hh,hh/2);g.fill();rr(x+w*.66,hy,w*.2,hh,hh/2);g.fill()}
  else{var lh=Math.max(2,sh*.045),ly=sy+sh*.355,br=DA.brake||Date.now()-DA.wshow<3000;g.fillStyle=br?'rgba(255,40,40,.98)':'rgba(214,40,40,.7)';rr(x+w*.2,ly,w*.28,lh,lh/2);g.fill();rr(x+w*.52,ly,w*.28,lh,lh/2);g.fill();if(br){g.globalAlpha=.35;rr(x+w*.17,ly-lh,w*.66,lh*3,lh);g.fill();g.globalAlpha=1}}}
 else{g.fillStyle='#cfd5db';rr(x+w*.08,y+h*.18,w*.84,h*.62,w*.1);g.fill()}}
var PED=new Image();PED.src='/icons/ped.webp';
function iconPed(x,y,w,h,c){if(PED.complete&&PED.naturalWidth){var ph=h,pw=ph*PED.naturalWidth/PED.naturalHeight;g.drawImage(PED,x+w/2-pw/2,y,pw,ph)}}
function iconCone(x,y,w,h,c){g.fillStyle='#f08a00';g.beginPath();g.moveTo(x+w*.5,y+h*.04);g.lineTo(x+w*.82,y+h*.84);g.lineTo(x+w*.18,y+h*.84);g.closePath();g.fill();
 g.fillStyle='#fff';g.beginPath();g.moveTo(x+w*.4,y+h*.3);g.lineTo(x+w*.6,y+h*.3);g.lineTo(x+w*.66,y+h*.46);g.lineTo(x+w*.34,y+h*.46);g.closePath();g.fill();
 g.beginPath();g.moveTo(x+w*.3,y+h*.58);g.lineTo(x+w*.7,y+h*.58);g.lineTo(x+w*.75,y+h*.72);g.lineTo(x+w*.25,y+h*.72);g.closePath();g.fill();
 g.fillStyle='#d86f00';rr(x+w*.06,y+h*.84,w*.88,h*.14,w*.04);g.fill()}
function iconBike(x,y,w,h,c){g.strokeStyle='#cfd5db';g.lineWidth=Math.max(2,w*.1);g.fillStyle='#cfd5db';g.beginPath();g.arc(x+w*.5,y+h*.84,h*.14,0,6.3);g.stroke();
 g.fillStyle=c;g.beginPath();g.arc(x+w*.5,y+h*.14,h*.1,0,6.3);g.fill();rr(x+w*.28,y+h*.26,w*.44,h*.42,w*.12);g.fill()}
function iconUnk(x,y,w,h,c){g.fillStyle=c;g.beginPath();g.moveTo(x+w*.5,y+h*.1);g.lineTo(x+w*.9,y+h*.5);g.lineTo(x+w*.5,y+h*.9);g.lineTo(x+w*.1,y+h*.5);g.closePath();g.fill()}
var ICON={four_wheel:iconCar,pedestrian:iconPed,construction:iconCone,two_wheel:iconBike,unknown:iconUnk};
function chev(x,y,s,fill,glow){if(glow){g.fillStyle=glow;g.beginPath();g.moveTo(x+s*1.35+s/5,y+s+s/10);g.lineTo(x,y-s/5);g.lineTo(x-s*1.35-s/5,y+s+s/10);g.closePath();g.fill()}
 g.fillStyle=fill;g.beginPath();g.moveTo(x+s*1.25,y+s);g.lineTo(x,y);g.lineTo(x-s*1.25,y+s);g.closePath();g.fill()}
/* ---------- day / night ---------- */
var THEME='auto';try{THEME=localStorage.getItem('theme')||'auto'}catch(e){}
function sstep(a,b,x){x=Math.max(0,Math.min(1,(x-a)/(b-a)));return x*x*(3-2*x)}
function nightAmt(){if(THEME=='day')return 0;if(THEME=='night')return 1;var d=new Date(),h=d.getHours()+d.getMinutes()/60,doy=Math.floor((d-new Date(d.getFullYear(),0,0))/864e5),dl=12+4.3*Math.sin(2*Math.PI*(doy-80)/365),sr=12-dl/2,ss=12+dl/2;
 return Math.max(sstep(ss-.3,ss+.8,h),1-sstep(sr-.8,sr+.3,h))}
function hex2(c){return [parseInt(c.substr(1,2),16),parseInt(c.substr(3,2),16),parseInt(c.substr(5,2),16)]}
function mx(a,b,t){var A=hex2(a),B=hex2(b);return 'rgb('+Math.round(A[0]+(B[0]-A[0])*t)+','+Math.round(A[1]+(B[1]-A[1])*t)+','+Math.round(A[2]+(B[2]-A[2])*t)+')'}
function themeCycle(){THEME=THEME=='auto'?'night':(THEME=='night'?'day':'auto');try{localStorage.setItem('theme',THEME)}catch(e){}themeUI();return false}
function themeUI(){var a=document.getElementById('themesw');if(a)a.textContent='theme: '+THEME}
/* ---------- scene ---------- */
function draw(){var W=cv.clientWidth,H=cv.clientHeight,dpr=devicePixelRatio||1;cv.width=W*dpr;cv.height=H*dpr;g.setTransform(dpr,0,0,dpr,0,0);
 var vw=Math.min(W,H*1.8),f=vw*.85,hz=H*.44,hc=3.2,CB=7,cx=W/2,u=Math.max(.6,Math.min(1.4,vw/1200));
 var m0=S&&S.model||{},pp0=m0.p||[0,0,0,0],Lk=Math.min(18,lim(pp0,18)),yt=Math.max(-1.1,Math.min(1.1,Math.atan2(pv(pp0,Lk),Lk)))*.9;YS+=(yt-YS)*.25;
 var CY=YS;function P(d,y){var dd=d*Math.cos(CY)+y*Math.sin(CY)+CB,yy=y*Math.cos(CY)-d*Math.sin(CY);if(dd<1.5)return null;return [cx-f*yy/dd,hz+f*hc/dd,dd]}
 var NI=nightAmt();var sk=g.createLinearGradient(0,0,0,hz);sk.addColorStop(0,mx('#8fb3d9','#03060d',NI));sk.addColorStop(1,mx('#d9e3ec','#16233a',NI));g.fillStyle=sk;g.fillRect(0,0,W,hz+1);
 if(NI>.05){g.globalAlpha=Math.min(1,NI*1.2);for(var si=0;si<60;si++){var sx=(si*7919%1000)/1000*W,sy=(si*104729%1000)/1000*hz*.8,sr=.6+(si%3)*.4;g.fillStyle='rgba(220,230,255,'+(.35+(si%4)*.15)+')';g.beginPath();g.arc(sx,sy,sr*u,0,6.3);g.fill()}
  g.globalAlpha=1}
 g.fillStyle=mx('#55704a','#0c140f',NI);g.fillRect(0,hz,W,H-hz);
 var m=S&&S.model||{},lp=m.l&&m.pl>.25?m.l:[0,0,0,1.75],rp=m.r&&m.pr>.25?m.r:[0,0,0,-1.75],pp=m.p||[0,0,0,0];
 function band(a,b,off1,off2,col,x0,x1,st){x1=Math.min(x1,lim(a,x1),lim(b,x1));g.fillStyle=col;g.beginPath();var i,pt,first=true;for(i=x0;i<=x1;i+=st){pt=P(i,pv(a,i)+off1);if(!pt)continue;first?g.moveTo(pt[0],pt[1]):g.lineTo(pt[0],pt[1]);first=false}
  for(i=x1;i>=x0;i-=st){pt=P(i,pv(b,i)+off2);if(pt)g.lineTo(pt[0],pt[1])}g.closePath();g.fill()}
 var ra=g.createLinearGradient(0,hz,0,H);ra.addColorStop(0,mx('#6a6e74','#262b33',NI));ra.addColorStop(1,mx('#34373c','#0f1115',NI));
 band(lp,rp,3.4,-3.4,ra,-6,160,3);
 var pa=(m.pl>.25?m.pl:.9);g.globalAlpha=Math.min(1,.4+pa);band(lp,lp,.18,-.18,mx('#f2f2f2','#9aa4b2',NI),-6,150,2);band(rp,rp,.18,-.18,mx('#f2f2f2','#9aa4b2',NI),-6,150,2);g.globalAlpha=1;
 var eng=S&&S.op;var tg=g.createLinearGradient(0,H,0,hz);tg.addColorStop(0,eng?'rgba(24,160,90,.9)':'rgba(255,255,255,.55)');tg.addColorStop(1,eng?'rgba(24,160,90,.08)':'rgba(255,255,255,.05)');
 g.globalAlpha=1-.4*NI;band(pp,pp,.95,-.95,tg,-6,70,2);g.globalAlpha=1;
 if(!S){return}
 /* objects, far to near: icon + ground marker only; small info tags are placed afterwards */
 var dmax=Math.min(lim(lp,160),lim(rp,160));
 /* only what is on or right next to the road: sideways distance from the lane centre at that range (lead always kept); beyond the range where the road is drawn nothing is shown */
 var os=(S.o||[]).filter(function(o){if(o.d<=3||o.d>=150)return false;if(NON&&o.d<9)return false;if(S.lead&&Math.abs(S.lead.d-o.d)<2.5&&Math.abs(S.lead.y-o.y)<1.2)return true;
   if(o.d>dmax)return false;var cc=(pv(lp,o.d)+pv(rp,o.d))/2,off=Math.abs(o.y-cc);
   if(off>7)return false;if(o.c=='unknown'&&o.stationary&&off>4.5)return false;return true}).sort(function(a,b){return b.d-a.d}),tags=[],icons=[];
 os.forEach(function(o){var isl=S.lead&&Math.abs(S.lead.d-o.d)<2.5&&Math.abs(S.lead.y-o.y)<1.2,ec=eclass(o,isl),oc=ec[0],inf=ec[1],gp=P(o.d,o.y);if(!gp)return;var c=COL[oc]||COL.unknown,hh=Math.max(30*u,f*(HGT[oc]||1)/gp[2]),ww=Math.max(20*u,f*(WID[oc]||1)/gp[2]);
  if(oc=='unknown'&&o.stationary){var dr=Math.max(3*u,Math.min(9*u,f*.12/gp[2]));g.globalAlpha=.55;g.fillStyle=c;g.beginPath();g.arc(gp[0],gp[1]-dr,dr,0,6.3);g.fill();g.globalAlpha=1;return}
  var s=Math.max(7*u,Math.min(22*u,f*.3/gp[2]));if(isl)chev(gp[0],gp[1]+2,s*.75,'rgba(214,40,40,.9)',(DA.brake||Date.now()-DA.wshow<3000)?'#ff2d2d':'#f5c400');
  var bx=gp[0]-ww/2,by=gp[1]-hh-s*.2;g.globalAlpha=1;ONC=oc=='four_wheel'&&!o.stationary&&o.gs<-1.5&&!isl;(ICON[oc]||iconUnk)(bx,by,ww,hh,c);g.globalAlpha=1;
  icons.push({x:bx,y:by,w:ww,h:hh});
  var kmh=o.stationary?0:Math.round(o.gs*3.6),t=Math.round(o.d)+' m'+(oc=='construction'?'':' · '+kmh+' km/h')+(isl&&o.vr!=null?'  ('+(o.vr>=0?'+':'')+Math.round(o.vr*3.6)+')':'');
  tags.push({x:gp[0],top:by,c:isl?'#d62828':c,t:(isl?'Lead  ':'')+(inf&&!isl?'~ ':'')+t,pr:(isl?0:(oc=='pedestrian'||oc=='two_wheel'?1:2))*1000+o.d})});
 var fs=Math.round(12*u),th=fs*1.55,placed=icons.map(function(i){return {x:i.x,y:i.y,w:i.w,h:i.h}}),minY=H*.2;g.font='600 '+fs+'px system-ui,sans-serif';g.textAlign='center';
 tags.sort(function(a,b){return a.pr-b.pr});
 tags.forEach(function(T){var w=g.measureText(T.t).width+fs*1.1,x=Math.max(2,Math.min(W-w-2,T.x-w/2)),y=T.top-th-3,k=0;
  while(k++<4&&placed.some(function(p){var ph=p.h||th;return x<p.x+p.w+3&&x+w+3>p.x&&y<p.y+ph+2&&y+th+2>p.y}))y-=th+3;
  if(k>4||y<minY)return;placed.push({x:x,y:y,w:w,h:th});g.textAlign='left';g.lineJoin='round';g.lineWidth=fs*.28;g.strokeStyle='rgba(8,10,14,.85)';g.strokeText(T.t,x+fs*.5,y+th/2+fs*.34);g.fillStyle='#fff';g.fillText(T.t,x+fs*.5,y+th/2+fs*.34);g.textAlign='center'});
 /* HUD */
 var bw=150*u,bh=140*u,bx=22*u,by=22*u;g.fillStyle='rgba(60,66,75,.82)';rr(bx,by,bw,bh,20*u);g.fill();g.lineWidth=6*u;g.strokeStyle='#fff';rr(bx,by,bw,bh,20*u);g.stroke();
 g.textAlign='center';g.fillStyle='rgba(255,255,255,.85)';g.font='500 '+26*u+'px system-ui';g.fillText('MAX',bx+bw/2,by+40*u);g.fillStyle='#fff';g.font='700 '+68*u+'px system-ui';g.fillText(S.cruise>0&&S.cruise<250?Math.round(S.cruise):'N/A',bx+bw/2,by+112*u);
 if(S.lim){var lk=S.lim.kmh?S.lim.v:Math.round(S.lim.v*1.609),sx=bx+bw+62*u,sy=by+bh/2,sr=52*u;g.fillStyle=mx('#ffffff','#c9ced6',NI);g.beginPath();g.arc(sx,sy,sr,0,6.3);g.fill();g.strokeStyle='#d62828';g.lineWidth=11*u;g.beginPath();g.arc(sx,sy,sr-5.5*u,0,6.3);g.stroke();
  g.fillStyle='#111';g.textAlign='center';g.font='700 '+(lk>=100?36:44)*u+'px system-ui';g.fillText(lk,sx,sy+(lk>=100?13:16)*u)}
 var kmh=S.v*3.6,over=S.lim?kmh-(S.lim.kmh?S.lim.v:S.lim.v*1.609):0,spc=over>10?'#ff6b6b':(over>3?'#ffc247':'#fff');
 g.font='700 '+110*u+'px system-ui';g.fillStyle=spc;g.fillText(Math.round(S.v*3.6),W/2,H*.15);g.font='500 '+30*u+'px system-ui';g.fillStyle='rgba(255,255,255,.85)';g.fillText('kph',W/2,H*.15+34*u);
 if(S.pw){var pw=S.pw,pbw=260*u,pbh=14*u,px=W/2-pbw/2,py=H*.15+54*u,fr=Math.max(-1,Math.min(1,pw.kw/80));g.fillStyle='rgba(60,66,75,.75)';rr(px,py,pbw,pbh,7*u);g.fill();g.fillStyle=pw.kw>=0?'#e6e8ea':'#2fbf71';
  if(fr>=0)g.fillRect(px+pbw/2,py+2*u,pbw/2*fr,pbh-4*u);else g.fillRect(px+pbw/2+pbw/2*fr,py+2*u,-pbw/2*fr,pbh-4*u);
  g.fillStyle='rgba(255,255,255,.9)';g.fillRect(px+pbw/2-1,py-3*u,2,pbh+6*u);g.textAlign='center';g.font='600 '+16*u+'px system-ui';g.fillStyle='#fff';
  g.fillText(Math.abs(pw.kw).toFixed(0)+' kW motor '+(pw.kw<-0.5?'(regen)':'')+'   pedal '+pw.ped+'%',W/2,py+pbh+20*u)}
 if(DA.gap!=null){var gt=DA.gap.toFixed(1)+' s gap',gc=DA.gap<1?'#d62828':(DA.gap<1.5?'#e08a00':'#178644');g.font='700 '+20*u+'px system-ui';var gw=g.measureText(gt).width+28*u,gy=H*.15+92*u;g.fillStyle=gc;rr(W/2-gw/2,gy,gw,32*u,16*u);g.fill();g.fillStyle='#fff';g.textAlign='center';g.fillText(gt,W/2,gy+23*u)}
 if(Date.now()-DA.wshow<3000){g.fillStyle='rgba(214,40,40,.95)';rr(W/2-230*u,H*.15+160*u,460*u,58*u,14*u);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 '+30*u+'px system-ui';g.fillText('CLOSING FAST - BRAKE',W/2,H*.15+200*u)}
 else if(Date.now()-DA.tshow<3000){g.fillStyle='rgba(224,138,0,.95)';rr(W/2-170*u,H*.15+160*u,340*u,48*u,14*u);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 '+26*u+'px system-ui';g.fillText('TOO CLOSE',W/2,H*.15+194*u)}
 else if(DA.brake){g.fillStyle='rgba(224,138,0,.95)';rr(W/2-150*u,H*.15+160*u,300*u,48*u,14*u);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 '+26*u+'px system-ui';g.fillText('LEAD BRAKING',W/2,H*.15+194*u)}
 if(Date.now()-LM.show<6000){g.fillStyle='rgba(24,160,90,.95)';rr(W/2-210*u,H*.15+96*u,420*u,58*u,14*u);g.fill();g.fillStyle='#fff';g.textAlign='center';g.font='700 '+30*u+'px system-ui';g.fillText('LEAD CAR MOVING',W/2,H*.15+136*u)}
 g.fillStyle=eng?'#178644':'#4b5563';g.beginPath();g.arc(W-84*u,76*u,52*u,0,6.3);g.fill();g.strokeStyle='#fff';g.lineWidth=11*u;g.beginPath();g.arc(W-84*u,76*u,24*u,0,6.3);g.stroke();
 var cx=W-84*u,cy=76*u+52*u+22*u;g.textAlign='center';g.font='600 '+15*u+'px system-ui';g.fillStyle=eng?'#7be0a4':'rgba(255,255,255,.75)';g.fillText(eng?'ENGAGED':'DISENGAGED',cx,cy);
 [['ACC',!!S.acc],['ALC',!!S.alc]].forEach(function(e,i){var x=cx-62*u+i*66*u,y=cy+10*u;g.fillStyle=e[1]?'#178644':'rgba(75,85,99,.85)';rr(x,y,58*u,28*u,14*u);g.fill();g.fillStyle=e[1]?'#fff':'rgba(255,255,255,.6)';g.font='700 '+16*u+'px system-ui';g.fillText(e[0],x+29*u,y+20*u)});
 if(S.pw){var tp=[DA.D<10?DA.D.toFixed(2)+' km':DA.D.toFixed(1)+' km',(DA.D>.1?Math.round(DA.E*1000/DA.D)+' Wh/km':'-- Wh/km')];
  tp.forEach(function(t,i){var x=cx-62*u,y=cy+10*u+(28+8)*(i+1)*u;g.fillStyle='rgba(75,85,99,.85)';rr(x,y,124*u,28*u,14*u);g.fill();g.fillStyle='#fff';g.font='700 '+16*u+'px system-ui';g.textAlign='center';g.fillText(t,x+62*u,y+20*u)})}
 if(TS.show){var sm=[['Distance',DA.D.toFixed(1)+' km'],['Driving time',(function(t){t=Math.round(t);var h=Math.floor(t/3600),mi=Math.floor(t%3600/60),sc=t%60;return h?h+' h '+mi+' min':mi+' min '+sc+' s'})(TS.mv)],['Average speed',TS.mv>5?Math.round(DA.D/(TS.mv/3600))+' km/h':'--'],['Top speed',Math.round(TS.vmax*3.6)+' km/h'],['Energy',DA.E.toFixed(2)+' kWh'],['Efficiency',DA.D>.1?Math.round(DA.E*1000/DA.D)+' Wh/km':'--'],['Too-close alerts',''+TS.close],['Closing alerts',''+TS.fast],['Cut-ins',''+TS.cuts]],
  cw=440*u,ch=(74+sm.length*34+36)*u,x0=W/2-cw/2,y0=H*.3;g.fillStyle='rgba(18,22,28,.9)';rr(x0,y0,cw,ch,20*u);g.fill();g.strokeStyle='rgba(255,255,255,.35)';g.lineWidth=2*u;rr(x0,y0,cw,ch,20*u);g.stroke();
  g.textAlign='center';g.fillStyle='#fff';g.font='700 '+28*u+'px system-ui';g.fillText('Drive summary',W/2,y0+44*u);
  sm.forEach(function(r,i){var yy=y0+84*u+i*34*u+22*u;g.textAlign='left';g.font='500 '+19*u+'px system-ui';g.fillStyle='rgba(255,255,255,.7)';g.fillText(r[0],x0+28*u,yy);g.textAlign='right';g.font='700 '+21*u+'px system-ui';g.fillStyle=(i>=6&&r[1]!='0')?'#ffb454':'#fff';g.fillText(r[1],x0+cw-28*u,yy)});
  g.textAlign='center';g.font='500 '+14*u+'px system-ui';g.fillStyle='rgba(255,255,255,.5)';g.fillText('tap to close',W/2,y0+ch-14*u)}
 if(S.alert){g.fillStyle='rgba(180,50,40,.9)';rr(W/2-300*u,H-120*u,600*u,60*u,12*u);g.fill();g.fillStyle='#fff';g.font='600 '+26*u+'px system-ui';g.fillText(S.alert,W/2,H-80*u)}
 /* legend */
 var lg=[['construction','Construction'],['pedestrian','Pedestrian'],['two_wheel','Two-wheeler'],['four_wheel','Vehicle'],['unknown','Unknown']],lx=14,ly=H-26;g.font='500 '+13*u+'px system-ui';g.textAlign='left';
 lg.forEach(function(e){g.fillStyle=COL[e[0]];g.beginPath();g.arc(lx+6,ly,6,0,6.3);g.fill();g.fillStyle='rgba(255,255,255,.9)';g.fillText(e[1],lx+16,ly+4);lx+=g.measureText(e[1]).width+36});
 g.fillStyle='rgba(255,255,255,.7)';g.fillText('display only - does not control the car',lx+6,ly+4)}
function hexA(h,a){var n=parseInt(h.slice(1),16);return 'rgba('+(n>>16)+','+(n>>8&255)+','+(n&255)+','+a+')'}
/* ---------- data ---------- */
/* ---------- lead-car-moving alert: while you are stopped (< 1 m/s) and the lead (within 45 m) has been standing still for 1.5 s, alert once when it moves off (> 1 m/s over the ground for 0.4 s) ---------- */
var LM={still:0,moving:0,armed:false,show:0,snd:true,ac:null};
/* ---------- driving aids shown on the page (display only): time gap, closing warning, lead braking, trip efficiency ---------- */
var TS={mv:0,vmax:0,close:0,fast:0,cuts:0,still:0,show:0,hid:false,last:0};
function tsTick(s,now){var v=s.v||0,dt=TS.last?Math.min(1,(now-TS.last)/1000):0;TS.last=now;
 if(v>.5){TS.mv+=dt;TS.still=0;if(v>TS.vmax)TS.vmax=v;TS.show=0;TS.hid=false}
 else{if(!TS.still)TS.still=now;if(!TS.show&&!TS.hid&&now-TS.still>20000&&TS.mv>120&&DA.D>.5)TS.show=now}}
function tripReset(){DA.E=0;DA.D=0;TS.mv=0;TS.vmax=0;TS.close=0;TS.fast=0;TS.cuts=0;TS.show=0;TS.hid=false;return false}
/* LABELBTN: tap a button while driving instead of needing to ssh in; see do_POST /api/label server-side */
function mark(txt){var t=document.getElementById('labeltoast');
 fetch('/api/label',{method:'POST',headers:{'Content-Type':'text/plain'},body:txt}).then(function(r){return r.json()}).then(function(j){
  t.textContent=j.ok?('Marked: '+txt.replace('LABEL ','')):'Mark failed (no recording running?)';t.classList.add('show');
  clearTimeout(window._mt);window._mt=setTimeout(function(){t.classList.remove('show')},1800)
 }).catch(function(){t.textContent='Mark failed (offline?)';t.classList.add('show');clearTimeout(window._mt);window._mt=setTimeout(function(){t.classList.remove('show')},1800)});
 if(navigator.vibrate)navigator.vibrate(30);return false}
var DA={tg:0,tlast:0,tshow:0,gap:null,ttc:null,brake:false,wshow:0,wlast:0,hist:[],E:0,D:0,last:0};
function warn(){try{if(LM.snd){if(LM.warnau){LM.warnau.currentTime=0;var pr=LM.warnau.play();if(pr&&pr.catch)pr.catch(function(){if(LM.ac){tone(520,0,.16,.22);tone(520,.22,.16,.22);tone(520,.44,.16,.22)}})}else if(LM.ac){tone(520,0,.16,.22);tone(520,.22,.16,.22);tone(520,.44,.16,.22)}}}catch(e){}try{navigator.vibrate&&navigator.vibrate([200,100,200])}catch(e){}}
function aids(s,now){var l=s.lead,v=s.v||0;try{tsTick(s,now)}catch(e){}
 /* trip: motor energy and distance since this page was opened (regen counts back) */
 if(DA.last&&s.pw){var dt=Math.min(1,(now-DA.last)/1000);DA.E+=s.pw.kw*dt/3600;DA.D+=v*dt/1000}DA.last=now;
 DA.gap=null;DA.ttc=null;
 if(!l||l.d>120||v<2){DA.brake=false;DA.hist=[];return}
 DA.gap=l.d/v;
 if(l.vr<-.5)DA.ttc=l.d/(-l.vr);
 /* lead deceleration over the last ~1 s (ground speed of the lead = vr + v) */
 DA.hist.push([now,l.vr+v]);while(DA.hist.length&&now-DA.hist[0][0]>1200)DA.hist.shift();
 var a=0;if(DA.hist.length>3){var h0=DA.hist[0],h1=DA.hist[DA.hist.length-1];a=(h1[1]-h0[1])/Math.max(.3,(h1[0]-h0[0])/1000)}
 DA.brake=a<-3&&v>3&&l.d<60;
 if(DA.gap<.8&&v>8){if(!DA.tg)DA.tg=now;if(now-DA.tg>3000&&now-DA.tlast>10000){DA.tlast=now;DA.tshow=now;TS.close++;warn()}}else DA.tg=0;
 if(DA.ttc!=null&&DA.ttc<2.5&&l.d<60&&v>3&&now-DA.wlast>4000){DA.wlast=now;DA.wshow=now;TS.fast++;warn()}}
function tone(f,t,d,gv){var o=LM.ac.createOscillator(),o2=LM.ac.createOscillator(),gn=LM.ac.createGain(),g2=LM.ac.createGain(),T=LM.ac.currentTime+t;
 o.type='sine';o2.type='sine';o.frequency.value=f;o2.frequency.value=f*2.0;g2.gain.value=.22;o2.connect(g2);g2.connect(gn);o.connect(gn);gn.connect(LM.ac.destination);
 gn.gain.setValueAtTime(.0001,T);gn.gain.linearRampToValueAtTime(gv,T+.012);gn.gain.exponentialRampToValueAtTime(.0001,T+d);o.start(T);o2.start(T);o.stop(T+d+.05);o2.stop(T+d+.05)}
function chime(){tone(660,0,.55,.2);tone(880,.17,.7,.2)}                /* soft two-note chime (sine + a quiet octave, fast attack, smooth decay) */
function beep(){try{if(LM.snd){if(LM.custom){LM.custom.currentTime=0;var pr=LM.custom.play();if(pr&&pr.catch)pr.catch(function(){if(LM.ac)chime()})}else if(LM.ac)chime()}}catch(e){}
 try{navigator.vibrate&&navigator.vibrate([120,80,120])}catch(e){}}
function leadWatch(s,now){var l=s.lead;if(!l||l.d>45||s.v>1.0){LM.still=0;LM.moving=0;LM.armed=false;return}
 var gs=l.vr+s.v;
 if(gs<.5){if(!LM.still)LM.still=now;LM.moving=0;if(now-LM.still>1500)LM.armed=true}
 else if(LM.armed&&gs>=1.0){if(!LM.moving)LM.moving=now;if(now-LM.moving>400){LM.show=now;LM.armed=false;LM.still=0;LM.moving=0;beep()}}
 else LM.moving=0}
cv.addEventListener('click',function(){if(TS.show){TS.show=0;TS.hid=true;draw()}});
document.getElementById('snd').onclick=function(e){e.preventDefault();LM.snd=!LM.snd;if(LM.snd&&!LM.ac){try{LM.ac=new (window.AudioContext||window.webkitAudioContext)()}catch(x){}}if(LM.snd&&LM.ac&&LM.ac.resume)LM.ac.resume();this.textContent='lead-car chime: '+(LM.snd?'on':'off');
 if(LM.snd&&!LM.tried){LM.tried=true;try{var au=new Audio('/chime');au.oncanplaythrough=function(){LM.custom=au};au.onerror=function(){LM.custom=null};au.load();var aw=new Audio('/chime?n=warn');aw.oncanplaythrough=function(){LM.warnau=aw};aw.onerror=function(){LM.warnau=null};aw.load()}catch(x){}}
 if(LM.snd)setTimeout(beep,250)};
var NT=15,NFR=null,NON=false,NLOW=0;try{var q=localStorage.getItem('nearKmh');if(q!==null)NT=parseFloat(q)}catch(e){}
function nearUI(){var a=document.getElementById('nearsw');if(a)a.textContent='auto near view: '+(NT>0?'below '+NT+' km/h':'off')}
function nearCycle(){var L=[0,5,10,15,20,30],i=L.indexOf(NT);NT=L[(i+1)%L.length];try{localStorage.setItem('nearKmh',NT)}catch(e){}nearUI();return false}
function nearTick(s){var kmh=s.v*3.6,on=NON;if(NT<=0)on=false;else if(!NON&&kmh<=NT)on=true;else if(NON&&kmh>NT+3)on=false;if(TS.show)on=false;
 if(on!==NON){NON=on;if(on){if(!NFR){NFR=document.createElement('iframe');NFR.style.cssText='position:fixed;left:50%;bottom:-18vh;margin-left:calc(min(100vw,104vh)/-2);width:min(100vw,104vh);height:min(100vw,104vh);border:0;background:transparent;pointer-events:none;z-index:50';NFR.setAttribute('allowtransparency','true');NFR.src='/near?embed=1'+(/demo/.test(location.search)?'&demo=1':'');document.body.appendChild(NFR)}NFR.style.display='block'}else if(NFR)NFR.style.display='none'}}
var CI={h:{},last:0};
function cutwarn(){TS.cuts++;try{if(LM.snd&&LM.ac){tone(880,0,.09,.28);tone(880,.13,.09,.28);tone(1175,.26,.16,.28)}}catch(e){}try{navigator.vibrate&&navigator.vibrate([90,60,90,60,160])}catch(e){}}
/* audio-only cut-in warning: a moving car from the next lane drifting toward the centre of our lane (needs ~0.8 s of consistent sideways motion, so radar jitter does not trigger it) */
function cutin(s,now){var m=s.model||{},lp=m.l&&m.pl>.25?m.l:[0,0,0,1.75],rp=m.r&&m.pr>.25?m.r:[0,0,0,-1.75],seen={};
 if((s.v||0)<5){CI.h={};return}
 (s.o||[]).forEach(function(o){if(o.c=='pedestrian'||o.c=='construction'||o.c=='two_wheel'||o.stationary||o.pe<70||o.d<4||o.d>50||o.gs<3)return;
  var cc=(pv(lp,o.d)+pv(rp,o.d))/2,ao=Math.abs(o.y-cc),h=CI.h[o.s]||(CI.h[o.s]=[]);seen[o.s]=1;h.push([now,ao]);while(h.length&&now-h[0][0]>1600)h.shift();
  if(h.length<5)return;var a0=h[0],a1=h[h.length-1],dt=(a1[0]-a0[0])/1000;if(dt<.8)return;
  if(a0[1]>1.5&&a0[1]<4.2&&a1[1]<2.3&&a1[1]>.5&&(a0[1]-a1[1])/dt>.6&&now-CI.last>5000){CI.last=now;cutwarn()}});
 for(var k in CI.h)if(!seen[k])delete CI.h[k]}
function upd(s){S=s;leadWatch(s,Date.now());aids(s,Date.now());try{cutin(s,Date.now())}catch(e){}try{nearTick(s)}catch(e){}draw()}
function demoState(t){var o=[{d:48,y:-5.1,gs:1.2,vr:-16,c:'pedestrian',stationary:false,len:.5},{d:26,y:3.4,gs:-13,vr:-29,c:'four_wheel',stationary:false,len:4.5},{d:32,y:.2,gs:16.0,vr:-1.2,c:'four_wheel',stationary:false,len:4.5},{d:36,y:3.6,gs:0,vr:-17.2,c:'construction',stationary:true,len:.5},
 {d:52,y:2.6,gs:0,vr:-17.2,c:'construction',stationary:true,len:.5},{d:20,y:-3.2,gs:20,vr:2.8,c:'two_wheel',stationary:false,len:2},{d:70,y:-2.4,gs:15,vr:-2.2,c:'unknown',stationary:false,len:4.5},{d:40,y:4.4,gs:0,vr:-17.2,c:'unknown',stationary:true,len:.5}];
 var sw=Math.sin(t/3)*.002;return {v:17.2,ang:0,op:true,acc:true,alc:(Math.floor(t/6)%2==0),cruise:120,lead:{d:32,y:.2,vr:-1.2},pw:{kw:Math.round(40*Math.sin(t/2)),tq:90,rpm:3000,ped:20},o:o,model:{l:[0,0,sw,1.75],r:[0,0,sw,-1.75],p:[0,0,sw,0],pl:.9,pr:.9},alert:''}}
if(demo){document.getElementById('conn').textContent='demo data';var t0=Date.now();(function loop(){upd(demoState((Date.now()-t0)/1000));setTimeout(loop,100)})()}
else{var es=new EventSource('/api/stream');es.onmessage=function(e){document.getElementById('conn').textContent='live';upd(JSON.parse(e.data))};es.onerror=function(){document.getElementById('conn').textContent='disconnected, retrying…'}}
onresize=draw;draw();
nearUI();themeUI();
</script></body></html>
"""

CALIB = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Radar calibration</title><style>
:root{--bg:#0d0f12;--card:#161a1f;--fg:#e6e9ec;--mut:#9aa3ad;--ok:#2fbf71;--warn:#f0a020;--bad:#e5484d;--line:#2a3038}
*{box-sizing:border-box}html,body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,Segoe UI,sans-serif}
header{display:flex;gap:14px;align-items:center;padding:10px 16px;border-bottom:1px solid var(--line);flex-wrap:wrap}header h1{font-size:18px;margin:0;flex:1}
header a{color:var(--mut);text-decoration:none;border-bottom:1px solid #556;font-size:13px}
main{display:grid;grid-template-columns:1fr 1fr;gap:12px;padding:12px 16px;max-width:1200px;margin:0 auto}
@media(max-width:820px){main{grid-template-columns:1fr}}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
h2{font-size:13px;letter-spacing:.04em;text-transform:uppercase;color:var(--mut);margin:0 0 8px}
.big{font-size:54px;font-weight:700;line-height:1;font-variant-numeric:tabular-nums}.sub{color:var(--mut);font-size:13px;margin-top:4px}
.row{display:flex;gap:18px;flex-wrap:wrap;align-items:flex-end}
#gauge{position:relative;height:34px;background:#242a31;border-radius:17px;margin:14px 0 4px}
#gz{position:absolute;top:0;bottom:0;left:45%;width:10%;background:rgba(47,191,113,.28)}
#gc{position:absolute;top:-4px;bottom:-4px;left:50%;width:2px;background:#fff}
#gm{position:absolute;top:3px;width:28px;height:28px;border-radius:50%;background:var(--mut);margin-left:-14px;left:50%;transition:left .12s}
.scale{display:flex;justify-content:space-between;color:var(--mut);font-size:12px}
canvas{width:100%;height:360px;background:#0f1216;border-radius:8px;display:block}
table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}th,td{text-align:right;padding:3px 6px;border-bottom:1px solid var(--line)}th{color:var(--mut);font-weight:500}th:first-child,td:first-child{text-align:left}
input{background:#0f1216;color:var(--fg);border:1px solid var(--line);border-radius:6px;padding:6px 8px;width:90px;font-size:15px}
button{background:#1e5fd0;color:#fff;border:0;border-radius:8px;padding:8px 14px;font-size:14px;cursor:pointer}button.g{background:#2a3038}
ol{margin:6px 0 0;padding-left:20px;font-size:13px;color:var(--mut)}li{margin:3px 0}
#hint{font-size:15px;margin-top:6px}.pill{display:inline-block;padding:2px 10px;border-radius:12px;font-size:13px;font-weight:600}
</style></head><body>
<header><h1>Radar calibration <span style="color:var(--mut);font-size:13px;font-weight:400">read-only - sends nothing to the car</span></h1><span id=conn style="color:var(--mut);font-size:13px">connecting…</span><a href="/">road view</a><a href="/near">near view</a><a href="/top">top-down</a><a href="?demo=1">demo</a></header>
<main>
<section><h2>Pole on the centre line</h2>
 <div class=row><div><div class=big id=dy>-</div><div class=sub>dY sideways offset (m), + = left of the car's centre line</div></div>
 <div><div class=big id=dr style="font-size:42px">-</div><div class=sub>distance from the bumper (m)</div><div class=sub id=rd></div></div></div>
 <div id=gauge><div id=gz></div><div id=gc></div><div id=gm></div></div>
 <div class=scale><span>0.5 m left</span><span>on line</span><span>0.5 m right</span></div>
 <div id=hint></div>
 <div class=sub id=q></div>
 <div style="margin-top:12px" class=row><label class=sub>measured with a ruler, bumper to pole (m)<br><input id=md type=number step=0.01 placeholder="e.g. 3.00"></label>
 <label class=sub>radar sits behind the bumper by (m)<br><input id=off type=number step=0.01 value=0></label><button id=setoff class=g>Use this reading</button>
 <div><div id=dd style="font-size:22px;font-weight:700">-</div><div class=sub>shown minus measured</div></div></div>
 <div class=sub style="margin-top:8px">Put the pole on the centre line, measure from the front edge of the bumper to the pole and type it in, then press <b>Use this reading</b>. The page learns how far behind the bumper the radar sits and shows distances from the bumper. Move the pole to other distances: the difference should stay within a few cm.</div>
</section>
<section><h2>Top view (front radar, 0 - 8 m)</h2><canvas id=cv></canvas></section>
<section style="grid-column:1/-1"><h2>Driving check - is the radar aimed straight? <span style="text-transform:none;letter-spacing:0">(learns while you drive, no pole needed)</span></h2>
 <div class=row><div><div class=big id=dg style="font-size:46px">-</div><div class=sub>yaw error (degrees): + = radar looks too far right</div></div>
 <div><div class=big id=dofs style="font-size:46px">-</div><div class=sub>sideways offset at 0 m (m): + = left of car centre</div></div>
 <div><div class=big id=dn style="font-size:46px">0</div><div class=sub id=dns>samples</div></div>
 <div style="flex:1;min-width:200px"><div id=dv></div><div class=sub id=dh style="margin-top:6px"></div><div style="margin-top:8px"><button class=g id=dreset>Reset learning</button></div></div></div>
 <canvas id=dcv style="height:220px;margin-top:10px"></canvas>
 <div class=sub style="margin-top:6px">Collects the sideways position of the car in front (15 - 80 m, same lane, straight road above 40 km/h) against its distance. If the radar is turned, that position slides sideways as the distance grows: the slope is the yaw error. The constant part is where the radar sits across the car. With the camera lane model it measures against the lane centre, without it against the car's centre line (needs more samples, since traffic is not always centred).</div>
</section>
<section><h2>Readings you saved</h2>
 <div class=row><button id=cap>Save reading</button><button class=g id=clr>Clear</button><span class=sub id=capn></span></div>
 <table id=ct style="margin-top:8px"><thead><tr><th>#</th><th>from bumper (m)</th><th>dY (m)</th><th>measured</th><th>error</th></tr></thead><tbody></tbody></table>
 <div class=sub id=cs style="margin-top:6px"></div>
</section>
<section><h2>All radar tracks 1 - 8 m ahead</h2>
 <table id=tt><thead><tr><th>track</th><th>dRel</th><th>dY</th><th>speed</th><th>exists %</th></tr></thead><tbody></tbody></table>
 <ol><li>Pole on the centre line between your chalk points A and B, standing still.</li>
 <li>dY should read 0.0 at every spot along that line (green zone = within 5 cm). Save a reading at each distance.</li>
 <li>dY always the same side: the radar is turned. If dY is + (pole looks left), the radar is probably aimed too far right: turn it left a little, and the other way round. Move it in small steps.</li>
 <li>Distance too large or the pole is not seen: the radar is aimed too high or too low. Tilt it, then check again.</li>
 <li>The radar measures from its own face, not from the bumper. The offset above is that gap and is remembered in this browser.</li></ol>
</section>
</main>
<script>
var OFF=0;try{OFF=parseFloat(localStorage.getItem('radarOff'))||0}catch(e){}
var S=null,demo=/demo/.test(location.search),HIST=[],SEEN=[],SAVED=[],LO=1.5,HI=8,CL=0.9;       /* search window 1.5 - 8 m (the stock script uses 2.5 - 4.5 m); the pole must be within 0.9 m of the centre line */
function $(i){return document.getElementById(i)}
function f2(x){return (x>=0?'+':'')+x.toFixed(2)}
function pick(){var os=(S&&S.o)||[],best=null;os.forEach(function(o){if(o.d>=LO&&o.d<=HI&&Math.abs(o.y)<=CL&&(o.pe==null||o.pe>=30)){if(!best||Math.abs(o.y)<Math.abs(best.y))best=o}});return best}
function upd(s){S=s;var now=Date.now(),p=pick();try{drv(s,now)}catch(e){}SEEN.push(p?1:0);if(SEEN.length>30)SEEN.shift();
 if(p){HIST.push([now,p.d,p.y])}while(HIST.length&&now-HIST[0][0]>1200)HIST.shift();
 render(p,now)}
function avg(a){var s=0;a.forEach(function(x){s+=x});return a.length?s/a.length:null}
function render(p,now){var dr=avg(HIST.map(function(h){return h[1]})),dy=avg(HIST.map(function(h){return h[2]})),n=HIST.length;
 var live=HIST.length&&now-HIST[HIST.length-1][0]<1200;
 var db=live?dr-OFF:null;$('dy').textContent=live?f2(dy):'-';$('dr').textContent=live?db.toFixed(2):'-';$('rd').textContent=live?'radar reads '+dr.toFixed(2)+' m':'';
 var gm=$('gm'),ok=live&&Math.abs(dy)<=.05;gm.style.left=(live?50+Math.max(-50,Math.min(50,dy/.5*50)):50)+'%';gm.style.background=!live?'#566':(ok?'var(--ok)':(Math.abs(dy)<.15?'var(--warn)':'var(--bad)'));
 var h='';if(!live)h='<span class=pill style="background:#3a2a1a;color:#f0a020">No pole seen</span> between '+LO+' and '+HI+' m near the centre line. Place the pole, or tilt the radar if it is not picked up.';
 else if(ok)h='<span class=pill style="background:#14351f;color:#2fbf71">On the centre line</span> dY within 5 cm.';
 else h='<span class=pill style="background:#3a1a1c;color:#e5484d">Off the line</span> pole reads '+(dy>0?'LEFT':'RIGHT')+' by '+Math.abs(dy*100).toFixed(0)+' cm.';
 $('hint').innerHTML=h;
 var hits=SEEN.reduce(function(a,b){return a+b},0);$('q').textContent=live?('seen in '+hits+' of the last '+SEEN.length+' frames'+(n>3?' - jitter '+(Math.sqrt(avg(HIST.map(function(x){return (x[2]-dy)*(x[2]-dy)})))*100).toFixed(1)+' cm':'')+(p&&p.pe!=null?' - exists '+p.pe+'%':'')):'';
 var m=parseFloat($('md').value);$('dd').textContent=live&&!isNaN(m)?f2(db-m)+' m':'-';
 /* all tracks table */
 var os=((S&&S.o)||[]).filter(function(o){return o.d>=1&&o.d<=8&&Math.abs(o.y)<4}).sort(function(a,b){return a.d-b.d}).slice(0,14);
 $('tt').tBodies[0].innerHTML=os.map(function(o){return '<tr><td>'+(o.s!=null?o.s:'')+'</td><td>'+o.d.toFixed(2)+'</td><td>'+f2(o.y)+'</td><td>'+(o.stationary?'still':Math.round((o.gs||0)*3.6)+' km/h')+'</td><td>'+(o.pe!=null?o.pe:'')+'</td></tr>'}).join('')||'<tr><td colspan=5 style="text-align:center;color:var(--mut)">nothing in range</td></tr>';
 plot(os,p)}
function plot(os,p){var c=$('cv'),W=c.clientWidth,H=c.clientHeight,d=devicePixelRatio||1;c.width=W*d;c.height=H*d;var g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);
 var sx=W/6,sy=(H-20)/8,cx=W/2,y0=H-4;                                             /* +-3 m across, 0 - 8 m up; the radar is at the bottom centre */
 g.fillStyle='#0f1216';g.fillRect(0,0,W,H);g.strokeStyle='#222a32';g.fillStyle='#667';g.font='11px system-ui';g.lineWidth=1;
 for(var m=1;m<=8;m++){g.beginPath();g.moveTo(0,y0-m*sy);g.lineTo(W,y0-m*sy);g.stroke();g.fillText(m+' m',4,y0-m*sy-3)}
 for(var x=-3;x<=3;x++){g.beginPath();g.moveTo(cx+x*sx,y0);g.lineTo(cx+x*sx,0);g.stroke()}
 g.fillStyle='rgba(47,191,113,.12)';g.fillRect(cx-.05*sx,y0-HI*sy,.1*sx,(HI-LO)*sy);                 /* +-5 cm zone along the target window */
 g.strokeStyle='#2fbf71';g.lineWidth=1.5;g.setLineDash([6,5]);g.beginPath();g.moveTo(cx,y0);g.lineTo(cx,0);g.stroke();g.setLineDash([]);
 g.strokeStyle='rgba(240,160,32,.6)';[LO,HI].forEach(function(r){g.beginPath();g.moveTo(cx-3*sx,y0-r*sy);g.lineTo(cx+3*sx,y0-r*sy);g.stroke()});
 g.fillStyle='#cfd6dd';g.beginPath();g.moveTo(cx,y0-8);g.lineTo(cx-10,y0+2);g.lineTo(cx+10,y0+2);g.closePath();g.fill();
 os.forEach(function(o){var x=cx-o.y*sx,y=y0-o.d*sy,isp=p&&o.s===p.s&&o.d===p.d;g.fillStyle=isp?'#2fbf71':'#7d8691';g.beginPath();g.arc(x,y,isp?8:5,0,6.3);g.fill();if(isp){g.strokeStyle='#fff';g.lineWidth=2;g.stroke()}})}
$('cap').onclick=function(){var live=HIST.length&&Date.now()-HIST[HIST.length-1][0]<1200;if(!live)return;var dr=avg(HIST.map(function(h){return h[1]})),dy=avg(HIST.map(function(h){return h[2]})),m=parseFloat($('md').value);
 SAVED.push({dr:dr-OFF,dy:dy,m:isNaN(m)?null:m});showSaved()};
$('setoff').onclick=function(){var live=HIST.length&&Date.now()-HIST[HIST.length-1][0]<1200,m=parseFloat($('md').value);if(!live||isNaN(m))return;OFF=avg(HIST.map(function(h){return h[1]}))-m;$('off').value=OFF.toFixed(2);try{localStorage.setItem('radarOff',OFF)}catch(e){}};
$('off').onchange=function(){OFF=parseFloat(this.value)||0;try{localStorage.setItem('radarOff',OFF)}catch(e){}};
$('off').value=OFF.toFixed(2);
$('clr').onclick=function(){SAVED=[];showSaved()};
function showSaved(){$('ct').tBodies[0].innerHTML=SAVED.map(function(r,i){return '<tr><td>'+(i+1)+'</td><td>'+r.dr.toFixed(2)+'</td><td style="color:'+(Math.abs(r.dy)<=.05?'var(--ok)':'var(--bad)')+'">'+f2(r.dy)+'</td><td>'+(r.m!=null?r.m.toFixed(2):'')+'</td><td>'+(r.m!=null?f2(r.dr-r.m):'')+'</td></tr>'}).join('');
 var mx=SAVED.reduce(function(a,r){return Math.max(a,Math.abs(r.dy))},0);$('cs').textContent=SAVED.length?('largest |dY| '+(mx*100).toFixed(0)+' cm over '+SAVED.length+' readings - '+(mx<=.05?'calibrated within 5 cm':'keep adjusting')):''}
function demoState(t){var d=3.5+Math.sin(t/4)*.6,y=.18*Math.sin(t/6)+.03;return {v:0,o:[{s:7,d:d,y:y,pe:96,vr:0,gs:0,stationary:true},{s:3,d:6.2,y:-1.6,pe:80,vr:0,gs:0,stationary:true}]}}
if(demo){$('conn').textContent='demo data';var t0=Date.now();(function loop(){upd(demoState((Date.now()-t0)/1000));setTimeout(loop,100)})()}
else{var es=new EventSource('/api/stream');es.onmessage=function(e){$('conn').textContent='live';upd(JSON.parse(e.data))};es.onerror=function(){$('conn').textContent='disconnected, retrying…'}}
setInterval(function(){if(S)render(pick(),Date.now())},400);onresize=function(){if(S)render(pick(),Date.now())};

/* ---- driving check: least-squares fit of the lead car's sideways position against its range ---- */
var DRV={pts:[],last:{},cam:false};try{var _d=JSON.parse(localStorage.getItem('radarDrv')||'null');if(_d&&_d.pts)DRV.pts=_d.pts}catch(e){}
function pvd(p,x){return ((p[0]*x+p[1])*x+p[2])*x+p[3]}
function drv(s,now){var v=s.v||0;if(v<11||Math.abs(s.ang||0)>4)return;var m=s.model||{},pp=m.p||[0,0,0,0];if(Math.abs(pp[1])>.0004||Math.abs(pp[2])>.02)return;
 var l=s.lead;if(!l||l.d<15||l.d>80)return;var o=null;(s.o||[]).forEach(function(x){if(Math.abs(x.d-l.d)<2&&Math.abs(x.y-l.y)<.6)o=x});if(!o||o.stationary||(o.gs||0)<4||(o.pe!=null&&o.pe<80))return;
 var cam=m.l&&m.r&&m.pl>.5&&m.pr>.5,cc=cam?(pvd(m.l,l.d)+pvd(m.r,l.d))/2:0;DRV.cam=!!cam;
 var dy=l.y-cc;if(Math.abs(dy)>2.2)return;if(DRV.last[o.s]&&now-DRV.last[o.s]<500)return;DRV.last[o.s]=now;
 DRV.pts.push([+l.d.toFixed(1),+dy.toFixed(2)]);if(DRV.pts.length>1500)DRV.pts.shift();if(DRV.pts.length%25==0){try{localStorage.setItem('radarDrv',JSON.stringify({pts:DRV.pts}))}catch(e){}}}
function fit(pts){var P=pts.slice();for(var it=0;it<3;it++){var n=P.length;if(n<8)return null;var sx=0,sy=0,sxx=0,sxy=0;P.forEach(function(p){sx+=p[0];sy+=p[1];sxx+=p[0]*p[0];sxy+=p[0]*p[1]});var den=n*sxx-sx*sx;if(Math.abs(den)<1e-6)return null;var b=(n*sxy-sx*sy)/den,a=(sy-b*sx)/n,r=P.map(function(p){return p[1]-a-b*p[0]}),sd=Math.sqrt(r.reduce(function(q,x){return q+x*x},0)/n);
  if(it<2)P=P.filter(function(p,i){return Math.abs(r[i])<2.5*sd+.05});else{return {a:a,b:b,sd:sd,n:n,se:sd/Math.sqrt(Math.max(1,(sxx-sx*sx/n)))}}}}
function drvRender(){var f=fit(DRV.pts),n=DRV.pts.length,dmin=99,dmax=0;DRV.pts.forEach(function(p){if(p[0]<dmin)dmin=p[0];if(p[0]>dmax)dmax=p[0]});$('dn').textContent=n;$('dns').textContent='samples - range spread '+(n?Math.round(dmax-dmin):0)+' m'+(DRV.cam?' - lane centre':' - car centre line');
 var ok=f&&n>=150&&dmax-dmin>=25;if(!f){$('dg').textContent='-';$('dofs').textContent='-';$('dv').innerHTML='<span class=pill style="background:#242a31;color:#9aa3ad">Collecting</span>';$('dh').textContent='Drive above 40 km/h behind another car on a straight road.'}
 else{var ang=Math.atan(f.b)*180/Math.PI,seA=Math.atan(f.se)*180/Math.PI;$('dg').textContent=(ang>=0?'+':'')+ang.toFixed(2)+'\u00b0';$('dofs').textContent=f2(f.a);
  var aa=Math.abs(ang),verdict=!ok?['Not enough data yet','#242a31','#9aa3ad']:(aa<.3?['Radar is straight','#14351f','#2fbf71']:(aa<.8?['Slightly turned','#3a2f12','#f0a020']:['Turned - adjust','#3a1a1c','#e5484d']));
  $('dv').innerHTML='<span class=pill style="background:'+verdict[1]+';color:'+verdict[2]+'">'+verdict[0]+'</span>';
  $('dh').textContent=(ok&&aa>=.3?'Turn the radar '+(ang>0?'LEFT':'RIGHT')+' by about '+aa.toFixed(1)+'\u00b0 ('+(Math.tan(aa*Math.PI/180)*100).toFixed(1)+' mm of shift per 10 cm of lever) and drive again. ':'')+'Uncertainty +-'+(seA*2).toFixed(2)+'\u00b0; spread '+(f.sd*100).toFixed(0)+' cm.'}
 var c=$('dcv'),W=c.clientWidth,H=c.clientHeight,d=devicePixelRatio||1;c.width=W*d;c.height=H*d;var g=c.getContext('2d');g.setTransform(d,0,0,d,0,0);g.fillStyle='#0f1216';g.fillRect(0,0,W,H);
 var X=function(x){return 40+(x-10)/80*(W-50)},Y=function(y){return H/2-y/2.5*(H/2-14)};g.strokeStyle='#222a32';g.fillStyle='#667';g.font='11px system-ui';g.lineWidth=1;[-2,-1,0,1,2].forEach(function(y){g.beginPath();g.moveTo(40,Y(y));g.lineTo(W-10,Y(y));g.stroke();g.fillText(y+' m',4,Y(y)+4)});[20,40,60,80].forEach(function(x){g.fillText(x+' m',X(x)-12,H-3)});
 g.fillStyle='rgba(90,160,255,.55)';DRV.pts.forEach(function(p){g.beginPath();g.arc(X(p[0]),Y(p[1]),2.4,0,6.3);g.fill()});
 if(f){g.strokeStyle='#f0a020';g.lineWidth=2;g.beginPath();g.moveTo(X(10),Y(f.a+f.b*10));g.lineTo(X(90),Y(f.a+f.b*90));g.stroke()}}
$('dreset').onclick=function(){DRV.pts=[];try{localStorage.removeItem('radarDrv')}catch(e){}drvRender()};
setInterval(drvRender,1000);
</script></body></html>"""

TOP = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Radar live</title><style>
:root{--bg:#101215;--fg:#e6e8ea;--mut:#8b9199;--road:#1b1e22}
@media(prefers-color-scheme:light){:root{--bg:#f3f4f6;--fg:#1c1f23;--mut:#5b626b;--road:#dfe2e6}}
html,body{margin:0;height:100%;display:flex;flex-direction:column;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
#top{display:flex;gap:16px;align-items:baseline;padding:8px 16px;flex-wrap:wrap}#top b{font-size:22px}.m{color:var(--mut)}
#c{display:block;width:100%;flex:1;min-height:0;touch-action:none}
#bot{padding:6px 16px;display:flex;gap:14px;flex-wrap:wrap;align-items:center}
.k{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px}button{font:inherit;padding:4px 10px;border-radius:8px;border:1px solid var(--mut);background:none;color:var(--fg)}
#labelbar{display:flex;gap:6px;flex-wrap:wrap;padding:4px 16px 8px}#labelbar button{border:0;border-radius:14px;padding:6px 11px;font:600 12px system-ui,sans-serif;color:#fff}
#labeltoast{padding:0 16px;font:600 12px system-ui,sans-serif;color:var(--mut);min-height:16px}
</style></head><body>
<div id=top><a href="/" style="color:inherit">road view</a><a href="/learn" style="color:inherit">label objects</a><span><b id=v>0</b> <span class=m>km/h</span></span><span class=m>steer <span id=ang>0</span>°</span><span id=op class=m></span><span id=n class=m></span><span id=al class=m></span><span id=conn class=m>connecting…</span></div>
<canvas id=c></canvas>
<div id=labelbar>
 <button style="background:#1a73e8" onclick="return mark('LABEL pedestrian')">Pedestrian</button>
 <button style="background:#e5484d" onclick="return mark('LABEL vehicle')">Vehicle</button>
 <button style="background:#8e44ad" onclick="return mark('LABEL two_wheel')">Two-wheel</button>
 <button style="background:#f08a00" onclick="return mark('LABEL obstacle')">Obstacle</button>
 <button style="background:#2fbf71" onclick="return mark('LABEL animal')">Animal</button>
 <button style="background:#7d8691" onclick="return mark('LABEL missed')">Missed!</button>
 <button style="background:#444b54" onclick="return mark('LABEL false_positive')">False+</button>
</div>
<div id=labeltoast></div>
<div id=bot><button id=zi>zoom +</button><button id=zo>zoom −</button><span class=m id=rng></span>
<span><span class=k style="background:#2a8bf2"></span>car</span><span><span class=k style="background:#8e44ad"></span>2-wheel</span><span><span class=k style="background:#e5484d"></span>pedestrian</span>
<span><span class=k style="background:#f59e0b"></span>construction</span><span><span class=k style="background:#9aa0a6"></span>unknown</span><span class=m>hollow = stationary</span></div>
<script>
/* LABELBTN: same /api/label endpoint the road view uses - see its do_POST handler server-side */
function mark(txt){var t=document.getElementById('labeltoast');
 fetch('/api/label',{method:'POST',headers:{'Content-Type':'text/plain'},body:txt}).then(function(r){return r.json()}).then(function(j){
  t.textContent=j.ok?('Marked: '+txt.replace('LABEL ','')):'Mark failed (no recording running?)';
  clearTimeout(window._mt);window._mt=setTimeout(function(){t.textContent=''},1800)
 }).catch(function(){t.textContent='Mark failed (offline?)'});
 if(navigator.vibrate)navigator.vibrate(30);return false}
var COL={four_wheel:'#2a8bf2',two_wheel:'#8e44ad',pedestrian:'#e5484d',construction:'#f59e0b',unknown:'#9aa0a6'},R=80,S=null;
var cv=document.getElementById('c'),g=cv.getContext('2d');
function $(i){return document.getElementById(i)}
function draw(){var w=cv.clientWidth,h=cv.clientHeight,d=devicePixelRatio||1;cv.width=w*d;cv.height=h*d;g.setTransform(d,0,0,d,0,0);
 var cs=getComputedStyle(document.body),fg=cs.color,x0=w/2,y0=h-30,sy=(h-50)/R,sx=Math.min(sy*1.2,w/36);
 g.fillStyle=getComputedStyle(document.documentElement).getPropertyValue('--road');g.fillRect(x0-1.9*sx*1.0,0,3.8*sx,y0);
 g.strokeStyle=fg;g.globalAlpha=.25;g.lineWidth=1;g.font='11px system-ui';g.fillStyle=fg;
 for(var m=10;m<=R;m+=10){var y=y0-m*sy;g.beginPath();g.moveTo(0,y);g.lineTo(w,y);g.stroke();g.globalAlpha=.5;g.fillText(m+' m',4,y-2);g.globalAlpha=.25}
 for(var l=-10;l<=10;l+=5){g.beginPath();g.moveTo(x0-l*sx,y0);g.lineTo(x0-l*sx,0);g.stroke()}g.globalAlpha=1;
 g.fillStyle='#2a8bf2';g.fillRect(x0-.9*sx,y0-1,1.8*sx,3.5*sy);
 if(!S)return;
 S.o.forEach(function(o){if(o.d>R)return;var x=x0-o.y*sx,y=y0-o.d*sy,r=Math.max(4,Math.min(12,(o.len||1)*sx*.5+3)),c=COL[o.c]||'#9aa0a6';
  g.beginPath();g.arc(x,y,r,0,6.3);if(o.stationary){g.lineWidth=2;g.strokeStyle=c;g.stroke()}else{g.fillStyle=c;g.fill()}
  if(S.lead&&Math.abs(S.lead.d-o.d)<2&&Math.abs(S.lead.y-o.y)<1){g.lineWidth=3;g.strokeStyle='#e5484d';g.beginPath();g.arc(x,y,r+4,0,6.3);g.stroke()}
  if(o.d<R*.6){g.fillStyle=fg;g.font='11px system-ui';g.fillText(Math.round(o.d)+'m '+(o.stationary?'':Math.round(o.gs*3.6)+'k'),x+r+3,y+4)}});}
function upd(s){S=s;$('v').textContent=Math.round(s.v*3.6);$('ang').textContent=s.ang;$('op').textContent=s.op?'● engaged':'○ not engaged';
 $('n').textContent=s.o.length+' objects';var a=s.status&&s.status.alerts&&s.status.alerts.length?'radar alerts: '+s.status.alerts.join(', '):'';$('al').textContent=a;draw()}
$('zi').onclick=function(){R=Math.max(20,R-20);$('rng').textContent=R+' m';draw()};$('zo').onclick=function(){R=Math.min(200,R+20);$('rng').textContent=R+' m';draw()};
$('rng').textContent=R+' m';onresize=draw;
var es=new EventSource('/api/stream');es.onmessage=function(e){$('conn').textContent='live';upd(JSON.parse(e.data))};es.onerror=function(){$('conn').textContent='disconnected, retrying…'};
</script></body></html>"""

# LEARN: top-down view where you tap a specific object, then tap what it actually is. Unlike the plain
# flash/button marks (a timestamp only), this writes the exact track (slot, distance, lateral offset,
# velocity, length, what the radar itself guessed) paired with your label - a real (features, label) row,
# straight to LEARN_FILE as JSONL. It also drops a plain-text mark (same mechanism as /api/label) so the
# moment still shows up as an alert banner on replay.
# ML: what the classifier learned - model scores, label counts (yours / automatic), recent labels, live objects with
# the model's guess. Polls /api/ml every 2 s.
MLPAGE = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Radar learning</title><style>
:root{--bg:#101215;--fg:#e6e8ea;--mut:#8b9199;--card:#1b1e22;--line:#2a2e34;--ok:#3fb950;--warn:#d29922;--bar:#2a8bf2}
@media(prefers-color-scheme:light){:root{--bg:#f3f4f6;--fg:#1c1f23;--mut:#5b626b;--card:#fff;--line:#e1e4e8;--ok:#1a7f37;--warn:#9a6700;--bar:#1a73e8}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
#top{display:flex;gap:16px;padding:10px 16px;flex-wrap:wrap}#top a{color:inherit}
main{display:grid;gap:12px;padding:0 16px 24px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
section{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;min-width:0}
h2{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--mut);margin:0 0 10px}
.big{font-size:28px;font-weight:700}.m{color:var(--mut)}.ok{color:var(--ok)}.warn{color:var(--warn)}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}td,th{padding:4px 6px;text-align:left;border-bottom:1px solid var(--line)}
th{color:var(--mut);font-weight:600;font-size:12px}td.n{text-align:right}
.bar{height:8px;background:var(--bar);border-radius:4px;min-width:2px}
.wrap{overflow-x:auto}
button{font:inherit;padding:6px 12px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--fg);cursor:pointer}
.cmh{text-align:center}td.cmc{text-align:center}
</style></head><body>
<div id=top><a href="/">road view</a><a href="/top">top-down</a><a href="/learn">label objects</a><a href="/trips">trips map</a><b>What it learned</b><span id=conn class=m>loading…</span></div>
<main>
<section><h2>Model</h2><div id=model></div></section>
<section><h2>Training</h2><div id=train></div></section>
<section><h2>Labels per class</h2><div id=labels></div></section>
<section style="grid-column:1/-1"><h2>Models compared (cross-validated on drives it never trained on)</h2><div class=wrap id=cmp></div></section>
<section><h2>Per-class F1</h2><div class=wrap id=pcf></div></section>
<section style="grid-column:1/-1"><h2>Confusion matrix (rows = label, columns = model)</h2><div class=wrap id=cm></div></section>
<section style="grid-column:1/-1"><h2>Live objects - radar vs model</h2><div class=wrap id=objs></div></section>
<section style="grid-column:1/-1"><h2>Latest labels</h2><div class=wrap id=recent></div></section>
</main>
<script>
var $=function(i){return document.getElementById(i)};
function esc(x){return String(x==null?'':x).replace(/[&<>"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]})}
function pct(x){return x==null?'-':Math.round(x*100)+'%'}
function ago(t){if(!t)return '-';var s=Date.now()/1000-t;return s<90?Math.round(s)+' s ago':s<5400?Math.round(s/60)+' min ago':s<172800?Math.round(s/3600)+' h ago':Math.round(s/86400)+' days ago'}
function trainNow(){var b=$('tn');b.disabled=true;fetch('/api/train',{method:'POST'}).then(function(r){return r.json()}).then(function(j){b.textContent=j.ok?'training started':'already training';poll()}).catch(function(){b.disabled=false})}
var NAMES={rf:'Random Forest (INFRA3DRC)',mlp:'Neural net 2x10 (Ham et al.)',extratrees5:'Extra trees (depth 5)',extratrees8:'Extra trees (depth 8)'};
function render(j){
 var m=j.model;
 $('model').innerHTML=m?('<div class=big>'+pct(m.cv_macro_f1)+'</div><div class=m>score (cross-validated macro-F1) vs <b>'+pct(m.baseline_f1)+'</b> without the model</div>'+
  '<p>Trained on <b>'+m.rows+'</b> labels, '+ago(m.trained)+'<br>'+esc(m.algo)+' · '+(m.kind=='mlp'?'layers '+(m.layers||[]).join('-'):m.trees+' trees')+' · overrides the radar when ≥ '+pct(m.min_p)+' sure<br>Classes: '+esc((m.classes||[]).join(', '))+(m.prev?'<br><span class=m>previous model kept as .prev</span>':'')+'</p>')
  :'<div class=big>none yet</div><p class=m>The radar\'s own classes are shown until the first model is trained.</p>';
 var s=j.status||{},l=j.last_training;
 $('train').innerHTML='<p><button id=tn onclick="trainNow()"'+(s.training?' disabled':'')+'>'+(s.training?'training…':'Train now')+'</button></p><p>'+esc(s.msg||'-')+'</p><p>Labels: <b>'+j.total+'</b> · next training at <b>'+j.next_at+'</b> (when parked)<br>Automatic labelling: '+(j.autolabel_on?'<span class=ok>on</span>':'<span class=warn>off</span>')+'</p>'+
  (l?'<p class=m>Last run '+ago(l.t)+': '+esc(l.msg)+'</p>':'');
 var cls={};Object.keys(j.manual).forEach(function(k){cls[k]=1});Object.keys(j.auto).forEach(function(k){cls[k]=1});
 var ks=Object.keys(cls).sort(),mx=1;ks.forEach(function(k){mx=Math.max(mx,(j.manual[k]||0)+(j.auto[k]||0))});
 $('labels').innerHTML=ks.length?'<table><tr><th>class</th><th class=n>yours</th><th class=n>auto</th><th style="width:40%"></th></tr>'+ks.map(function(k){var a=j.manual[k]||0,b=j.auto[k]||0;
  return '<tr><td>'+esc(k)+'</td><td class=n>'+a+'</td><td class=n>'+b+'</td><td><div class=bar style="width:'+(100*(a+b)/mx)+'%"></div></td></tr>'}).join('')+'</table>':'<p class=m>No labels yet. Drive (automatic labels) or tap objects on <a href="/learn">/learn</a>.</p>';

 var cands=(m&&m.candidates)||(l&&l.candidates)||{},base=(m&&m.baseline)||(l&&l.baseline),ck=Object.keys(cands);
 function mrow(name,x,bold){return '<tr'+(bold?' style="font-weight:700"':'')+'><td>'+esc(name)+'</td><td class=n>'+pct(x.accuracy)+'</td><td class=n>'+pct(x.balanced_accuracy)+'</td><td class=n>'+pct(x.macro_f1)+'</td><td class=n>'+pct(x.weighted_f1)+'</td></tr>'}
 $('cmp').innerHTML=ck.length?'<table><tr><th>model</th><th class=n>accuracy</th><th class=n>balanced acc.</th><th class=n>macro-F1</th><th class=n>weighted-F1</th></tr>'+
  ck.map(function(k){return mrow((NAMES[k]||k)+(m&&m.algo&&((k=='rf'&&m.kind=='forest')||(k=='mlp'&&m.kind=='mlp'))?'  (installed)':''),cands[k],m&&((k=='rf'&&m.kind=='forest')||(k=='mlp'&&m.kind=='mlp')))}).join('')+(base?mrow('No model (radar + rules)',base,false):'')+'</table>'
  +'<p class=m>Same four numbers INFRA3DRC-Radar-Classification reports. Most labels come from the automatic rules, so this measures agreement with them; labels you tap on /learn are the real test.</p>':'<p class=m>Shown after the first model v2 training.</p>';
 var mt=m&&m.metrics,cls=(m&&m.classes)||[];
 $('pcf').innerHTML=mt&&mt.f1?'<table><tr><th>class</th><th class=n>model</th><th class=n>no model</th></tr>'+cls.map(function(c){return '<tr><td>'+esc(c)+'</td><td class=n>'+pct(mt.f1[c])+'</td><td class=n>'+(base&&base.f1?pct(base.f1[c]):'-')+'</td></tr>'}).join('')+'</table>':'<p class=m>-</p>';
 $('cm').innerHTML=mt&&mt.confusion?'<table><tr><th></th>'+cls.map(function(c){return '<th class=cmh>'+esc(c)+'</th>'}).join('')+'</tr>'+mt.confusion.map(function(row,i){var t=row.reduce(function(a,b){return a+b},0)||1;
  return '<tr><td>'+esc(cls[i])+'</td>'+row.map(function(v,jj){return '<td class=cmc style="background:rgba(42,139,242,'+(v/t*0.85).toFixed(2)+')">'+v+'</td>'}).join('')+'</tr>'}).join('')+'</table>':'<p class=m>-</p>';
 var o=(j.objects||[]).slice().sort(function(a,b){return a.d-b.d});
 $('objs').innerHTML=o.length?'<table><tr><th>slot</th><th class=n>dist</th><th class=n>lateral</th><th class=n>ground km/h</th><th>radar says</th><th>model says</th><th>shown</th></tr>'+o.map(function(x){
  var ml=x.ml?esc(x.ml.c)+' <span class=m>'+pct(x.ml.p)+'</span>':'<span class=m>-</span>';
  return '<tr><td>'+x.s+'</td><td class=n>'+x.d+' m</td><td class=n>'+x.y+' m</td><td class=n>'+Math.round(x.gs*3.6)+'</td><td>'+esc(x.rc)+' <span class=m>'+(x.pc==null?'':x.pc+'%')+'</span></td><td>'+ml+'</td><td>'+(x.inf?'~ ':'')+esc(x.c)+'</td></tr>'}).join('')+'</table>'
  :'<p class=m>No radar objects right now.</p>';
 var r=(j.recent||[]).slice().reverse();
 $('recent').innerHTML=r.length?'<table><tr><th>when</th><th>label</th><th>by</th><th>radar said</th><th class=n>dist</th><th class=n>ground km/h</th></tr>'+r.map(function(x){
  return '<tr><td>'+ago(x.t)+'</td><td>'+esc(x.c)+'</td><td>'+(x.auto?'auto':'you')+'</td><td>'+esc(x.radar_c)+'</td><td class=n>'+(x.d==null?'-':x.d+' m')+'</td><td class=n>'+(x.gs==null?'-':Math.round(x.gs*3.6))+'</td></tr>'}).join('')+'</table>':'<p class=m>None yet.</p>';
}
function poll(){fetch('/api/ml').then(function(r){return r.json()}).then(function(j){$('conn').textContent='live';render(j)}).catch(function(){$('conn').textContent='disconnected, retrying…'})}
poll();setInterval(poll,2000);
</script></body></html>"""

LEARN = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Label objects</title><style>
:root{--bg:#101215;--fg:#e6e8ea;--mut:#8b9199;--road:#1b1e22}
@media(prefers-color-scheme:light){:root{--bg:#f3f4f6;--fg:#1c1f23;--mut:#5b626b;--road:#dfe2e6}}
html,body{margin:0;height:100%;display:flex;flex-direction:column;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
#top{display:flex;gap:16px;align-items:baseline;padding:8px 16px;flex-wrap:wrap}#top b{font-size:22px}.m{color:var(--mut)}
#c{display:block;width:100%;flex:1;min-height:0;touch-action:none}
#sel{padding:6px 16px;font:600 13px system-ui,sans-serif;min-height:18px}
#pal{display:flex;gap:6px;flex-wrap:wrap;padding:6px 16px 10px}
#pal button{border:0;border-radius:14px;padding:8px 12px;font:600 12px system-ui,sans-serif;color:#fff;opacity:.5}
#pal button.on{opacity:1}
</style></head><body>
<div id=top><a href="/" style="color:inherit">road view</a><a href="/top" style="color:inherit">top-down</a><a href="/ml" style="color:inherit">what it learned</a><span><b id=v>0</b> <span class=m>km/h</span></span><span id=n class=m></span><span id=conn class=m>connecting…</span></div>
<canvas id=c></canvas>
<div id=sel>tap an object below, then tap what it is</div>
<div id=pal></div>
<script>
var COL={four_wheel:'#2a8bf2',two_wheel:'#8e44ad',pedestrian:'#e5484d',construction:'#f59e0b',unknown:'#9aa0a6'},R=60,S=null,SEL=null;
var CATS=[['pedestrian','#1a73e8'],['dog','#8d6e63'],['animal','#6d4c41'],['car','#e5484d'],['truck','#b71c1c'],
 ['two_wheel','#8e44ad'],['bus','#ad1457'],['construction','#f08a00'],['separator','#fbc02d'],['wall','#616161'],
 ['pole','#455a64'],['sign','#00897b'],['obstacle','#f57c00'],['not_real','#263238']];
var pal=document.getElementById('pal');
CATS.forEach(function(c){var b=document.createElement('button');b.textContent=c[0].replace('_',' ');b.style.background=c[1];b.onclick=function(){label(c[0])};pal.appendChild(b)});
var cv=document.getElementById('c'),g=cv.getContext('2d');
function $(i){return document.getElementById(i)}
function geo(){var w=cv.clientWidth,h=cv.clientHeight;return {w:w,h:h,x0:w/2,y0:h-30,sy:(h-50)/R,sx:Math.min((h-50)/R*1.2,w/36)}}
function draw(){var w=cv.clientWidth,h=cv.clientHeight,d=devicePixelRatio||1;cv.width=w*d;cv.height=h*d;g.setTransform(d,0,0,d,0,0);
 var G=geo(),x0=G.x0,y0=G.y0,sx=G.sx,sy=G.sy,cs=getComputedStyle(document.body),fg=cs.color;
 g.fillStyle=getComputedStyle(document.documentElement).getPropertyValue('--road');g.fillRect(x0-1.9*sx,0,3.8*sx,y0);
 g.strokeStyle=fg;g.globalAlpha=.25;g.lineWidth=1;g.font='11px system-ui';g.fillStyle=fg;
 for(var m=10;m<=R;m+=10){var y=y0-m*sy;g.beginPath();g.moveTo(0,y);g.lineTo(w,y);g.stroke();g.globalAlpha=.5;g.fillText(m+' m',4,y-2);g.globalAlpha=.25}
 g.globalAlpha=1;g.fillStyle='#2a8bf2';g.fillRect(x0-.9*sx,y0-1,1.8*sx,3.5*sy);
 if(!S)return;
 S.o.forEach(function(o){if(o.d>R)return;var x=x0-o.y*sx,y=y0-o.d*sy,r=Math.max(9,Math.min(18,(o.len||1)*sx*.5+6)),c=COL[o.c]||'#9aa0a6',isSel=SEL&&SEL.s===o.s;
  g.beginPath();g.arc(x,y,r,0,6.3);g.fillStyle=c;g.globalAlpha=isSel?1:.85;g.fill();g.globalAlpha=1;
  if(isSel){g.lineWidth=3;g.strokeStyle='#fff';g.beginPath();g.arc(x,y,r+5,0,6.3);g.stroke()}
  g.fillStyle=fg;g.font=(isSel?'700 ':'')+'11px system-ui';g.fillText(Math.round(o.d)+'m',x+r+3,y+4)});}
function nearest(px,py){var G=geo(),best=null,bd=1e9;(S?S.o:[]).forEach(function(o){if(o.d>R)return;var x=G.x0-o.y*G.sx,y=G.y0-o.d*G.sy,dd=(x-px)*(x-px)+(y-py)*(y-py);if(dd<bd){bd=dd;best=o}});
 return (best&&bd<40*40)?best:null}
function showSel(){var e=$('sel');if(!SEL){e.textContent='tap an object below, then tap what it is';return}
 e.textContent='Selected: slot '+SEL.s+' · '+SEL.d+' m · '+SEL.y+' m lateral · radar guess: '+SEL.c+' ('+SEL.pc+'%) - now tap a label'}
cv.addEventListener('click',function(e){var r=cv.getBoundingClientRect(),o=nearest(e.clientX-r.left,e.clientY-r.top);if(o){SEL=o;showSel();draw()}});
cv.addEventListener('touchstart',function(e){var r=cv.getBoundingClientRect(),t=e.touches[0],o=nearest(t.clientX-r.left,t.clientY-r.top);if(o){SEL=o;showSel();draw();e.preventDefault()}},{passive:false});
function label(cat){if(!SEL){$('sel').textContent='tap an object first';return}
 var body=JSON.stringify({s:SEL.s,c:cat,d:SEL.d,y:SEL.y,vr:SEL.vr,gs:SEL.gs,len:SEL.len,radar_c:SEL.rc||SEL.c,radar_pc:SEL.pc,ml:SEL.ml});
 fetch('/api/objlabel',{method:'POST',headers:{'Content-Type':'application/json'},body:body}).then(function(r){return r.json()}).then(function(j){
  $('sel').textContent=j.ok?('Labeled slot '+SEL.s+' as '+cat):'Label failed (no recording running?)';SEL=null;draw();
 }).catch(function(){$('sel').textContent='Label failed (offline?)'});
 if(navigator.vibrate)navigator.vibrate(30)}
function upd(s){S=s;$('v').textContent=Math.round(s.v*3.6);$('n').textContent=s.o.length+' objects'+(s.ml?' · model: '+s.ml.msg+(s.ml.auto?' · auto-labelled: '+Object.keys(s.ml.auto).map(function(k){return k+' '+s.ml.auto[k]}).join(', '):''):'');
 if(SEL){var still=s.o.find(function(o){return o.s===SEL.s});if(still)SEL=still;else{SEL=null;$('sel').textContent='track lost, selection cleared'}}
 draw()}
onresize=draw;
var es=new EventSource('/api/stream');es.onmessage=function(e){$('conn').textContent='live';upd(JSON.parse(e.data))};es.onerror=function(){$('conn').textContent='disconnected, retrying…'};
</script></body></html>"""

NEAR = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Surround view</title><style>
html,body{margin:0;height:100%;background:#0d0f12;overflow:hidden;font-family:system-ui,Segoe UI,sans-serif}
canvas{display:block;width:100vw;height:100vh}
#nav{position:fixed;right:10px;bottom:8px;z-index:50}#nav summary{list-style:none;cursor:pointer;width:36px;height:36px;display:flex;align-items:center;justify-content:center;background:#1c2128;border:1px solid #30363d;border-radius:8px;font-size:18px;color:#cfd6dd}#nav summary::-webkit-details-marker{display:none}#nav[open] summary{background:#262c35}#nav>div{position:absolute;right:0;bottom:44px;background:#1c2128;border:1px solid #30363d;border-radius:8px;padding:6px 0;min-width:190px;box-shadow:0 8px 24px rgba(0,0,0,.45)}#nav a,#nav>div span{display:block;padding:9px 14px;color:#cfd6dd;text-decoration:none;font-size:13px;white-space:nowrap}#nav a:hover{background:#262c35}
html.fr #nav{display:none}
html.emb,html.emb body{background:transparent}
#scene{position:fixed;inset:0}#gp{position:absolute;inset:0}
.m3{display:none;position:absolute}
html.emb #scene{perspective:2600px;perspective-origin:50% 40%}
html.emb #gp{transform-style:preserve-3d;transform-origin:50% 58%;transform:rotateX(68deg)}
html.emb #gp>*{position:absolute}
html.emb .m3{display:block}
html.emb #roof,html.emb #sl,html.emb #sr{display:none}
#sh{background:rgba(0,0,0,.6);filter:blur(12px);border-radius:40%}
.sd{background:linear-gradient(180deg,#2d353f 0,#3a444f 45%,#232a32 100%);box-shadow:inset 0 0 0 1px rgba(238,242,246,.35)}
</style></head><body><div id=scene><div id=gp><canvas id=c></canvas><div class='m3' id=sh></div><div class='m3 sd' id=sl></div><div class='m3 sd' id=sr></div><img class='m3' id=rear><img class='m3' id=roof></div></div><details id=nav><summary>☰</summary><div><span id=conn style="color:#9aa0a6">connecting…</span><a href="/">road view</a><a href="/top">top-down view</a><a href="?demo=1">demo</a></div></details>
<script>if(window.top!==window)document.documentElement.className='fr'+(/embed=1/.test(location.search)?' emb':'');var EMB=/embed=1/.test(location.search)</script><script>
/* all-round proximity "wave" view, display only. FRONT: the radar (left / centre / right) plus the two front corner parking sensors. REAR: the two rear corner parking sensors. A glowing wave follows the car outline and moves away from it as the distance grows (up to 2.8 m). */
var cv=document.getElementById('c'),g=cv.getContext('2d'),S=null,demo=/demo/.test(location.search),DMAX=4.8,RMAX=6.5;
function rr(x,y,w,h,r){g.beginPath();g.moveTo(x+r,y);g.arcTo(x+w,y,x+w,y+h,r);g.arcTo(x+w,y+h,x,y+h,r);g.arcTo(x,y+h,x,y,r);g.arcTo(x,y,x+w,y,r);g.closePath()}
function pval(P,k,sd){if(!P||!P[k])return null;var v=P[k][sd];return (v==null||v>=250)?null:v/100}
var CAR=new Image();CAR.onload=function(){draw()};CAR.src='/icons/car.webp';
var CARR=new Image();CARR.onload=function(){draw()};CARR.src='/icons/carr.webp';
var PROF=[[0.0,0.08],[0.017,0.508],[0.034,0.667],[0.051,0.789],[0.068,0.887],[0.085,0.938],[0.102,0.981],[0.119,0.99],[0.136,0.995],[0.153,0.999],[0.169,0.999],[0.186,0.999],[0.203,0.999],[0.22,0.999],[0.237,0.999],[0.254,0.995],[0.271,0.981],[0.288,0.971],[0.305,0.971],[0.322,0.971],[0.339,0.971],[0.356,0.967],[0.373,0.967],[0.39,0.967],[0.407,0.967],[0.424,0.962],[0.441,0.962],[0.458,0.962],[0.475,0.962],[0.492,0.962],[0.508,0.962],[0.525,0.962],[0.542,0.962],[0.559,0.957],[0.576,0.957],[0.593,0.957],[0.61,0.957],[0.627,0.957],[0.644,0.957],[0.661,0.967],[0.678,0.976],[0.695,0.985],[0.712,0.995],[0.729,1.004],[0.746,1.009],[0.763,1.009],[0.78,1.009],[0.797,1.009],[0.814,1.009],[0.831,1.009],[0.847,0.999],[0.864,0.99],[0.881,0.971],[0.898,0.938],[0.915,0.91],[0.932,0.873],[0.949,0.826],[0.966,0.732],[0.983,0.55],[1.0,0.08]];
function hwAt(u){if(u<=0)return PROF[0][1];for(var i=1;i<PROF.length;i++)if(u<=PROF[i][0]){var p=PROF[i-1],q=PROF[i];return lerp(p[1],q[1],(u-p[0])/(q[0]-p[0]))}return PROF[PROF.length-1][1]}
function lerp(a,b,t){return a+(b-a)*t}
function colr(d,a){var h=d>=1.6?lerp(50,38,Math.min(1,(d-1.6)/1.2)):(d>=.8?lerp(38,22,(1.6-d)/.8):lerp(22,2,Math.min(1,(.8-d)/.5)));return 'hsla('+h+',100%,'+(d<.5?55:52)+'%,'+a+')'}
function fmt(d){return d<1?Math.round(d*100)+' cm':d.toFixed(2)+' m'}
function draw(){var W=cv.clientWidth,H=cv.clientHeight,dpr=devicePixelRatio||1;cv.width=W*dpr;cv.height=H*dpr;g.setTransform(dpr,0,0,dpr,0,0);
 var u=Math.max(.7,Math.min(1.4,Math.min(W,H)/700)),sc=Math.min(H*.8/10.6,W/8),cx=W/2,cy=H*.54,cw=1.94*sc,cl=4.98*sc,a=cw/2,b=cl/2;
 if(EMB)g.clearRect(0,0,W,H);else{g.fillStyle='#10131a';g.fillRect(0,0,W,H)}
 var P=S&&S.park;
 /* radar objects by sector (front only); y is positive to the left */
 var rl=99,rc=99,rrr=99;((S&&S.o)||[]).forEach(function(o){if(o.d<=0||o.d>RMAX||Math.abs(o.y)>4.5||(o.pe!=null&&o.pe<50))return;if(o.y>.7){if(o.d<rl)rl=o.d}else if(o.y<-.7){if(o.d<rrr)rrr=o.d}else if(o.d<rc)rc=o.d});
 function nn(v){return v>=99||v>DMAX+.3?null:v}
 /* anchors around the car, angles in canvas degrees (0 = right, clockwise, -90 = front, +90 = rear); null = nothing detected */
 var fa=Math.atan2(b,a)*180/Math.PI,ca=180-fa,cb=fa;      /* canvas angle of the car's front corners: -ca (left), -cb (right); rear corners: +ca / +cb */
 var A=[{a:-180,d:null,g:'x'},{a:-ca-30,d:null,g:'x'},{a:-ca,d:pval(P,'f','l'),g:'f'},{a:-ca+(ca-90)*.45,d:nn(rl),g:'f'},{a:-97,d:nn(rc==99?99:rc+.2),g:'f'},{a:-90,d:nn(rc),g:'f'},{a:-83,d:nn(rc==99?99:rc+.2),g:'f'},{a:-cb-(90-cb)*.45,d:nn(rrr),g:'f'},{a:-cb,d:pval(P,'f','r'),g:'f'},{a:-cb+30,d:null,g:'x'},
  {a:cb-30,d:null,g:'x'},{a:cb,d:pval(P,'r','r'),g:'r'},{a:ca,d:pval(P,'r','l'),g:'r'},{a:ca+30,d:null,g:'x'},{a:180,d:null,g:'x'}];
 function dist(th){for(var i=0;i<A.length-1;i++){if(th>=A[i].a&&th<=A[i+1].a){var d0=A[i].d==null?DMAX+1.2:A[i].d,d1=A[i+1].d==null?DMAX+1.2:A[i+1].d,t=(th-A[i].a)/(A[i+1].a-A[i].a);t=t*t*(3-2*t);return lerp(d0,d1,t)}}return DMAX+2}
 var BR=null;function bnd(t){if(!BR){BR=[];for(var k=0;k<=720;k++){var tt=k/720*2*Math.PI-Math.PI,lo=0,hi=Math.max(a,b)*1.2;for(var n=0;n<18;n++){var m=(lo+hi)/2,x=Math.abs(Math.cos(tt))*m,y=Math.sin(tt)*m,uu=(y+b)/(2*b);(uu>=0&&uu<=1&&x<=hwAt(uu)*sc)?lo=m:hi=m}BR.push(lo)}}var f=(t+Math.PI)/(2*Math.PI)*720,i0=Math.max(0,Math.min(719,Math.floor(f)));return lerp(BR[i0],BR[i0+1],f-i0)}      /* real Model S outline (traced from the blueprint) */
 function sdist(th){var t=0,n=0;for(var k=-20;k<=20;k+=2){var w=1-Math.abs(k)/24;t+=dist(Math.max(-180,Math.min(180,th+k)))*w;n+=w}return t/n}      /* smoothed so the wave is one soft curve */
 /* car body */
 if(EMB){if(CAR.naturalWidth){var iw=cl*CAR.naturalWidth/CAR.naturalHeight,rh=iw/1.423,Hh=rh*.97,E=function(id,l,t,w,h,tr,o){var e=document.getElementById(id);e.style.left=l+'px';e.style.top=t+'px';e.style.width=w+'px';e.style.height=h+'px';e.style.transform=tr;e.style.transformOrigin=o||'50% 50%';return e};
   var roof=E('roof',cx-iw/2,cy-b,iw,cl,'translateZ('+Hh+'px)');if(!roof.src)roof.src=CAR.src;
   var rear=E('rear',cx-iw/2,cy+b-rh,iw,rh,'rotateX(-90deg)','50% 100%');if(!rear.src)rear.src=CARR.src;
   E('sh',cx-iw*.48,cy+b-cl*.14,iw*.96,cl*.2,'');
   E('sl',cx-a*.97-Hh/2,cy-b*.92,Hh,cl*.92,'rotateY(90deg)');E('sr',cx+a*.97-Hh/2,cy-b*.92,Hh,cl*.92,'rotateY(90deg)')}}
 else if(CAR.complete&&CAR.naturalWidth){var iw=cl*CAR.naturalWidth/CAR.naturalHeight;g.shadowColor='rgba(0,0,0,.6)';g.shadowBlur=18*u;g.drawImage(CAR,cx-iw/2,cy-b,iw,cl);g.shadowBlur=0}
 /* the wave: one smooth band per run of detected angles, drawn as layered glow + a thin bright edge */
 var pulse=.75+.25*Math.sin(Date.now()/150),runs=[],cur=null;
 function pt(th){var d=sdist(th),t=th*Math.PI/180,r=bnd(t)+.16*sc+Math.min(d,DMAX+.9)*sc*.2;return {x:cx+Math.cos(t)*r,y:cy+Math.sin(t)*r,d:d,th:th}}
 for(var th=-180;th<=180;th+=1){var p=pt(th);if(p.d<DMAX+.9){if(!cur){cur=[];runs.push(cur)}cur.push(p)}else cur=null}
 g.lineCap='round';g.lineJoin='round';
 runs.forEach(function(run){if(run.length<2)return;var mn=99;run.forEach(function(p){if(p.d<mn)mn=p.d});var k=mn<.4?pulse:1;
  function path(){g.beginPath();run.forEach(function(p,i){i?g.lineTo(p.x,p.y):g.moveTo(p.x,p.y)})}
  var wide=(1.35-Math.min(1,mn/DMAX)*.6);
  [[64,.07],[46,.11],[30,.17],[18,.26]].forEach(function(L){g.lineWidth=L[0]*u*wide;g.strokeStyle=colr(mn,L[1]*k);path();g.stroke()});
  g.lineWidth=6*u;for(var i=1;i<run.length;i++){var a0=run[i-1],a1=run[i],dd=(a0.d+a1.d)/2;g.strokeStyle=colr(dd,Math.min(1,k*(1.1-dd/(DMAX*1.4))));g.beginPath();g.moveTo(a0.x,a0.y);g.lineTo(a1.x,a1.y);g.stroke()}});
 /* distance labels at the nearest point of the front and of the rear wave ("Stop" when about to touch) */
 [['f'],['r']].forEach(function(grp){var best=null;A.forEach(function(p){if(p.g==grp[0]&&p.d!=null&&p.d<DMAX&&(!best||p.d<best.d))best=p});if(!best)return;
  var t=best.a*Math.PI/180,r=bnd(t)+.16*sc+Math.min(best.d,DMAX)*sc*.2+50*u,x=cx+Math.cos(t)*r,y=cy+Math.sin(t)*r,txt=best.d<.3?'Stop':fmt(best.d);g.font='700 '+Math.round(26*u)+'px system-ui';g.textAlign='center';g.lineWidth=5;g.strokeStyle='rgba(0,0,0,.75)';g.strokeText(txt,x,y);g.fillStyle='#fff';g.fillText(txt,x,y)});
 g.textAlign='center';g.font='500 '+Math.round(16*u)+'px system-ui';g.fillStyle=EMB?'rgba(0,0,0,0)':'rgba(255,255,255,.7)';g.fillText(S?Math.round(S.v*3.6)+' km/h':'',W/2,H*.06);
 var any=A.some(function(p){return p.d!=null&&p.d<DMAX});if(!any&&!EMB){g.font='600 '+Math.round(22*u)+'px system-ui';g.fillStyle='rgba(255,255,255,.55)';g.fillText('clear',W/2,H*.12)}
 g.textAlign='left';g.font='500 '+Math.round(13*u)+'px system-ui';g.fillStyle=EMB?'rgba(0,0,0,0)':'rgba(255,255,255,.6)';g.fillText(P?'front: radar + corner sensors - rear: corner sensors (decode not yet verified) - display only':'front: radar only - no parking-sensor data - display only',14,H-40)}
function upd(s){S=s;draw()}
function demoState(t){return {v:0.6,park:{f:{l:Math.round(70+(Math.sin(t/2.5)+1)*60),r:255,ss:1},r:{l:Math.round(45+(Math.sin(t/1.7)+1)*100),r:Math.round(90+(Math.sin(t/2.1+1)+1)*90),ss:1}},o:[{d:1.2+(Math.sin(t/2)+1)*1.2,y:.1,pe:90}]}}
if(demo){document.getElementById('conn').textContent='demo data';var t0=Date.now();(function loop(){upd(demoState((Date.now()-t0)/1000));setTimeout(loop,60)})()}
else{var es=new EventSource('/api/stream');es.onmessage=function(e){document.getElementById('conn').textContent='live';upd(JSON.parse(e.data))};es.onerror=function(){document.getElementById('conn').textContent='disconnected, retrying…'}}
setInterval(function(){if(S&&!demo)draw()},100);onresize=draw;draw();
</script></body></html>
"""



MAPP = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>Tesla CAN data</title><style>
:root{--bg:#101215;--card:#1a1d21;--fg:#e6e8ea;--mut:#8b9199;--ok:#2fb36b;--bd:#2a2e34;--warn:#d9822b;--acc:#3b82f6}
@media(prefers-color-scheme:light){:root{--bg:#f3f4f6;--card:#fff;--fg:#1c1f23;--mut:#5b626b;--bd:#d9dce0}}
body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{display:flex;gap:14px;align-items:center;flex-wrap:wrap;padding:10px 16px;border-bottom:1px solid var(--bd)}header a{color:var(--mut)}
.chipb{padding:3px 10px;border-radius:99px;border:1px solid #4b5563;background:transparent;color:inherit;cursor:pointer}.chip{padding:3px 12px;border-radius:99px;background:#4b5563;color:#fff;font-weight:700}.chip.on{background:var(--ok)}
nav{display:flex;gap:6px;padding:8px 16px;flex-wrap:wrap;border-bottom:1px solid var(--bd)}
nav button{font:inherit;padding:6px 14px;border-radius:99px;border:1px solid var(--bd);background:none;color:var(--fg);cursor:pointer}nav button.on{background:var(--acc);border-color:var(--acc);color:#fff}
nav small{opacity:.7;margin-left:4px}
main{display:grid;grid-template-columns:repeat(auto-fill,minmax(330px,1fr));gap:12px;padding:12px 16px}
section{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:10px 14px}
h2{margin:0 0 6px;font-size:13px;letter-spacing:.04em;color:var(--mut)}h2 small{float:right;font-weight:400}
table{width:100%;border-collapse:collapse}td{padding:2px 0;vertical-align:top}td:last-child{text-align:right;font-variant-numeric:tabular-nums;word-break:break-all}td:first-child{color:var(--mut)}
.mono{font-family:ui-monospace,Menlo,monospace;font-size:12px}input{font:inherit;padding:4px 8px;border-radius:8px;border:1px solid var(--bd);background:var(--bg);color:var(--fg);width:90%;margin-bottom:6px}
.big{font-size:30px;font-weight:700}.wide{grid-column:1/-1}.stale h2{color:var(--warn)}.empty{color:var(--mut);padding:16px}
</style></head><body>
<header><b>Tesla CAN data</b><span id=v>0 km/h</span><span class=chip id=acc>ACC</span><span class=chip id=alc>ALC</span><span id=conn style="color:var(--mut)">connecting...</span>
<a href="/">road view</a><a href="/video">video</a><a href="?demo=1">demo</a><span style="color:var(--mut)" id=logn></span></header>
<nav id=nav></nav>
<main id=m></main>
<script>
var demo=/demo=1/.test(location.search),D=null,TAB='map',Q='';
var TABS=[['map','Map / nav'],['pt','Powertrain & chassis'],['gw','Gateway & config'],['das','Driver assist'],['body','Body & seat'],['all','All CAN & openpilot']];
function cat(n){n=n.replace(/^(PT_|M3_)/,'');
 if(/^(UI_driverAssistMapData|UI_driverAssistRoadSign|UI_gps|MCU_location|UI_csa|UI_roadCurv|UI_radarMapData)/.test(n))return 'map';
 if(/^(DI_|ESP_|EPB_|EPAS|SBW_|BrakeMessage|STW_ANGL)/.test(n))return 'pt';
 if(/^(GTW_|MCU_chassis|MCU_cluster|UI_status|UI_autopilotControl|UI_driverAssistControl|UI_telemetry|UI_driverAssistAnon|UI_solar)/.test(n))return 'gw';
 if(/^(DAS_|AutopilotStatus)/.test(n))return 'das';
 return 'body'}
function fv(x){return x===null||x===undefined?'-':(typeof x=='number'?(Math.round(x*1000)/1000):(typeof x=='object'?JSON.stringify(x):x))}
function hx(k){return '0x'+(+k).toString(16)}
function card(m,raw){var r=raw[m.addr]||{};var h='<section'+(r.age>2?' class=stale':'')+'><h2>'+m.name+' <small>'+hx(m.addr)+' &middot; bus '+r.src+' &middot; '+r.hz+' Hz &middot; age '+r.age+'s'+(m.mux!==null&&m.mux!==undefined?' &middot; mux '+m.mux:'')+'</small></h2><table>';
 Object.keys(m.sig).forEach(function(k){h+='<tr><td>'+k+'</td><td>'+fv(m.sig[k])+'</td></tr>'});return h+'</table></section>'}
// "Every CAN address": change tracking in the browser (no extra work on the EON). Each 0.5 s poll is compared with the
// previous one, per byte and per bit: when it last changed, how often. Bytes that change in > 30% of polls (counters,
// checksums, live values) are greyed; a byte that changed < 1.5 s ago is red, < 5 s orange, < 30 s faint. "Mark" stores
// the current bytes: then outlined bytes / the "mark" column show what differs since you pressed it (press a button in
// the car, see what lit up). Click a row for the bit grid and its last values.
var CH={},SORT='recent',VIEW='all',MARKT=0,OPEN={};
function track(d){var raw=d.raw||{},t=Date.now()/1000;Object.keys(raw).forEach(function(k){
 var b=(raw[k].hex||'').split(' ').filter(Boolean).map(function(x){return parseInt(x,16)}),c=CH[k];
 if(!c){CH[k]={b:b,first:t,last:0,bl:{},bn:{},byl:{},byn:{},polls:0,chg:0,hist:[[t,raw[k].hex]],vals:[[t,b]],base:MARKT?b.slice():null};return}
 c.vals.push([t,b]);if(c.vals.length>120)c.vals.shift();   // 60 s of values for the byte graphs
 c.polls++;var any=false;
 for(var i=0;i<Math.max(b.length,c.b.length);i++){var x=(b[i]===undefined||c.b[i]===undefined)?255:(b[i]^c.b[i]);if(!x)continue;any=true;c.byl[i]=t;c.byn[i]=(c.byn[i]||0)+1;
  for(var j=0;j<8;j++)if(x>>j&1){c.bl[i*8+j]=t;c.bn[i*8+j]=(c.bn[i*8+j]||0)+1}}
 if(any){c.chg++;c.last=t;c.hist.unshift([t,raw[k].hex]);if(c.hist.length>12)c.hist.pop()}
 c.b=b})}
function noisy(c,n){return c.polls>=10&&(n||0)/c.polls>0.3}
function qbit(c,n){return !noisy(c,c.byn[n>>3])}   // a bit counts as quiet when its byte isn't a counter / live value
function qlast(c){var m=0;for(var k in c.bl)if(qbit(c,+k)&&c.bl[k]>m)m=c.bl[k];return m}   // last change of a quiet bit
function nbits(c){return Object.keys(c.bn).length}
function dmark(c){if(!c.base)return 0;var n=0;for(var i=0;i<Math.max(c.b.length,c.base.length);i++){var x=(c.b[i]||0)^(c.base[i]||0);while(x){n+=x&1;x>>=1}}return n}
function bcol(c,i,t){var st='';if(c.base&&c.b[i]!==c.base[i])st='box-shadow:inset 0 0 0 2px #1a73e8;';
 if(noisy(c,c.byn[i]))return st+'color:var(--mut)';if(!c.byl[i])return st;var a=t-c.byl[i];
 return st+(a<1.5?'background:#e5484d;color:#fff':a<5?'background:#f08a00;color:#000':a<30?'background:rgba(240,138,0,.25)':'')}
function ago(t,x){return x?(t-x<100?(t-x).toFixed(1):Math.round(t-x))+'':'-'}
function canSort(m){SORT=m;show(D)}function canView(m){VIEW=m;show(D)}function canOpen(k){OPEN[k]=!OPEN[k];show(D)}
function canMark(){MARKT=Date.now()/1000;for(var k in CH)CH[k].base=CH[k].b.slice();show(D)}
function canReset(){CH={};MARKT=0;OPEN={};if(D)track(D);show(D)}
function bitgrid(c,t){var h='<table class=mono style="margin:6px 0"><tr><td>byte \\ bit</td>';for(var j=7;j>=0;j--)h+='<td>'+j+'</td>';h+='<td>hex</td></tr>';
 for(var i=0;i<c.b.length;i++){h+='<tr><td>'+i+'</td>';for(var j=7;j>=0;j--){var n=i*8+j,f=c.bn[n]||0,a=c.bl[n]?t-c.bl[n]:1e9,
   bg=a<1.5&&qbit(c,n)?'#e5484d':!qbit(c,n)&&f?'rgba(128,128,128,.35)':f?'rgba(26,115,232,'+Math.min(0.9,0.2+0.15*Math.log(1+f))+')':'';
   h+='<td title="bit '+n+' (Intel start bit) &middot; '+f+' flips'+(c.bl[n]?' &middot; last '+ago(t,c.bl[n])+' s ago':'')+'" style="text-align:center;'+(bg?'background:'+bg:'')+'">'+(c.b[i]>>j&1)+'</td>'}
  h+='<td>'+('0'+c.b[i].toString(16)).slice(-2)+'</td></tr>'}
 h+='</table>'+bytegraphs(c,t)+'<div style="color:var(--mut)">blue = flipped (darker = more often), grey = in a byte that changes constantly, red = just now. Hover a bit for its number and count.</div><div class=mono style="margin-top:6px">';
 c.hist.forEach(function(e){h+=ago(t,e[0])+' s ago &nbsp; '+e[1]+'<br>'});return h+'</div>'}
function bytegraphs(c,t){   // value of each byte over the last 60 s (like SavvyCAN's graphs), only bytes that changed
 var h='<div style="display:flex;flex-wrap:wrap;gap:8px;margin:6px 0">',W=180,H=44;
 for(var i=0;i<c.b.length;i++){if(!c.byn[i])continue;var pts=c.vals.map(function(v){return ((60-(t-v[0]))/60*W).toFixed(1)+','+(H-2-((v[1][i]||0)/255)*(H-4)).toFixed(1)}).join(' ');
  var lo=255,hi=0;c.vals.forEach(function(v){var x=v[1][i]||0;if(x<lo)lo=x;if(x>hi)hi=x});
  h+='<div><div style="color:var(--mut)">byte '+i+' &nbsp; '+lo+'..'+hi+(MARKT&&t-MARKT<60?' &nbsp; <span style="color:#2fbf71">| = mark</span>':'')+'</div><svg width='+W+' height='+H+' style="background:rgba(128,128,128,.12);border-radius:4px">'+
   (MARKT&&t-MARKT<60?'<line x1='+((60-(t-MARKT))/60*W).toFixed(1)+' x2='+((60-(t-MARKT))/60*W).toFixed(1)+' y1=0 y2='+H+' stroke="#2fbf71"/>':'')+
   '<polyline fill=none stroke="#1a73e8" stroke-width=1.5 points="'+pts+'"/></svg></div>'}
 return h+'</div>'}
function allcan(raw){var t=Date.now()/1000,q=Q.toLowerCase(),btn=function(cur,v,f,l){return '<button class=chipb'+(cur==v?' style="background:#1a73e8;color:#fff"':'')+' onclick="'+f+'(\''+v+'\')">'+l+'</button>'};
 var h='<section class=wide><h2>Every CAN address seen (any bus) <small>'+Object.keys(raw).length+' addresses'+(MARKT?' &middot; mark set '+ago(t,MARKT)+' s ago':'')+'</small></h2>'+
  '<div style="display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:6px 0">sort '+btn(SORT,'recent','canSort','last change')+btn(SORT,'rate','canSort','changes/s')+btn(SORT,'bits','canSort','bits changing')+btn(SORT,'addr','canSort','address')+btn(SORT,'hz','canSort','Hz')+btn(SORT,'bus','canSort','bus')+
  ' &nbsp; show '+btn(VIEW,'all','canView','all')+btn(VIEW,'changing','canView','changed in 30 s')+btn(VIEW,'mark','canView','differs from mark')+btn(VIEW,'static','canView','never changed')+
  ' &nbsp; <button class=chipb onclick="canMark()" style="background:#2fbf71;color:#fff">Mark</button><button class=chipb onclick="canReset()">Reset stats</button></div>'+
  '<div style="color:var(--mut);margin-bottom:6px">Changed <span style="background:#e5484d;color:#fff">&lt;1.5 s</span> <span style="background:#f08a00;color:#000">&lt;5 s</span> <span style="background:rgba(240,138,0,.25)">&lt;30 s</span> ago &middot; grey = changes almost every update (counter / live value) &middot; <span style="box-shadow:inset 0 0 0 2px #1a73e8">outlined</span> = differs from Mark &middot; "last chg" ignores grey bytes. Bytes of IDs nothing decodes refresh twice a second, so very short blips can be missed.</div>'+
  '<input id=f1 placeholder="filter by address / name / bus, e.g. 0x3d8, UI_, src 1" value="'+Q+'" oninput="Q=this.value;show(D);var e=document.getElementById(\'f1\');e.focus();e.setSelectionRange(Q.length,Q.length)">'+
  '<div style="overflow-x:auto"><table class=mono><tr><td>addr</td><td>name (if in DBC)</td><td>bus</td><td>Hz</td><td>age s</td><td>last chg s</td><td>chg/s</td><td>bits</td><td>mark</td><td>data</td></tr>';
 var ks=Object.keys(raw).filter(function(k){var e=raw[k],c=CH[k];if(!c)return false;if(q&&(hx(k)+' '+e.name+' src '+e.src).toLowerCase().indexOf(q)<0)return false;
  if(VIEW=='changing')return t-qlast(c)<30;if(VIEW=='mark')return dmark(c)>0;if(VIEW=='static')return !c.chg;return true});
 var rate=function(k){var c=CH[k];return c.chg/Math.max(1,t-c.first)},key={recent:function(k){return -qlast(CH[k])},rate:function(k){return -rate(k)},bits:function(k){return -nbits(CH[k])},addr:function(k){return +k},hz:function(k){return -raw[k].hz},bus:function(k){return raw[k].src*4096+(+k)}}[SORT];
 ks.sort(function(a,b){return (key(a)-key(b))||(a-b)});
 ks.forEach(function(k){var e=raw[k],c=CH[k],ql=qlast(c),dm=dmark(c),d='';
  for(var i=0;i<c.b.length;i++)d+='<span style="padding:0 2px;border-radius:3px;'+bcol(c,i,t)+'" title="byte '+i+' &middot; '+(c.byn[i]||0)+' changes'+(c.byl[i]?' &middot; last '+ago(t,c.byl[i])+' s ago':'')+'">'+('0'+c.b[i].toString(16)).slice(-2)+'</span> ';
  h+='<tr onclick="canOpen(\''+k+'\')" style="cursor:pointer'+(OPEN[k]?';background:rgba(26,115,232,.12)':'')+'"><td>'+hx(k)+'</td><td>'+(e.name||'')+'</td><td>'+e.src+'</td><td>'+e.hz+'</td><td>'+e.age+'</td><td>'+ago(t,ql)+'</td><td>'+rate(k).toFixed(2)+'</td><td>'+nbits(c)+'</td><td>'+(c.base?dm:'')+'</td><td style="text-align:right;white-space:nowrap">'+d+'</td></tr>';
  if(OPEN[k])h+='<tr><td colspan=10>'+bitgrid(c,t)+'</td></tr>'});
 return h+'</table></div>'}
function show(d){D=d;document.getElementById('v').textContent=Math.round((d.v||0)*3.6)+' km/h';
 document.getElementById('acc').className='chip'+(d.acc?' on':'');document.getElementById('alc').className='chip'+(d.alc?' on':'');
 document.getElementById('logn').textContent=d.logfile?'logging to '+d.logfile:'';
 var raw=d.raw||{},ms=d.msgs||{},groups={map:[],pt:[],gw:[],das:[],body:[]};
 Object.keys(ms).forEach(function(k){groups[cat(ms[k].name)].push(ms[k])});
 var nav='';TABS.forEach(function(t){var n=groups[t[0]]?groups[t[0]].length:Object.keys(raw).length;nav+='<button class="'+(TAB==t[0]?'on':'')+'" onclick="TAB=\''+t[0]+'\';show(D)">'+t[1]+'<small>'+n+'</small></button>'});
 document.getElementById('nav').innerHTML=nav;
 var h='';
 if(TAB=='all'){
  h+=allcan(raw);
  h+='</section><section class=wide><h2>openpilot carState / controlsState fields</h2><table class=mono>';
  ['car','ctl'].forEach(function(g){var o=d[g]||{};Object.keys(o).forEach(function(k){h+='<tr><td>'+g+'.'+k+'</td><td>'+fv(o[k])+'</td></tr>'})});h+='</table></section>'}
 else{
  if(TAB=='map'){var md=groups.map.filter(function(m){return m.name=='UI_driverAssistMapData'})[0];
   if(md){var s=md.sig,u=(s.UI_mapSpeedUnits?' mph':' km/h');h+='<section><h2>Speed limit from the map</h2><div class=big>'+(s.UI_mapSpeedLimit?s.UI_mapSpeedLimit*5+u:'-')+'</div><div style="color:var(--mut)">raw '+s.UI_mapSpeedLimit+' (x5 assumed), type '+s.UI_mapSpeedLimitType+', road class '+s.UI_roadClass+'</div></section>'}}
  var L=groups[TAB].sort(function(a,b){return a.addr-b.addr});L.forEach(function(m){h+=card(m,raw)});
  if(!L.length)h='<div class=empty>Nothing seen yet for this group. If the car is on and the "All CAN" tab lists addresses, MCU2 may use different IDs than the S/X DBC.</div>'}
 document.getElementById('m').innerHTML=h}
function demoHex(){var n=Math.floor(Date.now()/500),h=function(x){return ('0'+(x&255).toString(16)).slice(-2)};return [h(n%16),'12',h(Math.floor(n/12)%2?0x40:0),'00',h(n*7),'00','00',h(n*13)].join(' ')}   // demo: a counter, a rare flag, a checksum
function demoD(){return {v:17.2,acc:true,alc:true,msgs:{'968:UI_driverAssistMapData':{name:'UI_driverAssistMapData',addr:968,mux:null,sig:{UI_mapSpeedLimit:24,UI_mapSpeedUnits:0,UI_navRouteActive:1,UI_nextBranchDist:'120 m'}},
 '984:MCU_locationStatus':{name:'MCU_locationStatus',addr:984,mux:null,sig:{MCU_latitude:'44.4268 deg',MCU_longitude:'26.1025 deg'}},
 '872:DI_state':{name:'DI_state',addr:872,mux:null,sig:{DI_vehicleSpeed:'62 kph',DI_gear:4}},'920:GTW_carConfig':{name:'GTW_carConfig',addr:920,mux:null,sig:{GTW_dasHw:2,GTW_parkAssistInstalled:1}},
 '513:SDM1':{name:'SDM1',addr:513,mux:null,sig:{SDM1_Chime:0}}},
 raw:{968:{src:0,hz:10,age:.05,hex:'00 18 00 20 00 00 00 00',name:'UI_driverAssistMapData'},984:{src:0,hz:5,age:.1,hex:'11 22 33 44 55 66 77 88',name:'MCU_locationStatus'},872:{src:0,hz:50,age:0,hex:demoHex(),name:'DI_state'},920:{src:0,hz:1,age:.2,hex:'00',name:'GTW_carConfig'},513:{src:0,hz:50,age:0,hex:'00',name:'SDM1'}},car:{vEgo:17.2},ctl:{enabled:true}}}
if(demo){document.getElementById('conn').textContent='demo data';setInterval(function(){var d=demoD();track(d);if(document.activeElement.tagName!='INPUT')show(d);else D=d},500)}
else{(function poll(){fetch('/api/map').then(function(r){return r.json()}).then(function(d){document.getElementById('conn').textContent='live';track(d);if(document.activeElement.tagName!='INPUT')show(d);else D=d}).catch(function(){document.getElementById('conn').textContent='disconnected'}).then(function(){setTimeout(poll,500)})})()}
</script></body></html>
"""

# ---------------- camera feed (read-only): MJPEG from openpilot's camera stream (VisionIPC) ----------------
CAMS = {'road': ('VISION_STREAM_ROAD', 'VISION_STREAM_YUV_BACK', 'VISION_STREAM_RGB_BACK'),
        'driver': ('VISION_STREAM_DRIVER', 'VISION_STREAM_YUV_FRONT', 'VISION_STREAM_RGB_FRONT'),
        'wide': ('VISION_STREAM_WIDE_ROAD', 'VISION_STREAM_YUV_WIDE', 'VISION_STREAM_RGB_WIDE')}
VID = {}        # cam -> {'jpg': bytes, 'n': int, 'err': str, 'want': last request time, 'run': bool}
VLOCK = threading.Lock()

def _open_cam(cam):
    try:
        try: from cereal.visionipc import VisionIpcClient, VisionStreamType
        except ImportError: from cereal.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType
    except Exception as e:
        return None, 'no VisionIPC python module in this openpilot (%s)' % e
    st = None
    for n in CAMS[cam]:
        if hasattr(VisionStreamType, n): st = getattr(VisionStreamType, n); break
    if st is None: return None, 'no %s stream type in this openpilot' % cam
    try:
        c = VisionIpcClient('camerad', st, True)
        if not c.connect(True): return None, 'camerad not running (turn the car on / start openpilot)'
        return c, ''
    except Exception as e:
        return None, 'cannot open camera stream: %s' % e

def _to_jpeg(buf, maxw=720, q=60):
    import numpy as np, cv2
    w, h = int(buf.width), int(buf.height); arr = np.asarray(buf.data).reshape(-1)
    if arr.size >= w * h * 3 and arr.size < w * h * 3 // 2 * 2 + 4096 * 8 and arr.size == w * h * 3:      # packed RGB buffer
        img = cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    else:                                                                                                 # NV12 (Y plane then interleaved UV)
        stride = int(getattr(buf, 'stride', 0) or w)
        yuv = np.ascontiguousarray(arr[:stride * (h * 3 // 2)].reshape(h * 3 // 2, stride)[:, :w])
        img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    if w > maxw: img = cv2.resize(img, (maxw, int(h * maxw / w)))
    ok, j = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
    return j.tobytes() if ok else None

def cam_worker(cam):
    V = VID[cam]
    try:
        import numpy, cv2
    except ImportError:
        V['err'] = 'numpy/cv2 not available on this device'; V['run'] = False; return
    c, err = _open_cam(cam)
    if c is None and 'VisionIPC' in err:                      # old openpilot (0.6.x): no camera module, show the EON screen (camera + overlay) instead
        V['err'] = ''; V['mode'] = 'screen'
        while time.time() - V['want'] < 10:
            try:
                j, ct = _screen_jpeg(); V['jpg'] = j; V['ct'] = ct; V['n'] += 1; V['err'] = ''
            except Exception as e:
                V['err'] = 'screen capture failed: %s' % e; time.sleep(1)
            time.sleep(.15)
        V['run'] = False; return
    if c is None:
        V['err'] = err; V['run'] = False; return
    V['err'] = ''; last = 0
    while time.time() - V['want'] < 10:             # stops by itself when nobody is watching
        try:
            b = c.recv()
            if b is None: V['err'] = 'no frames (camerad stopped?)'; time.sleep(.5); continue
            if time.time() - last < 0.08: continue  # ~12 fps max
            last = time.time(); j = _to_jpeg(b)
            if j: V['jpg'] = j; V['n'] += 1; V['err'] = ''
        except Exception as e:
            V['err'] = 'frame error: %s' % e; time.sleep(.5)
    V['run'] = False

def cam_want(cam):
    with VLOCK:
        V = VID.setdefault(cam, {'jpg': None, 'n': 0, 'err': 'starting…', 'want': 0, 'run': False})
        V['want'] = time.time()
        if not V['run']:
            V['run'] = True; V['err'] = 'starting…'; threading.Thread(target=cam_worker, args=(cam,), daemon=True).start()
    return V

# ---------------- EON screen recording (same command the REC button uses: Android screenrecord -> /sdcard/videos) ----------------
VIDEOS_DIR = os.environ.get('RADARWEB_VIDEOS', '/sdcard/videos')
REC = {'on': False, 'file': '', 't0': 0, 'thread': None, 'msg': ''}
RLOCK = threading.Lock()

def _sr_pids():
    out = []
    try:
        for d in os.listdir('/proc'):
            if d.isdigit():
                try:
                    cl = open('/proc/%s/cmdline' % d, 'rb').read().split(b'\0')[0]
                    if os.path.basename(cl.decode('utf8', 'ignore')) == 'screenrecord': out.append(int(d))
                except Exception: pass
    except Exception: pass
    return out

def _rec_files():
    fs = []
    try:
        for n in os.listdir(VIDEOS_DIR):
            if n.endswith('.mp4'):
                p = os.path.join(VIDEOS_DIR, n); st = os.stat(p); fs.append({'n': n, 'mb': round(st.st_size / 1e6, 1), 't': int(st.st_mtime)})
    except Exception: pass
    fs.sort(key=lambda f: -f['t'])
    return fs

def _rec_loop():
    import subprocess
    while REC['on']:
        if not _sr_pids():
            try:
                if not os.path.isdir(VIDEOS_DIR): os.makedirs(VIDEOS_DIR)
                try: subprocess.call(['input', 'keyevent', '224'])         # wake the EON screen (a sleeping screen records black)
                except Exception: pass
                REC['file'] = 'web_%s.mp4' % time.strftime('%Y%m%d-%H%M%S'); REC['t0'] = time.time()
                subprocess.Popen(['screenrecord', '--time-limit', '180', os.path.join(VIDEOS_DIR, REC['file'])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                REC['msg'] = ''
                old = [f for f in _rec_files() if f['n'].startswith('web_')][20:]          # keep the newest 20 web recordings; saved_* and the REC button's files are never touched
                for f in old:
                    try: os.remove(os.path.join(VIDEOS_DIR, f['n']))
                    except Exception: pass
            except Exception as e:
                REC['msg'] = 'cannot start screenrecord: %s' % e; REC['on'] = False; break
        time.sleep(1)

def rec_cmd(cmd):
    import signal
    with RLOCK:
        if cmd == 'start' and not REC['on']:
            REC['on'] = True; REC['msg'] = ''; t = threading.Thread(target=_rec_loop, daemon=True); REC['thread'] = t; t.start()
        elif cmd == 'stop':
            REC['on'] = False
            for p in _sr_pids():
                try: os.kill(p, signal.SIGINT)                      # SIGINT lets screenrecord finish the mp4 properly
                except Exception: pass
    time.sleep(.3)
    return rec_status()

def rec_status():
    pids = _sr_pids()
    return {'on': REC['on'], 'running': bool(pids), 'ext': bool(pids) and not REC['on'], 'file': REC['file'], 'sec': int(time.time() - REC['t0']) if pids else 0,
            'msg': REC['msg'], 'dir': VIDEOS_DIR, 'files': _rec_files()[:12]}

def _screen_jpeg(maxw=960, q=55):
    import subprocess
    png = subprocess.check_output(['screencap', '-p'], timeout=8)
    try:
        import numpy as np, cv2
        img = cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            h, w = img.shape[:2]
            if w > maxw: img = cv2.resize(img, (maxw, int(h * maxw / w)))
            ok, j = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), q])
            if ok: return j.tobytes(), 'image/jpeg'
    except Exception: pass
    return png, 'image/png'                                         # no OpenCV: send the PNG as is (slower)


# ---------------- camera recalibration (the one action that is not read-only): same effect as the EON's "Reset Calibration" ----------------
RECAL = {'t': 0, 'msg': '', 'stage': ''}
def _pids_of(name):
    out = []
    try:
        for d in os.listdir('/proc'):
            if d.isdigit() and int(d) != os.getpid():
                try:
                    cl = open('/proc/%s/cmdline' % d, 'rb').read().replace(b'\0', b' ').decode('utf8', 'ignore')
                    if name in cl and 'radarweb' not in cl and 'grep' not in cl: out.append(int(d))
                except Exception: pass
    except Exception: pass
    return out

def _recal_worker():
    import signal
    try:
        for p in _pids_of('calibrationd'):
            try: os.kill(p, signal.SIGTERM)                     # stop it first, or it would write its old calibration back
            except Exception: pass
        t = time.time()
        while _pids_of('calibrationd') and time.time() - t < 5: time.sleep(.2)
        from common.params import Params
        Params().delete('CalibrationParams')
        RECAL['stage'] = 'waiting'; RECAL['msg'] = 'calibration cleared, waiting for calibrationd to restart…'
        t = time.time()
        while time.time() - t < 12:
            if _pids_of('calibrationd'): RECAL['stage'] = 'done'; RECAL['msg'] = 'calibrationd is running again and is learning the camera angle: drive straight for a few minutes'; return
            time.sleep(.5)
        RECAL['stage'] = 'manual'; RECAL['msg'] = 'calibration cleared, but calibrationd did not restart by itself: turn the car off and on (or restart openpilot) and it relearns from scratch'
    except Exception as e:
        RECAL['stage'] = 'error'; RECAL['msg'] = 'failed: %s' % e

def recal_start():
    if STATE.get('v', 0) > 0.5 or STATE.get('op'):
        return {'ok': False, 'msg': 'only when the car is stopped and openpilot is not engaged'}
    if RECAL['stage'] in ('running', 'waiting') and time.time() - RECAL['t'] < 30:
        return {'ok': False, 'msg': 'already in progress'}
    try: import common.params       # noqa: only on the EON
    except Exception as e: return {'ok': False, 'msg': 'not on the EON, nothing to reset (%s)' % e}
    RECAL.update({'t': time.time(), 'stage': 'running', 'msg': 'stopping calibrationd…'})
    threading.Thread(target=_recal_worker, daemon=True).start()
    return {'ok': True, 'msg': RECAL['msg']}


VIDEO = r"""<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<title>Camera</title><style>
html,body{margin:0;height:100%;background:#0d0f12;color:#cfd6dd;font-family:system-ui,Segoe UI,sans-serif;overflow:hidden}
#wrap{position:fixed;inset:0;display:flex;align-items:center;justify-content:center}
img{max-width:100vw;max-height:100vh;object-fit:contain;background:#000}
#msg{position:fixed;left:0;right:0;top:45%;text-align:center;font-size:18px;opacity:.85;padding:0 20px}
#tabs{position:fixed;left:10px;top:8px}#tabs a{display:inline-block;margin-right:8px;padding:5px 12px;border-radius:14px;background:rgba(60,66,75,.8);color:#fff;text-decoration:none;font-size:14px}
#tabs a.on{background:#178644}
#nav{position:fixed;right:10px;bottom:8px;z-index:50}#nav summary{list-style:none;cursor:pointer;width:36px;height:36px;display:flex;align-items:center;justify-content:center;background:#1c2128;border:1px solid #30363d;border-radius:8px;font-size:18px;color:#cfd6dd}#nav summary::-webkit-details-marker{display:none}#nav[open] summary{background:#262c35}#nav>div{position:absolute;right:0;bottom:44px;background:#1c2128;border:1px solid #30363d;border-radius:8px;padding:6px 0;min-width:190px;box-shadow:0 8px 24px rgba(0,0,0,.45)}#nav a,#nav>div span{display:block;padding:9px 14px;color:#cfd6dd;text-decoration:none;font-size:13px;white-space:nowrap}#nav a:hover{background:#262c35}
#rec{position:fixed;right:10px;top:8px;text-align:right;font-size:13px}#rb{font:700 15px system-ui;padding:7px 16px;border-radius:18px;border:2px solid #fff;background:rgba(60,66,75,.85);color:#fff;cursor:pointer}#rb.on{background:#d62828}#cb{font:600 13px system-ui;padding:6px 12px;border-radius:16px;border:2px solid #fff;background:rgba(60,66,75,.85);color:#fff;cursor:pointer}#cs{margin-top:6px;max-width:340px;margin-left:auto;font-size:13px}#rs{margin-left:8px}#rf{margin-top:6px;opacity:.9}#rf a{display:block;color:#cfd6dd;text-decoration:none;border-bottom:1px solid #667;margin:2px 0}
</style></head><body><div id=wrap><img id=im alt=""></div><div id=msg>connecting to the camera…</div>
<div id=tabs><a href="?cam=road" id=t_road>road</a><a href="?cam=driver" id=t_driver>driver</a><a href="?cam=wide" id=t_wide>wide</a></div>
<div id=rec><button id=rb>REC</button><span id=rs></span><div id=rf></div><div style="margin-top:10px"><button id=cb class=sec>Reset calibration</button></div><div id=cs></div></div>
<details id=nav><summary>☰</summary><div><a href="/">road view</a><a href="/top">top-down view</a><a href="/near">near view</a></div></details>
<script>
var cam=(/cam=(\w+)/.exec(location.search)||[0,'road'])[1],im=document.getElementById('im'),msg=document.getElementById('msg');
document.getElementById('t_'+cam)&&(document.getElementById('t_'+cam).className='on');
im.onload=function(){msg.style.display='none'};
function start(){im.src='/video.mjpg?cam='+cam+'&t='+Date.now()}
im.onerror=function(){msg.style.display='block';setTimeout(start,2000)};
setInterval(function(){fetch('/api/video?cam='+cam).then(function(r){return r.json()}).then(function(d){if(d.err){msg.style.display='block';msg.textContent=d.err}else if(!d.n){msg.style.display='block';msg.textContent='connecting to the camera…'}}).catch(function(){})},1500);
function rec(cmd){fetch('/api/rec'+(cmd?'?cmd='+cmd:'')).then(function(r){return r.json()}).then(function(d){var b=document.getElementById('rb');
 b.className=d.running?'on':'';b.textContent=d.running?'STOP':'REC';b.onclick=function(){rec(d.running?'stop':'start')};
 document.getElementById('rs').textContent=d.msg||(d.running?(d.ext?'recording (started on the EON) ':'recording ')+d.sec+' s  '+d.file:'');
 document.getElementById('rf').innerHTML=d.files.map(function(f){return '<a href="/rec/'+f.n+'">'+f.n+'  '+f.mb+' MB</a>'}).join('')}).catch(function(){})}
rec();setInterval(rec,2000);
function cal(){fetch('/api/recal').then(function(r){return r.json()}).then(function(d){document.getElementById('cs').textContent=d.msg||''}).catch(function(){})}
document.getElementById('cb').onclick=function(){if(!confirm('Reset the camera calibration?\n\nopenpilot will forget the camera angle and relearn it while you drive. Only works while the car is stopped and openpilot is not engaged.'))return;
 fetch('/api/recal',{method:'POST',headers:{'X-Recal':'1'}}).then(function(r){return r.json()}).then(function(d){document.getElementById('cs').textContent=d.msg}).catch(function(e){document.getElementById('cs').textContent='request failed'})};
cal();setInterval(cal,1500);
start();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _snap(self):
        with LOCK: return SNAP[0] if SNAP[0] and time.time() - SNAP[1] < 0.3 else json.dumps(STATE)   # built once per tick, shared by all streams
    def _authed(self):
        if not AUTH: return True
        import base64, hmac
        h = self.headers.get('Authorization', '')
        try: ok = h.startswith('Basic ') and hmac.compare_digest(base64.b64decode(h[6:]), AUTH.encode())
        except Exception: ok = False
        if not ok:
            self.send_response(401); self.send_header('WWW-Authenticate', 'Basic realm="radarweb"'); self.send_header('Content-Length', '0'); self.end_headers()
        return ok
    def do_GET(self):
        if not self._authed(): return
        if self.path == '/api/state':
            b = self._snap().encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == '/api/map':
            msgs, rawv = build_map(time.time()); d = {'msgs': msgs, 'raw': rawv, 'car': MAP['car'], 'ctl': MAP['ctl'], 'logfile': MAP.get('logfile'), 'v': STATE.get('v', 0), 'acc': STATE.get('acc', False), 'alc': STATE.get('alc', False)}
            b = json.dumps(d, default=str).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == '/api/trips':
            b = json.dumps(trips_list()).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.startswith('/api/trip?'):
            import urllib.parse as _up
            f_ = (_up.parse_qs(self.path.split('?', 1)[1]).get('f') or [''])[0]
            b = json.dumps(trip_get(f_)).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/trips':
            import radartrip
            b = radartrip.VIEWER.encode(); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == '/api/ml':
            b = json.dumps(ml_summary(), default=str).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/ml':
            b = MLPAGE.encode(); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/map':
            b = MAPP.encode(); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == '/api/stream':
            self.send_response(200); self.send_header('Content-Type', 'text/event-stream'); self.send_header('Cache-Control', 'no-cache'); self.end_headers()
            try:
                while True:
                    self.wfile.write(('data: ' + self._snap() + '\n\n').encode()); self.wfile.flush(); time.sleep(0.1)
            except Exception: pass
        elif self.path.split('?')[0] in ('/', '/index.html', '/top', '/near', '/calib', '/learn'):
            p = self.path.split('?')[0]
            b = (TOP if p == '/top' else NEAR if p == '/near' else CALIB if p == '/calib' else LEARN if p == '/learn' else ROAD).encode(); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/video':
            b = VIDEO.encode(); self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/api/video':
            import re as _re
            m_ = _re.search(r'cam=(\w+)', self.path); cam = m_.group(1) if m_ and m_.group(1) in CAMS else 'road'
            V = cam_want(cam); b = json.dumps({'err': V['err'], 'n': V['n']}).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/video.mjpg':
            import re as _re
            m_ = _re.search(r'cam=(\w+)', self.path); cam = m_.group(1) if m_ and m_.group(1) in CAMS else 'road'
            V = cam_want(cam); self.send_response(200); self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame'); self.send_header('Cache-Control', 'no-cache'); self.end_headers()
            n = -1
            try:
                while True:
                    V['want'] = time.time()
                    if V['jpg'] and V['n'] != n:
                        n = V['n']; j = V['jpg']
                        self.wfile.write(('--frame\r\nContent-Type: %s\r\nContent-Length: %d\r\n\r\n' % (V.get('ct', 'image/jpeg'), len(j))).encode()); self.wfile.write(j); self.wfile.write(b'\r\n'); self.wfile.flush()
                    time.sleep(0.04)
                    if not V['run'] and not V['jpg']: time.sleep(1.0); cam_want(cam)
            except Exception: pass
        elif self.path.split('?')[0] == '/api/rec':
            import re as _re
            m_ = _re.search(r'cmd=(\w+)', self.path); b = json.dumps(rec_cmd(m_.group(1)) if m_ and m_.group(1) in ('start', 'stop') else rec_status()).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.startswith('/icons/'):
            import re as _re2
            n_ = self.path[7:].split('?')[0]
            if not _re2.match(r'^[\w.\-]+\.webp$', n_) or not os.path.isfile(os.path.join(ICON_DIR, n_)):
                self.send_response(404); self.end_headers()
            else:
                p_ = os.path.join(ICON_DIR, n_); sz = os.path.getsize(p_)
                self.send_response(200)
                self.send_header('Content-Type', 'image/webp')
                self.send_header('Cache-Control', 'public, max-age=31536000, immutable')
                self.send_header('Content-Length', str(sz))
                self.end_headers()
                with open(p_, 'rb') as fh:
                    self.wfile.write(fh.read())
        elif self.path.startswith('/rec/'):
            import re as _re
            n_ = self.path[5:].split('?')[0]
            if not _re.match(r'^[\w.\-]+\.mp4$', n_) or not os.path.isfile(os.path.join(VIDEOS_DIR, n_)):
                self.send_response(404); self.end_headers()
            else:
                p_ = os.path.join(VIDEOS_DIR, n_); sz = os.path.getsize(p_); self.send_response(200); self.send_header('Content-Type', 'video/mp4'); self.send_header('Content-Length', str(sz)); self.end_headers()
                try:
                    with open(p_, 'rb') as fh:
                        while True:
                            ch = fh.read(65536)
                            if not ch: break
                            self.wfile.write(ch)
                except Exception: pass
        elif self.path.split('?')[0] == '/api/recal':
            b = json.dumps({'stage': RECAL['stage'], 'msg': RECAL['msg'], 'v': STATE.get('v', 0), 'op': bool(STATE.get('op'))}).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/chime':      # optional: your own alert sound, put chime.mp3 / .wav / .ogg / .m4a next to radarweb.py or in /data
            fn = None
            import re as _re
            _m = _re.search(r'n=(\w+)', self.path); base = 'warn' if _m and _m.group(1) == 'warn' else 'chime'
            for d in (os.path.dirname(os.path.abspath(__file__)), '/data'):
                for ext, ct in (('mp3', 'audio/mpeg'), ('wav', 'audio/wav'), ('ogg', 'audio/ogg'), ('m4a', 'audio/mp4')):
                    q = os.path.join(d, base + '.' + ext)
                    if fn is None and os.path.isfile(q): fn = (q, ct)
            if fn is None:
                self.send_response(404); self.end_headers()
            else:
                b = open(fn[0], 'rb').read(); self.send_response(200); self.send_header('Content-Type', fn[1]); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        else:
            self.send_response(404); self.end_headers()

    def do_POST(self):
        if not self._authed(): return
        if self.path.split('?')[0] == '/api/recal' and self.headers.get('X-Recal') == '1':
            b = json.dumps(recal_start()).encode(); self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/api/label':
            # LABELBTN: body is the raw label text (plain text, not JSON - keeps the client's fetch() tiny).
            try:
                n = int(self.headers.get('Content-Length', '0') or '0')
                txt = self.rfile.read(min(n, 200)).decode('utf8', 'replace').strip()
            except Exception:
                txt = ''
            ok = False
            if txt:
                LAST_LABEL['t'] = time.time(); LAST_LABEL['text'] = txt
                try:
                    os.makedirs(os.path.dirname(LABEL_FILE), exist_ok=True)
                    open(LABEL_FILE, 'w').write(txt)   # same file canraw.py/rawlog.py already poll for a mark
                    ok = True
                except Exception:
                    pass
            b = json.dumps({'ok': ok, 'text': txt, 'file': LABEL_FILE}).encode()
            self.send_response(200 if ok else 500); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path == '/api/train':
            ok = start_training(True)
            b = json.dumps({'ok': ok, 'msg': AUTO['msg']}).encode()
            self.send_response(200); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        elif self.path.split('?')[0] == '/api/objlabel':
            # LEARN: body is JSON {s,c,d,y,vr,gs,len,radar_c,radar_pc} from the /learn page's tap-to-select UI.
            # Added here: 'h' = the last HIST_S of that track and 'v' = ego speed, so the trainer gets the same
            # history features the live model uses (only if the slot still holds the object you tapped).
            ok = False; row = {}
            try:
                n = int(self.headers.get('Content-Length', '0') or '0')
                row = json.loads(self.rfile.read(min(n, 2000)).decode('utf8', 'replace'))
                row['t'] = time.time(); row['v'] = STATE['v']
                try:
                    h = TRK.get(int(row['s']), row['t'])
                    if h and abs(h[-1][1] - float(row['d'])) < radarml.JUMP_D and abs(h[-1][2] - float(row['y'])) < radarml.JUMP_Y: row['h'] = h; row['hf'] = TRK.fields
                    for o in list(OBJFULL.values()):      # fields the page doesn't send (moving state, meas flag ...) from the live object
                        if o['s'] == int(row['s']):
                            for key in ('mst', 'meas', 'pe', 'ls', 'la', 'dz', 'po', 'pno'): row.setdefault(key, o.get(key))
                except Exception:
                    pass
                os.makedirs(os.path.dirname(OBJLABEL_FILE), exist_ok=True)
                open(OBJLABEL_FILE, 'a').write(json.dumps(row, separators=(',', ':')) + '\n')
                try:
                    os.makedirs(os.path.dirname(LABEL_FILE), exist_ok=True)
                    open(LABEL_FILE, 'w').write('LABEL %s s%s d%sm y%sm' % (row.get('c'), row.get('s'), row.get('d'), row.get('y')))
                except Exception:
                    pass
                ok = True
            except Exception:
                pass
            b = json.dumps({'ok': ok, 'row': row, 'file': OBJLABEL_FILE}).encode()
            self.send_response(200 if ok else 500); self.send_header('Content-Type', 'application/json'); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
        else:
            self.send_response(403); self.send_header('Content-Length', '0'); self.end_headers()

class S(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    def handle_error(self, request, client_address):
        # BrokenPipeError/ConnectionResetError just means the browser tab closed, reloaded, or
        # cancelled the request mid-response (page nav, the SSE stream, /video.mjpg stopping) -
        # normal client behavior, not a server bug, so don't spam the console with a traceback.
        # Anything else still prints normally so a real bug stays visible.
        import sys
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)): return
        super().handle_error(request, client_address)

if __name__ == '__main__':
    threading.Thread(target=worker, daemon=True).start()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); s.connect(('10.255.255.255', 1)); ip = s.getsockname()[0]
    except Exception: ip = '<EON-IP>'
    try: open('/data/radarweb.log', 'a').write('%s open http://%s:%d/\n' % (time.strftime('%H:%M:%S'), ip, PORT))
    except Exception: print('open http://%s:%d/' % (ip, PORT))
    S((BIND, PORT), H).serve_forever()