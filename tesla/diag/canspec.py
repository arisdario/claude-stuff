"""CAN messages recorded raw + decoded for the lateral-control investigation.
Signal layout copied from the project DBC (same table radarweb.py uses): name -> (start, length, signed, scale, offset, unit).
All are big-endian (Motorola) in these messages. stdlib only."""

CAN_SPEC = {
  0x370: ('EPAS_sysStatus', {                 # steering angle used by openpilot (EPAS_internalSAS) + EPAS status
    'EPAS_internalSAS': (37, 14, False, 0.1, -819.2), 'EPAS_torsionBarTorque': (19, 12, False, 0.01, -20.5),
    'EPAS_eacStatus': (55, 3, False, 1, 0), 'EPAS_eacErrorCode': (23, 4, False, 1, 0), 'EPAS_handsOnLevel': (39, 2, False, 1, 0),
    'EPAS_steeringFault': (2, 1, False, 1, 0), 'EPAS_steeringReduced': (3, 1, False, 1, 0),
    'EPAS_steeringRackForce': (1, 10, False, 50, -25575), 'EPAS_currentTuneMode': (7, 4, False, 1, 0)}),
  0x0E: ('STW_ANGLHP_STAT', {                 # steering-wheel sensor (second, independent angle source)
    'StW_AnglHP': (5, 14, False, 0.1, -819.2), 'StW_AnglHP_Spd': (21, 14, False, 0.5, -4096),
    'StW_AnglHP_Sens_Stat': (33, 2, False, 1, 0)}),
  0x488: ('DAS_steeringControl', {            # what openpilot commands (angle request)
    'DAS_steeringControlType': (23, 2, False, 1, 0), 'DAS_steeringAngleRequest': (6, 15, False, 0.1, -1638.35),
    'DAS_steeringHapticRequest': (7, 1, False, 1, 0)}),
  0x101: ('GTW_epasControl', {'GTW_epasControlType': (15, 2, False, 1, 0), 'GTW_epasPowerMode': (6, 4, False, 1, 0),
                              'GTW_epasTuneRequest': (2, 3, False, 1, 0)}),
  0x108: ('DI_torque1', {'DI_torqueDriver': (0, 13, False, 0.25, 0), 'DI_torqueMotor': (16, 13, False, 0.25, 0),
                         'DI_motorRPM': (32, 16, False, 1, 0), 'DI_pedalPos': (48, 8, False, 0.4, 0)}),
  0x118: ('DI_torque2', {'DI_gear': (12, 3, False, 1, 0), 'DI_brakePedal': (15, 1, False, 1, 0),
                         'DI_vehicleSpeed': (16, 12, False, 0.05, -25), 'DI_brakePedalState': (36, 2, False, 1, 0)}),
  0x155: ('ESP_B', {'ESP_vehicleSpeed': (47, 16, False, 0.01, 0), 'ESP_vehicleSpeedQF': (57, 2, False, 1, 0)}),
  0x2B9: ('DAS_control', {'DAS_accState': (12, 4, False, 1, 0), 'DAS_setSpeed': (0, 12, False, 0.1, 0)}),
  0x368: ('DI_state', {'DI_cruiseState': (12, 4, False, 1, 0), 'DI_analogSpeed': (16, 12, False, 0.1, 0)}),
}
DEFAULT_RAW_ADDRS = tuple(CAN_SPEC)
# message addresses whose silence matters for lateral control (gap detection); others are only counted
WATCH_ADDRS = (0x370, 0x0E, 0x108, 0x118, 0x155)


def _bits_be(v, start, length):
  x, i = 0, start
  for _ in range(length):
    x = (x << 1) | ((v >> ((i // 8) * 8 + i % 8)) & 1)
    i = i + 15 if i % 8 == 0 else i - 1
  return x


def decode(addr, data):
  """data: bytes (<= 8). Returns {signal: value} or None for an address that is not in the table."""
  spec = CAN_SPEC.get(addr)
  if spec is None:
    return None
  v = int.from_bytes(bytes(data), 'little')
  out = {}
  for name, (st, ln, sg, sc, of) in spec[1].items():
    x = _bits_be(v, st, ln)
    if sg and x >= 1 << (ln - 1):
      x -= 1 << ln
    out[name] = round(x * sc + of, 4) if (sc != 1 or of != 0) else int(x)
  return out
