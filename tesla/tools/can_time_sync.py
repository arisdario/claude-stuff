#!/usr/bin/env python3
"""can_time_sync.py - set the EON system clock from the CAR'S OWN clock, read off CAN. No GPS, no
network: works on a bench as long as the car is powered and the panda is passing CAN to openpilot.

The gateway broadcasts full date+time in GTW_carState (CAN 0x318 / 792, main bus). Bit layout from
opendbc tesla_can.dbc (Intel/little-endian):
    YEAR 0|7 (+2000)   MONTH 8|4   SECOND 16|6   Hour 24|5   DAY 32|5   MINUTE 40|6
This reads the shared 'can' cereal socket (the same CAN stream carstate uses), decodes that one
message by hand (no DBC/CANParser needed), sanity-checks the date, and sets the clock with `date -u -s`.

Read-only w.r.t. the car: only subscribes to 'can', never sends anything.

  python3 can_time_sync.py              # one-shot: read car time, set the clock, exit
  python3 can_time_sync.py --print      # just print the decoded car time, do NOT touch the clock

NOTE on UTC vs local: the car broadcasts the time shown on its dash, which is usually LOCAL time, so
the EON (GMT) clock ends up on the car's local time - off from true UTC by your timezone offset. That
is completely fine for the purpose here (a dead-RTC EON needs a roughly-correct clock on boot so TLS
cert validation / NTP can work; being off by a timezone is harmless for that). NTP refines it to exact
UTC once the EON is online. Use --print and compare against the dash before trusting it.

Env: TIMEOUT_S (default 15), FLOOR_EPOCH_S (sanity floor, default 2025-01-01), DATE_CMD (default date).
"""
import datetime
import os
import subprocess
import sys
import time

ADDR = 0x318  # GTW_carState
TIMEOUT_S = float(os.environ.get("TIMEOUT_S", "15"))
FLOOR_EPOCH_S = int(os.environ.get("FLOOR_EPOCH_S", "1735689600"))  # 2025-01-01T00:00:00Z
CEILING_EPOCH_S = FLOOR_EPOCH_S + 20 * 365 * 86400                   # ~2045, reject garbage far future
DATE_CMD = os.environ.get("DATE_CMD", "date")


def decode_gtw_carstate(dat):
  """dat: the 8 payload bytes of GTW_carState (0x318). Returns a naive UTC datetime, or None if the
  fields don't form a valid date. Fields are little-endian bit offsets per opendbc tesla_can.dbc."""
  if dat is None or len(dat) < 6:
    return None
  v = int.from_bytes(bytes(dat[:8]).ljust(8, b"\0"), "little")

  def f(start, length):
    return (v >> start) & ((1 << length) - 1)

  year = f(0, 7) + 2000
  month = f(8, 4)
  second = f(16, 6)
  hour = f(24, 5)
  day = f(32, 5)
  minute = f(40, 6)
  try:
    return datetime.datetime(year, month, day, hour, minute, second)
  except ValueError:
    return None


def read_car_time(timeout_s):
  try:
    import cereal.messaging as messaging
  except Exception as e:
    sys.stderr.write("can_time_sync: cannot import cereal (%s) - run from /data/openpilot with PYTHONPATH set\n" % e)
    return None
  sub = messaging.sub_sock("can", timeout=100)
  deadline = time.monotonic() + timeout_s
  seen_any = 0
  seen_318 = 0
  while time.monotonic() < deadline:
    for msg in messaging.drain_sock(sub):
      for c in msg.can:
        seen_any += 1
        if c.address == ADDR and c.src == 0:
          seen_318 += 1
          dt = decode_gtw_carstate(c.dat)
          if dt is not None:
            return dt
    time.sleep(0.02)
  if seen_any == 0:
    sys.stderr.write("can_time_sync: no CAN messages at all - is openpilot/boardd running and panda connected?\n")
  elif seen_318 == 0:
    sys.stderr.write("can_time_sync: CAN is flowing but no 0x318 (GTW_carState) on bus 0 - is the car powered/awake?\n")
  else:
    sys.stderr.write("can_time_sync: saw 0x318 but its date fields never decoded sanely\n")
  return None


def set_clock(dt):
  spec = dt.strftime("%Y-%m-%d %H:%M:%S")
  try:
    subprocess.run([DATE_CMD, "-u", "-s", spec], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  except Exception as e:
    sys.stderr.write("can_time_sync: '%s -u -s %s' failed (%s) - are we root?\n" % (DATE_CMD, spec, e))
    return False
  print("can_time_sync: system clock set to %s (from the car's CAN clock, GTW_carState 0x318)" % spec)
  return True


def main(argv):
  just_print = "--print" in argv
  dt = read_car_time(TIMEOUT_S)
  if dt is None:
    sys.stderr.write("can_time_sync: could not read a car time within %ds\n" % int(TIMEOUT_S))
    return 1

  epoch = int(dt.replace(tzinfo=datetime.timezone.utc).timestamp())
  if just_print:
    print("car time (GTW_carState 0x318): %s UTC  (epoch %d)" % (dt.strftime("%Y-%m-%d %H:%M:%S"), epoch))
    print("  (clock NOT changed - this is --print mode. Compare against the dash; likely local time.)")
    return 0

  if not (FLOOR_EPOCH_S <= epoch <= CEILING_EPOCH_S):
    sys.stderr.write("can_time_sync: decoded time %s (epoch %d) is outside the sane window - refusing to set\n"
                     % (dt.strftime("%Y-%m-%d %H:%M:%S"), epoch))
    return 1

  return 0 if set_clock(dt) else 1


if __name__ == "__main__":
  sys.exit(main(sys.argv[1:]))
