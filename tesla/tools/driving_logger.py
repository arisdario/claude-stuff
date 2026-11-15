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
import glob
import gzip
import json
import os
import re
import shutil
import struct
import sys
import threading
import time

# ---------------------------------------------------------------------------------------- RECORDER (--auto)
# THE ONE LOGGER. `--auto` (started at boot, see launch line in the chat) runs forever and records every drive by
# itself - a drive = carState is being published (openpilot started) - into /data/drivelog/<drive start>/c0001.jsonl.gz,
# c0002 ..., one compressed chunk every CHUNK_S seconds. log_uploader.py sends each finished chunk to GitHub right away
# (live, while driving) and deletes it locally. Every line is JSON with "src":
#   car   this logger's own 20 Hz sample (everything below in the docstring: state, leads, model, GPS, ...)
#   acc   ACC_module.py's decision line, followed from /data/acclog (button pressed + reason, see _acc_log there)
#   map   radarweb's map/radar line, followed from /data/maplog (radar objects, raw CAN snapshot, ~1/s)
#   fcw   fcw_shadow.py's comparison computed here (stock FCW vs fcw_v2 vs the model's) - the separate
#         fcw_shadow process is no longer needed
#   meta  drive start / end
# Lines from the followed files are copied as text (no re-parsing): {"src":"acc","r":<line>} / {"src":"map","d":<line>}.
DRIVE_ROOT = '/data/drivelog'
CHUNK_S = 120.0
DRIVE_END_S = 30.0            # no carState for this long = the drive ended (openpilot stopped)
FOLLOW = (('acc', 'r', '/data/acclog/acc_*.jsonl'), ('map', 'd', '/data/maplog/map_*.jsonl'))
MAPLOG_MAX_BYTES = 500 * 1024 * 1024   # radarweb's /data/maplog: oldest files deleted above this (all merged into chunks)
DRIVELOG_MAX_BYTES = 1024 * 1024 * 1024  # chunks not uploaded yet (e.g. no internet for days): oldest deleted above this
# PICTURES: camerad publishes the road camera's frame as /dev/c920_live.jpg (~10/s, ~230 kB). The recorder keeps the last
# FRAME_KEEP_S seconds (one frame per FRAME_EVERY_S) in memory and saves them around interesting moments - an ACC press,
# |accel| > 0.5 m/s2, the lead appearing/disappearing, an FCW event - plus one picture every FRAME_PERIODIC_S; at most
# FRAMES_PER_CHUNK per 2-minute chunk, packed as cNNNN.frames.tar next to the chunk (uploaded live too).
FRAME_SRC = '/dev/c920_live.jpg'
FRAME_EVERY_S = 2.0
FRAME_KEEP_S = 10.0
FRAME_AFTER_S = 6.0
FRAME_PERIODIC_S = 60.0
FRAMES_PER_CHUNK = 12
FRAMES_PRIORITY_EXTRA = 12    # moments that matter most (FRAME_PRIORITY) may use this many more per chunk ...
FRAME_PRIORITY = ('lead_out', 'lead_in', 'gap_short', 'cancel', 'fcw')   # ... and get the full window before/after;
                              # routine moments (ACC press, |accel| > 0.5) get one frame before and one after
                              # (15 Nov 19:47: the 12 were used up by 19:47:14, nothing of the 0.5 s-gap approach after)
GAP_SHORT_S = 1.5             # picture trigger: following closer than this (lead seen, above 36 km/h)
# RADAR OBJECTS: radard's liveTracks (every object the radar tracks, not only openpilot's two leads) are written
# TRACKS_HZ times a second as {"src":"tracks","t":..,"o":[[trackId, dRel m, yRel m, vRel m/s], ...]}, within TRACKS_MAX_M
TRACKS_HZ = 5
TRACKS_MAX_M = 250.
FRAMES_OFF = '/data/recorder_frames_off'      # touch to record no pictures (saves mobile data)


def gzip_file(src, dst):
  """src -> dst.gz atomically (tmp + rename), then src removed. Empty src: just removed."""
  if os.path.getsize(src) > 0:
    tmp = dst + '.tmp'
    with open(src, 'rb') as fi, gzip.open(tmp, 'wb', compresslevel=6) as fo:
      shutil.copyfileobj(fi, fo, 1 << 20)
    os.rename(tmp, dst)
  os.remove(src)


class ChunkWriter(object):
  """one drive: <root>/<YYYYmmdd_HHMMSS>/cNNNN.jsonl.part, closed + gzipped (in a thread) every chunk_s seconds"""
  def __init__(self, root=DRIVE_ROOT, chunk_s=CHUNK_S, clock=time.monotonic, background=True):
    self.drive = time.strftime('%Y%m%d_%H%M%S')
    self.dir = os.path.join(root, self.drive)
    os.makedirs(self.dir, exist_ok=True)
    self.chunk_s, self.clock, self.background, self.n, self.f = chunk_s, clock, background, 0, None
    self._threads = []
    self._open()

  def _open(self):
    self.n += 1
    self.part = os.path.join(self.dir, 'c%04d.jsonl.part' % self.n)
    self.f = open(self.part, 'w')
    self.t0 = self.clock()
    self.frames = 0

  def add_frame(self, t_wall, data, why):
    """PICTURES: one camera frame for this chunk (at most FRAMES_PER_CHUNK, + FRAMES_PRIORITY_EXTRA for the moments in
    FRAME_PRIORITY); returns True if kept"""
    if self.frames >= FRAMES_PER_CHUNK + (FRAMES_PRIORITY_EXTRA if why in FRAME_PRIORITY else 0):
      return False
    d = os.path.join(self.dir, 'c%04d.frames' % self.n)
    os.makedirs(d, exist_ok=True)
    name = '%s_%s.jpg' % (time.strftime('%H%M%S', time.localtime(t_wall)) + ('%.1f' % (t_wall % 1))[1:], why)
    with open(os.path.join(d, name), 'wb') as h:
      h.write(data)
    self.frames += 1
    return True

  def _finish(self):
    self.f.close()
    part, dst = self.part, self.part[:-len('.part')] + '.gz'
    fdir = os.path.join(self.dir, 'c%04d.frames' % self.n)
    def work():
      gzip_file(part, dst)
      if os.path.isdir(fdir):
        tar_dir(fdir, fdir + '.tar')
    if self.background:
      th = threading.Thread(target=work); th.start(); self._threads.append(th)
    else:
      work()

  def write(self, s):
    self.f.write(s)
    if self.clock() - self.t0 >= self.chunk_s:
      self._finish(); self._open()

  def flush(self):
    self.f.flush()

  def close(self):
    self._finish()
    for th in self._threads:
      th.join()


def tar_dir(src_dir, dst):
  """pictures of one chunk -> one .tar (jpgs are compressed already), atomically; the folder is removed"""
  import tarfile
  tmp = dst + '.tmp'
  with tarfile.open(tmp, 'w') as tf:
    for n in sorted(os.listdir(src_dir)):
      tf.add(os.path.join(src_dir, n), arcname=n)
  os.rename(tmp, dst)
  shutil.rmtree(src_dir, ignore_errors=True)


class Frames(object):
  """PICTURES: rolling memory of recent camera frames; save() writes the ones around a moment into the chunk"""
  def __init__(self, src=FRAME_SRC):
    self.src, self.buf, self.t_grab, self.save_until, self.saved_t, self.t_periodic = src, [], 0., 0., set(), 0.

  def tick(self, now, writer):
    if now - self.t_grab >= FRAME_EVERY_S:
      self.t_grab = now
      try:
        if now - os.path.getmtime(self.src) < 2.:
          with open(self.src, 'rb') as h:
            self.buf.append((now, h.read()))
      except OSError:
        pass
      self.buf = [b for b in self.buf if now - b[0] <= FRAME_KEEP_S]
      if now - self.t_periodic >= FRAME_PERIODIC_S:
        self.t_periodic = now
        self.mark(now, 'periodic', writer, before=0., after=0.)
      elif now <= self.save_until and self.buf:
        self._save(self.buf[-1], self.why, writer)

  def mark(self, now, why, writer, before=FRAME_KEEP_S, after=FRAME_AFTER_S):
    """something happened: keep the frames of the last `before` seconds and the next FRAME_AFTER_S"""
    self.why = why
    for b in self.buf:
      if now - b[0] <= before or b is self.buf[-1]:
        self._save(b, why, writer)
    self.save_until = max(self.save_until, now + after)

  def _save(self, b, why, writer):
    if b[0] in self.saved_t:
      return
    if writer.add_frame(b[0], b[1], why):
      self.saved_t.add(b[0])
      if len(self.saved_t) > 200:
        self.saved_t = set(sorted(self.saved_t)[-50:])


def frame_window(why):
  """PICTURES: (seconds before, seconds after) to save around a moment - full window only for the priority ones"""
  return (FRAME_KEEP_S, FRAME_AFTER_S) if why in FRAME_PRIORITY else (FRAME_EVERY_S, FRAME_EVERY_S)


def tracks_line(lt, t_wall, max_m=TRACKS_MAX_M):
  """RADAR OBJECTS: one 'tracks' line from a liveTracks list (any objects with trackId, dRel, yRel, vRel)"""
  o = []
  for p in lt:
    try:
      d = float(p.dRel)
      if 0. < d <= max_m:
        o.append([int(p.trackId), round(d, 1), round(float(p.yRel), 1), round(float(p.vRel), 1)])
    except Exception:
      pass
  o.sort(key=lambda x: x[1])
  return json.dumps({'src': 'tracks', 't': round(t_wall, 3), 'o': o}, separators=(',', ':'))


class Follow(object):
  """follows the newest file matching a pattern like `tail -F`: returns complete new lines only"""
  def __init__(self, pattern):
    self.pattern, self.path, self.pos = pattern, None, 0

  def _newest(self):
    files = glob.glob(self.pattern)
    return max(files, key=os.path.getmtime) if files else None

  def skip_to_end(self):
    self.path = self._newest()
    self.pos = os.path.getsize(self.path) if self.path else 0

  def read(self, max_bytes=4 << 20):
    out = []
    newest = self._newest()
    if newest is None:
      return out
    if self.path is not None and newest != self.path:
      out += self._read_from(self.path, max_bytes)       # finish the old file, then the new one from its start
      self.path, self.pos = newest, 0
    elif self.path is None:
      self.path, self.pos = newest, 0
    return out + self._read_from(self.path, max_bytes)

  def _read_from(self, path, max_bytes):
    try:
      with open(path, 'rb') as f:
        f.seek(self.pos)
        data = f.read(max_bytes)
    except OSError:
      return []
    end = data.rfind(b'\n')
    if end < 0:
      return []
    self.pos += end + 1
    return [l.decode('utf-8', 'replace') for l in data[:end].split(b'\n') if l.strip()]


def cap_dir(pattern, max_bytes, keep=()):
  """delete the oldest files matching pattern while their total is above max_bytes (never the ones in keep)"""
  files = sorted((f for f in glob.glob(pattern) if f not in keep), key=os.path.getmtime)
  total = sum(os.path.getsize(f) for f in files)
  while files and total > max_bytes:
    f = files.pop(0); total -= os.path.getsize(f)
    try:
      os.remove(f)
    except OSError:
      pass


def finish_leftovers(root=DRIVE_ROOT, active=None):
  """chunks left half-written by a reboot / crash: gzip them so they get uploaded too; picture folders whose chunk is
  finished (no .part next to them) are packed into their .tar. `active`: the folder of the drive being recorded now -
  its open chunk is left alone."""
  for part in glob.glob(os.path.join(root, '*', 'c*.jsonl.part')):
    if active and os.path.dirname(part) == active:
      continue
    try:
      gzip_file(part, part[:-len('.part')] + '.gz')
    except Exception:
      pass
  for fdir in glob.glob(os.path.join(root, '*', 'c*.frames')):
    if os.path.exists(fdir[:-len('.frames')] + '.jsonl.part'):
      continue                                        # its chunk is still being written
    try:
      tar_dir(fdir, fdir + '.tar')
    except Exception:
      pass

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
LEAD_FIELDS = ('status', 'dRel', 'vRel', 'yRel', 'aRel', 'vLeadK', 'aLeadK', 'vLead', 'aLeadTau', 'modelProb', 'radar', 'fcw')
LEAD_BOOLS = ('status', 'radar', 'fcw')   # radar: False = camera-only lead; modelProb 0 = radar-only lead


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
      out[f] = bool(v) if f in LEAD_BOOLS else round(float(v), 3)
    except Exception:
      out[f] = None
  return out


PLAN_FIELDS = (('src', 'longitudinalPlanSource'), ('v_start', 'vStart'), ('a_start', 'aStart'), ('v_target', 'vTarget'),
               ('a_target', 'aTarget'), ('v_target_future', 'vTargetFuture'), ('v_cruise', 'vCruise'), ('a_cruise', 'aCruise'),
               ('has_lead', 'hasLead'))


def plan_dict(p):
  """PLANLOG: the planner's output (m/s, m/s2). Any field missing on this schema -> None, never a crash."""
  out = {}
  for k, name in PLAN_FIELDS:
    try:
      v = getattr(p, name)
      out[k] = str(v) if k == 'src' else bool(v) if k == 'has_lead' else round(float(v), 3)
    except Exception:
      out[k] = None
  return out


BUTTONS_FILE = '/data/openpilot/selfdrive/car/tesla/buttons.msg'   # the screen's custom buttons (UIBT_module): name, label, label2
PLANNER_FILE = '/data/openpilot/selfdrive/controls/lib/planner.py'


def acc_mode(path=BUTTONS_FILE):
  """MODELOG: the ACC button's mode as shown on the screen ('follow', 'auto', 'cc', 'xnor' ...), None if unknown"""
  try:
    d = open(path, 'rb').read()
    for i in range(0, len(d) - 22, 23):
      name, _, label2 = struct.unpack('6s6s11s', d[i:i + 23])
      if name.rstrip(b'\0') == b'acc':
        return label2.rstrip(b'\0').decode('utf8', 'replace') or None
  except Exception:
    pass
  return None


def gap_patch_active(path=PLANNER_FILE):
  """MODELOG: True when tools/patch_planner_gap.py is applied to the installed planner (driver's gap), None if unknown"""
  try:
    return 'GAPSHIFT (tesla/tools/patch_planner_gap.py)' in open(path).read()
  except Exception:
    return None


def long_ctrl_dict(c):
  """PLANLOG: openpilot's longitudinal state (the planner only plans in 'pid') and the set speed it plans for (km/h)"""
  out = {'long_control_state': None, 'v_cruise_kph': None}
  try:
    out['long_control_state'] = str(c.longControlState)
  except Exception:
    pass
  try:
    out['v_cruise_kph'] = round(float(c.vCruise), 1)
  except Exception:
    pass
  return out


# ------------------------------------------------------------------ CLASSIFIED EXTRAS (field guide: LOG_FORMAT.md)
CTRL_FIELDS = (('v_pid', 'vPid'), ('v_target_lead', 'vTargetLead'), ('a_target', 'aTarget'), ('up_accel', 'upAccelCmd'),
               ('ui_accel', 'uiAccelCmd'), ('uf_accel', 'ufAccelCmd'), ('force_decel', 'forceDecel'), ('active', 'active'),
               ('engageable', 'engageable'), ('angle_steers_des', 'angleSteersDes'))
CC_FIELDS = (('pcm_speed', 'cruiseControl.speedOverride'), ('accel_override', 'cruiseControl.accelOverride'),
             ('cancel', 'cruiseControl.cancel'), ('hud_lead', 'hudControl.leadVisible'), ('hud_set_speed', 'hudControl.setSpeed'),
             ('gas', 'actuators.gas'), ('brake', 'actuators.brake'))
MODEL_LEAD_FIELDS = (('prob', 'lead.prob'), ('dist', 'lead.dist'), ('std', 'lead.std'), ('rel_vel', 'lead.relVel'),
                     ('rel_y', 'lead.relY'), ('future_prob', 'leadFuture.prob'), ('future_dist', 'leadFuture.dist'))
PATH_FIELDS = (('d_poly', 'dPoly'), ('lane_width', 'laneWidth'), ('l_prob', 'lProb'), ('r_prob', 'rProb'),
               ('angle_des', 'angleSteers'), ('mpc_valid', 'mpcSolutionValid'))
ALERT_FIELDS = (('text1', 'alertText1'), ('text2', 'alertText2'), ('status', 'alertStatus'), ('type', 'alertType'))


def fields_dict(obj, fields, nd=3):
  """{key: value} for (key, 'attr.sub') pairs of a message; numbers rounded, bools kept, lists of numbers kept, enums as
  text, anything missing -> None. Never raises (a field this schema does not have is just None)."""
  out = {}
  for k, a in fields:
    try:
      v = obj
      for part in a.split('.'):
        v = getattr(v, part)
      if isinstance(v, bool):
        out[k] = v
      elif isinstance(v, (int, float)):
        out[k] = round(float(v), nd)
      elif isinstance(v, str):
        out[k] = v
      elif hasattr(v, '__len__'):
        out[k] = [round(float(x), 5) for x in v]
      else:
        out[k] = str(v)
    except Exception:
      out[k] = None
  return out


SYS_PROCS = ('camerad', '_modeld', 'selfdrive.controls.controlsd', 'selfdrive.controls.plannerd',
             'selfdrive.controls.radard', 'boardd', '_ui', 'manager.py')
SYS_RESCAN_S = 5.0     # the process list is searched every 5 s; their CPU time is read every second


class SysSampler(object):
  """SYS class, once a second: what the EON itself is doing - so a camera hang, a crashing / restarting process, heat,
  memory or disk trouble shows in the log instead of being guessed afterwards. Reads /proc and /sys only."""
  def __init__(self, proc='/proc', sysfs='/sys', frame=FRAME_SRC, data='/data'):
    self.proc, self.sysfs, self.frame, self.data = proc, sysfs, frame, data
    self.pids, self.t_scan = {}, -1e9
    try:
      self.tick = os.sysconf('SC_CLK_TCK')
    except Exception:
      self.tick = 100

  def _scan(self):
    pids = {}
    for pid in os.listdir(self.proc):
      if not pid.isdigit():
        continue
      try:
        cmd = open(os.path.join(self.proc, pid, 'cmdline'), 'rb').read().replace(b'\0', b' ').decode('utf8', 'replace')
      except Exception:
        continue
      for w in SYS_PROCS:
        if w in cmd and w not in pids:
          pids[w] = int(pid)
    self.pids = pids

  def _cpu(self, pid):
    try:
      st = open(os.path.join(self.proc, str(pid), 'stat')).read().rsplit(')', 1)[1].split()
      return round((int(st[11]) + int(st[12])) / float(self.tick), 1)
    except Exception:
      return None

  def _read(self, *parts):
    try:
      return open(os.path.join(*parts)).read().strip()
    except Exception:
      return None

  def sample(self, now_mono, t_wall):
    if now_mono - self.t_scan >= SYS_RESCAN_S:
      self.t_scan = now_mono
      try:
        self._scan()
      except Exception:
        pass
    procs = {}
    for w, pid in list(self.pids.items()):
      cpu = self._cpu(pid)
      if cpu is None:
        del self.pids[w]                                # gone: found again (with a new pid) at the next scan
      else:
        procs[w] = [pid, cpu]
    temps = []
    tdir = os.path.join(self.sysfs, 'class', 'thermal')
    try:
      for z in sorted(os.listdir(tdir), key=lambda s: int(''.join(c for c in s if c.isdigit()) or 0)):
        v = self._read(tdir, z, 'temp')
        if v and v.lstrip('-').isdigit():
          temps.append(int(v) // 1000 if abs(int(v)) > 1000 else int(v))
    except Exception:
      pass
    mem = None
    try:
      for line in open(os.path.join(self.proc, 'meminfo')):
        if line.startswith('MemAvailable:'):
          mem = int(line.split()[1]) // 1024
    except Exception:
      pass
    try:
      sv = os.statvfs(self.data); free_gb = round(sv.f_bavail * sv.f_frsize / 1e9, 2)
    except Exception:
      free_gb = None
    try:
      st = os.stat(self.frame); frame = [round(t_wall - st.st_mtime, 2), st.st_size]
    except Exception:
      frame = None
    load = self._read(self.proc, 'loadavg')
    up = self._read(self.proc, 'uptime')
    bat = self._read(self.sysfs, 'class', 'power_supply', 'battery', 'capacity')
    usbfs = self._read(self.sysfs, 'module', 'usbcore', 'parameters', 'usbfs_memory_mb')
    return {'src': 'sys', 't': round(t_wall, 3), 'procs': procs,
            'temps': temps, 'temp_max': max(temps) if temps else None, 'mem_avail_mb': mem, 'data_free_gb': free_gb,
            'load': [float(x) for x in load.split()[:3]] if load else None,
            'uptime_s': round(float(up.split()[0]), 1) if up else None,
            'battery_pct': int(bat) if bat and bat.isdigit() else None,
            'camera_frame': frame, 'usbfs_mb': int(usbfs) if usbfs and usbfs.isdigit() else None}


KMSG_KEEP = re.compile(r'usb|uvc|xhci|camera|ispif|msm_cam|thermal|oom|out of memory|killed process|panda', re.I)
KMSG_DROP = re.compile(r'avc:|audit', re.I)


class Kmsg(object):
  """KERNEL class: new kernel messages about USB / the camera / heat / memory, read from /dev/kmsg without blocking
  (e.g. 'usb 1-1.3: can't re-read device descriptor ... -110', 'usb_submit_urb returned -12' - the camera hangs)"""
  def __init__(self, path='/dev/kmsg'):
    try:
      self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
      os.lseek(self.fd, 0, os.SEEK_END)
    except Exception:
      self.fd = None

  def read(self, max_n=200):
    out = []
    if self.fd is None:
      return out
    for _ in range(max_n):
      try:
        rec = os.read(self.fd, 8192)
      except BlockingIOError:
        break
      except OSError:
        continue                                        # records overwritten while we were away (EPIPE): skip
      if not rec:
        break
      line = kmsg_line(rec)
      if line:
        out.append(line)
    return out


def kmsg_line(rec):
  """KERNEL class: the message text of one /dev/kmsg record if it is about USB / camera / heat / memory, else None"""
  line = rec.decode('utf8', 'replace').split(';', 1)[-1].split('\n')[0].strip()
  return line[:300] if KMSG_KEEP.search(line) and not KMSG_DROP.search(line) else None


VERSION_FILES = (('acc', '/data/openpilot/selfdrive/car/tesla/ACC_module.py'),
                 ('carcontroller', '/data/openpilot/selfdrive/car/tesla/carcontroller.py'),
                 ('planner', PLANNER_FILE), ('radard', '/data/openpilot/selfdrive/controls/radard.py'),
                 ('controlsd', '/data/openpilot/selfdrive/controls/controlsd.py'),
                 ('camerad', '/data/openpilot/selfdrive/camerad/camerad'))


def versions(files=VERSION_FILES, repo='/data/claude-stuff'):
  """META class, at drive start: md5 of the installed openpilot files that matter + which patches are in, + the
  claude-stuff commit - so every drive says exactly what was running"""
  import hashlib
  out = {}
  for k, p in files:
    try:
      out[k] = hashlib.md5(open(p, 'rb').read()).hexdigest()
    except Exception:
      out[k] = None
  try:
    out['radard_lead_patch'] = 'RADARLEAD (tesla/tools/patch_radard_lead.py)' in open(dict(files)['radard']).read()
  except Exception:
    out['radard_lead_patch'] = None
  try:
    head = open(os.path.join(repo, '.git', 'HEAD')).read().strip()
    out['claude_stuff'] = open(os.path.join(repo, '.git', head[5:])).read().strip()[:10] if head.startswith('ref: ') else head[:10]
  except Exception:
    out['claude_stuff'] = None
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
def run(out_path, hz=20.0, duration=None, log=print, auto=False):
  import cereal.messaging as messaging  # local import: keeps this file importable/testable off-car
  sl = _load_steer_limits()

  topics = ('controlsState', 'carState', 'radarState', 'model', 'carControl', 'gpsLocationExternal') + (('plan', 'liveTracks', 'pathPlan') if auto else ())
  # openpilot's message queues allow a limited number of readers per message and may not exist yet right after boot:
  # connecting can raise MessagingError. The essential ones are retried until they work; the extras are optional and
  # retried every 30 s, so one busy queue (model is the busiest) can never stop the recording.
  REQUIRED = ('controlsState', 'carState', 'radarState')
  def connect(name, conflate=True):
    try:
      return messaging.sub_sock(name, conflate=conflate)
    except Exception as e:
      return e
  subs, retry_at = {}, {}
  for name in topics + ('sendcan',):
    while True:
      s = connect(name, conflate=(name != 'sendcan'))
      if not isinstance(s, Exception):
        subs[name] = s; break
      if name not in REQUIRED:
        log('driving_logger: %s not available yet (%r) - recording without it, retry in 30 s' % (name, s))
        subs[name] = None; retry_at[name] = time.monotonic() + 30.; break
      log('driving_logger: cannot subscribe to %s yet (%r) - retrying in 5 s' % (name, s)); time.sleep(5.)
  sc = subs['sendcan']    # drained in full every tick, like lightdash - a flood here never starves us
  def rx(name):
    s = subs.get(name)
    if s is None:
      if name in retry_at and time.monotonic() >= retry_at[name]:
        s = connect(name, conflate=(name != 'sendcan'))
        if isinstance(s, Exception):
          retry_at[name] = time.monotonic() + 30.; return None
        subs[name] = s; del retry_at[name]; log('driving_logger: %s connected' % name)
      else:
        return None
    return messaging.recv_one_or_none(s)

  clf = Classifier()
  last_ctrl = last_long = last_steer = None
  last_das = decode_0x659(None)
  last_actuator_angle = None
  last_model = model_dict(None)
  last_gps = gps_dict(None)
  t_ctrl = t_659 = None
  prev_request, prev_t_659 = None, None     # for the steer_limits hypotheticals, see module docstring
  reported_fields = False
  last_leads, t_rad = (lead_dict(None), lead_dict(None)), None
  last_plan, t_plan, last_long_ctrl = plan_dict(None), None, {'long_control_state': None, 'v_cruise_kph': None}
  last_mode, t_mode = None, None
  last_tracks, t_tracks = None, None      # RADAR OBJECTS: last liveTracks list
  last_ctrl_x, last_cc_x = fields_dict(None, CTRL_FIELDS), fields_dict(None, CC_FIELDS)    # CLASSIFIED EXTRAS (LOG_FORMAT.md)
  last_model_lead, last_path = fields_dict(None, MODEL_LEAD_FIELDS), fields_dict(None, PATH_FIELDS)
  last_alert, written_alert = None, None
  sysm, kmsg, t_sys = (SysSampler(), Kmsg(), -1e9) if auto else (None, None, -1e9)          # MODELOG: ACC button mode, re-read every 2 s (you can change it while driving)

  # RECORDER: in --auto the output is a ChunkWriter opened when a drive starts and closed when it ends
  follows, shadow, last_car_t, last_plan_fcw, last_mm = [], None, None, False, None
  if auto:
    f = None
    finish_leftovers()
    follows = [(src, key, Follow(p)) for src, key, p in FOLLOW]
    frames = Frames()
    last_lead_seen, t_trig = None, 0.
    try:
      sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
      import fcw_shadow
      shadow = fcw_shadow.ShadowCompare()
    except Exception as e:
      fcw_shadow = None
      log('driving_logger: FCW comparison not available (%r)' % (e,))
  else:
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

      if sc is None and 'sendcan' in retry_at and time.monotonic() >= retry_at['sendcan']:
        s_ = connect('sendcan', conflate=False)
        if isinstance(s_, Exception):
          retry_at['sendcan'] = time.monotonic() + 30.
        else:
          sc = subs['sendcan'] = s_; del retry_at['sendcan']; log('driving_logger: sendcan connected')
      for ev in (messaging.drain_sock(sc, wait_for_one=False) if sc is not None else ()):
        for c in ev.sendcan:
          if c.address == 0x659 and c.src == 0:
            last_das = decode_0x659(bytes(c.dat))
            last_long, last_steer = last_das['long_enabled'], last_das['steer_enabled']
            t_659 = now_mono

      cs = rx('controlsState')
      if cs is not None:
        try:
          last_ctrl = bool(cs.controlsState.enabled)
        except Exception:
          last_ctrl = None
        last_long_ctrl = long_ctrl_dict(cs.controlsState)
        last_ctrl_x = fields_dict(cs.controlsState, CTRL_FIELDS)
        last_alert = fields_dict(cs.controlsState, ALERT_FIELDS)
        t_ctrl = now_mono

      car = rx('carState')
      rad = rx('radarState')
      if auto:
        pl = rx('plan')
        if pl is not None:
          try:
            last_plan_fcw = bool(pl.plan.fcw)
          except Exception:
            pass
          last_plan, t_plan = plan_dict(pl.plan), now_mono
        if car is not None:
          last_car_t = now_mono
        driving = last_car_t is not None and now_mono - last_car_t < DRIVE_END_S
        if driving and f is None:                       # a drive starts: new chunk set, followed files from their end
          f = ChunkWriter()
          for _, _, fo in follows:
            fo.skip_to_end()
          if fcw_shadow is not None:
            shadow = fcw_shadow.ShadowCompare()
          f.write(json.dumps({'src': 'meta', 'meta': 'drive start', 't': round(time.time(), 3), 'hz': hz, 'drive': f.drive,
                              'acc_mode': acc_mode(), 'planner_gap_patch': gap_patch_active(), 'versions': versions()}) + '\n')
          log('driving_logger: drive %s started' % f.drive)
        elif not driving and f is not None:             # the drive ended: last chunk out, keep the disk in bounds
          f.write(json.dumps({'src': 'meta', 'meta': 'drive end', 't': round(time.time(), 3)}) + '\n')
          f.close(); log('driving_logger: drive %s ended, %d chunks' % (f.drive, f.n)); f = None
          finish_leftovers()                              # anything the last chunk left behind goes up too
          cap_dir('/data/maplog/map_*.jsonl', MAPLOG_MAX_BYTES)
          cap_dir(os.path.join(DRIVE_ROOT, '*', 'c*.jsonl.gz'), DRIVELOG_MAX_BYTES)
        if f is None:                                   # not driving: nothing to record, poll slowly
          time.sleep(0.5); next_t = time.monotonic()
          continue
      cc = rx('carControl')
      if cc is not None:
        try:
          last_actuator_angle = float(cc.carControl.actuators.steerAngle)
        except Exception:
          last_actuator_angle = None
        last_cc_x = fields_dict(cc.carControl, CC_FIELDS)
      pp = rx('pathPlan') if auto else None
      if pp is not None:
        last_path = fields_dict(pp.pathPlan, PATH_FIELDS)

      mm = rx('model')
      if mm is not None:
        last_mm = mm
        try:
          last_model = model_dict(mm.model)
        except Exception:
          pass
        last_model_lead = fields_dict(mm.model, MODEL_LEAD_FIELDS)
      gp = rx('gpsLocationExternal')
      if gp is not None:
        try:
          last_gps = gps_dict(gp.gpsLocationExternal)
        except Exception:
          pass

      state = clf.update(now_mono, last_ctrl, None if t_ctrl is None else now_mono - t_ctrl,
                          last_long, None if t_659 is None else now_mono - t_659,
                          last_steer, None if t_659 is None else now_mono - t_659)

      sample = {'t': round(time.time(), 3), 'state': state}
      if auto:
        sample = {'src': 'car', 't': sample['t'], 'state': state}
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

      # radarState comes at ~20 Hz, like this loop, so some ticks get no new message: keep the last leads ("sticky",
      # like model / GPS) instead of writing all-None - that looked like the lead car flickering in and out.
      if rad is not None:
        try:
          last_leads = (lead_dict(rad.radarState.leadOne), lead_dict(rad.radarState.leadTwo))
        except Exception:
          last_leads = (lead_dict(None), lead_dict(None))
        t_rad = now_mono
      fresh = t_rad is not None and now_mono - t_rad < 0.5
      sample['lead1'], sample['lead2'] = last_leads if fresh else (lead_dict(None), lead_dict(None))

      # PLANLOG: what openpilot's longitudinal planner wants (the ACC redesign follows it) - None when older than 0.5 s
      sample['plan'] = last_plan if t_plan is not None and now_mono - t_plan < 0.5 else plan_dict(None)
      if t_mode is None or now_mono - t_mode > 2.:
        last_mode, t_mode = acc_mode(), now_mono
      sample['acc_mode'] = last_mode
      sample.update(last_long_ctrl if t_ctrl is not None and now_mono - t_ctrl < STALE_S else long_ctrl_dict(None))
      sample['model'] = last_model
      if auto:                                     # CLASSIFIED EXTRAS - see LOG_FORMAT.md
        sample['ctrl'] = last_ctrl_x               # openpilot's longitudinal control (controlsState)
        sample['cc'] = last_cc_x                   # what openpilot sent: pcm_speed (speed to cruise), HUD lead ...
        sample['model_lead'] = last_model_lead     # the CAMERA model's own lead (prob < 0.5 -> no camera lead)
        sample['path'] = last_path                 # predicted path / lanes (pathPlan) - places radar objects
      sample['gps'] = last_gps

      if not reported_fields and car is not None and rad is not None:
        f.write(json.dumps({'src': 'meta', 'meta': 'fields_seen_once', 't': time.time(),
                             'carState_sample_keys': sorted(k for k in sample if not k.startswith('das_')),
                             'lead1': sample['lead1']}) + '\n')
        reported_fields = True

      f.write(json.dumps(sample) + '\n')
      n += 1
      if auto:
        trig = None
        for src, key, fo in follows:                     # ACC decisions + radar/map lines written since the last tick
          for line in fo.read():
            f.write('{"src":"%s","%s":%s}\n' % (src, key, line))
            if src == 'acc' and trig != 'cancel':
              try:
                b_ = json.loads(line)[10]
                if b_ == 1:
                  trig = 'cancel'                        # cruise cancelled (regen): a priority moment
                elif b_ and trig is None:
                  trig = 'acc_press'
              except Exception:
                pass
        # PICTURES: what happened this tick that is worth a look
        seen = bool((sample.get('lead1') or {}).get('status'))
        if last_lead_seen is not None and seen != last_lead_seen and trig in (None, 'acc_press'):
          trig = 'lead_in' if seen else 'lead_out'
        last_lead_seen = seen
        l1_ = sample.get('lead1') or {}
        v_ = sample.get('v_ego') or 0.
        if seen and v_ > 10. and l1_.get('dRel') and l1_['dRel'] / v_ < GAP_SHORT_S and trig in (None, 'acc_press'):
          trig = 'gap_short'                             # following closer than 1.5 s
        # RADAR OBJECTS: everything the radar tracks, TRACKS_HZ times a second
        lt_ = rx('liveTracks')
        if lt_ is not None:
          last_tracks, t_tracks = lt_.liveTracks, now_mono
        if last_tracks is not None and now_mono - t_tracks < 0.5 and n % max(1, int(hz / TRACKS_HZ)) == 0:
          f.write(tracks_line(last_tracks, time.time()) + '\n')
        # ALERT class: what openpilot showed you, when it changes
        if last_alert is not None and last_alert != written_alert:
          written_alert = last_alert
          f.write(json.dumps(dict(last_alert, src='alert', t=round(time.time(), 3))) + '\n')
        # SYS + KERNEL classes, once a second
        if now_mono - t_sys >= 1.0:
          t_sys = now_mono
          try:
            f.write(json.dumps(sysm.sample(now_mono, time.time()), separators=(',', ':')) + '\n')
          except Exception:
            pass
          kl = kmsg.read()
          if kl:
            f.write(json.dumps({'src': 'kernel', 't': round(time.time(), 3), 'lines': kl}) + '\n')
        if abs(sample.get('a_ego') or 0.) > 0.5 and trig is None:
          trig = 'accel' if (sample.get('a_ego') or 0.) > 0 else 'decel'
        if shadow is not None and car is not None and rad is not None:   # FCW comparison (fcw_shadow.py's logic)
          try:
            l1, cs_ = rad.radarState.leadOne, car.carState
            mf = fcw_shadow.model_fcw_from(last_mm.model, cs_.brakePressed) if last_mm is not None else None
            for e in shadow.step(time.time(), cs_.vEgo, l1.status, l1.dRel, l1.vLead, l1.yRel,
                                 cs_.leftBlinker or cs_.rightBlinker, last_plan_fcw, mf):
              e['src'] = 'fcw'
              f.write(json.dumps(e) + '\n')
              trig = trig or 'fcw'
          except Exception:
            pass
        if not os.path.exists(FRAMES_OFF):
          now_w = time.time()
          if trig and now_w - t_trig > 4.:              # one moment at a time; the window covers the next seconds
            t_trig = now_w; before_, after_ = frame_window(trig); frames.mark(now_w, trig, f, before=before_, after=after_)
          frames.tick(now_w, f)
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
    if f is not None:
      f.flush()
      f.close()
    log('driving_logger: stopped, %d samples in %s' % (n, DRIVE_ROOT if auto else out_path))


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('---')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--out', default='/data/drivelog_%d.jsonl' % int(time.time()))
  ap.add_argument('--hz', type=float, default=20.0)
  ap.add_argument('--duration', type=float, default=None, help='seconds; default: run until Ctrl-C')
  ap.add_argument('--auto', action='store_true', help='THE recorder: run forever, record every drive in chunks to %s' % DRIVE_ROOT)
  args = ap.parse_args(argv)
  if args.auto:
    os.makedirs(DRIVE_ROOT, exist_ok=True)
    since = os.path.join(DRIVE_ROOT, '.since')     # log_uploader: separate maplog/acclog files after this are already in the chunks
    if not os.path.exists(since):
      open(since, 'w').close()
    import fcntl
    lock = open(os.path.join(DRIVE_ROOT, '.lock'), 'w')
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)        # one recorder only (boot line + a manual start)
    except OSError:
      print('driving_logger: the recorder is already running'); return
  def log(msg):                                        # flushed: the boot line sends this to a file (/data/recorder.log)
    print('%s %s' % (time.strftime('%Y-%m-%d %H:%M:%S'), msg), flush=True)
  if args.auto:
    log('driving_logger: recorder running, waiting for a drive (openpilot started)')
  if not args.auto:
    run(args.out, hz=args.hz, duration=args.duration, log=log)
    return
  while True:                                          # the recorder never stays down: any error -> restart in 10 s
    try:
      run(args.out, hz=args.hz, duration=args.duration, log=log, auto=True)
      return
    except KeyboardInterrupt:
      return
    except Exception as e:
      import traceback
      log('driving_logger: recorder error %r - restarting in 10 s\n%s' % (e, traceback.format_exc()))
      time.sleep(10.)


if __name__ == '__main__':
  main()
