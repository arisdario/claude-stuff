# Instrument cluster (IC) integration: what the CAN dumps show

The cluster shows Autopilot state, lane lines, lead/side cars and cruise speed from DAS messages the panda fakes
(`do_fake_DAS` in `safety_tesla.h`), filled with data openpilot sends to the panda in 0x659 (status, steering),
0x557 (lanes, lead), 0x559 (objects), 0x554 (warnings, emulation on/off), 0x556 (signs), 0x65A (beams, fleet speed).

## Findings from the 25 Oct radarweb CAN dumps

**1. 22:44 drive: DAS emulation was off.** None of the cluster messages (0x399 DAS_status, 0x389 DAS_status2,
0x239 DAS_lanes, 0x3E9 DAS_bodyControls, ...) appear at all, while 0x488 / 0x214 (sent *before*
`if (enable_das_emulation == 0) return;`) do. openpilot sends that flag in 0x554 from `enable_das_emulation` in
`/data/bb_openpilot.cfg` (default **False**).

**2. 21:31 drive: emulation on, but the panda never used openpilot's status data.** For all 48 minutes every cluster
message was identical: Autopilot state 1 (unavailable), no lane lines, no objects, cruise limit "not available"
(204.6 mph = 0x3FF), lane change state 5 - the panda's start-up defaults. Meanwhile openpilot was engaged 1,292 s and
sent op_status 5 / cruise state 2 / a cruise limit in 0x659 every second:

| 0x659 from openpilot | 0x399 / 0x389 to the cluster | seconds |
|---|---|---|
| op_status 5 (active), cc_state 2 | op_status 1, limit SNA | 1,096 |
| op_status 2-3 | op_status 1, limit SNA | 1,741 |
| steering angle bytes | same angle in 0x488 | 2,894 of 2,901 |

So the firmware on the panda takes the **steering** bytes from 0x659 but not the **status** bytes. In the
`safety_tesla.h` you have, the same 0x659 handler reads both (and openpilot's packing in
`teslacan.create_fake_DAS_msg` matches that parser byte for byte), so **the panda is running different, probably older,
firmware** - the handler's comment `if (addr == 0x659) { //0x553) {` suggests a version that took the status from
0x553, which openpilot no longer sends. The car clock the panda uses for its 2 s staleness check (0x318) decodes
correctly, so that is not the cause.

**3. Smaller bugs that corrupt the lane display:**
- `safety_tesla.h`: `int lWidth = 29;` is packed into a 4-bit field (`lWidth << 4`); its top bit spills into the
  lane view range byte. The 21:31 cluster data shows exactly that: width 6.06 m, range 161 m.
- `teslacan.py` (fixed here): lane width was packed without clipping; 0 m before the first pathPlan wrapped to 5.1 m.

## What to do

1. `/data/bb_openpilot.cfg`: `enable_das_emulation = True` (and keep `enable_radar_emulation = True`), reboot.
2. **Reflash the panda** with firmware built from the `safety_tesla.h` that matches this openpilot (the one with the
   full 0x659 handler). Check what is flashed first: openpilot normally reflashes only when the version string differs,
   so a panda with older code but the same version string is never updated. Use your fork's flash procedure
   (on these trees usually `cd /data/openpilot/panda/board && make` with `boardd` stopped).
3. In `safety_tesla.h`, fix the default lane width (one line) - it only shows before openpilot's first 0x557, but it
   is what the cluster displays whenever the panda has no lane data:
   ```c
   int lWidth = 5;            // 4-bit field: 2.0 + 5 * 0.3125 = 3.56 m. Was 29, which overflowed into laneRange
   ```
   and mask it where DAS_lanes is packed:
   ```c
   MLB = ((lWidth & 0x0F) << 4) + (rLine << 1) + lLine + (laneRange << 8) + (curvC0 << 16) + (curvC1  << 24);
   ```

## Check after reflashing

Drive with openpilot engaged and radarweb running, then on the EON (newest log):

```
python3 - <<'EOF'
import json, glob, collections
f = sorted(glob.glob('/data/maplog/map_*.jsonl'))[-1]; c = collections.Counter()
for l in open(f):
    r = json.loads(l).get('raw', {}); a, s = r.get('0x659'), r.get('0x399')
    if a and s: c[(int(a[:2], 16) & 15, int(s[:2], 16) & 15)] += 1
print(f, '(op_status sent by openpilot, op_status sent to the cluster): seconds', dict(c))
EOF
```

Fixed when the two numbers match (e.g. `(5, 5)` while engaged). If 0x399 doesn't appear at all, the DAS emulation is
still off (step 1).

The C snippets were not compiled here (no panda toolchain in this environment).

## MCU2: bench results (26 Oct) and conclusion

`bench/benchdas.py` sent the complete DAS set (op_status 5, both lanes, lead at 40 m, set speed 80) with the panda's
exact packing (`--selftest` reproduces the car's frames, including EPB 0x214):

| Test | Cluster |
|---|---|
| A: DAS set, car's own 0x398 (no AP) | nothing; set speed stays at the drive inverter's 0 (0x368 `DI_cruiseSet`) |
| B: + spoofed 0x398 (dasHw/radar/autopilot = 1) | nothing; parking-brake fault (the EPB expected the AP computer's 0x214) |
| B2: + 0x214 | nothing |

The DAS messages have the same IDs and layouts in the Model 3 DBC (0x239, 0x2B9, 0x309, 0x389, 0x399, 0x3E9), and
none of the Model 3 UI/config messages (0x7FF, 0x3F8, 0x3FD, 0x293) appear on this car's CAN. Tinkla, whose fork this
is, states that on MCU2 openpilot works but does not display on the instrument cluster, and that its Tinkla Buddy
(which sits between the MCU and the cluster or the AP computer) supports MCU1 only. So on MCU2 the cluster draws the
Autopilot view from the MCU2 itself, which takes the "AP fitted" setting from the gateway config, not from these CAN
messages. Nothing the panda sends can change that; don't spoof 0x398 or 0x368 in the car.
