# Steering-angle limits: car side + panda firmware, as ONE change

Why: our panda firmware (BogGyver `master`, v1.7.3 as uploaded) checks the steering request per message with the 250 ms tables
x 25 = about 626 deg per message, i.e. no limit, and our carcontroller's rate clip is commented out. Nothing limited how fast
the request could move. The tesla-unity carcontroller limits it (rate table + clip to +-20 deg of the measured angle).

Two halves that only work together:

| half | file | what |
|---|---|---|
| car | `tesla/steer_limits.py` (opt-in, `/data/steer_limits_on`) | tesla-unity table, per 20 ms message: up 5/4/.75, down 5/5.5/2.4 deg at 0/5/15 m/s, clip +-20 deg to measured. Strength 1.0 = full |
| firmware | `tesla/panda/angle_rate_per_message.patch` | per-message check with factor 1 and tables 2x the car side: up 10/8/1.5, down 10/11/4.8 deg at 2/7/17 m/s (+1 deg margin) |

A violation in the firmware clears `controls_allowed` = openpilot disengages. So:
- firmware patched + car layer off or partial (e.g. 0.25) => legitimate steps can trip it => disengagements ("WILL TRIP")
- firmware unpatched + car layer on => safe, only the car limits apply ("UNPATCHED")
- both => "PAIRED"

## Order (do not skip)
1. `git pull origin offline-nav` in /data/claude-stuff, then `sh /data/claude-stuff/tesla/deploy_car.sh --yes` (copies steer_limits.py and the carcontroller), reboot.
2. `echo 1 > /data/steer_limits_on` (full strength). Drive with the car-side layer first; watch with lightlog (`ang_cmd` vs `ang`).
3. `python3 /data/claude-stuff/tesla/panda/check_pairing.py` -> must say UNPATCHED (before) .
4. Apply the firmware patch to the tree your flasher builds from:
   `cd /data/openpilot/panda && patch -p1 --dry-run < /data/claude-stuff/tesla/panda/angle_rate_per_message.patch && patch -p1 < /data/claude-stuff/tesla/panda/angle_rate_per_message.patch`
5. `python3 /data/claude-stuff/tesla/panda/check_pairing.py` -> must say PAIRED.
6. Flash with your tool: `pkill -f manager.py ; sleep 2 ; pkill boardd ; python3 /data/pandatool.py flash` and check `status`.

## Undo
`cd /data/openpilot/panda && patch -R -p1 < /data/claude-stuff/tesla/panda/angle_rate_per_message.patch`, flash again. Car side: `rm /data/steer_limits_on`.

## What was and was not verified
- Verified: patch applies cleanly to the uploaded v1.7.3 `safety_tesla.h`; GMLAN lines (pandatool's check) untouched; a Python replica of the
  firmware check had 0 violations on 80,000 messages of car-limited steering at 3-35 m/s and trips on a 40 deg step;
  `check_pairing.py` and its tests (`tests/test_panda_pairing.py`).
- NOT verified: compilation for the STM32 (use your `build_st`), the real 0x659 message rate (assumed ~50 Hz), behaviour on the car.
- Does not stop a slow 0.35-0.4 Hz wiggle (peak rate ~25 deg/s is below both limits): it protects against sudden jumps.
