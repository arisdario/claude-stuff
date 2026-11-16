# Drive log format (`driving_logger.py --auto`)

One drive = `/data/drivelog/<YYYYmmdd_HHMMSS>/cNNNN.jsonl.gz` (2 minutes each) + `cNNNN.frames.tar` (pictures), uploaded
live to the `eon-logging` repo as release `drive-<YYYYmmdd_HHMMSS>`. Every line is one JSON object; **`src` says which
class it is**. Times `t` are the EON clock (seconds since 1970, UTC). Distances m, speeds m/s, accelerations m/s² unless
the name says `kph`. A field the message does not have is `null` (never a crash).

| class (`src`) | rate | what it answers |
|---|---|---|
| `meta` | drive start / end | what was running: ACC mode, patches, file checksums |
| `car` | 20 / s | everything about the car and openpilot at that moment |
| `tracks` | 5 / s | every object the radar tracks (not only the two openpilot picked) |
| `acc` | 20 / s while ACC is on | what the stalk-ACC saw and pressed, and why |
| `alert` | on change | what openpilot showed you |
| `sys` | 1 / s | the EON's health: processes, CPU, heat, memory, disk, camera frames |
| `kernel` | when it happens | kernel messages about USB / camera / heat / memory |
| `fcw` | on events | FCW shadow comparison (`fcw_shadow.py`) |
| `ahb` | 5 / s while `/data/ahb_vision_on` exists | what a camera automatic high beam WOULD do (shadow, nothing switched) |
| `map` | as written | radarweb map lines (only when radarweb runs) |

## `meta`
`{"src":"meta","meta":"drive start","t","hz","drive","acc_mode","planner_gap_patch","versions":{...}}` and `"drive end"`.
- `acc_mode`: the ACC button's mode (`follow`, `auto`, `cc`, `xnor`).
- `ahb_vision_on`: whether camerad's light finder was switched on for this drive.
- `planner_gap_patch` / `versions.radard_lead_patch`: whether those patches are installed.
- `versions`: md5 of the installed `acc` (ACC_module.py), `carcontroller`, `planner`, `radard`, `controlsd`, `camerad`
  + `claude_stuff` (commit of /data/claude-stuff). Compare with git to know exactly which code drove.

## `car` (20 / s)
| group | fields | source |
|---|---|---|
| state | `state` (human / engaged / …), `ctrl_enabled`, `das_*` (what was sent to the cluster: 0x659 bits) | controlsState + sendcan |
| vehicle | `v_ego`, `a_ego`, `standstill`, `steering_angle`, `steering_rate`, `steering_torque`, `brake_lights` | carState |
| driver | `gas`, `gas_pressed`, `steering_pressed`, `left_blinker`, `right_blinker` (`brake_pressed_UNRELIABLE` is always False on this car) | carState |
| cruise | `cruise_speed` (m/s, your set speed as openpilot sees it), `cruise_enabled_UNRELIABLE` | carState |
| `acc_mode` | ACC button mode, re-read every 2 s | buttons.msg |
| planner | `plan` {`src` cruise/mpc1/mpc2/model, `v_start`, `a_start`, `v_target`, `a_target`, `v_target_future` (what xnor follows), `v_cruise`, `a_cruise`, `has_lead`}, `long_control_state` (planning only in `pid`), `v_cruise_kph` | plan, controlsState |
| `ctrl` | `v_pid`, `v_target_lead`, `a_target`, `up_accel`, `ui_accel`, `uf_accel`, `force_decel`, `active`, `engageable`, `angle_steers_des` | controlsState |
| `cc` | `pcm_speed` (speed openpilot asks cruise for), `accel_override`, `cancel`, `hud_lead`, `hud_set_speed`, `gas`, `brake` | carControl |
| leads | `lead1`, `lead2` {`status`, `dRel`, `vRel`, `yRel`, `aRel`, `vLead`, `vLeadK`, `aLeadK`, `aLeadTau`, `modelProb`, `radar`, `fcw`} — `radar` False = camera-only lead, `modelProb` 0 = radar-only lead (radard patch), `lead2` = the car beyond the lead with the patch | radarState |
| `model_lead` | the CAMERA model's own lead: `prob` (below 0.5 openpilot does not accept a car), `dist`, `std`, `rel_vel`, `rel_y`, `future_prob`, `future_dist` | model |
| `model_plan` | what the camera model ITSELF plans — the 0.7.5 planner never reads it, it is logged to compare: `v` / `a` = the speed (m/s, absolute) / acceleration it plans at t = 0, 1 … 9 s; `p_brake`, `p_gas`, `p_steer` = its probability that the driver brakes / presses gas / steers; `p_engaged`; model FCW `hard_brake`, `p_brake5`, `p_brake3` (P of braking 5 / 3 m/s²). All `null` when the model message is older than 0.5 s | model (0.8.12 adapter) |
| `model` | path / lane points every 4 m to 100 m, lane probabilities | model |
| `path` | `d_poly` (predicted path, used to place radar objects), `lane_width`, `l_prob`, `r_prob`, `angle_des`, `mpc_valid` | pathPlan |
| `gps` | `latitude`, `longitude`, `altitude`, `accuracy`, `speed`, `bearingDeg` (the car's own GPS) | gpsLocationExternal |
| steering limits | `actuator_steer_angle(_neg)`, `sl20_angle`, `sl25_angle` (hypothetical limits, never applied) | carControl |

## `tracks` (5 / s)
`{"src":"tracks","t","o":[[trackId, dRel, yRel, vRel], ...]}` — every radar object within 250 m, nearest first.
`yRel` positive = left. Absolute speed of an object = `v_ego` + `vRel`; ~0 = stationary, negative = oncoming.

## `acc` (20 / s while ACC is on)
`{"src":"acc","r":[t, v_ego, a_ego, set kph (car), your kph, lead seen, lead dRel, lead vRel, lead aRel, curve cap kph,
button, reason, cruise state, brake, gas, ...]}` — button: 0 none, 16 +1, 4 +5, 32 −1, 8 −5, 1 cancel. `reason` is the
ACC's own text (`xnor: -1 (plan 112 -> aim 110.5)`, `xnor: cancel (braking needed 0.8 m/s2)`, …). Cruise state 2 = on.

## `alert` (on change)
`{"src":"alert","t","text1","text2","status","type"}`.

## `sys` (1 / s)
`{"src":"sys","t","procs":{name:[pid, cpu seconds]},"temps":[°C per thermal zone],"temp_max","mem_avail_mb",
"data_free_gb","load":[1,5,15 min],"uptime_s","battery_pct","camera_frame":[age s, bytes],"usbfs_mb"}`
- `procs`: camerad, _modeld, controlsd, plannerd, radard, boardd, _ui, manager.py. A **pid change = that process was
  restarted**; **camerad cpu not growing + `camera_frame` age growing = the camera hung**.

## `kernel` (when it happens)
`{"src":"kernel","t","lines":[...]}` — new kernel messages about usb / uvc / xhci / camera / thermal / out-of-memory
(e.g. `usb 1-1.3: ... -110` = the C920 stopped answering, `usb_submit_urb returned -12` = USB transfer memory full).

## `ahb` (5 / s, only while camerad's light finder runs - `tesla/camerad/AHB_VISION.md`)
`{"src":"ahb","t","state","why","v","lead_d","n","lamps","luma","horizon","lights":[[x,y,area,halo],...]}`
- `state`: `high` / `dip` = what it would do. `why`: `clear`, or the first reason to dip: `no_camera`, `slow`
  (< 30 km/h; high again from 40), `headlights`, `car_ahead` (openpilot's lead < 150 m), `lit_road` (> 2 street lamps
  above the horizon), `bright` (band brightness > 90). High only after 3 s of `clear`.
- `n`: light sources near the horizon; `lights`: their full-res pixel (upright frame), core area, halo share.

## Pictures (`cNNNN.frames.tar`)
`HHMMSS.s_<why>.jpg` (EON local time; the raw C920 frame is upside down — rotate 180°). Priority moments (`lead_out`,
`lead_in`, `gap_short` = following closer than 1.5 s, `cancel`, `fcw`) get ±10 s / +6 s of pictures and 12 extra per
chunk; routine ones (`acc_press`, `accel`, `decel`) one before and one after; plus one `periodic` per minute.
`ahb_dip` / `ahb_high` (the shadow high-beam decision changed) have their own 12 per chunk.
