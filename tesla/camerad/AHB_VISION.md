# Shadow automatic high beams from the C920 (camerad `ahb_vision.cc` + logger `ahb_shadow.py`)

**Status: SHADOW ONLY. Nothing switches the lights.** It records what an automatic high beam *would* do, with pictures,
so a night drive shows how often it is right before it is ever connected. Whether it *can* be connected through the
light lever is a separate question, answered by the parked test in `tesla/ahb_stalk.py`.

Meant to work like Tesla's own: you push the lever out (high beams); the system dips for traffic and restores when clear.

## How it sees

| question | answer from | how |
|---|---|---|
| oncoming car? | camera | a **light source**: a saturated, compact core **with a glow halo** around it, in a band around the horizon |
| car ahead? | radar + model lead (`lead1`) | already tracked to 160–200 m; tail lights are not searched in the picture |
| lit road? | camera | street lamps = light sources clearly **above** the horizon |
| dark enough? | camera | brightness of the band around the horizon |

Geometry: a car's lights are 0.6–1 m above the road and the C920 ~1.2 m, so at any distance they appear at or a
little below the horizon. Street lamps (6–12 m up) appear clearly above it. Lane markings, signs and guardrail
reflectors lit by our own beam are reflections: elongated and/or without a halo, so they are ignored.

## What was measured (368 night pictures, 15 Nov 2026)

- Plain "bright spot" search: useless. Guardrail reflectors look like tail lights; our own lit lane markings gave about 9
  false "lights" per picture.
- Core + halo + horizon band (this code): finds the oncoming cars (19:54:55, 19:54:59) and cars across the
  central barrier, drops most markings. Still wrong sometimes: a guardrail reflector (19:40:57), a lit marking (16:25:44).
  **Accuracy is not measured yet**: that is what the shadow drive is for.
- The C++ code gives exactly the Python prototype's result on all 368 pictures. Busy scene: 0.28 ms per run on a PC.
- On those pictures the decision would say: headlights 167, lit road 130, clear 71 (mostly lit roads that evening).

## Switches and files

- `touch /data/ahb_vision_on` starts it (camerad checks every 2 s); `rm` stops it. Without the file, camerad does nothing.
- `/data/c920_horizon`: optional horizon row (default 325, from the pictures) if the camera is moved.
- `/dev/c920_lights.json`: camerad's result, 5 times a second (`n`, `lamps_above`, `band_luma`, `lights` = x, y, area, halo).
- The logger writes an `ahb` line 5 times a second and pictures named `*_ahb_dip.jpg` / `*_ahb_high.jpg` when the
  decision changes (their own 12 per chunk). Field guide: `tesla/tools/LOG_FORMAT.md`.

## Build safety

Same rule as sign detection: compiled in only if a trial compile + link succeeds in `SConscript`; every call site is
behind `#ifdef EXT_WANT_AHB_VISION`; identical in both frame callbacks (`tests/test_ahb_vision.py`,
`tests/test_camera_h264_isolation.py`). Synthetic-scene tests: `test/ext_c920/ahb_vision_test.cc`.
