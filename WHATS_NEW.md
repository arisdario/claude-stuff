# What's new vs stock openpilot 0.7.5a (BogGyver / Tinkla)

Everything in this repo compared against the stock `BogGyver/openpilot` 0.7.5a tree. Each item links to its file and
is marked:

- 🚗 **drives** — changes what the car actually does (longitudinal control, model, signals sent). Test on a known
  empty road, foot over the brake.
- 👁 **advisory / observe** — only reads, displays, warns or records. Never commands the car.

> Note: `tesla/CHANGES.md` says the first commit of `tesla/` is "the code exactly as it was on the car", so a couple
> of diffs against pristine 0.7.5a may be pre-existing car tweaks. Every new file below, and every change carrying an
> uppercase tag comment (`PIDTUNE`, `APGATE`, `LANEWIDTH`, `OBSTUNE`, `MAPCURVE`, `CURVEBRAKE`, `AHBAUTO`, …), is
> deliberate work added in this repo.

---

## 1. radarweb — a monitoring & analysis layer (👁 all observe-only)

A separate web app that reads openpilot's `can` / `carState` / `radarState` sockets read-only. None of this exists in
stock openpilot, and none of it sends anything to the car.

| File | What it is |
|---|---|
| [`radarweb.py`](radarweb.py) | The dashboard/web server: road view, `/dash`, `/near`, `/top`, `/map`, `/events`, `/trips`, `/ml`, diagnostics, and one global menu on every page |
| [`radarml.py`](radarml.py), [`radar_classify_live.py`](radar_classify_live.py), [`radarlearn_train.py`](radarlearn_train.py) | Radar object classifier and its training |
| [`radartrip.py`](radartrip.py) | Trip viewer |
| [`radarevents.py`](radarevents.py) | Intervention & close-pass recorder (what openpilot did before you took over) |
| [`radarpark.py`](radarpark.py) | Radar distance vs parking-sensor gap at stops → radar mounting offset |
| [`radarsens.py`](radarsens.py) | Wheels (slip / tyre pressure from pulse counters), sun/glare, fleet speed, map-curvature check |
| [`radaruds.py`](radaruds.py) | Read-only module diagnostics (VIN, part/software, fault codes) |

Enable the diagnostics (off by default): `touch /data/uds_read`, then only in **Park, stopped, openpilot off**.

## 2. New files on the driving side

| File | | What it is |
|---|---|---|
| [`tesla/pcc_curves.py`](tesla/pcc_curves.py) | 🚗 | Curve approach-speed math + learned pedal map; feeds `PCC_module` |
| [`tesla/model083/`](tesla/model083/) | 🚗 | The openpilot 0.8.3 driving-model drop-in (adapter + `driving.cc/.h` + tools). **Optional, separate deploy** — compile into `selfdrive/modeld/models/` with the 0.8.3 `.dlc`. See its [README](tesla/model083/README.md) |
| [`tesla/mapcurved.py`](tesla/mapcurved.py) | 🚗 | Standalone map-based curve speed limiter |
| [`tesla/obstacle_warn.py`](tesla/obstacle_warn.py) | 👁 | Advisory obstacle detector (warns, never commands) |
| [`tesla/uds_relay.py`](tesla/uds_relay.py) | 👁 | Read-only UDS relay for radarweb; Park+stopped+OP-off only, off unless `/data/uds_read` |
| [`tesla/panda_uds_read_only.patch`](tesla/panda_uds_read_only.patch) | 👁 | Panda firmware: refuses anything but reads on diagnostic IDs |
| [`tesla/bench/`](tesla/bench/) | — | Bench test scripts (never with the car connected) |

## 3. Modified stock openpilot files

### 🚗 Change driving behaviour

| File | Change |
|---|---|
| [`tesla/PCC_module.py`](tesla/PCC_module.py) | Curve slowing reworked as a braking problem, smoother acceleration, learned pedal map, and the 9 fixes in [`CHANGES.md`](tesla/CHANGES.md) (map limit was interpolated backwards; 4 Hz map data read in a 20 Hz loop; `TypeError` before the first `radarState`; speed frozen in any bend; etc.) |
| [`tesla/interface.py`](tesla/interface.py) | Longitudinal PID re-tuned + gas map changed (`PIDTUNE`, 0.6.6 SP values) |
| [`tesla/teslacan.py`](tesla/teslacan.py) | Lane-width field clipped to the valid 2.0–6.69 m range — was wrapping around (`LANEWIDTH`) |
| [`tesla/carstate.py`](tesla/carstate.py) | `apEnabled` latch disabled (`APGATE`, "as in 0.6.6") |

### 👁 Advisory / plumbing only

| File | Change |
|---|---|
| [`tesla/AHB_module.py`](tesla/AHB_module.py) | Auto high beam + code-managed high beam (enable with `touch /data/ahb_auto`) |
| [`tesla/carcontroller.py`](tesla/carcontroller.py) | Wires in AHB auto and the UDS relay (both guarded so they can't break the control loop) |
| [`tesla/radar_interface.py`](tesla/radar_interface.py) | Hooks in the advisory `ObstacleDetector` |
| [`tesla/radar_tools/calibrateRadar.py`](tesla/radar_tools/calibrateRadar.py) | Radar-offset calibration helper improvements |

## 4. Docs & tests (repo only — do not deploy to the car)

[`tesla/CHANGES.md`](tesla/CHANGES.md), [`tesla/CLUSTER.md`](tesla/CLUSTER.md), the `model083` and `radar_tools`
READMEs, and the `tests/` + `tesla/tests/` suites.

---

## Enable flags (create only what you want, then restart openpilot)

| Flag | Turns on |
|---|---|
| `touch /data/ahb_auto` | Code-managed auto high beam |
| `touch /data/uds_read` | Read-only module diagnostics |
| `touch /data/ahb_test` | Parked-only AHB beam test |
| `touch /data/mapcurve_debug` | PCC map-curve debug prints |

## The short version

Genuinely new **and touches the car**: the PCC curve-slowing rework, the longitudinal PID re-tune, the lane-width
clip, auto high beam, and (optional, deliberate) the 0.8.3 model. Everything else — the whole radarweb suite, the
diagnostics, the obstacle warnings, the recorders — is observe/advise only.
