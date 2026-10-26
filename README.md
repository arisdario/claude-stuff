# radarweb + radar object classifier

Live Tesla radar view in a browser (runs on the EON next to openpilot), plus a machine-learning classifier that learns
from the objects you label and retrains itself on the EON.

**Intended for:** Tesla Model S **pre-AP** (no factory Autopilot) running openpilot on an EON, with a retrofitted
**Bosch MRRevo14F** radar (the Tesla AP1 radar). The radar reports up to **32 tracked objects over CAN** (frame pairs
0x310-0x36E), each with distance, lateral offset, relative speed, class, length and more - that is what radarweb
shows and what the classifier learns from.

| File | Runs on | What it does |
|---|---|---|
| `radarweb.py` | EON | Web server: road/top/learn pages. Reads CAN from openpilot's `can` socket (nothing is sent to the car). Saves `/learn` labels with 2 s of track history, runs the model, retrains automatically when parked. |
| `radarml.py` | EON (+ laptop) | Pure python (no numpy/sklearn): full radar object-frame decoder (bit layout from tesla-radar-driver's DBC), INFRA3DRC-style features (mean/std/min/max/median/span of each signal over the track history), model inference, and the on-car trainer. |
| `radarlearn_train.py` | laptop | Optional, more thorough trainer (scikit-learn, nested grid search, group cross-validation). |
| `radartrip.py` | EON (+ laptop) | Drive logs on a map. Used by `/trips`; on a laptop `python3 radartrip.py map_*.jsonl` writes a self-contained `.html` per log. The output contains your GPS track: keep it private. |
| `radar_classify_live.py` | Pi/laptop + CAN adapter | Optional: same classifier on a radar on a **bench**, via [tesla-radar-driver](https://github.com/SamSkjord/tesla-radar-driver). Never on a car. |

`radarweb.py` also needs your `radarlog.py` (decoders) next to it - it is not in this repo.

## Dependencies

**EON (`radarweb.py`, `radarml.py`)** - nothing to install, openpilot already has it all:

| Needed | For | Where it comes from |
|---|---|---|
| Python 3 | everything | openpilot |
| `radarlog.py` | radar frame decoders | your file, in `/data/` next to radarweb.py |
| `cereal` (openpilot) | CAN + carState/controlsState/model data | `PYTHONPATH=/data/openpilot` in the start command |
| `numpy`, `cv2` | lane/path drawing, camera snapshots | openpilot (only those features need them) |
| `common.params` (openpilot) | camera recalibration page | openpilot |
| `nice` | low-priority automatic training | system command, already on the EON |

The ML part (`radarml.py`) is pure Python standard library on purpose: no numpy, scikit-learn or cantools on the car.

**Laptop (`radarlearn_train.py`, optional):** Python 3.9+, `pip install numpy pandas "scikit-learn>=1.4" matplotlib`
(matplotlib only for `--plots`). Keep `radarml.py` in the same folder.

**Bench (`radar_classify_live.py`, optional):** Python 3.8+, `pip install python-can cantools`, a clone of
[tesla-radar-driver](https://github.com/SamSkjord/tesla-radar-driver), a CAN adapter (socketcan, e.g. a Waveshare
CAN hat on a Raspberry Pi; CANalyst-II on macOS also needs `pip install canalystii pyusb`), and `radarml.py` in the same folder.

## Run on the EON

```
scp radarweb.py radarml.py root@<EON-IP>:/data/          # next to /data/radarlog.py
ssh root@<EON-IP>
cd /data/openpilot && PYTHONPATH=/data/openpilot:/data nohup python3 /data/radarweb.py 8088 >/dev/null 2>&1 &
```

Open `http://<EON-IP>:8088/` on a phone on the same network. `/learn` = tap an object, then tap what it is.
`/trips` = your drives on a map (OpenStreetMap, free): route coloured by speed / vs the speed limit / openpilot,
stops, and what the radar saw at its real position - vehicles, oncoming traffic, people and bikes crossing in front,
static obstacles - plus a speed chart (click it to jump the map). Each drive in `/data/maplog` is processed once in the
background (`nice`), cached as `map_*.trip.json`.
`/ml` = what it learned: model score vs no model, label counts per class (yours / automatic), the latest labels,
and every live object with what the radar says, what the model says (with confidence) and what is shown.

## Automatic labelling (objects label themselves)

radarweb follows every radar track over its whole life. When a track ends it gets a label only if its behaviour
clearly fits one, and snapshots from across its life (as the object looked at that moment) are saved with that label
to `/data/canraw/autolabels.jsonl`:

| Track behaviour | Label |
|---|---|
| moved over the ground at 6+ m/s (22 km/h) for 1 s | `four_wheel` (`two_wheel` if the radar was sure and it stayed under 43 km/h) |
| radar held one class at 80%+ confidence for most of the track | that class |
| never moved while you drove 20 m+ past it, seen 3 s+ | `static` (poles, signs, walls, parked objects) |
| lived under 0.8 s with low existence probability, or range rate and distance change disagree | `not_real` (ghost) |
| anything else | not used |

At most one track per class every 15 s is kept (balanced classes, small file; it stops at 50 MB). The labels are only
as good as these rules: the model learns to recognise those classes earlier and in the hard cases (a parked car looks
like the cars it saw move). Labels you tap on `/learn` are still used and count 3x. `AUTOLABEL=0` turns it off.

## Crossing pedestrians and cyclists

The radar measures speed along its line of sight, so someone crossing in front of you looks stationary to it and stays
class "unknown". radarweb detects crossing from the track's sideways motion (last 2 s of positions, checked against the
radar's lateral speed): 2-5 km/h = pedestrian, 5-25 km/h = cyclist, faster = vehicle. Your own rotation in turns makes
everything ahead sweep sideways; that sweep is measured from the stationary objects and subtracted, and detection pauses
in sharp turns and while manoeuvring (steering past 90 deg). Shown with `~` like other inferred classes.

## Logs

`/data/maplog/map_YYYYMMDD_HHMMSS.jsonl` is written by radarweb itself, one line per second (speed, openpilot/cruise
state, decoded CAN incl. GPS and speed limit, raw CAN bytes, carState, and radarweb's own objects/crossings). A new file
starts each time radarweb starts, named after the EON clock. About 55 MB per hour and nothing deletes them: check
`du -sh /data/maplog` now and then.

## Model v2: how it all fits together

The radar does its own detection, tracking and classification inside (raw data -> FFTs -> angles -> detections ->
tracking/classification) and only sends the result over CAN: up to 32 objects with distance, offsets, speeds,
acceleration, length, height, probabilities, tracker uncertainties and its class. radarweb works after that:

1. **Track history** (2 s per object) and derived signals: sideways ground speed (with your own rotation removed),
   ground acceleration (radar's relative + yours), 2-D ground speed, correlations between position and speed.
2. **Features** (148): mean/std/min/max/median/span of every signal over the history, as in INFRA3DRC (there over the
   points of an object, here over time), plus the object-level kinematic features of Ham et al., Sensors 2025.
3. **Labels**: automatic ones from track behaviour + yours from `/learn` (3x weight).
4. **Training** (parked: gear in P for 30 s or stopped 2 min; or **Train now** on `/ml`; or
   `python3 radarml.py --autotrain labels.jsonl,autolabels.jsonl radar_model.json`), at the lowest CPU priority.
   Two candidates, both on the EON (numpy, which openpilot ships):
   - Random Forest, 100 trees, as in INFRA3DRC-Radar-Classification (bootstrap, best split, balanced classes)
   - neural network 2 hidden layers x 10, as in Ham et al. (ReLU, RMSprop lr 0.01, batch 64, early stopping)
   Each is scored with cross-validation that keeps whole drives out of training; the better one is installed if it
   beats "no model" and the installed model. `/ml` shows both with INFRA3DRC's metrics (accuracy, balanced accuracy,
   macro-F1, weighted-F1), per-class F1 and the confusion matrix. Without numpy a plain-python trainer is used.
5. **Live**: each object is classified once per second (plain python, smoothed per track); the model replaces the
   radar's class when it is at least 60% sure, then physical limits and crossing detection apply.

A model trained on older features is replaced automatically at the next stop.

## Automatic training

- Labels: `/data/canraw/labels.jsonl` (yours) + `/data/canraw/autolabels.jsonl` (automatic).
- With 60+ labels in total and 30+ new since the last training, radarweb retrains when the car has been parked for a minute
  (under 1 km/h, openpilot not engaged), as a separate `nice -n 19` process. Driving off stops it; it retries at the next stop.
- The new model is installed (`/data/radar_model.json`, old one kept as `.prev`) only if its cross-validated macro-F1
  beats both "no model" and the installed model. radarweb reloads it without a restart.
- Status is shown on `/learn` next to the object count. Errors: `/tmp/radar_autotrain.log`.
- Without a model file radarweb behaves exactly as before.

| Environment variable | Default | |
|---|---|---|
| `AUTOTRAIN_NEW` | `30` | new labels needed before retraining; `0` = off |
| `RADAR_MODEL` | `/data/radar_model.json` | model file |
| `LEARN_LABELS` | `/data/canraw/labels.jsonl` | your labels file |
| `AUTOLABEL` | `1` | `0` = no automatic labels |
| `AUTOLABELS` | `/data/canraw/autolabels.jsonl` | automatic labels file |

## Optional: train on a laptop

```
pip install numpy pandas scikit-learn matplotlib
scp root@<EON-IP>:/data/canraw/labels.jsonl .
python3 radarlearn_train.py labels.jsonl --plots
scp radar_model.json root@<EON-IP>:/data/radar_model.json       # picked up without a restart
```
