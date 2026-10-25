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

## Automatic training

- Labels go to `/data/canraw/labels.jsonl`.
- With 60+ labels and 30+ new since the last training, radarweb retrains when the car has been parked for a minute
  (under 1 km/h, openpilot not engaged), as a separate `nice -n 19` process. Driving off stops it; it retries at the next stop.
- The new model is installed (`/data/radar_model.json`, old one kept as `.prev`) only if its cross-validated macro-F1
  beats both "no model" and the installed model. radarweb reloads it without a restart.
- Status is shown on `/learn` next to the object count. Errors: `/tmp/radar_autotrain.log`.
- Without a model file radarweb behaves exactly as before.

| Environment variable | Default | |
|---|---|---|
| `AUTOTRAIN_NEW` | `30` | new labels needed before retraining; `0` = off |
| `RADAR_MODEL` | `/data/radar_model.json` | model file |
| `LEARN_LABELS` | `/data/canraw/labels.jsonl` | labels file |

## Optional: train on a laptop

```
pip install numpy pandas scikit-learn matplotlib
scp root@<EON-IP>:/data/canraw/labels.jsonl .
python3 radarlearn_train.py labels.jsonl --plots
scp radar_model.json root@<EON-IP>:/data/radar_model.json       # picked up without a restart
```
