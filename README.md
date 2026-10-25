# radarweb + radar object classifier

Live Tesla radar view in a browser (runs on the EON next to openpilot), plus a machine-learning classifier that learns
from the objects you label and retrains itself on the EON.

| File | Runs on | What it does |
|---|---|---|
| `radarweb.py` | EON | Web server: road/top/learn pages. Reads CAN from openpilot's `can` socket (nothing is sent to the car). Saves `/learn` labels with 2 s of track history, runs the model, retrains automatically when parked. |
| `radarml.py` | EON (+ laptop) | Pure python (no numpy/sklearn): full radar object-frame decoder (bit layout from tesla-radar-driver's DBC), INFRA3DRC-style features (mean/std/min/max/median/span of each signal over the track history), model inference, and the on-car trainer. |
| `radarlearn_train.py` | laptop | Optional, more thorough trainer (scikit-learn, nested grid search, group cross-validation). |
| `radar_classify_live.py` | Pi/laptop + CAN adapter | Optional: same classifier on a radar on a **bench**, via [tesla-radar-driver](https://github.com/SamSkjord/tesla-radar-driver). Never on a car. |

`radarweb.py` also needs your `radarlog.py` (decoders) next to it - it is not in this repo.

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
