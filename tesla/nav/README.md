# Offline navigation prototype (route planning only)

Plans and follows a road route from a local OSM extract with no internet, detects leaving the route and recalculates. **Advisory only**:
nothing here imports vehicle control (a test enforces it) and nothing commands steering or braking. `mapcurve.sqlite`, `mapcurved`, CSA
and PCC are untouched. See `SCHEMA_V2_AND_RUNTIME.md` for the mapcurve schema this is deliberately separate from.

| file | role |
|---|---|
| `osm_rules.py` | oneway / access / maxspeed rules, diacritic-insensitive names |
| `routegraph.py` | builds `routegraph.sqlite` (directed edges keyed by OSM node IDs, all drivable classes, raw tags kept) and reads it |
| `router.py` | A* over directed-edge states: shared-node connectivity only, no U-turns, turn penalties; `find_route`, `find_route_from_edge` |
| `instructions.py` | turn-by-turn steps (incl. roundabout exits) and junction summaries |
| `tracker.py` | edge tracking: connectivity + accumulated evidence, so a parallel road is never mistaken for the route |
| `matcher.py` | route matching, off-route detection with hysteresis, recalculation from the tracked edge (`Navigator`) |
| `oracle.py` | **independent** reachability oracle for validation (own SQL, own oneway/access rules, own BFS; no router/snapping) |
| `validate.py` | route validation + simulation + report (`python3 -m tesla.nav.validate`) |

## Running the two Predeal routes (needs your OSM extract; not runnable in the sandbox, which blocks all map hosts)
```
python3 -m tesla.nav.validate --pbf myarea.osm.pbf --build routegraph.sqlite        # pip install pyrosm; builds a SEPARATE graph, ~minutes
python3 -m tesla.nav.validate --graph routegraph.sqlite \
    --route "Predeal" "Timișu de Sus" --route "Predeal" "Timișu de Jos" \
    --reference reference.gpx --report predeal_report.md                              # reference: .gpx / .geojson / .csv(lat,lon) from a trusted router
```
Place names are matched against `place=*` nodes in the data (diacritics/case ignored; all candidates are listed in the report; `lat,lon` also works).
A small bbox extract around Predeal-Brasov is enough (`--bbox minlon,minlat,maxlon,maxlat` clips while building). Exit code 1 if any problem is found.
Without `--reference` the report says the comparison was **not performed**; no reference is ever invented.

## What the report checks
connectivity (shared node between every edge pair, geometry continuity, no reversals, oneway/access re-derived from raw tags) · a noisy GPS drive
(must stay on route and arrive) · a wrong turn (detected? recalculated from the wrong road? the new route legal and followed to the end?) judged by
the independent oracle, classified `DETECTION_FAILURE` / `RECALC_FAILURE` / `REROUTE_UNREACHABLE` / `SNAP_BACK` / `NO_ROUTE_CORRECT` / `REROUTED`,
every failure naming the wrong-road edge, junction node, oracle verdict and navigator status · planner-vs-oracle parity on every directed edge near
the route, from the edge and from mid-edge GPS fixes · `NO ROUTE` diagnosed as disconnected data vs a oneway/access block.

## Benchmark
`BENCHMARK.md` is the seeded benchmark (147 conditions x 30 seeds, ground truth + the independent oracle, ambiguity classes never pooled, Wilson 95% intervals); reproduce it with
`python3 -m tesla.nav.evaluate --all --seeds 30 --md BENCHMARK.md`. It lists the known defects D1-D5 (pinned as expected-failure tests in `tests/test_nav_known_defects.py`). The older figures below
predate the tracker fixes and are superseded by it.

## Measured behaviour (OLD, superseded by BENCHMARK.md - synthetic parallel road, 30 seeds per cell)
Wrong road with **no connection** (oracle: unreachable): **0 snap-backs in 360 runs**; detected 358/360 (the 2 misses: 8 m separation, GPS noise >= 5 m).
Driving the route beside a parallel road: **no false alarms at >= 12 m separation with <= 5 m GPS noise, none at >= 20 m at any tested noise**;
at 8 m separation (about one GPS error) false alarms are 3/30 (5 m noise) and 6/30 (8 m noise). With a legitimate connection the recalculation starts on
the road the car is on in 30/30 runs at 2 m noise, but only 22-24/29 at 8 m separation and 28-29/30 at 20 m when noise is 5-8 m (the track is mis-assigned).
Roads closer than about one GPS error cannot be reliably told apart by any per-fix method; this is a limit, not a tuned-away bug. Using each fix's reported
accuracy as sigma was tried and measured worse (see `tracker.py`).

## Not supported / known limits
- **Turn-restriction relations** (no_left_turn, ...), time-conditional access, vehicle limits, tolls, traffic: not in the data used. Routes may contain a forbidden turn.
- Roads within ~1 GPS error of each other (see above). Reroute starts from the tracked edge; if the track is lost (GPS gap/teleport) it falls back to snapping.
- Display in Openpilot's navigation UI and any `liveMapData`/cereal publication are **not implemented**; `Navigator.update()` returns a `NavState` dict only.
- Pedestrian/ferry/track roads are excluded; `service` roads are allowed except parking aisles/driveways.
