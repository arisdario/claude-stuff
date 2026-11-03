# Offline navigation prototype - technical handover

Branch `offline-nav`. Planning/advisory only: nothing in `tesla/nav/` imports or talks to steering, braking, acceleration, CAN, UDS, CSA, mapcurve or PCC.
No network at runtime (OSM PBF processed locally; Geofabrik extract supplied by the owner).

## Architecture (data flow)
PBF --`routegraph.build` (pyrosm `driving+service` + pyosmium raw tags)--> `routegraph.sqlite` (schema v2: directed edges keyed by OSM node ids, bridge/tunnel/layer/
maxspeed, raw oneway/access tags, R-tree `edge_rtree` with grid fallback, places, provenance meta)
--> `router.find_route` (A* over directed-edge states, no U-turn, turn penalties) --> `instructions` (steps/junctions)
--> GPS: `tracker.EdgeTracker` (connectivity + CUSUM evidence) + `matcher.RouteMatcher/Navigator` (on-route, off-route hysteresis, recalculation)
--> validation: `oracle.Oracle` (independent BFS, shares no routing code), `validate.py`, `evaluate.py`+`scenarios.py` (seeded benchmark), `speedplot.py` (advisory speed profile).

## Files changed this phase (commits 9679ccd, 5e00033 and the commit that adds this file)
- `routegraph.py` R-tree + deterministic snap; `matcher.py` two GPS false-reroute fixes (below); `speedplot.py` (new, unchanged logic + `suspicious()`);
- `run_logged.sh` (nohup/setsid wrapper: records .pid .log .exit .start .end .seconds);
- `validation/`: both route reports, speed graphs, `SUSPICIOUS_GEOMETRY.md`;
- tests: `test_nav_data.py` (schema v2, tag preservation, R-tree), `test_nav_deviation.py` (+ route-doubles-back test), `test_nav_known_defects.py` (D4 now passes).

## GPS false-reroute fixes (matcher.py; no threshold of the existing logic changed)
1. **Ambiguous-parallel evidence rule.** When the tracker says the car is on another road within AMBIG_M of the route, the mean separation over the bad run is
   compared with 2 x the fix accuracy (the benchmark's own "ambiguous" definition). If below, `OFF_FIXES_PARALLEL` (30) bad fixes are required instead of 8. Clear separations
   (e.g. 8 m with 1.5 m noise) keep the old speed (regression test passes). Fixes ramp/merge false reroutes (km 14.7, 18.9 etc.).
2. **Continuity-first route matching.** `RouteMatcher.match` searches `NEAR_AHEAD_M` (80 m + speed term) first; only widens to 300 m if nothing is within ON_ROUTE_M.
   A loop ramp (km 3.2 of Fundulea->Bucuresti) made progress jump +304 m to a later pass of the same route and caused a reroute at "cross-track 367 m".
Result on the real graph (single seed, 5 m noise): on-route simulation 0 reroutes in both directions (was 1 and 3).

## Tests
`python3 -m unittest discover -s tesla/tests -p "test_nav*.py"` -> 132 tests OK, 4 expected failures (known defects D1, D2, D3, D5 documented as tests).
Whole `tesla/tests`: 341 run, 2 import errors (`test_UIBT_module`, `test_readconfig`: need openpilot `selfdrive`; present before my changes).
NOT re-run: the benchmark (`BENCHMARK.md` is from commit dd77b97; the matcher/snap changes may shift its numbers), parity check after the matcher fix, full `validate` runs after the fix.

## Known bugs / open items
- Fundulea->Bucuresti deviation test: after the correct reroute a SECOND reroute occurs (~11.8 km into the new route, "on another road (secondary)") -> validator still reports
  "expected exactly 1 recalculation". Not diagnosed. Re-run validators to confirm final report state.
- Report label bug: "connectivity FAIL" is triggered by the word "edge" in unrelated problem text (validate.py ~line 516). Cosmetic; connectivity itself passes.
- D1 (>=3-fix GPS jump false recalc), D2 (lost-track snap-back), D3 (second-deviation latency), D5 (missed deviation at 8 m / 2 m noise) unfixed (see test_nav_known_defects.py).
- Turn restrictions, conditional access, tolls not modelled. No reference route comparison done.
- Predeal-Brasov graph must be rebuilt with the service-road fix and re-audited; the Timisu routes were never validated.
- Speed recommendation: curvature is estimated from route geometry (10 m resample, +-30 m Menger) - rougher than the mapcurve builder. Suspicious points are listed in
  `validation/SUSPICIOUS_GEOMETRY.md` (spikes, and fast-road reductions that may be braking lookahead rather than geometry). Real-drive speed overlay not built.

## Next steps (in order)
1. Re-run both validations detached: `tesla/nav/run_logged.sh OUT NAME -- python3 -W ignore -m tesla.nav.validate --graph bf.sqlite --route "Fundulea" "București" --report OUT/x.md` (~8-14 min each; do not run two at once).
2. Diagnose the second reroute above; add a regression test.
3. Re-run the seeded benchmark once (`python3 -m tesla.nav.evaluate ...`, ask first) and update BENCHMARK.md.
4. Fix the report label bug; rebuild/audit Predeal graph.
5. Openpilot integration, incremental: (a) read-only process publishing a `navState` message (planning result only) fed by existing GPS; (b) on-EON replay of logged drives;
   (c) display/voice only. No path from nav to car control; keep `/data/tesla_csa_off`, CSA shadow-only and mapcurve/PCC untouched. Large DBs/PBFs/logs stay out of git.


## Update (supersedes the GPS-reroute section above where they differ)
- **Matcher reference = `745d287`.** I tried a further matcher change (tracker re-anchoring + outlier-proof ambiguity evidence, commit `d30f32f`, kept ONLY on branch
  `matcher-resync-experiment`) and ran the full benchmark against `745d287` (4410 runs each, same seeds). Clear class: identical. Ambiguous class: false recalc 2% -> 0% but
  detection 64% -> 49% and confident wrong-road recalc 2% -> 3%: no overall improvement, so it was reverted on `offline-nav` (matcher.py and test_nav_deviation.py are exactly `745d287`).
  The "second reroute ~11.8 km" is therefore NOT fixed on this branch; cause (documented): the connectivity-based tracker locks onto a road 1-10 m beside the route, then two outlier
  fixes (16.9 m, 24.5 m) trigger a recalculation from the parallel road. The two real-route validations were stopped (not completed) when the matcher was reverted; re-run with
  `run_logged.sh` if needed. No further matcher work until asked.
- **Focus is now curve-ahead announcement** (advisory only, no control path):
  `tesla/nav/curveahead.py` (find_curves = curvature along the CONFIRMED route's geometry, 800 m horizon, v = sqrt(1.85/kappa), curve if v < 108 km/h and >= 20 m long;
  `CurveAnnouncer` = each curve announced once with a stable id, distance-to-start updated every fix, dropped when passed, never announced while road confidence is not 'high';
  `CurveAhead` wraps the existing Navigator for position/confidence), `tesla/nav/curve_replay.py` (replay + report), `tesla/tests/test_nav_curveahead.py`.
- Replay results (Bucharest-Fundulea graph, 5 m GPS noise, 25 m/s): both directions every real curve announced exactly once (32/32 and 18/18), 0 duplicates, 0 spurious, 0 false slowdowns,
  distance never grows while approaching. Road match with the reference matcher: median progress error 3.3 m, p95 ~10 m, 0 high-confidence fixes with progress wrong by > 40 m
  (6 runs: 2 directions x 15/25/33 m/s); ~10% of fixes are low-confidence (no speed advised). Reports: `tesla/nav/validation/curve/`.
- Not done / next: replay against a REAL logged drive (user's map_*.jsonl) to compare announcements with what the car actually did; decide how the message is transported
  (cereal) - separate step, still advisory only; approach-speed/braking fields exist in `advise()` but are not part of the announcement by request.

## Update 2 - CurveCue comparison (github.com/mhathiyari/curve-call, MIT; web demo https://mhathiyari.github.io/curve-call/)
The user tested that demo on Bucharest-Fundulea (15 curves: 2 gentle, 5 moderate, 4 firm, 4 sharp) and said it is what they want. Its engine is the same pipeline as ours
(10 m interpolation, Menger curvature, segmentation, lateral-g speed). Adopted from it (not copied code): severity bands by radius (gentle > 200 m, moderate 100-200, firm 50-100,
sharp 25-50, hairpin < 25), direction, spoken text ("Sharp left ahead, slow to 35"), and 'standard' verbosity (announce moderate and tighter; gentle only on request).
Result on our graph (offline, existing matcher, 25 m/s, 5 m noise): Bucuresti->Fundulea **15 curves announced (6 moderate, 4 firm, 4 sharp, 1 hairpin)**, Fundulea->Bucuresti 12; each once,
0 duplicates, distances only shrink. Routes differ from the demo's (theirs 39.6 km, ours 41.2 km via the A2), so the counts agree but are not the same curves.
Not adopted yet (CurveCue has them): compound/S-bend merging, 'tightening' modifier, companion mode without a destination (walk forward along the road at forks), trigger-distance timing.

## Update 3 - advisory target speed (still advisory only)
`CurveAhead.update()` now returns `msg['advisory']` = {confidence, target_speed_ms, slow_down, curve_distance_m, curve_speed_ms, curve_radius_m, curve_severity}.
`target_speed()` = min over announced curves of sqrt(v_curve^2 + 2*DECEL*distance_to_bend_start): the highest speed the car may have now and still reach each bend at its curve speed using only
DECEL (1.0 m/s^2, configurable). It falls smoothly toward the curve speed, is None when no curve is ahead or road confidence is not 'high', and `slow_down` = current speed > target.
`simulate_follow()` checks it on clean geometry: a car cruising at 25 or 33 m/s and obeying the target never brakes harder than DECEL and reaches every bend at <= curve speed + 0.75 m/s,
except a hairpin 218 m (Bucuresti->Fundulea) / 619 m (Fundulea->Bucuresti at 33 m/s) after the route START, where the replay starts at cruise speed with too little distance (start-up artefact).
Per-fix output: `python3 -m tesla.nav.curve_replay ... --csv advisory.csv` (sample in tesla/nav/validation/curve/). Nothing consumes it yet: no cereal message, no controller.
Next (separate, needs a decision): publish this dict as a message; any consumer that turns target_speed into acceleration limits is a vehicle-control change and must go through the PCC/CSA safety review.

## Update 4 - ONE curve-detection engine: CurveCue's (supersedes our own detector described in Updates 1-3)
Per the user's instruction our own curvature/curve detector was REMOVED from `curveahead.py`. Detection is now `tesla/nav/curvecall_engine.py`: a statement-by-statement Python port of CurveCue's
`web-demo/engine.js` (MIT, attribution in the file header; Python because the EON has no JVM/node). Ported: 5 m interpolation, Menger radius + direction, spike rejection, 5-point rolling average,
500 m threshold segmentation with 50 m gap merge, severity/direction/modifiers, advisory speed. NOT ported: compound/S-bend detection, lean angle, narration.
Proof of equivalence: `tesla/tests/test_curvecall_engine.py` compares the port with the ORIGINAL engine.js run under node (`tesla/tests/data/`: engine.js copy, run_golden.mjs, routes, golden outputs)
on Bucharest<->Fundulea: identical curves (indices, severity, direction, modifiers, advisory speed; radius/distance to 1e-5).
`curveahead.py` is now only glue: run the engine once per route (cached), read the curves ahead of the matched progress (800 m horizon), announce each once (stable id, distance updates), and
turn them into `target_speed()` (sqrt(v_curve^2 + 2*DECEL*d), CurveCue's braking relation). Speed uses a_lat = 1.85 m/s^2 (the repo's mapcurve value, = 0.19 g); CurveCue's own default is 0.35 g
(1.9x higher a_lat, ~1.4x higher speeds) - OPEN DECISION: pass lateral_g to match CurveCue or keep mapcurve's comfort value.
Comparison on Bucharest<->Fundulea (>= moderate): CurveCue engine finds 9 / 8 curves, our old detector found 15 / 12 - every CurveCue curve was also in ours; ours added 5 / 4 more (tight bends, e.g. an
18 m-radius turn 150 m from the start) because CurveCue averages over ~25 m and merges nearby curves. Neither is verified against ground truth; that needs a real drive log.
Replay with the CurveCue engine (25 and 33 m/s, 5 m GPS noise, existing matcher): each curve announced once, 0 false, 0 curves entered above curve speed at 25 m/s (1 at 33 m/s: a hairpin shortly after the route start), max braking 1.00 m/s^2.

## Update 5 - the "hairpin entered too fast at 33 m/s" (investigated, no behaviour change)
Cause: replay start-up, NOT announcement timing. At 33 m/s the first bend on Bucharest->Fundulea (CurveCue: sharp, R = 29 m, curve speed 8.9 m/s) starts 413 m after the first fix; slowing 33 -> 8.9 m/s
at DECEL 1.0 needs 505 m. It is announced at the very first fix (inside the 800 m horizon), the target speed brakes at the full 1.0 m/s^2 from then on, and the car reaches the bend at 15.5 m/s - the
best physically possible (sqrt(33^2 - 2*413) = 16.2 m/s). Every other bend is entered at or below curve speed at 15, 25 and 33 m/s on both routes. Only `simulate_follow`'s reporting was changed (it
printed a bend's END as its start); tests added: `TestLateEntryIsOnlyPhysicallyUnavoidable` (any late entry must be a bend closer than its braking distance, and braking must have been the physical maximum).
Limit worth knowing: with an 800 m horizon and 1.0 m/s^2 the advisory can only be early enough for entry speeds up to ~40 m/s (146 km/h) at the slowest curve speed (8.9 m/s).
Settings unchanged: CurveCue engine, a_lat 1.85 m/s^2 (not 0.35 g), advisory only, no new detector.

## Update 6 - validation on a REAL drive (map_20261101_175314, Bucharest->Fundulea, 32.7 km, 30 min)  [`python3 -m tesla.nav.real_drive_validate`]
- Driven route reconstructed from the GPS trace (waypoint legs every 300 m): 33.1 km, every fix within 8 m of it. The CurveCue engine finds only 4 curves on it (A2 is nearly straight): a 22 m hairpin at km 2.54-2.77,
  a firm right (R 72) at km 27.97, and two gentle bends (not announced at standard verbosity).
- Announced: 2 curves, no duplicates (a duplicate in an earlier replay was the replay's own dead-reckoning overshoot, fixed by keeping progress monotone). Hairpin: announced 791 m ahead, the driver went 67 -> 45 (100 m before)
  -> 28 km/h in the bend (curve speed 32): agrees, and was already at/below the advisory envelope. Firm right: announced 795 m ahead; no usable speed samples near it (GPS too sparse) -> inconclusive.
- Driver slow-downs with NO curve in the geometry (km 0.4 and 1.3 urban, 7.5 and 26.9 on the A2: 103->76, 112->85 km/h): traffic/ramps; nothing was announced there (no false slowdowns).
- **Real-data limits found:** (1) the logged GPS position refreshes only every ~14 s (134 distinct positions in 1812 s; 92% duplicates) - progress must be dead-reckoned from speed between updates (the validator does; the production
  CurveAhead does NOT yet); (2) the reference Navigator recalculated 11-22 times on this drive (the driver did not follow the planner's route, and the sparse GPS produces jumps) - rerouting was therefore excluded from this validation;
  (3) only 2 announceable curves: this drive cannot validate curve timing statistically. A winding-road drive is needed for that.
- MCU2 navigation: the DBC in the repo defines passive CAN signals from the MCU (UI_driverAssistMapData 0x3C8: UI_navRouteActive, UI_nextBranchDist, UI_roadClass, UI_mapSpeedLimit, UI_gpsRoadMatch;
  MCU_locationStatus 0x3D8 lat/lon; UI_gpsVehicleSpeed 0x2F8 heading; MCU_locationStatus2 0x68 navigonExpectedSpeed). No next-turn direction, route geometry or destination signal exists. Our drive logs recorded only 0x3D8 and 0x2F8,
  (CORRECTED in Update 12: the logger stores the raw bytes of EVERY CAN id it sees, 253 ids, and 0x3C8 / 0x68 are simply absent from those drives.)

## Update 7 - speed-based progress estimate (dead reckoning) in `CurveAhead`
When the GPS position has NOT changed since the last update (the MCU log refreshes it only every ~14 s), the position is no longer fed to the Navigator as a new fix (that froze the matcher, which saw
"the car moved 25 m/s but stayed put"); instead progress advances by the car's speed (trapezoid of the last two speeds) and confidence is whatever the last REAL position update gave, dropping to 'low' after
DR_MAX_AGE_S = 60 s without a new position. A new position slightly behind the dead-reckoned progress (<= 50 m) does not move the car backwards (no re-announcement). `dead_reckon=False` restores the old behaviour.
Message: `advisory.dead_reckoned`, `advisory.position_age_s`. Tests: `TestDeadReckoning` (14 s updates at 8 m/s: max progress error < 30 m vs > 60 m without; the bend announced once; confidence stays high).
Also changed: the tracker's `contested` flag fires when a long position jump leaves many candidate edges ahead ON THE SAME road; `CurveAhead._contested_off_route` now only counts evidence for edges that are NOT on the route
(a parallel road still lowers confidence). Replay on Bucharest<->Fundulea at 25 m/s: unchanged results (9 / 8 curves, once each, 0 false), high-confidence fixes 1469->1559 and 1413->1494, still 0 with progress wrong > 40 m.
FINDING (CurveCue engine, not modified): the engine resamples the route at 5 m and smooths the RADIUS with a plain average (radius capped 10,000 m). On a polyline whose vertices are >= 10 m apart, every other resampled point lies
on a straight chord (infinite radius) and swamps the average: a real R = 99 m arc built from 10 m chords came out as R = 8010 m and no curve at all; with vertices <= 5 m apart it is detected correctly. Real OSM geometry has irregular
vertex spacing, so some bends may be under-reported (this is the likely reason the engine finds fewer curves than our old 30 m-baseline detector on the same routes). Mitigation available WITHOUT changing the engine: pass a larger
`spacing` (the engine option exists) or densify route geometry; not done - needs a decision and a comparison against real bends.

## Update 8 - how warnings reach the driver today (traced, read-only; no code changed)
- Lane departure warning -> the car's INSTRUMENT CLUSTER, not the EON screen: `carcontroller.py:838` `_should_ldw` (enable_ldw config, v >= 16 m/s, no blinker, numb period after blinker) -> `:754-766` ldwStatus 1-4 from
  lane probability + lane offset (LDW_WARNING_1 0.9 m, LDW_WARNING_2 0.5 m) -> `:632-638` `teslacan.create_fake_DAS_warning` (`teslacan.py:210-232`, CAN 0x554, byte 3 = ldwStatus) -> panda `do_fake_DAS` (safety_tesla.h,
  NOT in this repo; per tesla/CLUSTER.md) -> fake DAS messages the cluster draws. 0x554 has no spare field for a curve advisory: using the cluster means a new/extended CAN message + panda firmware change.
- EON-screen alerts use openpilot events: `interface.py` `create_event('steerUnavailable', [ET.NO_ENTRY, ET.WARNING])` etc. -> controlsd -> alert definitions in openpilot's `alerts.py` (NOT in this repo, so the exact
  yellow/white styling could not be verified here).
- `obstacle_warn.py` / `radar_interface.py:132` computes `obstacle_level` but NOTHING in this repo reads it: that warning is not displayed anywhere yet.

## Update 9 - what openpilotkr/openpilot (github.com/openpilotkr/openpilot, cloned read-only at /home/user/openpilotkr/openpilot, commit 3ff6dc6) already does
- OSM curve slowdown there = `selfdrive/mapd` (the move-fast "mapd"): every 1 Hz it queries OSM around the car (Overpass online, or a local osm3s server at /data/osm if installed), `lib/Route.py` follows the road AHEAD
  with no destination (picks the continuation by bearing at forks), `lib/NodesData.py` fits a spline through the OSM nodes (data "enhanced" with a node every 15 m where nodes are >= 50 m apart), evaluates curvature every 5 m,
  makes a turn-speed section where curvature >= 0.002 1/m (R < 500 m), splits sections by sign and by curvature peak, limit = sqrt(2.3 m/s^2 / curvature). Published as `liveMapData.turnSpeedLimit` (+ ...Ahead lists).
- The consumer is HYUNDAI-only: `selfdrive/car/hyundai/navicontrol.py` interpolates the limit through user tables (OCurvSpeedC/T, VCurvSpeedC/T), picks Vision/OSM/both via CurvDecelOption (OSM only when the camera model
  does not already see the curve: osm_wait_timer) and then spams the cruise buttons to change the set speed. Not applicable to Tesla: PCC sets speed directly.
- I ran mapd's NodesData curvature code unchanged (scipy) on our two Bucharest<->Fundulea routes: 37 and 30 turn-speed sections vs CurveCue engine 10 / 11 and our old detector 15 / 12: it fragments real bends into many
  sections (e.g. the A2 on-ramp loop at km 14.2-14.6 -> 9 sections) and reports junction corners with R of 2-10 m. Noisier than CurveCue's engine for announcing; its value is the road-following (Route.py) and the data pipeline.
- Needs internet (Overpass) or a local OSM server; cereal there has liveMapData.turnSpeedLimit* fields that the 0.7.5 Tesla cereal does not.

## Update 10 - the three detectors combined and scored on the REAL drive; our own detector restored as PRIMARY
Detectors: OWN (tesla/nav/own_detector.py, restored from c98e2db), CC (CurveCue engine port), MAPD (openpilotkr mapd spline curvature, unmodified, scipy). Script: `tesla/tools/detector_compare.py` (outside tesla/nav on purpose: it imports openpilotkr selfdrive code, which the nav-isolation test forbids inside the package), output
`tesla/nav/validation/curve/three_detectors_real_drive.md`. Driven route (33.1 km, rebuilt from the GPS trace), curves with R < 200 m, scored against what the driver did (rule fixed in advance: speed drop >= 12 km/h
between 400..100 m before and the slowest point -30..+120 m, or slowest point within 10 km/h of the curve speed). 6 clusters, 5 with speed data, all 5 slowed:
OWN 5/5, MAPD 5/5, CurveCue 1/5 (it misses the short sharp bends: km 0.4 R32, 27.5, 27.65, 30.3 - the driver went 54->20, 91->76, 91->67, 56->30 km/h). No detector announced a curve the driver ignored.
Null baseline: the same rule fires at 12% of random places (14 of 113) -> 5 of 5 by luck ~ 0.00003. Combinations ('OWN or CC', 'any two agree', union) add nothing here; 'all three' is as weak as CurveCue.
CAVEATS: ONE drive, FIVE scored curves; 'driver slowed' is a proxy (traffic, junctions also slow people); the rule also fired at km 1.0-1.2, 4.8-9.5, 13.5, 22.0, 26.8-27.0 where NO detector saw a curve (possible misses by all three, or traffic).
Decision implemented: `curveahead.DEFAULT_DETECTOR = 'own'`; each curve also carries `confirmed` (True when the CurveCue engine finds a curve at the same place) and the announcer passes it through, so downstream can
treat 'confirmed' differently from 'own only'. CurveCue stays available (`detector='curvecall'`), still verified against the original engine.js. mapd is NOT used (needs Overpass/OSM server + scipy, and is noisy on sparse geometry).
Replay on Bucharest<->Fundulea at 25 m/s with OWN: 15 / 12 curves announced once each, 0 false, max braking 1.0 m/s^2; the one curve entered above curve speed is a hairpin 150 m after the replay's first fix (start-up).

## Update 11 - simulated drive Pitesti -> Ramnicu Valcea (`python3 -m tesla.nav.advisory_drive_test`, route from the national mapcurve v1 DB via `tesla/nav/national_route.py`)
No routing graph / PBF exists for that area (and none can be downloaded here), so the route (64.2 km, trunk+primary, 11,913 points) was built by joining the national DB's roads where they share a vertex (one-way codes honoured).
This is a TEST helper, not the routing engine; it was not checked against a map of the real DN7/DN65 line.
Result with the advisory active (own detector, a_lat 1.85, decel 1.0), constant cruise: 94 curves of moderate or tighter (92 also found by CurveCue; 2 hairpin-class are sharp/hairpin: 24 sharp, 22 firm, 140 moderate counted over the two speed runs),
the car slows into EVERY one (0 entered above curve speed, max braking 1.00 m/s^2, lowest 32 km/h) and runs straight stretches at full speed, but it is below cruise for 51% of the drive at 79 km/h and 68% at 101 km/h:
the road is winding (km 30-52 nearly continuous bends). OPEN TUNING QUESTION (not changed): at a_lat 1.85 m/s^2 a moderate bend of R 150-190 m caps the speed at 60-69 km/h - may be too conservative for a driver's feel.
Simulation only: a constant-speed car obeying target_speed(); no real car / openpilot / PCC was involved.


## Update 12 - the enable option (advisory_runtime.py) and the drive plan for Pitesti -> Ramnicu Valcea
- CORRECTION: radarweb's /data/maplog/map_*.jsonl already stores, once a second, the raw bytes of every CAN id seen (253 distinct ids in the Nov 1 drives) plus the decoded messages. MCU navigation messages 0x3C8
  (UI_driverAssistMapData: navRouteActive, nextBranchDist, roadClass...) and 0x68 (navigonExpectedSpeed) are NOT in those drives: either the MCU does not broadcast them on this car/bus or MCU navigation was not active.
  No logger change is needed. On the next drive, use the MCU navigation for part of the trip and look for '0x3c8' / '0x68' in the log's `raw`.
- `tesla/nav/advisory_runtime.py` (+ `tesla/tests/test_nav_advisory_runtime.py`, 8 tests): switches on the EON, same convention as tesla_csa. Default = SHADOW (computes and logs to /data/curveadvice_shadow.log, never notifies);
  `touch /data/tesla_curveadvice_enable` = text notifications ON, only while road confidence is high, speed >= 8 m/s, GPS position <= 30 s old, curve moderate or tighter, within braking + 3 s reaction distance (>= 200 m), once per curve;
  `touch /data/tesla_curveadvice_off` overrides everything. No control output, no vehicle-control imports (the nav isolation test passes).
- NOT WIRED: nothing on the EON calls this module yet and no screen shows the text. Wiring (a process feeding GPS/speed into CurveAhead + the display path) is the next step and needs explicit approval because the EON-screen alert path
  lives in the car interface / controls events (see Update 8).
- Gate run over the simulated Pitesti drive (enabled): 94 notifications, each curve once, at both 79 and 101 km/h cruise; 8 per run came with less distance than the braking distance - an artifact of that demo, which passes only the nearest
  curve to the gate; the real integration passes all active curves.
- DRIVE PLAN: drive Pitesti -> Ramnicu Valcea with radarweb logging on (it does by default); use the MCU navigation for a while; upload the /data/maplog/map_*.jsonl. Then: `python3 -m tesla.nav.real_drive_validate` (driven route, announcements vs
  what you did) and the three-detector comparison (`tesla/tools/detector_compare.py`) on a winding road.
