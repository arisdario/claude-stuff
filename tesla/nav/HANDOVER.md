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
