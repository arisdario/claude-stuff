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
