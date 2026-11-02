# mapcurve schema v2 and runtime interfaces (as of commit 0ad00df)

Source of truth: `tesla/tools/mapcurve_build.py` (builder) and `tesla/mapcurved.py` (runtime). This documents what exists;
nothing in the navigation prototype modifies either.

## Database (`/data/mapcurve.sqlite`)

`pts` - one row per sample along a road (original OSM vertices plus filler points every <= 6 m):

| column | type | meaning |
|---|---|---|
| `lat`, `lon` | REAL | WGS84, 7 decimals |
| `speed` | REAL | curve speed in m/s = sqrt(1.85 / abs(curvature)), clipped to 8.94 .. 255 |
| `cell_lat`, `cell_lon` | INTEGER | `floor(lat / 0.01)`, `floor(lon / 0.01)` grid index |
| `way_id` | TEXT | OSM way id |
| `segment_id` | INTEGER | piece of the way (0, 1, ...) |
| `seq` | INTEGER | order of the sample inside the segment |
| `direction` | INTEGER | oneway code: 1 = forward only along `seq`, -1 = reverse only, 0 = both |
| `highway` | TEXT | OSM highway class |
| `curvature` | REAL | signed Menger curvature, 1/m |

`segments` - `(way_id, segment_id)` primary key; `highway`, `name`, `oneway`, `point_count`, and (v2) `start_node_id`,
`end_node_id` (OSM node IDs of the first/last sample; NULL when built with `--no-nodes`).
`meta` - `schema_version` ("2"), `node_ids` ("yes"/"no"), `a_y`.
Indexes: `idx_pts_cell(cell_lat, cell_lon)`, `idx_pts_road_order(way_id, segment_id, seq)`, `idx_pts_way`,
`idx_seg_start`, `idx_seg_end`.

Connectivity rule: ways are split into segments at every node shared by two drivable ways, so two segments are connected
**iff** one's `end_node_id`/`start_node_id` equals the other's. Never by coordinates. A v1 database (no node columns, no
`meta`) is detected by the runtime (`has_nodes == False`) and never joined.

## Runtime (`mapcurved.MapCurveLookup`)

`MapCurveLookup(db_path)` opens the DB read-only (`PRAGMA query_only=ON`); `has_nodes` is true only if `segments` has
`start_node_id`, `end_node_id` and `oneway`.

`query_ahead(lat, lon, heading_deg, v_ego=0.0)` returns `None` or `(speed_ms, dist_m, match)`:
- `speed_ms` stored curve speed of the binding point (lowest `approach_speed`), `dist_m` its distance along the road;
- `match = {way_id, segment_id, match_m, heading_error, direction, connections[], truncated, node_ids}`;
- `connections[]` = hops taken through shared nodes: `{way_id, segment_id, node, deviation, candidates, same_way, at_m}`;
- `truncated` = why the walk ended: `lookahead | dead_end | ambiguous | sharp_turn | oneway | no_node_ids | no_node_id |
  hop_limit | no_bearing`.
Matching: nearest sample per segment within `MATCH_RADIUS_M` (35 m), travel direction from heading vs the segment tangent
(`MATCH_HEADING_DEG` 45), honouring `direction`. NaN/inf/None/out-of-range position or heading returns `None`.
Junction choice: same OSM way is taken directly; between different roads the best must be within `STRAIGHT_DEG` (30) of
straight and beat the runner-up by `AMBIGUITY_MARGIN_DEG` (25), a lone continuation up to `MAX_TURN_DEG` (100); else stop.

Output path: `mapcurved.main()` publishes `liveMapData` (`mapValid`, `curvatureValid`, `curvature = 1.85 / speed^2`,
`distToTurn`) which `PCC_module` consumes. `mapcurve_launcher.ensure_running()` starts it from `carcontroller.__init__`.

## Why this database cannot be the routing graph

1. **Only 6 road classes** (motorway/trunk/primary + links). Predeal's local streets and the Timisu villages are mostly
   secondary/tertiary/residential/unclassified - absent.
2. **No access tags** (`access`, `motor_vehicle`, `vehicle`, ...), **no `maxspeed`/`ref`**, no turn restrictions.
3. **Oneway is partial**: roundabout (`junction=*`) implied oneway and `oneway:*` variants are not read.
4. **Segments are split for curve lookup**, not for routing, and samples are densified for curvature.
5. The current national `.sqlite` is **v1 (no node IDs)**: nothing in it can be joined.

So routing uses a **separate** graph database (`routegraph.sqlite`, built by `tesla/nav/routegraph.py` from the same PBF with
all drivable classes and access tags). The mapcurve database and runtime are untouched; the national database is not
rebuilt or replaced.
