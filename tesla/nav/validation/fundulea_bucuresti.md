# Offline navigation - route validation report

Graph: routegraph schema 2, turn restrictions: **not modelled**. Planning only - no vehicle control.

Edges excluded by access/oneway rules while building the graph: access=customers x611, access=delivery x77, access=permit x225, class:bus_stop x5, motor_vehicle=customers x183, motor_vehicle=delivery x17, motor_vehicle=permit x63, service:drive-through x919, service:driveway x13220.

## Fundulea -> București

- from: Fundulea (town, exact match) at 44.46837, 26.50968; to: București (city, exact match) at 44.43614, 26.10268
- **distance 39.05 km**, estimated drive time 30 min (road-class speeds), straight line 32.50 km (x1.20)
- reverse route 41.20 km (differences are expected: oneway streets, turn costs)
- 640 directed edges, 19 instruction steps, 639 junction nodes passed

### Roads in order

| # | road | class | km |
|---|---|---|---|
| 1 | Strada Bradului | residential | 0.47 |
| 2 | Strada Primăverii | residential | 0.04 |
| 3 | Bulevardul 22 Decembrie (DN3) | primary | 0.03 |
| 4 | (unnamed secondary_link) | secondary_link | 0.05 |
| 5 | Strada Mihail Kogălniceanu (DJ402) | secondary | 1.36 |
| 6 | DJ402 | secondary | 0.83 |
| 7 | (unnamed motorway_link) | motorway_link | 0.69 |
| 8 | Autostrada Soarelui (A2) | motorway | 24.74 |
| 9 | Pasajul Theodor Pallady | primary | 0.48 |
| 10 | Bulevardul Theodor Pallady | primary | 4.12 |
| 11 | Bulevardul Camil Ressu | primary | 2.33 |
| 12 | Calea Dudești | secondary | 1.15 |
| 13 | Strada Lucian Blaga | secondary | 0.21 |
| 14 | Bulevardul Unirii | primary | 1.52 |
| 15 | Piața Unirii | primary | 0.24 |
| 16 | Bulevardul Ion Constantin Brătianu (DN4) | primary | 0.56 |
| 17 | Bulevardul Ion C. Brătianu (DN4) | primary | 0.18 |
| 18 | Piața Universității (DN4;DN2;DN3) | primary | 0.03 |
| 19 | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | primary | 0.03 |

### Instructions

-  0.00 km  Head south-east on Strada Bradului
-  0.47 km  Turn right onto Strada Primăverii  (node 1601699109, 2 exits)
-  0.51 km  Turn left onto Bulevardul 22 Decembrie (DN3)  (node 1601699064, 2 exits)
-  0.54 km  Continue onto (unnamed secondary_link)  (node 13749468053, 2 exits)
-  0.59 km  Bear right onto Strada Mihail Kogălniceanu (DJ402)  (node 13749468058, 3 exits)
-  1.95 km  Continue onto DJ402  (node 405138541, 1 exits)
-  2.78 km  Turn left onto (unnamed motorway_link)  (node 480295643, 3 exits)
-  3.47 km  Continue onto Autostrada Soarelui (A2)  (node 444684775, 1 exits)
- 28.21 km  Continue onto Pasajul Theodor Pallady  (node 10276563088, 1 exits)
- 28.68 km  Continue onto Bulevardul Theodor Pallady  (node 3121630897, 1 exits)
- 32.80 km  Continue onto Bulevardul Camil Ressu  (node 660928, 2 exits)
- 35.13 km  Continue onto Calea Dudești  (node 2198300941, 2 exits)
- 36.28 km  Turn right onto Strada Lucian Blaga  (node 2092822511, 2 exits)
- 36.49 km  Turn left onto Bulevardul Unirii  (node 578689, 2 exits)
- 38.01 km  Turn right onto Piața Unirii  (node 2400144428, 1 exits)
- 38.25 km  Continue onto Bulevardul Ion Constantin Brătianu (DN4)  (node 2404241703, 2 exits)
- 38.81 km  Continue onto Bulevardul Ion C. Brătianu (DN4)  (node 2160093499, 1 exits)
- 39.01 km  At the roundabout take exit 1 onto Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7)  (node 1823167126, 2 exits)
- 39.05 km  Arrive at your destination

### Junctions with alternatives (106 of 639 nodes passed)

| at km | OSM node | legal exits | turn | from | to |
|---|---|---|---|---|---|
| 0.06 | 1601699046 | 2 | -1 deg | Strada Bradului | Strada Bradului |
| 0.18 | 1601699034 | 2 | -4 deg | Strada Bradului | Strada Bradului |
| 0.21 | 1601698889 | 2 | +2 deg | Strada Bradului | Strada Bradului |
| 0.27 | 1601698870 | 2 | -0 deg | Strada Bradului | Strada Bradului |
| 0.47 | 1601699109 | 2 | +91 deg | Strada Bradului | Strada Primăverii |
| 0.51 | 1601699064 | 2 | -96 deg | Strada Primăverii | Bulevardul 22 Decembrie (DN3) |
| 0.52 | 1601698953 | 2 | -1 deg | Bulevardul 22 Decembrie (DN3) | Bulevardul 22 Decembrie (DN3) |
| 0.54 | 13749468053 | 2 | +12 deg | Bulevardul 22 Decembrie (DN3) | (unnamed secondary_link) |
| 0.59 | 13749468058 | 3 | +24 deg | (unnamed secondary_link) | Strada Mihail Kogălniceanu (DJ402) |
| 0.64 | 2326854339 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 0.75 | 265956451 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 0.98 | 2437681061 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.00 | 2437681081 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.20 | 774544501 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.25 | 8041791326 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.30 | 2411790979 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.51 | 303821145 | 2 | -1 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.54 | 2326854318 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.66 | 2326854333 | 3 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.82 | 2326854312 | 3 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 1.92 | 265956448 | 2 | -2 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 2.78 | 480295643 | 3 | -88 deg | DJ402 | (unnamed motorway_link) |
| 2.99 | 2385996570 | 2 | +7 deg | (unnamed motorway_link) | (unnamed motorway_link) |
| 16.49 | 670057 | 2 | +0 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 18.74 | 2576337665 | 2 | -0 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 23.50 | 3536364594 | 2 | +1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 25.06 | 226237489 | 2 | +0 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 26.68 | 275179590 | 2 | -1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 28.15 | 660956 | 2 | -1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 28.89 | 13370577734 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 29.19 | 13482121182 | 2 | +1 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 30.26 | 2518777549 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 30.36 | 2254075245 | 3 | +0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 30.71 | 5458667396 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 30.77 | 5458667409 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 30.88 | 6525688943 | 2 | +0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.17 | 2378400448 | 3 | +0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.37 | 257409038 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.52 | 257410026 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.64 | 257403137 | 3 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.70 | 257406074 | 2 | +0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 31.98 | 13990890353 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 32.05 | 255365250 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 32.07 | 6908483544 | 2 | +2 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 32.40 | 257341834 | 2 | -0 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 32.68 | 257344611 | 2 | -2 deg | Bulevardul Theodor Pallady | Bulevardul Theodor Pallady |
| 32.80 | 660928 | 2 | -2 deg | Bulevardul Theodor Pallady | Bulevardul Camil Ressu |
| 32.82 | 242404005 | 2 | -2 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.18 | 257415098 | 2 | +1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.48 | 242409588 | 2 | +1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.66 | 242409226 | 2 | -1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.67 | 6558338869 | 2 | +2 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.85 | 630366888 | 2 | -1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 33.94 | 630367283 | 2 | +0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.02 | 885600907 | 2 | +0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.11 | 886483295 | 2 | -0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.19 | 886483298 | 2 | -0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.28 | 2365377250 | 2 | -4 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.31 | 6261902802 | 2 | -0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.32 | 2400993409 | 2 | -0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.52 | 256698630 | 2 | -1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.87 | 574840 | 2 | +0 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 34.89 | 2152005747 | 2 | -1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 35.05 | 494426602 | 2 | -1 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 35.09 | 3683088659 | 2 | -5 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 35.12 | 2198300957 | 2 | -4 deg | Bulevardul Camil Ressu | Bulevardul Camil Ressu |
| 35.13 | 2198300941 | 2 | +3 deg | Bulevardul Camil Ressu | Calea Dudești |
| 35.18 | 6266512118 | 2 | +1 deg | Calea Dudești | Calea Dudești |
| 35.23 | 2092822371 | 2 | +1 deg | Calea Dudești | Calea Dudești |
| 35.27 | 2330839618 | 2 | -0 deg | Calea Dudești | Calea Dudești |
| 35.29 | 2298559235 | 2 | +0 deg | Calea Dudești | Calea Dudești |
| 35.37 | 2092822399 | 3 | +1 deg | Calea Dudești | Calea Dudești |
| 35.43 | 2092822369 | 2 | +4 deg | Calea Dudești | Calea Dudești |
| 35.48 | 2092822411 | 2 | -1 deg | Calea Dudești | Calea Dudești |
| 35.59 | 2092822390 | 4 | +0 deg | Calea Dudești | Calea Dudești |
| 35.64 | 2092822555 | 3 | -1 deg | Calea Dudești | Calea Dudești |
| 35.69 | 2092822472 | 3 | -0 deg | Calea Dudești | Calea Dudești |
| 35.73 | 2204349553 | 2 | -2 deg | Calea Dudești | Calea Dudești |
| 35.74 | 2330839642 | 2 | -0 deg | Calea Dudești | Calea Dudești |
| 35.77 | 2092822492 | 3 | +1 deg | Calea Dudești | Calea Dudești |
| ... | 26 more | | | | |

### Checks

- connectivity (shared OSM node between every pair of edges, geometry continuity, no reversals, oneway + access re-checked from raw tags): **FAIL**
- simulated noisy GPS drive (2604 fixes, 5 m noise): {'on_route': 2586, 'checking': 12, 'rerouted': 3, 'arrived': 3}, reroutes 3 -> **FAIL**
- leave-the-route test: wrong-road edge 236940 (way 866279335), junction node 670057, oracle reachable=True, navigator status=arrived -> **PASS**
  - outcome REROUTED; detected=True, recalculated=True after 1072 fixes, new route 22.47 km, final status arrived
- independent oracle vs planner (300 directed edges near the route from-edge, 144 mid-edge GPS fixes): **PASS**
- reference comparison: **not performed** (no trusted reference route supplied; pass --reference)

Connectivity problems: 

- **simulated on-route drive triggered 3 reroute(s) / 0 off-route fixes (false positive)**
- **expected exactly 1 recalculation, got 4: wrong-road edge 236940 (way 866279335), junction node 670057, oracle reachable=True, navigator status=arrived**

## Limitations

- Turn-restriction relations (no_left_turn, only_straight_on, ...) are not in the graph: a route may contain a turn a sign forbids.
- Time-conditional access, vehicle weight/height limits and tolls are not modelled; travel times use class/maxspeed speeds, not traffic.
- Advisory only: nothing here is connected to steering or braking.
