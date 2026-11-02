# Offline navigation - route validation report

Graph: routegraph schema 2, turn restrictions: **not modelled**. Planning only - no vehicle control.

Edges excluded by access/oneway rules while building the graph: access=customers x611, access=delivery x77, access=permit x225, class:bus_stop x5, motor_vehicle=customers x183, motor_vehicle=delivery x17, motor_vehicle=permit x63, service:drive-through x919, service:driveway x13220.

## București -> Fundulea

- from: București (city, exact match) at 44.43614, 26.10268; to: Fundulea (town, exact match) at 44.46837, 26.50968
- **distance 41.20 km**, estimated drive time 29 min (road-class speeds), straight line 32.50 km (x1.27)
- reverse route 39.05 km (differences are expected: oneway streets, turn costs)
- 659 directed edges, 20 instruction steps, 658 junction nodes passed

### Roads in order

| # | road | class | km |
|---|---|---|---|
| 1 | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | primary | 0.17 |
| 2 | (unnamed tertiary_link) | tertiary_link | 0.01 |
| 3 | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | primary | 0.22 |
| 4 | Piața Universității (DN4;DN2;DN3) | primary | 0.02 |
| 5 | Bulevardul Ion Constantin Brătianu (DN4) | primary | 0.74 |
| 6 | Strada Halelor | primary | 0.12 |
| 7 | Piața Unirii | primary | 0.61 |
| 8 | Splaiul Unirii | secondary | 10.82 |
| 9 | Șoseaua de Centură (DNCB) | primary | 1.50 |
| 10 | (unnamed primary_link) | primary_link | 0.06 |
| 11 | (unnamed tertiary) | tertiary | 0.11 |
| 12 | (unnamed motorway_link) | motorway_link | 0.46 |
| 13 | Autostrada Soarelui (A2) | motorway | 23.02 |
| 14 | (unnamed motorway_link) | motorway_link | 0.54 |
| 15 | DJ402 | secondary | 0.83 |
| 16 | Strada Mihail Kogălniceanu (DJ402) | secondary | 1.39 |
| 17 | Bulevardul 22 Decembrie (DN3) | primary | 0.06 |
| 18 | Strada Primăverii | residential | 0.04 |
| 19 | Strada Bradului | residential | 0.47 |

### Instructions

-  0.00 km  Head north-west on Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7)
-  0.17 km  Turn left onto (unnamed tertiary_link)  (node 11072303045, 3 exits)
-  0.18 km  Turn left onto Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7)  (node 11072303046, 1 exits)
-  0.43 km  At the roundabout take exit 1 onto Bulevardul Ion Constantin Brătianu (DN4)  (node 6565086693, 2 exits)
-  1.17 km  Bear right onto Strada Halelor  (node 6908633254, 1 exits)
-  1.29 km  Bear left onto Piața Unirii  (node 581512, 2 exits)
-  1.65 km  Turn left onto Piața Unirii  (node 31404739, 4 exits)
-  1.90 km  Continue onto Splaiul Unirii  (node 31404741, 3 exits)
- 12.72 km  Turn left onto Șoseaua de Centură (DNCB)  (node 6376968050, 2 exits)
- 14.22 km  Continue onto (unnamed primary_link)  (node 241780058, 2 exits)
- 14.29 km  Bear right onto (unnamed tertiary)  (node 1374720800, 1 exits)
- 14.39 km  At the roundabout take exit 2 onto (unnamed motorway_link)  (node 1374720913, 2 exits)
- 14.85 km  Continue onto Autostrada Soarelui (A2)  (node 2369448696, 1 exits)
- 37.87 km  Continue onto (unnamed motorway_link)  (node 670107, 2 exits)
- 38.41 km  Turn right onto DJ402  (node 480295643, 3 exits)
- 39.24 km  Continue onto Strada Mihail Kogălniceanu (DJ402)  (node 405138541, 1 exits)
- 40.63 km  Turn left onto Bulevardul 22 Decembrie (DN3)  (node 13749468052, 2 exits)
- 40.69 km  Turn right onto Strada Primăverii  (node 1601699064, 2 exits)
- 40.73 km  Turn left onto Strada Bradului  (node 1601699109, 2 exits)
- 41.20 km  Arrive at your destination

### Junctions with alternatives (89 of 658 nodes passed)

| at km | OSM node | legal exits | turn | from | to |
|---|---|---|---|---|---|
| 0.00 | 9422684451 | 2 | +4 deg | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) |
| 0.17 | 11072303045 | 3 | -89 deg | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | (unnamed tertiary_link) |
| 0.24 | 13841774313 | 2 | -0 deg | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) |
| 0.41 | 494777621 | 2 | +15 deg | Bulevardul Nicolae Bălcescu (DN1;DN1A;DN7) | Piața Universității (DN4;DN2;DN3) |
| 0.43 | 6565086693 | 2 | +33 deg | Piața Universității (DN4;DN2;DN3) | Bulevardul Ion Constantin Brătianu (DN4) |
| 0.53 | 391087090 | 2 | +1 deg | Bulevardul Ion Constantin Brătianu (DN4) | Bulevardul Ion Constantin Brătianu (DN4) |
| 0.60 | 257682899 | 2 | -12 deg | Bulevardul Ion Constantin Brătianu (DN4) | Bulevardul Ion Constantin Brătianu (DN4) |
| 0.62 | 2160093491 | 3 | +13 deg | Bulevardul Ion Constantin Brătianu (DN4) | Bulevardul Ion Constantin Brătianu (DN4) |
| 0.95 | 2160103784 | 2 | +0 deg | Bulevardul Ion Constantin Brătianu (DN4) | Bulevardul Ion Constantin Brătianu (DN4) |
| 1.29 | 581512 | 2 | -54 deg | Strada Halelor | Piața Unirii |
| 1.37 | 682438 | 2 | -1 deg | Piața Unirii | Piața Unirii |
| 1.49 | 11040070381 | 2 | +1 deg | Piața Unirii | Piața Unirii |
| 1.56 | 254337787 | 2 | -1 deg | Piața Unirii | Piața Unirii |
| 1.65 | 31404739 | 4 | -68 deg | Piața Unirii | Piața Unirii |
| 1.90 | 31404741 | 3 | -2 deg | Piața Unirii | Splaiul Unirii |
| 2.02 | 967064848 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 2.52 | 647188956 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 2.54 | 8356039147 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 2.67 | 254451008 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 2.76 | 2351625031 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 2.89 | 254451696 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 2.91 | 234958282 | 2 | -6 deg | Splaiul Unirii | Splaiul Unirii |
| 3.05 | 11432887493 | 2 | -4 deg | Splaiul Unirii | Splaiul Unirii |
| 3.07 | 2160062536 | 2 | +1 deg | Splaiul Unirii | Splaiul Unirii |
| 3.27 | 8329247442 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 3.29 | 151020447 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 3.37 | 254458066 | 2 | +1 deg | Splaiul Unirii | Splaiul Unirii |
| 3.39 | 234953759 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 3.76 | 254459309 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 4.03 | 254460877 | 2 | +1 deg | Splaiul Unirii | Splaiul Unirii |
| 4.11 | 254461141 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 4.16 | 6225649411 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 4.40 | 116822454 | 2 | -2 deg | Splaiul Unirii | Splaiul Unirii |
| 4.42 | 101068095 | 2 | -3 deg | Splaiul Unirii | Splaiul Unirii |
| 4.74 | 1835210930 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 6.09 | 11178261082 | 2 | +1 deg | Splaiul Unirii | Splaiul Unirii |
| 6.14 | 100815543 | 3 | -7 deg | Splaiul Unirii | Splaiul Unirii |
| 7.55 | 7704918522 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 7.58 | 2643797684 | 2 | -1 deg | Splaiul Unirii | Splaiul Unirii |
| 7.64 | 2643797767 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 7.77 | 6530473936 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 9.54 | 12175155691 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 9.58 | 11432129459 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 9.59 | 12053835865 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 10.00 | 922636774 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 10.80 | 11765618336 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 10.91 | 8929313694 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 11.06 | 7439317447 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 11.07 | 10599447134 | 2 | +0 deg | Splaiul Unirii | Splaiul Unirii |
| 11.18 | 11053736555 | 2 | -0 deg | Splaiul Unirii | Splaiul Unirii |
| 12.72 | 6376968050 | 2 | -101 deg | Splaiul Unirii | Șoseaua de Centură (DNCB) |
| 12.78 | 2461869053 | 2 | -0 deg | Șoseaua de Centură (DNCB) | Șoseaua de Centură (DNCB) |
| 12.81 | 13401999133 | 2 | -0 deg | Șoseaua de Centură (DNCB) | Șoseaua de Centură (DNCB) |
| 12.91 | 13401999124 | 2 | +2 deg | Șoseaua de Centură (DNCB) | Șoseaua de Centură (DNCB) |
| 12.95 | 254489694 | 2 | +4 deg | Șoseaua de Centură (DNCB) | Șoseaua de Centură (DNCB) |
| 14.11 | 2278362978 | 2 | +0 deg | Șoseaua de Centură (DNCB) | Șoseaua de Centură (DNCB) |
| 14.22 | 241780058 | 2 | +13 deg | Șoseaua de Centură (DNCB) | (unnamed primary_link) |
| 14.38 | 1374720770 | 2 | -21 deg | (unnamed tertiary) | (unnamed tertiary) |
| 14.39 | 1374720913 | 2 | +15 deg | (unnamed tertiary) | (unnamed motorway_link) |
| 15.24 | 2653647788 | 2 | +1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 15.97 | 2651589315 | 2 | -1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 17.57 | 670045 | 2 | -1 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 21.89 | 2337590256 | 2 | -0 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 24.81 | 2212516053 | 2 | +0 deg | Autostrada Soarelui (A2) | Autostrada Soarelui (A2) |
| 37.87 | 670107 | 2 | +11 deg | Autostrada Soarelui (A2) | (unnamed motorway_link) |
| 38.20 | 2385996570 | 3 | +2 deg | (unnamed motorway_link) | (unnamed motorway_link) |
| 38.41 | 480295643 | 3 | +88 deg | (unnamed motorway_link) | DJ402 |
| 39.27 | 265956448 | 2 | +2 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.37 | 2326854312 | 3 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.53 | 2326854333 | 3 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.65 | 2326854318 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.68 | 303821145 | 2 | +1 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.89 | 2411790979 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.94 | 8041791326 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 39.99 | 774544501 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 40.19 | 2437681081 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 40.21 | 2437681061 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 40.44 | 265956451 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 40.55 | 2326854339 | 2 | -0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| 40.60 | 13749468058 | 2 | +0 deg | Strada Mihail Kogălniceanu (DJ402) | Strada Mihail Kogălniceanu (DJ402) |
| ... | 9 more | | | | |

### Checks

- connectivity (shared OSM node between every pair of edges, geometry continuity, no reversals, oneway + access re-checked from raw tags): **FAIL**
- simulated noisy GPS drive (2747 fixes, 5 m noise): {'on_route': 2727, 'checking': 12, 'rerouted': 1, 'off_route': 5, 'arrived': 2}, reroutes 1 -> **FAIL**
- leave-the-route test: wrong-road edge 171864 (way 347290634), junction node 670045, oracle reachable=True, navigator status=arrived -> **FAIL (SNAP_BACK)**
  - outcome SNAP_BACK; detected=True, recalculated=True after 881 fixes, new route 26.48 km, final status arrived
- independent oracle vs planner (300 directed edges near the route from-edge, 52 mid-edge GPS fixes): **PASS**
- reference comparison: **not performed** (no trusted reference route supplied; pass --reference)

Connectivity problems: 

- **simulated on-route drive triggered 1 reroute(s) / 5 off-route fixes (false positive)**
- **SNAP-BACK - the new route does not start on the wrong road the car is on: wrong-road edge 171864 (way 347290634), junction node 670045, oracle reachable=True, navigator status=arrived**

## Limitations

- Turn-restriction relations (no_left_turn, only_straight_on, ...) are not in the graph: a route may contain a turn a sign forbids.
- Time-conditional access, vehicle weight/height limits and tolls are not modelled; travel times use class/maxspeed speeds, not traffic.
- Advisory only: nothing here is connected to steering or braking.
