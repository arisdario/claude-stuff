# Three detectors vs what the driver did - two real drives (re-run after the replay-harness fix)

## Drive 1: Bucuresti -> Fundulea on the A2 (map_20261101_175314)

| km | OLD | CC | MAPD | min R (m) | driver max before -> min in bend (km/h) | driver slowed? |
|---|---|---|---|---|---|---|
| 0.40 | x |  | x | 32 | 54 -> 20 | YES |
| 2.37 | x | x | x | 19 | 64 -> 52 | YES |
| 27.47 | x |  | x | 1 | 112 -> 87 | YES |
| 27.65 | x |  | x | 101 | 110 -> 82 | YES |
| 27.93 | x | x | x | 28 | 88 -> 84 | no |
| 30.26 | x |  | x | 29 | 56 -> 56 | no |

Scored on the 6 curves with driver evidence (4 slowed):

| detector / combination | real slow-downs announced | announced but driver did not slow | real slow-downs missed | precision | recall |
|---|---|---|---|---|---|
| OLD | 4 | 2 | 0 | 67% | 100% |
| CC | 1 | 1 | 3 | 50% | 25% |
| MAPD | 4 | 2 | 0 | 67% | 100% |
| OLD or CC (union) | 4 | 2 | 0 | 67% | 100% |
| any two agree | 4 | 2 | 0 | 67% | 100% |
| all three | 1 | 1 | 3 | 50% | 25% |
| all of the above (union) | 4 | 2 | 0 | 67% | 100% |

Null baseline (positions >= 400 m from any detected curve, with speed data): the rule fires at 16 of 113 (14%) -> chance that 5 of 5 curves look slowed by luck alone ~ 0.0001
positions where it fires (km): [1.0, 1.2, 4.8, 5.0, 6.5, 7.5, 7.8, 8.0, 9.0, 9.8, 14.0, 22.5, 28.8, 31.0, 31.2, 31.5]

## Drive 2: Fundulea -> Bucuresti on DN3 (map_20261101_182944, 22 km of driving, 85 min parked in the middle)

| km | OLD | CC | MAPD | min R (m) | driver max before -> min in bend (km/h) | driver slowed? |
|---|---|---|---|---|---|---|
| 10.38 | x | x | x | 13 | 76 -> 50 | YES |
| 12.15 | x | x | x | 27 | 62 -> 61 | no |
| 12.64 | x |  | x | 148 | 62 -> 47 | YES |
| 13.19 |  |  | x | 161 | 73 -> 79 | no |
| 18.74 | x |  | x | 45 | 50 -> 50 | no |
| 21.87 | x | x |  | 18 | 49 -> 51 | no |
| 22.19 | x |  | x | 155 | 62 -> 55 | YES |
| 26.33 | x | x | x | 31 | 68 -> -0 | YES |
| 26.97 | x | x | x | 31 | 64 -> 0 | YES |
| 27.29 | x | x | x | 14 | 59 -> 34 | YES |

Scored on the 10 curves with driver evidence (6 slowed):

| detector / combination | real slow-downs announced | announced but driver did not slow | real slow-downs missed | precision | recall |
|---|---|---|---|---|---|
| OLD | 6 | 3 | 0 | 67% | 100% |
| CC | 4 | 2 | 2 | 67% | 67% |
| MAPD | 6 | 3 | 0 | 67% | 100% |
| OLD or CC (union) | 6 | 3 | 0 | 67% | 100% |
| any two agree | 6 | 3 | 0 | 67% | 100% |
| all three | 4 | 1 | 2 | 80% | 67% |
| all of the above (union) | 6 | 4 | 0 | 60% | 100% |

Null baseline (positions >= 400 m from any detected curve, with speed data): the rule fires at 17 of 81 (21%) -> chance that 5 of 5 curves look slowed by luck alone ~ 0.0004
positions where it fires (km): [4.2, 5.8, 6.0, 6.2, 7.2, 7.5, 9.0, 11.2, 16.5, 16.8, 17.8, 18.2, 19.2, 19.8, 20.5, 21.2, 25.0]
