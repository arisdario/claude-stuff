# Three detectors on the DRIVEN route (33.1 km): curves with radius < 200 m

| km | OLD | CC | MAPD | min R (m) | driver max before -> min in bend (km/h) | driver slowed? |
|---|---|---|---|---|---|---|
| 0.40 | x |  | x | 32 | 54 -> 20 | YES |
| 2.37 | x | x | x | 19 | 65 -> 42 | YES |
| 27.47 | x |  | x | 1 | 91 -> 76 | YES |
| 27.65 | x |  | x | 101 | 91 -> 67 | YES |
| 27.93 | x | x | x | 28 | - | no data |
| 30.26 | x |  | x | 29 | 56 -> 30 | YES |

Scored on the 5 curves with driver evidence (5 slowed):

| detector / combination | real slow-downs announced | announced but driver did not slow | real slow-downs missed | precision | recall |
|---|---|---|---|---|---|
| OLD | 5 | 0 | 0 | 100% | 100% |
| CC | 1 | 0 | 4 | 100% | 20% |
| MAPD | 5 | 0 | 0 | 100% | 100% |
| OLD or CC (union) | 5 | 0 | 0 | 100% | 100% |
| any two agree | 5 | 0 | 0 | 100% | 100% |
| all three | 1 | 0 | 4 | 100% | 20% |
| all of the above (union) | 5 | 0 | 0 | 100% | 100% |

Null baseline (positions >= 400 m from any detected curve, with speed data): the rule fires at 14 of 113 (12%) -> chance that 5 of 5 curves look slowed by luck alone ~ 0.0000
positions where it fires (km): [1.0, 1.2, 4.8, 6.2, 6.5, 7.2, 7.5, 7.8, 8.8, 9.5, 13.5, 22.0, 26.8, 27.0]
