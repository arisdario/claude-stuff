# Curve-ahead replay: Fundulea -> București

route 39.05 km, replay speed 25 m/s (90 km/h), GPS noise 5 m, A_LAT 1.85 m/s^2, DECEL 1.00 m/s^2, horizon 800 m. Advisory only.

12 real curves of severity >= moderate (radius < 200 m; curve speed < 108 km/h, >= 2 samples) in the route geometry; announced with high confidence: **12**.

| # | at km | radius m | curve km/h | announced at (m before) | approach km/h then | slow-down must start (m before) | start slowing in (m) | early enough? |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.45 | 41 | 32 | 448 | 113 | 273 | 176 | yes |
| 2 | 2.77 | 25 | 32 | 786 | 147 | 273 | 522 | yes |
| 3 | 3.10 | 162 | 62 | 524 | 132 | 162 | 362 | yes |
| 4 | 3.33 | 38 | 32 | 754 | 143 | 273 | 482 | yes |
| 5 | 35.38 | 134 | 57 | 785 | 157 | 188 | 636 | yes |
| 6 | 35.88 | 133 | 56 | 787 | 154 | 189 | 597 | yes |
| 7 | 36.27 | 37 | 32 | 748 | 143 | 273 | 475 | yes |
| 8 | 36.48 | 26 | 32 | 788 | 146 | 273 | 515 | yes |
| 9 | 38.00 | 35 | 32 | 782 | 152 | 273 | 576 | yes |
| 10 | 38.24 | 76 | 43 | 787 | 149 | 242 | 545 | yes |
| 11 | 38.81 | 194 | 68 | 782 | 164 | 133 | 725 | yes |
| 12 | 39.00 | 82 | 44 | 788 | 170 | 237 | 0 | yes |

## Announcements (each curve announced once, distance then updated every fix)

12 announcements for 12 real curves: **duplicate announcements 0**, announcements not matching a real curve **0**, real curves never announced **none**, announced curves whose distance ever grew by > 25 m while approaching **0**

| id | announcement | first announced at route m | curve starts at route m | distance when announced | radius m | curve speed km/h | distance updates |
|---|---|---|---|---|---|---|---|
| 1 | Sharp right ahead, slow to 30 | 2 | 440 | 438 | 41 | 32 | 25 |
| 2 | Firm left ahead, slow to 35 | 1974 | 2760 | 786 | 53 | 36 | 39 |
| 3 | Moderate right ahead | 2576 | 3040 | 464 | 162 | 62 | 24 |
| 4 | Sharp right ahead, slow to 30 | 2576 | 3220 | 644 | 38 | 32 | 37 |
| 5 | Moderate right ahead | 34585 | 35360 | 775 | 176 | 65 | 35 |
| 6 | Moderate left ahead | 35093 | 35870 | 777 | 133 | 56 | 33 |
| 7 | Sharp right ahead, slow to 30 | 35522 | 36240 | 718 | 37 | 32 | 36 |
| 8 | Sharp left ahead, slow to 30 | 35692 | 36470 | 778 | 26 | 32 | 33 |
| 9 | Moderate right ahead | 37208 | 37990 | 782 | 115 | 53 | 34 |
| 10 | Moderate right ahead | 37427 | 38160 | 733 | 172 | 64 | 37 |
| 11 | Moderate left ahead | 38051 | 38780 | 729 | 194 | 68 | 32 |
| 12 | Firm left ahead, slow to 45 | 38216 | 38960 | 744 | 82 | 44 | 32 |

by severity: 0 gentle, 6 moderate, 2 firm, 4 sharp, 0 hairpin

## Following the advisory target speed (clean-geometry simulation, cruise 25 m/s)

curves entered: 12; entered above curve speed (+0.75 m/s): **0**; max braking used: **1.00 m/s^2** (limit 1.00); lowest speed reached 8.9 m/s (32 km/h)


announced early enough to slow at 1.0 m/s^2: **12 / 12**
false slowdowns (advisory pointing at a curve that is not in the route geometry): **0 of 1562 fixes** []
road match: progress error median 3.3 m, p95 9.8 m, max 102.5 m; high-confidence fixes 1413, of which progress wrong by > 40 m (wrong forward geometry): **0**; recalculations during replay: 0
fixes with advisory active: 82; fixes without high road confidence (no advised speed published): 149
