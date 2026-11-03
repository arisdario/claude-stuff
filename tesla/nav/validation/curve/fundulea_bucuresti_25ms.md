# Curve-ahead replay: Fundulea -> București

route 39.05 km, replay speed 25 m/s (90 km/h), GPS noise 5 m, A_LAT 1.85 m/s^2, DECEL 1.00 m/s^2, horizon 800 m. Advisory only.

8 curves of severity >= moderate found by the CurveCue engine in the route geometry; announced with high confidence: **8**.

| # | at km | radius m | curve km/h | announced at (m before) | approach km/h then | slow-down must start (m before) | start slowing in (m) | early enough? |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.51 | 55 | 36 | 507 | 120 | 262 | 245 | yes |
| 2 | 2.81 | 56 | 37 | 787 | 147 | 261 | 526 | yes |
| 3 | 3.23 | 35 | 32 | 655 | 134 | 273 | 382 | yes |
| 4 | 35.37 | 117 | 53 | 785 | 152 | 205 | 580 | yes |
| 5 | 35.88 | 113 | 52 | 787 | 152 | 208 | 579 | yes |
| 6 | 36.26 | 94 | 47 | 738 | 146 | 226 | 512 | yes |
| 7 | 38.23 | 48 | 34 | 800 | 148 | 268 | 532 | yes |
| 8 | 38.97 | 34 | 32 | 795 | 147 | 273 | 523 | yes |

## Announcements (each curve announced once, distance then updated every fix)

8 announcements for 8 real curves: **duplicate announcements 0**, announcements not matching a real curve **0**, real curves never announced **none**, announced curves whose distance ever grew by > 25 m while approaching **0**

| id | announcement | first announced at route m | curve starts at route m | distance when announced | radius m | curve speed km/h | distance updates |
|---|---|---|---|---|---|---|---|
| 1 | Firm right ahead, slow to 35 | 2 | 508 | 507 | 55 | 36 | 24 |
| 2 | Firm left ahead, slow to 35 | 2019 | 2806 | 787 | 56 | 37 | 36 |
| 3 | Sharp right ahead, slow to 30 | 2576 | 3231 | 655 | 35 | 32 | 34 |
| 4 | Moderate right ahead | 34585 | 35370 | 785 | 117 | 53 | 34 |
| 5 | Moderate left ahead | 35093 | 35880 | 787 | 113 | 52 | 32 |
| 6 | Firm left ahead, slow to 45 | 35522 | 36260 | 738 | 94 | 47 | 32 |
| 7 | Sharp right ahead, slow to 35 | 37427 | 38227 | 800 | 48 | 34 | 36 |
| 8 | Sharp left ahead, slow to 30 | 38176 | 38972 | 795 | 34 | 32 | 33 |

by severity: 0 gentle, 2 moderate, 3 firm, 3 sharp, 0 hairpin

## Following the advisory target speed (clean-geometry simulation, cruise 25 m/s)

curves entered: 8; entered above curve speed (+0.75 m/s): **0**; max braking used: **1.00 m/s^2** (limit 1.00); lowest speed reached 8.9 m/s (32 km/h)


announced early enough to slow at 1.0 m/s^2: **8 / 8**
false slowdowns (advisory pointing at a curve that is not in the route geometry): **0 of 1562 fixes** []
road match: progress error median 3.3 m, p95 9.8 m, max 102.5 m; high-confidence fixes 1413, of which progress wrong by > 40 m (wrong forward geometry): **0**; recalculations during replay: 0
fixes with advisory active: 64; fixes without high road confidence (no advised speed published): 149
