# Curve-ahead replay: București -> Fundulea

route 41.20 km, replay speed 25 m/s (90 km/h), GPS noise 5 m, A_LAT 1.85 m/s^2, DECEL 1.00 m/s^2, horizon 800 m. Advisory only.

9 curves of severity >= moderate found by the CurveCue engine in the route geometry; announced with high confidence: **9**.

| # | at km | radius m | curve km/h | announced at (m before) | approach km/h then | slow-down must start (m before) | start slowing in (m) | early enough? |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.41 | 29 | 32 | 410 | 108 | 273 | 138 | yes |
| 2 | 1.10 | 30 | 32 | 772 | 145 | 273 | 499 | yes |
| 3 | 1.36 | 166 | 63 | 787 | 156 | 159 | 628 | yes |
| 4 | 1.67 | 152 | 60 | 671 | 145 | 172 | 499 | yes |
| 5 | 2.30 | 199 | 69 | 699 | 151 | 129 | 571 | yes |
| 6 | 12.86 | 95 | 48 | 782 | 150 | 225 | 557 | yes |
| 7 | 14.25 | 10 | 32 | 798 | 147 | 273 | 525 | yes |
| 8 | 38.30 | 79 | 43 | 794 | 150 | 240 | 554 | yes |
| 9 | 40.68 | 175 | 65 | 798 | 158 | 150 | 648 | yes |

## Announcements (each curve announced once, distance then updated every fix)

9 announcements for 9 real curves: **duplicate announcements 0**, announcements not matching a real curve **0**, real curves never announced **none**, announced curves whose distance ever grew by > 25 m while approaching **0**

| id | announcement | first announced at route m | curve starts at route m | distance when announced | radius m | curve speed km/h | distance updates |
|---|---|---|---|---|---|---|---|
| 1 | Sharp right ahead, slow to 30 | 2 | 413 | 410 | 29 | 32 | 21 |
| 2 | Sharp right ahead, slow to 30 | 331 | 1103 | 772 | 30 | 32 | 34 |
| 3 | Moderate left ahead | 575 | 1362 | 787 | 166 | 63 | 33 |
| 4 | Moderate left ahead | 1001 | 1672 | 671 | 152 | 60 | 30 |
| 5 | Moderate right ahead | 1598 | 2297 | 699 | 199 | 69 | 31 |
| 6 | Firm right ahead, slow to 50 | 12080 | 12861 | 782 | 95 | 48 | 34 |
| 7 | Hairpin right ahead, slow to 30 | 13449 | 14246 | 798 | 10 | 32 | 45 |
| 8 | Firm right ahead, slow to 45 | 37501 | 38296 | 794 | 79 | 43 | 34 |
| 9 | Moderate right ahead | 39884 | 40682 | 798 | 175 | 65 | 33 |

by severity: 0 gentle, 4 moderate, 2 firm, 2 sharp, 1 hairpin

## Following the advisory target speed (clean-geometry simulation, cruise 25 m/s)

curves entered: 9; entered above curve speed (+0.75 m/s): **0**; max braking used: **1.00 m/s^2** (limit 1.00); lowest speed reached 8.9 m/s (32 km/h)


announced early enough to slow at 1.0 m/s^2: **9 / 9**
false slowdowns (advisory pointing at a curve that is not in the route geometry): **0 of 1649 fixes** []
road match: progress error median 3.3 m, p95 10.3 m, max 263.0 m; high-confidence fixes 1469, of which progress wrong by > 40 m (wrong forward geometry): **0**; recalculations during replay: 0
fixes with advisory active: 48; fixes without high road confidence (no advised speed published): 180
