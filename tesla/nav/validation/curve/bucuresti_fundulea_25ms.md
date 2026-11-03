# Curve-ahead replay: București -> Fundulea

route 41.20 km, replay speed 25 m/s (90 km/h), GPS noise 5 m, A_LAT 1.85 m/s^2, DECEL 1.00 m/s^2, horizon 800 m. Advisory only.

15 real curves of severity >= moderate (radius < 200 m; curve speed < 108 km/h, >= 2 samples) in the route geometry; announced with high confidence: **15**.

| # | at km | radius m | curve km/h | announced at (m before) | approach km/h then | slow-down must start (m before) | start slowing in (m) | early enough? |
|---|---|---|---|---|---|---|---|---|
| 1 | 0.16 | 18 | 32 | 158 | 72 | 273 | -115 | NO (announced 115 m late) |
| 2 | 0.41 | 191 | 68 | 408 | 123 | 136 | 271 | yes |
| 3 | 1.14 | 42 | 32 | 789 | 150 | 273 | 561 | yes |
| 4 | 1.29 | 46 | 33 | 781 | 146 | 270 | 511 | yes |
| 5 | 1.36 | 53 | 36 | 785 | 147 | 263 | 521 | yes |
| 6 | 1.64 | 34 | 32 | 639 | 133 | 273 | 367 | yes |
| 7 | 2.28 | 155 | 61 | 682 | 146 | 169 | 513 | yes |
| 8 | 12.71 | 28 | 32 | 779 | 146 | 273 | 507 | yes |
| 9 | 12.87 | 124 | 55 | 780 | 154 | 197 | 605 | yes |
| 10 | 14.37 | 32 | 32 | 766 | 145 | 273 | 494 | yes |
| 11 | 14.54 | 41 | 32 | 787 | 150 | 273 | 553 | yes |
| 12 | 14.82 | 177 | 65 | 762 | 155 | 149 | 613 | yes |
| 13 | 38.02 | 102 | 49 | 785 | 152 | 218 | 584 | yes |
| 14 | 38.40 | 27 | 32 | 776 | 145 | 273 | 503 | yes |
| 15 | 40.62 | 28 | 32 | 781 | 163 | 273 | 716 | yes |

## Announcements (each curve announced once, distance then updated every fix)

15 announcements for 15 real curves: **duplicate announcements 0**, announcements not matching a real curve **0**, real curves never announced **none**, announced curves whose distance ever grew by > 25 m while approaching **0**

| id | announcement | first announced at route m | curve starts at route m | distance when announced | radius m | curve speed km/h | distance updates |
|---|---|---|---|---|---|---|---|
| 1 | Hairpin left ahead, slow to 30 | 2 | 150 | 148 | 18 | 32 | 9 |
| 2 | Moderate left ahead | 2 | 380 | 378 | 191 | 68 | 18 |
| 3 | Firm right ahead, slow to 45 | 331 | 1090 | 759 | 91 | 47 | 36 |
| 4 | Sharp left ahead, slow to 35 | 499 | 1270 | 771 | 46 | 33 | 34 |
| 5 | Firm left ahead, slow to 35 | 575 | 1340 | 765 | 53 | 36 | 34 |
| 6 | Sharp left ahead, slow to 30 | 1001 | 1630 | 629 | 34 | 32 | 30 |
| 7 | Moderate right ahead | 1598 | 2130 | 532 | 155 | 61 | 31 |
| 8 | Sharp left ahead, slow to 30 | 11931 | 12700 | 769 | 28 | 32 | 34 |
| 9 | Moderate right ahead | 12080 | 12850 | 770 | 148 | 60 | 37 |
| 10 | Moderate right ahead | 13449 | 14210 | 761 | 107 | 51 | 39 |
| 11 | Firm left ahead, slow to 45 | 13661 | 14430 | 769 | 86 | 45 | 38 |
| 12 | Moderate left ahead | 14058 | 14810 | 752 | 177 | 65 | 34 |
| 13 | Moderate right ahead | 37225 | 37980 | 755 | 120 | 54 | 37 |
| 14 | Firm right ahead, slow to 40 | 37501 | 38260 | 759 | 60 | 38 | 38 |
| 15 | Sharp left ahead, slow to 30 | 39853 | 40600 | 747 | 28 | 32 | 37 |

by severity: 0 gentle, 6 moderate, 4 firm, 4 sharp, 1 hairpin

announced early enough to slow at 1.0 m/s^2: **14 / 15**
false slowdowns (advisory pointing at a curve that is not in the route geometry): **0 of 1649 fixes** []
road match: progress error median 3.3 m, p95 10.3 m, max 263.0 m; high-confidence fixes 1469, of which progress wrong by > 40 m (wrong forward geometry): **0**; recalculations during replay: 0
fixes with advisory active: 76; fixes without high road confidence (no advised speed published): 180
