# Real-drive validation of the curve announcements

10986 GPS fixes, 11250 s, 27.9 km driven; map-matched on the routing graph (driven route 28.1 km, within 8 m of the trace); 306 distinct GPS positions (37 s between updates on average); a_lat 1.85 m/s^2, decel 1.0 m/s^2. Advisory only.

fixes with HIGH road confidence (position within 15 m of the driven route): 9438 of 10986 (86%)

## Announced curves vs what the driver did

| id | announcement | radius m | curve km/h | announced m before | driver km/h when announced | driver km/h 100 m before | driver km/h at entry | driver min km/h within the bend | slowed >= 15 km/h approaching? | driver at/below advisory target? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Firm right ahead, slow to 40 | 59 | 38 | 780 | 78 | 60 | 73 | 73 | no | yes |
| 2 | Moderate right ahead | 184 | 66 | 776 | 58 | 61 | 62 | 62 | no | yes |
| 3 | Moderate left ahead | 132 | 56 | 768 | 62 | 62 | 62 | 62 | no | yes |
| 4 | Moderate right ahead | 192 | 68 | 667 | 62 | 35 | 51 | 51 | no | yes |
| 5 | Hairpin right ahead, slow to 30 | 24 | 32 | 1 | 49 | - | 49 | 48 | no | NO |
| 6 | Firm left ahead, slow to 35 | 59 | 37 | 121 | 49 | 49 | 46 | 43 | no | yes |
| 7 | Hairpin right ahead, slow to 30 | 22 | 32 | 251 | 49 | 45 | - | 0 | yes | yes |

curves announced: 7; driver slowed >= 15 km/h into them: **1**; driver already at/below the advisory envelope 100 m before the bend: **6**

## Driver slow-downs (>= 25 km/h within 400 m, ending above 15 km/h) and whether a curve was announced near their end (+-150 m)

7 slow-downs, **1** with an announced curve nearby; the others are traffic, junctions or ramps - listed for inspection:

| at route km | from km/h | to km/h | announced curve nearby |
|---|---|---|---|
| 7.26 | 89 | 63 | no |
| 10.22 | 77 | 50 | no |
| 10.61 | 74 | 47 | no |
| 10.69 | 48 | 19 | no |
| 12.49 | 62 | 28 | yes |
| 13.75 | 89 | 59 | no |
| 16.52 | 89 | 59 | no |