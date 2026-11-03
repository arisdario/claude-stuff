# Real-drive validation of the curve announcements

1757 GPS fixes, 1812 s, 32.7 km driven; map-matched on the routing graph (driven route 33.1 km, within 8 m of the trace); 134 distinct GPS positions (14 s between updates on average); a_lat 1.85 m/s^2, decel 1.0 m/s^2. Advisory only.

fixes with HIGH road confidence (position within 15 m of the driven route): 1671 of 1757 (95%)

## Announced curves vs what the driver did

| id | announcement | radius m | curve km/h | announced m before | driver km/h when announced | driver km/h 100 m before | driver km/h at entry | driver min km/h within the bend | slowed >= 15 km/h approaching? | driver at/below advisory target? |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | Hairpin left ahead, slow to 30 | 22 | 32 | 791 | 67 | 45 | - | 28 | yes | yes |
| 2 | Firm right ahead, slow to 40 | 72 | 41 | 795 | 84 | - | - | - | no | NO |

curves announced: 2; driver slowed >= 15 km/h into them: **1**; driver already at/below the advisory envelope 100 m before the bend: **1**

## Driver slow-downs (>= 25 km/h within 400 m, ending above 15 km/h) and whether a curve was announced near their end (+-150 m)

5 slow-downs, **1** with an announced curve nearby; the others are traffic, junctions or ramps - listed for inspection:

| at route km | from km/h | to km/h | announced curve nearby |
|---|---|---|---|
| 0.41 | 53 | 20 | no |
| 1.26 | 57 | 30 | no |
| 2.60 | 64 | 28 | yes |
| 7.51 | 103 | 76 | no |
| 26.90 | 112 | 85 | no |