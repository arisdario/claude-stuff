#!/usr/bin/env python3
"""Windshield case for a Logitech C920 (body without its clip/stand), LDWS-camera style.

  python3 c920_case.py          -> c920_case_body.stl, c920_case_lid.stl  (already in PRINT orientation)

Needs: pip install trimesh manifold3d numpy.   ALL sizes are in mm; edit the numbers below, rebuild, print.

Shape: a box that holds the camera, a short LENS HOOD in front (stops dashboard reflections in the glass - it is the
most important part, keep the inside matte black) and a slanted wedge on top whose flat face sits on the windshield
(WS_ANGLE = angle of the glass from horizontal). The camera looks horizontally. The back is closed by a lid with two
screws; the camera's cable leaves through a slot at the bottom of the lid.
Axes while designing: x = width, y = forward (0 = back of the case), z = up, lens centre at z = 0.
"""
import math, numpy as np, trimesh

import sys
BARE = len(sys.argv) > 1 and sys.argv[1] == 'bare'   # python3 c920_case.py bare  -> case for the bare board (82 x 22 x 17 mm)
SEALED = 'sealed' in sys.argv      # hood runs all the way to the windshield and is closed by the glass (lens fully enclosed)
SUFFIX = ('_bare' if BARE else '') + ('_sealed' if SEALED else '')
HA, VA = 33.0, 22.0
for a in sys.argv:
    if a.startswith('ha='): HA = float(a[3:])
    if a.startswith('va='): VA = float(a[3:])
SUFFIX += '' if (HA, VA) == (33.0, 22.0) else '_%d_%d' % (HA, VA)
#               # SEALED: half view angles the hood must clear (the model uses +-32.7 x +-21.5 deg)
# ---- camera (MEASURE yours with calipers and edit) ----
CAM_W, CAM_H, CAM_D = (82.0, 22.0, 17.0) if BARE else (94.0, 29.0, 24.0)   # width, height, depth (bare: depth includes the lens)
LENS_DX, LENS_DZ = 0.0, 0.0                # lens offset from the body centre (ASSUMED centred - measure it)
LENS_HOLE = 18.0 if BARE else 28.0         # hole in the front wall around the lens barrel/ring
# ---- case ----
CLR = 0.6                                  # clearance per side around the camera
WALL = 2.6
LIP = 3.0                                  # lid plug depth
FOAM = 1.2                                 # room for a thin foam pad behind the camera
HOOD_L, HOOD_W, HOOD_H = (15.0, 46.0, 30.0) if BARE else (18.0, 58.0, 36.0)  # hood length, inner width, inner height at the lens (clear of the 70x43 deg view)
WS_ANGLE = 33.0                            # windshield surface angle from horizontal (Model S is very raked: measure yours)
EAR_W, EAR_T, SCREW_D = 10.0, 12.0, 2.6    # screw ears (M3 x 12 self-tapping), hole in the body; 3.4 in the lid
CABLE_X, CABLE_W, CABLE_H = 0.0, 8.0, 9.0  # cable slot in the lid (bottom)

cW, cH = CAM_W + 2 * CLR, CAM_H + 2 * CLR  # cavity
CD = CAM_D + FOAM + LIP                    # cavity depth (back .. inner front wall)
oW, oH = cW + 2 * WALL, cH + 2 * WALL
YF0 = CD + WALL                            # front face of the box
YF = YF0 + HOOD_L                          # end of the hood
HT = 2.4                                   # hood wall
zg_front = HOOD_H / 2 + HT                 # glass height at the hood end
if SEALED:
    YF = YF0; zg_front = oH / 2 + 0.8       # the glass plane passes just above the box front; the hood is cut by it
t = math.tan(math.radians(WS_ANGLE))
zg = lambda y: zg_front + t * (YF - y)     # glass plane: higher towards the driver


def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=trimesh.transformations.translation_matrix([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))


def cyl_y(r, y0, y1, x=0.0, z=0.0):   # cylinder along y
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=48)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1) / 2, z]); return c


def wedge(x0, x1, y0, y1, zb):        # solid from height zb up to the glass plane
    pts = [(x, y, z) for x in (x0, x1) for (y, z) in ((y0, zb), (y1, zb), (y0, zg(y0)), (y1, zg(y1)))]
    return trimesh.convex.convex_hull(np.array(pts))


def union(*m): return trimesh.boolean.union(list(m), engine='manifold')
def diff(a, *m): return trimesh.boolean.difference([a] + list(m), engine='manifold')


# ---------------- body ----------------
SX, SZ = oW / 2 - 8, (oH / 2 + zg(0)) / 2     # lid screws: into the solid wedge above the camera, no ears
def frustum(y0, hw0, hh0, y1, hw1, hh1):
    pts = [(sx * (hw0 if a else hw1), y, sz * (hh0 if a else hh1)) for a, y in ((1, y0), (0, y1)) for sx in (-1, 1) for sz in (-1, 1)]
    return trimesh.convex.convex_hull(np.array(pts))
if not SEALED:
    body = union(
        box(-oW / 2, oW / 2, 0, YF0, -oH / 2, oH / 2),
        box(-HOOD_W / 2 - HT, HOOD_W / 2 + HT, YF0 - 0.1, YF, -HOOD_H / 2 - HT, HOOD_H / 2 + HT),
        wedge(-oW / 2, oW / 2, 0, YF0, oH / 2 - 1),
        wedge(-HOOD_W / 2 - HT, HOOD_W / 2 + HT, YF0 - 0.1, YF, HOOD_H / 2 + HT - 1))
    hood_in = box(-HOOD_W / 2, HOOD_W / 2, YF0 - 0.01, YF + 1, -HOOD_H / 2, HOOD_H / 2)
else:
    a0 = LENS_HOLE / 2 + 0.5
    tH, tV = math.tan(math.radians(HA)), math.tan(math.radians(VA))
    S = (zg_front + a0 + HT) / (t - tV) + 5         # length at which the glass meets the lower edge of the hood
    body = union(
        box(-oW / 2, oW / 2, 0, YF0, -oH / 2, oH / 2),
        wedge(-oW / 2, oW / 2, 0, YF0, oH / 2 - 1),
        frustum(YF0 - 0.1, a0 + HT, a0 + HT, YF0 + S, a0 + HT + S * tH, a0 + HT + S * tV))
    hood_in = frustum(YF0 - 0.01, a0, a0, YF0 + S + 1, a0 + (S + 1) * tH, a0 + (S + 1) * tV)
    below = trimesh.convex.convex_hull(np.array([(x, y, z) for x in (-300, 300) for (y, z) in ((-5, -300), (YF0 + S + 20, -300), (-5, zg(-5)), (YF0 + S + 20, zg(YF0 + S + 20)))]))
    body = trimesh.boolean.intersection([body, below], engine='manifold')
body = diff(body,
    box(-cW / 2, cW / 2, -1, CD, -cH / 2, cH / 2),
    cyl_y(LENS_HOLE / 2, CD - 1, YF0 + 1, LENS_DX, LENS_DZ),
    hood_in,
    cyl_y(SCREW_D / 2, -1, EAR_T, SX, SZ), cyl_y(SCREW_D / 2, -1, EAR_T, -SX, SZ))
if SEALED: body = trimesh.boolean.intersection([body, below], engine='manifold')   # the glass plane also opens the hood's inside

# ---------------- lid (plate behind the body + plug into the cavity) ----------------
lid = union(box(-oW / 2, oW / 2, -3, 0, -oH / 2, zg(0) - 0.3),   # whole rear plate, like an OEM rear cover
            box(-cW / 2 + 0.3, cW / 2 - 0.3, -0.01, LIP, -cH / 2 + 0.3, cH / 2 - 0.3))
lid = diff(lid,
    cyl_y(1.7, -4, 1, SX, SZ), cyl_y(1.7, -4, 1, -SX, SZ),
    box(CABLE_X - CABLE_W / 2, CABLE_X + CABLE_W / 2, -4, LIP + 1, -oH / 2 - 1, -oH / 2 + WALL + CABLE_H))

# print orientation: back of the case (y = 0) on the bed
R = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
out = {}
for name, m in (('body', body), ('lid', lid)):
    m = m.copy(); m.apply_transform(R); m.apply_translation(-m.bounds[0] * [0, 0, 0] - [0, 0, m.bounds[0][2]])
    out[name] = m
    m.export('c920_case%s_%s.stl' % (SUFFIX, name))
    print(name, 'watertight', m.is_watertight, 'size mm', np.round(m.extents, 1), 'volume cm3', round(m.volume / 1000, 1))
