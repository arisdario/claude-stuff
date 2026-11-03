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

# ---- camera (MEASURE yours with calipers and edit) ----
CAM_W, CAM_H, CAM_D = 94.0, 29.0, 24.0     # C920 body without clip: width, height, depth
LENS_DX, LENS_DZ = 0.0, 0.0                # lens offset from the body centre
LENS_HOLE = 28.0                           # hole in the front wall around the lens ring
# ---- case ----
CLR = 0.6                                  # clearance per side around the camera
WALL = 2.6
LIP = 3.0                                  # lid plug depth
FOAM = 1.2                                 # room for a thin foam pad behind the camera
HOOD_L, HOOD_W, HOOD_H = 18.0, 58.0, 36.0  # hood length, inner width, inner height at the lens (clear of the 70x43 deg view)
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
ex = oW / 2 + EAR_W
body = union(
    box(-oW / 2, oW / 2, 0, YF0, -oH / 2, oH / 2),
    box(-HOOD_W / 2 - HT, HOOD_W / 2 + HT, YF0 - 0.1, YF, -HOOD_H / 2 - HT, HOOD_H / 2 + HT),
    wedge(-oW / 2, oW / 2, 0, YF0, oH / 2 - 1),
    wedge(-HOOD_W / 2 - HT, HOOD_W / 2 + HT, YF0 - 0.1, YF, HOOD_H / 2 + HT - 1),
    box(oW / 2 - 1, ex, 0, EAR_T, -6, 6), box(-ex, -oW / 2 + 1, 0, EAR_T, -6, 6))
body = diff(body,
    box(-cW / 2, cW / 2, -1, CD, -cH / 2, cH / 2),
    cyl_y(LENS_HOLE / 2, CD - 1, YF0 + 1, LENS_DX, LENS_DZ),
    box(-HOOD_W / 2, HOOD_W / 2, YF0 - 0.01, YF + 1, -HOOD_H / 2, HOOD_H / 2),
    cyl_y(SCREW_D / 2, -1, EAR_T - 1.5, oW / 2 + EAR_W / 2 + 0.5, 0), cyl_y(SCREW_D / 2, -1, EAR_T - 1.5, -oW / 2 - EAR_W / 2 - 0.5, 0))

# ---------------- lid (plate behind the body + plug into the cavity) ----------------
lid = union(box(-ex, ex, -3, 0, -oH / 2, oH / 2),
            box(-cW / 2 + 0.3, cW / 2 - 0.3, -0.01, LIP, -cH / 2 + 0.3, cH / 2 - 0.3))
lid = diff(lid,
    cyl_y(1.7, -4, 1, oW / 2 + EAR_W / 2 + 0.5, 0), cyl_y(1.7, -4, 1, -oW / 2 - EAR_W / 2 - 0.5, 0),
    box(CABLE_X - CABLE_W / 2, CABLE_X + CABLE_W / 2, -4, LIP + 1, -oH / 2 - 1, -oH / 2 + WALL + CABLE_H))

# print orientation: back of the case (y = 0) on the bed
R = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
out = {}
for name, m in (('body', body), ('lid', lid)):
    m = m.copy(); m.apply_transform(R); m.apply_translation(-m.bounds[0] * [0, 0, 0] - [0, 0, m.bounds[0][2]])
    out[name] = m
    m.export('c920_case_%s.stl' % name)
    print(name, 'watertight', m.is_watertight, 'size mm', np.round(m.extents, 1), 'volume cm3', round(m.volume / 1000, 1))
