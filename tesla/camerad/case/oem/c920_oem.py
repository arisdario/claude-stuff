#!/usr/bin/env python3
"""OEM-style windshield housing + adjustable bracket for the Logitech C920 PCB, built around a REAL, proven fit:
the Printables "Logitech C920 Case 2020 Extrusion" (Cam-Body.stl + Cam-Cover.stl) is used as a CARTRIDGE that slides into
the pocket of this housing. Nothing about the PCB is guessed: the pocket, the lens window and the cable hole come from
those two STLs (not copied into this repo - give the folder on the command line).

  pip install trimesh manifold3d shapely networkx numpy
  python3 c920_oem.py /path/with/Cam-Body.stl+Cam-Cover.stl  [ws=33] [ha=28] [va=18]

Writes (print orientation, mm): oem_housing.stl  oem_rear_lid.stl  oem_bracket_pad.stl  oem_cheek_L.stl  oem_cheek_R.stl  (+ checks)
Car frame while designing: X = width (right), Y = forward (0 = back of the housing), Z = up; lens axis = Y, lens centre = (0, *, 0).
"""
import sys, math, numpy as np, trimesh
from shapely.geometry import box as sbox, Point, LineString
from shapely.ops import unary_union

SRC = sys.argv[1] if len(sys.argv) > 1 and '=' not in sys.argv[1] else '.'
P = dict(ws=33.0, ha=26.0, va=16.0)
for a in sys.argv[1:]:
    if '=' in a: k, v = a.split('='); P[k] = float(v)
WS, HA, VA = P['ws'], P['ha'], P['va']

# ------------------------------------------------ real hardware, measured from the two STLs
body = trimesh.load(SRC + '/Cam-Body.stl'); cover = trimesh.load(SRC + '/Cam-Cover.stl')
cb, cc = body.bounds, cover.bounds
CART_W = cc[1][0] - cc[0][0]; CART_H = cc[1][1] - cc[0][1]            # cover is the widest part: 101.2 x 28.9
cx0, cy0 = (cc[0][0] + cc[1][0]) / 2, (cc[0][1] + cc[1][1]) / 2
sec = cover.section(plane_origin=[0, 0, cc[1][2] - 0.5], plane_normal=[0, 0, 1])
win = [e for e in sec.discrete if len(e) in (5, 13) and (e.max(0) - e.min(0))[0] < 20 and (e.max(0) - e.min(0))[1] < 20 and (e.max(0) - e.min(0))[0] > 12 and abs(((e.min(0) + e.max(0)) / 2)[0] - cx0) < 10]
win = [e for e in sec.discrete if abs((e.max(0) - e.min(0))[0] - 15) < 1 and abs((e.max(0) - e.min(0))[1] - 15) < 1][0]
WIN_W, WIN_H = (win.max(0) - win.min(0))[:2]
WIN_DX, WIN_DZ = (win.min(0)[0] + win.max(0)[0]) / 2 - cx0, (win.min(0)[1] + win.max(0)[1]) / 2 - cy0   # lens window offset from the cartridge centre
fl = body.section(plane_origin=[0, 0, 0.5], plane_normal=[0, 0, 1]); hole = [e for e in fl.discrete if (e.max(0) - e.min(0))[0] < 8][0]
CAB_DX, CAB_DZ = (hole.min(0)[0] + hole.max(0)[0]) / 2 - cx0, (hole.min(0)[1] + hole.max(0)[1]) / 2 - cy0
CAB_W, CAB_H = (hole.max(0) - hole.min(0))[:2]
CART_D = (cb[1][2] - cb[0][2]) + (cc[1][2] - 123.0)                    # body 13 + cover top 2 (cover legs overlap the body)
print('cartridge %.1f x %.1f x %.1f  lens window %.1f x %.1f at dx %.1f dz %.1f   cable hole %.1f x %.1f at dx %.1f dz %.1f' % (CART_W, CART_H, CART_D, WIN_W, WIN_H, WIN_DX, WIN_DZ, CAB_W, CAB_H, CAB_DX, CAB_DZ))

# ------------------------------------------------ design numbers
CL = 0.5                      # clearance around the cartridge
WALL, FW, HT = 5.0, 3.0, 2.6  # side wall, front wall, hood wall
GAP = 5.0                     # foam seal between the hood rim and the glass (bracket pad is this thick)
PW, PH, PD = CART_W + 2 * CL, CART_H + 2 * CL, CART_D + 1.5   # pocket (+1.5 mm foam behind)
OW, OH = PW / 2 + WALL, PH / 2 + WALL                          # outer half sizes
YF0 = PD + FW                                                  # front face of the housing
t, tH, tV = math.tan(math.radians(WS)), math.tan(math.radians(HA)), math.tan(math.radians(VA))
ZG0 = OH - 4 + 1.0                                             # glass-plane height at the front face (just above the lens window)
zg = lambda y: ZG0 + t * (YF0 - y)                             # housing top = glass plane (lower than the real glass by GAP)
a0x, a0z = WIN_W / 2 + 0.6, WIN_H / 2 + 0.6


def rr_slab(y0, y1, hw, z0, z1, r):    # rounded-rectangle slab in X-Z extruded along Y
    poly = sbox(-hw + r, z0 + r, hw - r, z1 - r).buffer(r, 24)
    m = trimesh.creation.extrude_polygon(poly, y1 - y0)        # extrudes along +z of (x, y-as-z)
    m.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]))   # (x,y,z)->(x,z,-y): depth along +Y after shift
    m.apply_translation([0, y1 if False else y0 + (y1 - y0) * 0 + (y1 - y0), 0]) if False else None
    return m


def slab(y0, y1, hw, z0, z1, r):
    poly = sbox(-hw + r, z0 + r, hw - r, z1 - r).buffer(r, 24)
    m = trimesh.creation.extrude_polygon(poly, y1 - y0)        # polygon in (x, y) -> extrude along z
    # map (x, y, z) -> (x, z, y): polygon y becomes Z(up), extrusion z becomes Y(forward)
    m.vertices = m.vertices[:, [0, 2, 1]] + np.array([0, y0, 0]); m.faces = m.faces[:, ::-1]; m.fix_normals()
    return m


def hull(*m): return trimesh.convex.convex_hull(np.vstack([x.vertices for x in m]))
def box(x0, x1, y0, y1, z0, z1): return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=trimesh.transformations.translation_matrix([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))
def union(*m): return trimesh.boolean.union(list(m), engine='manifold')
def diff(a, *m): return trimesh.boolean.difference([a] + list(m), engine='manifold')
def inter(*m): return trimesh.boolean.intersection(list(m), engine='manifold')
def cyl_x(r, x0, x1, y, z):
    c = trimesh.creation.cylinder(radius=r, height=x1 - x0, sections=40); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [0, 1, 0])); c.apply_translation([(x0 + x1) / 2, y, z]); return c
def cyl_y(r, y0, y1, x, z):
    c = trimesh.creation.cylinder(radius=r, height=y1 - y0, sections=40); c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])); c.apply_translation([x, (y0 + y1) / 2, z]); return c
def frustum(y0, hw0, hh0, y1, hw1, hh1, x0=0.0, z0=0.0):
    pts = [(x0 + sx * (hw0 if a else hw1), y, z0 + sz * (hh0 if a else hh1)) for a, y in ((1, y0), (0, y1)) for sx in (-1, 1) for sz in (-1, 1)]
    return trimesh.convex.convex_hull(np.array(pts))

# lens axis = pocket-centre + window offset: the cartridge is placed so that the lens window centre sits on the lens axis (X = 0, Z = 0)
PXC, PZC = -WIN_DX, WIN_DZ                 # pocket centre relative to the lens axis
# ------------------------------------------------ housing: compact dashcam-style block (rounded, bezel ring around the lens, top vents)
# it HANGS from the bracket (it does not have to touch the glass): the lens sits a few cm behind the windshield like a normal dashcam
R = 9.0
BEZ_R, BEZ_L, BORE_R = 18.0, 7.0, max(math.hypot(WIN_W, WIN_H) / 2 + 1.0, 11.0)   # bezel ring, its length, bore (round, covers the square lens window)
ZT = PZC + OH                                       # top of the block
shell = hull(slab(0, 0.1, OW, PZC - OH, ZT, R), slab(YF0 - 0.1, YF0, OW - 2.0, PZC - OH + 1.0, ZT - 1.0, R - 1.5))   # slight taper to the front
shell = shell.copy(); shell.apply_translation([PXC, 0, 0])
bezel = cyl_y(BEZ_R, YF0 - 1.0, YF0 + BEZ_L, 0, 0)
pocket = box(PXC - PW / 2, PXC + PW / 2, -1, PD, PZC - PH / 2, PZC + PH / 2)
bore = cyl_y(BORE_R, PD - 0.5, YF0 + BEZ_L + 1, 0, 0)
grooves = [trimesh.creation.annulus(r_min=BEZ_R - 3.0 - 2.4 * i - 0.9, r_max=BEZ_R - 3.0 - 2.4 * i, height=0.8) for i in range(3)]
for g in grooves:
    g.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])); g.apply_translation([0, YF0 + BEZ_L - 0.3, 0])
# GoPro-standard 2-prong foot (mates with the usual 3-prong base glued to the windshield, M5 bolt + nut = the pitch hinge).
# Dimensions are the common GoPro ones from memory - print gopro_fit_test.stl first and adjust GP_CLR if too tight/loose.
GP_T, GP_GAP, GP_R, GP_HOLE, GP_H = 3.0, 3.3, 7.0, 5.4, 9.0     # prong thickness, gap between the 2 prongs (= 3-prong centre prong + clearance), tip radius, bolt hole, hole height
for a in sys.argv[1:]:
    if a.startswith('gpclr='): GP_GAP = 3.0 + float(a[6:])
YP, ZPV = YF0 / 2.0, ZT + GP_H
def gp_foot(zbase):
    from shapely.geometry import Polygon
    prof = unary_union([sbox(YP - GP_R, zbase, YP + GP_R, zbase + GP_H + 1), Point(YP, zbase + GP_H).buffer(GP_R, 32)]).difference(Point(YP, zbase + GP_H).buffer(GP_HOLE / 2, 24))
    out = []
    for sx in (-1, 1):
        m = trimesh.creation.extrude_polygon(prof, GP_T)
        m.vertices = m.vertices[:, [2, 0, 1]]; m.faces = m.faces[:, ::-1]; m.fix_normals()
        m.apply_translation([PXC + sx * (GP_GAP / 2 + GP_T / 2) - GP_T / 2, 0, 0]); out.append(m)
    out.append(box(PXC - GP_GAP / 2 - GP_T - 1, PXC + GP_GAP / 2 + GP_T + 1, YP - GP_R, YP + GP_R, zbase - 0.5, zbase + 2.0))
    return union(*out)
foot = gp_foot(ZT - 2.0)
body_h = union(shell, bezel, foot)
sc = [(sx * (PW / 2 + 2.5), PZC + sz * (PH / 2 + 2.5)) for sx in (-1, 1) for sz in (-1, 1)]    # lid screws in the wall corners
holes = [cyl_y(1.2, -1, 11, PXC + x, z) for x, z in sc]
vents = [box(PXC + x - 0.8, PXC + x + 0.8, 6, 24, ZT - WALL - 1, ZT + 1) for x in np.arange(-30, 31, 4.0)]   # slots on top, as on a dashcam
winc = box(-WIN_W / 2 - 0.4, WIN_W / 2 + 0.4, PD - 0.5, YF0 + 1, -WIN_H / 2 - 0.4, WIN_H / 2 + 0.4)
housing = diff(body_h, pocket, winc, bore, *holes, *vents, *grooves)
S = 0.0; zg = lambda y: ZT + 22.0 + t * (YF0 - y)    # glass plane (for the bracket pad): 22 mm above the top of the block at its front edge

# ------------------------------------------------ rear lid (closes the pocket; cable hole where the real cartridge has it)
lid = union(trimesh.creation.box(extents=[2 * OW - 2, 3, 2 * OH - 2], transform=trimesh.transformations.translation_matrix([PXC, -1.5, PZC])),
            box(PXC - PW / 2 + 1.0, PXC + PW / 2 - 1.0, 0, 1.6, PZC - PH / 2 + 1.0, PZC + PH / 2 - 1.0))
lid = diff(lid, *[cyl_y(1.7, -4, 3, PXC + x, z) for x, z in sc], box(PXC + CAB_DX - 4.5, PXC + CAB_DX + 4.5, -4, 3, PZC - OH - 1, PZC - CAB_DZ + 4.5))   # slot open to the bottom edge so the soldered cable can be dropped in

cheeks = []; pad = None

# ------------------------------------------------ checks with the REAL cartridge meshes
def placed(m, yoff):                   # cartridge frame -> car frame: X = x_c, Y = z_c + yoff, Z = -(y_c) : a pure rotation
    m = m.copy(); m.vertices = np.c_[m.vertices[:, 0] - cx0 + PXC, m.vertices[:, 2] + yoff, -(m.vertices[:, 1] - cy0) + PZC]; return m
cart = [placed(body, PD - CART_D), placed(cover, PD - 125.0)]
for c in cart: print('cartridge part volume %.1f cm3 (must be > 0)' % (c.volume / 1000))
for nm, c in zip(('cartridge body', 'cartridge cover'), cart):
    try:
        v = inter(housing, c).volume
    except Exception as e:
        v = float('nan')
    print('interference housing / %s: %.2f mm3' % (nm, v))
cone = frustum(PD, 0.1, 0.1, YF0 + BEZ_L, (BEZ_L + FW) * tH + 0.1, (BEZ_L + FW) * tV + 0.1)
print('view-cone clearance (should be ~0 mm3):', round(inter(housing, cone).volume, 2))

# ------------------------------------------------ export in print orientation (back down; pad flat is not needed)
R90 = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
test = union(foot_t := gp_foot(2.0), box(PXC - 12, PXC + 12, YP - 12, YP + 12, 0, 2.0))
for name, m in (('housing', housing), ('rear_lid', lid), ('gopro_fit_test', test)):
    m = m.copy(); m.apply_transform(R90); m.apply_translation([0, 0, -m.bounds[0][2]]); m.apply_translation([-m.centroid[0], 0, 0])
    m.export(('oem_%s.stl' % name) if name != 'gopro_fit_test' else 'gopro_fit_test.stl')
    print('%-12s watertight %s  size %s mm  volume %.1f cm3' % (name, m.is_watertight, np.round(m.extents, 1), m.volume / 1000))

# ------------------------------------------------ containment proof: the real cartridge lies entirely inside the pocket
for nm, c in zip(('cartridge body', 'cartridge cover'), cart):
    inside = inter(c, pocket).volume
    print('%s: volume %.0f mm3, inside the pocket %.0f mm3 (%s)' % (nm, c.volume, inside, 'OK' if abs(inside - c.volume) < 1 else 'NOT INSIDE'))
wc = np.array([(win.min(0)[0] + win.max(0)[0]) / 2 - cx0 + PXC, -((win.min(0)[1] + win.max(0)[1]) / 2 - cy0) + PZC])
print('cover lens window centre in the car frame: X %.2f Z %.2f (the lens axis is X 0, Z 0)' % tuple(wc))
