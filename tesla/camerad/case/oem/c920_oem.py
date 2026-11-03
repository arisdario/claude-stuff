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
P = dict(ws=33.0, ha=28.0, va=18.0)
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
# ------------------------------------------------ housing
R = 9.0
S = (ZG0 + a0z + HT) / (t - tV) + 4         # hood length: the glass plane closes the hood
shell = hull(slab(0, 0.1, OW, PZC - OH, zg(0), R), slab(YF0 - 0.1, YF0, OW, PZC - OH, zg(YF0), R))
shell = shell.copy(); shell.apply_translation([PXC, 0, 0])
hood_out = frustum(YF0 - 0.2, a0x + HT, a0z + HT, YF0 + S, a0x + HT + S * tH, a0z + HT + S * tV)
below = trimesh.convex.convex_hull(np.array([(x, y, z) for x in (-300, 300) for (y, z) in ((-5, -300), (YF0 + S + 30, -300), (-5, zg(-5)), (YF0 + S + 30, zg(YF0 + S + 30)))]))
pocket = box(PXC - PW / 2, PXC + PW / 2, -1, PD, PZC - PH / 2, PZC + PH / 2)
winc = box(-WIN_W / 2 - 0.4, WIN_W / 2 + 0.4, PD - 0.5, YF0 + 1, -WIN_H / 2 - 0.4, WIN_H / 2 + 0.4)
hood_in = frustum(YF0 - 0.01, a0x, a0z, YF0 + S + 2, a0x + (S + 2) * tH, a0z + (S + 2) * tV)
YP, ZP, ZA = 8.0, OH - 7, -(OH - 7)       # pitch pivot and adjuster bosses on both sides (Z measured from the pocket centre)
bosses = []
for sx in (-1, 1):
    for zz in (ZP, ZA):
        bosses.append(cyl_x(6.5, 0, 6.0, YP, PZC + zz).copy())
        bosses[-1].apply_translation([PXC + sx * (OW - 0.2 + (3.0 if sx > 0 else -3.0)) , 0, 0]) if False else None
body_h = union(shell, hood_out)
for sx in (-1, 1):
    for zz in (ZP, ZA):
        body_h = union(body_h, cyl_x(6.5, PXC + sx * (OW - 1), PXC + sx * (OW + 4.0), YP, PZC + zz))
body_h = inter(body_h, below)
sc = [(sx * (PW / 2 + 2.5), PZC + sz * (PH / 2 + 2.5)) for sx in (-1, 1) for sz in (-1, 1)]    # lid screws in the wall corners
holes = [cyl_y(1.2, -1, 11, PXC + x, z) for x, z in sc]
holes += [cyl_x(2.1, PXC + sx * (OW - 6), PXC + sx * (OW + 5), YP, PZC + zz) for sx in (-1, 1) for zz in (ZP, ZA)]
vents = [box(PXC + x - 1.0, PXC + x + 1.0, 4, PD - 3, PZC - OH - 1, PZC - PH / 2 + 0.3) for x in np.arange(-34, 35, 6.0)]   # slots in the underside wall: dust cannot fall in
housing = diff(body_h, pocket, winc, hood_in, *holes, *vents)
housing = inter(housing, below)

# ------------------------------------------------ rear lid (closes the pocket; cable hole where the real cartridge has it)
lid = union(trimesh.creation.box(extents=[2 * OW - 2, 3, 2 * OH - 2], transform=trimesh.transformations.translation_matrix([PXC, -1.5, PZC])),
            box(PXC - PW / 2 + 1.0, PXC + PW / 2 - 1.0, 0, 1.6, PZC - PH / 2 + 1.0, PZC + PH / 2 - 1.0))
lid = diff(lid, *[cyl_y(1.7, -4, 3, PXC + x, z) for x, z in sc], box(PXC + CAB_DX - 4.5, PXC + CAB_DX + 4.5, -4, 3, PZC - OH - 1, PZC - CAB_DZ + 4.5))   # slot open to the bottom edge so the soldered cable can be dropped in

# ------------------------------------------------ bracket: glass pad + two cheeks with pitch arc slots
PADT = GAP - 2.0     # 2 mm of air between pad and housing so the housing can pitch
pad_y0, pad_y1 = -6.0, YF0 - 4.0
CHW = 4.0
pad_pts = [(x, y, z) for x in (PXC - OW - 4 - CHW, PXC + OW + 4 + CHW) for (y, z) in ((pad_y0, zg(pad_y0) + 2), (pad_y1, zg(pad_y1) + 2), (pad_y0, zg(pad_y0) + GAP), (pad_y1, zg(pad_y1) + GAP))]
pad = trimesh.convex.convex_hull(np.array(pad_pts))
def cheek(side):
    # 2-D (Y,Z): arm from the pad down to the pivot, pivot hole, arc slot about the pivot for the adjuster boss
    from shapely.geometry import Polygon
    poly = Polygon([(pad_y0, PZC + ZA - 9), (pad_y1, PZC + ZA - 9), (pad_y1, zg(pad_y1) + GAP), (pad_y0, zg(pad_y0) + GAP)])
    arc = LineString([(YP + (PZC + ZP - (PZC + ZA)) * math.sin(math.radians(a)), PZC + ZP - (PZC + ZP - (PZC + ZA)) * math.cos(math.radians(a))) for a in np.linspace(-9, 9, 19)]).buffer(2.3)
    poly = poly.difference(arc).difference(Point(YP, PZC + ZP).buffer(2.3))
    m = trimesh.creation.extrude_polygon(poly, CHW)
    m.vertices = m.vertices[:, [2, 0, 1]]; m.faces = m.faces[:, ::-1]; m.fix_normals()     # (y,z,e)->(x=e,y,z)
    x0 = PXC + (OW + 4.2 if side > 0 else -OW - 4.2 - CHW)
    m.apply_translation([x0, 0, 0]); return m
cheeks = [cheek(1), cheek(-1)]

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
cone = frustum(PD, 0.1, 0.1, YF0 + S, (S + FW) * tH + 0.1, (S + FW) * tV + 0.1)
print('pad / housing interference mm3: %.2f' % inter(housing, pad).volume)
print('view-cone clearance (should be ~0 mm3):', round(inter(housing, cone).volume, 2))

# ------------------------------------------------ export in print orientation (back down; pad flat is not needed)
R90 = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
for name, m in (('housing', housing), ('rear_lid', lid), ('bracket_pad', pad), ('cheek_R', cheeks[0]), ('cheek_L', cheeks[1])):
    m = m.copy(); m.apply_transform(R90); m.apply_translation([0, 0, -m.bounds[0][2]]); m.apply_translation([-m.centroid[0], 0, 0])
    m.export('oem_%s.stl' % name)
    print('%-12s watertight %s  size %s mm  volume %.1f cm3' % (name, m.is_watertight, np.round(m.extents, 1), m.volume / 1000))

# ------------------------------------------------ containment proof: the real cartridge lies entirely inside the pocket
for nm, c in zip(('cartridge body', 'cartridge cover'), cart):
    inside = inter(c, pocket).volume
    print('%s: volume %.0f mm3, inside the pocket %.0f mm3 (%s)' % (nm, c.volume, inside, 'OK' if abs(inside - c.volume) < 1 else 'NOT INSIDE'))
wc = np.array([(win.min(0)[0] + win.max(0)[0]) / 2 - cx0 + PXC, -((win.min(0)[1] + win.max(0)[1]) / 2 - cy0) + PZC])
print('cover lens window centre in the car frame: X %.2f Z %.2f (the lens axis is X 0, Z 0)' % tuple(wc))
