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
CL = 1.2                      # clearance around the cartridge (room to swivel it)
FOAM_F, SWING = 5.0, 3.7     # foam pad between the cartridge and the front wall; swivel room behind the cartridge (adjusting screws push forward against the foam)
WALL, FW, HT = 5.0, 3.0, 2.6  # side wall, front wall, hood wall
GAP = 5.0                     # foam seal between the hood rim and the glass (bracket pad is this thick)
PW, PH, PD = CART_W + 2 * CL, CART_H + 2 * CL, CART_D + FOAM_F + 2 * SWING   # pocket
OW, OH = PW / 2 + WALL, PH / 2 + WALL                          # outer half sizes
YF0 = PD + FW                                                  # front face of the housing
t, tH, tV = math.tan(math.radians(WS)), math.tan(math.radians(HA)), math.tan(math.radians(VA))
tHc, tVc = math.tan(math.radians(33.0)), math.tan(math.radians(22.0))     # the model's real view (+-32.7 x +-21.5 deg) with a little margin
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
BELLOWS = 'nobellows' not in sys.argv     # flexible accordion hood (TPU) that presses on the glass: seals the lens from cabin air, follows any glass angle
COL_HW, COL_L = 16.0, 9.0                 # collar the bellows slides over (half size, length)
bezel = slab(YF0 - 1.0, YF0 + COL_L, COL_HW, -COL_HW, COL_HW, 8.0) if BELLOWS else cyl_y(BEZ_R, YF0 - 1.0, YF0 + BEZ_L, 0, 0)
pocket = box(PXC - PW / 2, PXC + PW / 2, -1, PD, PZC - PH / 2, PZC + PH / 2)
bore = cyl_y(BORE_R, PD - 0.5, YF0 + (COL_L if BELLOWS else BEZ_L) + 1, 0, 0)
grooves = [] if BELLOWS else [trimesh.creation.annulus(r_min=BEZ_R - 3.0 - 2.4 * i - 0.9, r_max=BEZ_R - 3.0 - 2.4 * i, height=0.8) for i in range(3)]
for g in grooves:
    g.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])); g.apply_translation([0, YF0 + BEZ_L - 0.3, 0])
# GoPro-standard 2-prong foot (mates with the usual 3-prong base glued to the windshield, M5 bolt + nut = the pitch hinge).
# Dimensions measured from a GoPro mount that already prints and fits (C920_GoPro_Mount.stl); gpclr=0.2 adds clearance if needed.
GP_T, GP_GAP, GP_R, GP_HOLE, GP_H = 3.0, 3.0, 7.4, 5.2, 7.9     # MEASURED from the user's C920_GoPro_Mount.stl: prong 3.0 thick, 3.0 gap, 14.8 deep (tip radius 7.4), 5.2 bolt hole, hole centre 6.9 above the base
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
foot = gp_foot(ZT - 1.0)
# light shield ("apron"), like the flocked trapezoid of a factory ADAS camera: a flat tray under/in front of the lens that widens away
# from it and slopes DOWN steeper than the lowest view line, so it never blocks the picture. Cover it with black flocking/felt.
APRON_L, APRON_DEG, APRON_SIDE = (0.0 if BELLOWS else 60.0), 28.0, 6.0    # side lip: 6 mm is the most that stays out of the view (9 mm already clips it)
for a in sys.argv[1:]:
    if a.startswith('apron='): APRON_L = float(a[6:])
    if a.startswith('apron_deg='): APRON_DEG = float(a[10:])
    if a.startswith('apron_side='): APRON_SIDE = float(a[11:])
apron = None
if APRON_L > 0:
    from shapely.geometry import Polygon
    w0 = BORE_R + 4.0; w1 = w0 + APRON_L * math.tan(math.radians(40.0)) / math.cos(math.radians(APRON_DEG))
    outer = Polygon([(-w0, -3.0), (w0, -3.0), (w1, APRON_L), (-w1, APRON_L)]); inner = outer.buffer(-2.4)
    plate = trimesh.creation.extrude_polygon(outer, 2.4)
    rim = trimesh.creation.extrude_polygon(outer.difference(inner), APRON_SIDE); rim.apply_translation([0, 0, 2.4])
    ap = union(plate, rim)                                   # local frame: x across, y along the slope, z = thickness/up
    ap.vertices = ap.vertices[:, [0, 1, 2]]
    pivot = np.array([0.0, YF0 + BEZ_L - 3.0, -(BORE_R + 3.0)])
    ap.apply_transform(trimesh.transformations.rotation_matrix(-math.radians(APRON_DEG), [1, 0, 0]))   # tilt down towards the front
    ap.apply_translation(pivot)
    apron = ap
body_h = union(shell, bezel, foot, *( [apron] if apron is not None else [] ))
sc = [(sx * (PW / 2 + 2.5), PZC + sz * (PH / 2 + 2.5)) for sx in (-1, 1) for sz in (-1, 1)]    # lid screws in the wall corners
holes = [cyl_y(1.2, -1, 11, PXC + x, z) for x, z in sc]
vents = [box(PXC + x - 0.8, PXC + x + 0.8, 6, 24, ZT - WALL - 1, ZT + 1) for x in np.arange(-30, 31, 4.0)]   # slots on top, as on a dashcam
winc = box(-WIN_W / 2 - 0.4, WIN_W / 2 + 0.4, PD - 0.5, YF0 + 1, -WIN_H / 2 - 0.4, WIN_H / 2 + 0.4)
housing = diff(body_h, pocket, winc, bore, *holes, *vents, *grooves)
S = 0.0; zg = lambda y: ZT + 22.0 + t * (YF0 - y)    # glass plane (for the bracket pad): 22 mm above the top of the block at its front edge

# ------------------------------------------------ bellows (print in TPU 95A): accordion hood from the collar to the glass
bellows = None
if BELLOWS:
    BL, FOLDS, WALLB, PEAK = 58.0, 6, 1.2, 7.0          # length (relaxed), number of folds, wall, how far the folds stand out
    CUFF = 6.0                                          # how far the cuff slides over the collar
    y_start = YF0 + COL_L - CUFF
    ys = [y_start] + [y_start + CUFF + (BL - CUFF) * k / (2 * FOLDS) for k in range(0, 2 * FOLDS + 1)]
    def prof(y, k):                                     # inner half sizes (x, z) at y; folds alternate valley / peak
        if k == 0: return COL_HW + 0.0, COL_HW + 0.0
        base_x = max((y - PD) * tHc + 6.0, COL_HW + 1.0); base_z = max((y - PD) * tVc + 6.0, COL_HW + 1.0)
        pk = PEAK if (k % 2 == 0) else 0.0
        return base_x + pk, base_z + pk
    rings_in = [prof(y, k if k > 0 else 0) for k, y in enumerate(ys)]
    def rr(y, hx, hz, r, d=0.0):
        hx, hz = hx + d, hz + d; return slab(y, y + 0.05, hx, -hz, hz, max(min(r + d, hx - 0.1, hz - 0.1), 0.5))
    def envelope(d):
        parts = []
        for i in range(len(ys) - 1):
            a, b = rings_in[i], rings_in[i + 1]
            parts.append(hull(rr(ys[i], a[0], a[1], 8.0, d), rr(ys[i + 1], b[0], b[1], 8.0, d)))
        return union(*parts)
    outer = envelope(WALLB)
    lip_y = ys[-1]
    lip = hull(rr(lip_y - 0.05, rings_in[-1][0], rings_in[-1][1], 8.0, WALLB + 6.0), rr(lip_y - 2.5, rings_in[-1][0], rings_in[-1][1], 8.0, WALLB + 6.0))   # flat sealing lip (put 2-3 mm foam tape on it if you like)
    cuff = hull(rr(y_start, COL_HW, COL_HW, 8.0, 2.6), rr(y_start + CUFF, COL_HW, COL_HW, 8.0, 2.6))
    bellows = diff(union(outer, lip, cuff), envelope(0.0), box(-COL_HW, COL_HW, y_start - 1, y_start + CUFF + 0.01, -COL_HW, COL_HW), box(-50, 50, lip_y - 0.01, lip_y + 5, -50, 50))
    # (the inner void above uses the inside profile; the collar socket is cut separately so the cuff hugs the collar)

# ------------------------------------------------ rear lid (closes the pocket; cable hole where the real cartridge has it)
LID_T = 6.0
lid = union(trimesh.creation.box(extents=[2 * OW - 2, LID_T, 2 * OH - 2], transform=trimesh.transformations.translation_matrix([PXC, -LID_T / 2, PZC])),
            box(PXC - PW / 2 + 1.0, PXC + PW / 2 - 1.0, 0, 1.6, PZC - PH / 2 + 1.0, PZC + PH / 2 - 1.0))
JACK = [(sx * 40.0, sz * 9.0) for sx in (-1, 1) for sz in (-1, 1)]       # 4 M3 adjusting screws (self-tapping into the printed lid): push the cartridge forward against the foam
lid = diff(lid, *[cyl_y(1.7, -LID_T - 1, 3, PXC + x, z) for x, z in sc], *[cyl_y(1.25, -LID_T - 1, 3, PXC + x, PZC + z) for x, z in JACK], box(PXC + CAB_DX - 4.5, PXC + CAB_DX + 4.5, -LID_T - 1, 3, PZC - OH - 1, PZC - CAB_DZ + 4.5))   # slot open to the bottom edge so the soldered cable can be dropped in

cheeks = []; pad = None

# ------------------------------------------------ checks with the REAL cartridge meshes
def placed(m, yoff):                   # cartridge frame -> car frame: X = x_c, Y = z_c + yoff, Z = -(y_c) : a pure rotation
    m = m.copy(); m.vertices = np.c_[m.vertices[:, 0] - cx0 + PXC, m.vertices[:, 2] + yoff, -(m.vertices[:, 1] - cy0) + PZC]; return m
cart = [placed(body, PD - FOAM_F - CART_D), placed(cover, PD - FOAM_F - 125.0)]
for c in cart: print('cartridge part volume %.1f cm3 (must be > 0)' % (c.volume / 1000))
for nm, c in zip(('cartridge body', 'cartridge cover'), cart):
    try:
        v = inter(housing, c).volume
    except Exception as e:
        v = float('nan')
    print('interference housing / %s: %.2f mm3' % (nm, v))
coneL = YF0 + BEZ_L + (APRON_L if APRON_L > 0 else 0) + 20
cone = frustum(PD, 0.1, 0.1, coneL, (coneL - PD) * tHc + 0.1, (coneL - PD) * tVc + 0.1)
cone = trimesh.boolean.difference([cone, bore], engine='manifold') if False else cone
print('view-cone clearance (should be ~0 mm3; the bezel ring itself is outside the cone):', round(inter(housing, cone).volume, 2))

# ------------------------------------------------ export in print orientation (back down; pad flat is not needed)
R90 = trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0])
test = union(foot_t := gp_foot(2.0), box(PXC - 12, PXC + 12, YP - 12, YP + 12, 0, 2.0))
if bellows is not None:
    print('bellows / view cone interference (should be 0 mm3): %.1f' % inter(bellows, cone).volume)
for name, m in (('housing', housing), ('rear_lid', lid), ('gopro_fit_test', test)) + ((('bellows_TPU', bellows),) if bellows is not None else ()):
    m = m.copy(); m.apply_transform(R90); m.apply_translation([0, 0, -m.bounds[0][2]]); m.apply_translation([-m.centroid[0], 0, 0])
    m.export(('oem_%s.stl' % name) if name != 'gopro_fit_test' else 'gopro_fit_test.stl')
    print('%-12s watertight %s  size %s mm  volume %.1f cm3' % (name, m.is_watertight, np.round(m.extents, 1), m.volume / 1000))

# ------------------------------------------------ swivel check: the cartridge turned to the limits (yaw +-5, pitch +-5) must not touch the housing or lid
pivot = np.array([PXC, PD - FOAM_F - CART_D / 2, PZC])
hl = union(housing, lid)
worst = 0.0
for yaw in (-5, 0, 5):
    for pitch in (-5, 0, 5):
        Rm = trimesh.transformations.concatenate_matrices(trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1], pivot), trimesh.transformations.rotation_matrix(math.radians(pitch), [1, 0, 0], pivot))
        for c in cart:
            cc2 = c.copy(); cc2.apply_transform(Rm); v_ = inter(hl, cc2).volume; worst = max(worst, v_)
            if v_ > 0.5: print('   touches at yaw %d pitch %d: %.1f mm3' % (yaw, pitch, v_))
print('swivel check (yaw/pitch +-5 deg, 9 poses): worst interference %.2f mm3 (must be 0)' % worst)
hb = union(housing, bellows) if bellows is not None else housing
wv = {}
for yaw in (-5, 0, 5):
    for pitch in (-5, 0, 5):
        Rm = trimesh.transformations.concatenate_matrices(trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1], pivot), trimesh.transformations.rotation_matrix(math.radians(pitch), [1, 0, 0], pivot))
        c2 = cone.copy(); c2.apply_transform(Rm); wv[(yaw, pitch)] = inter(hb, c2).volume
print('view cone (+-33 x 22 deg) vs housing+bellows when swivelled:', {k: round(v, 1) for k, v in wv.items() if v > 0.5} or 'clear in all 9 poses')
# ------------------------------------------------ containment proof: the real cartridge lies entirely inside the pocket
for nm, c in zip(('cartridge body', 'cartridge cover'), cart):
    inside = inter(c, pocket).volume
    print('%s: volume %.0f mm3, inside the pocket %.0f mm3 (%s)' % (nm, c.volume, inside, 'OK' if abs(inside - c.volume) < 1 else 'NOT INSIDE'))
wc = np.array([(win.min(0)[0] + win.max(0)[0]) / 2 - cx0 + PXC, -((win.min(0)[1] + win.max(0)[1]) / 2 - cy0) + PZC])
print('cover lens window centre in the car frame: X %.2f Z %.2f (the lens axis is X 0, Z 0)' % tuple(wc))
