#!/usr/bin/env python3
"""Move the GoPro prongs ("ears") of C920_GoPro_Mount.stl from the rim of the plate onto the big flat face (z = 0),
standing perpendicular to it, pointing away from it (a 90 degree turn about the X axis; the bolt hole axis stays along X).
Everything else is untouched.

  python3 rotate_gopro_prongs.py in.stl out.stl [split_y=23.4] [yc=9.0] [sink=0.4] [fill=1]

split_y = height of the rim where the prongs start, yc = where along Y the prongs stand on the face (centre), sink = overlap.
"""
import sys, math, numpy as np, trimesh
src, dst = sys.argv[1], sys.argv[2]
kv = dict(a.split('=') for a in sys.argv[3:])
split, yc, sink = float(kv.get('split_y', 23.4)), float(kv.get('yc', 9.0)), float(kv.get('sink', 0.4))
m = trimesh.load(src)
big = 1000
def slab(y0, y1): return trimesh.creation.box(extents=[big, y1 - y0, big], transform=trimesh.transformations.translation_matrix([0, (y0 + y1) / 2, 0]))
base = trimesh.boolean.intersection([m, slab(-big, split)], engine='manifold')
prongs = trimesh.boolean.intersection([m, slab(split, big)], engine='manifold')
zc = (prongs.bounds[0][2] + prongs.bounds[1][2]) / 2
v = prongs.vertices.copy()
# (x, y, z) -> (x, yc + (z - zc), sink - (y - split))   a proper rotation (-90 deg about X) plus a move; the prongs end up at z < 0
prongs.vertices = np.c_[v[:, 0], yc + (v[:, 2] - zc), sink - (v[:, 1] - split)]
parts = [base, prongs]
if kv.get('fill', '1') == '1':      # the plate has a hollow slot (x 12..36, y 2..10.5) right behind the face; fill it so the moved prongs stand on solid material
    parts.append(trimesh.creation.box(extents=[24.0, 8.5, 6.0], transform=trimesh.transformations.translation_matrix([24.0, 6.25, 3.0])))
out = trimesh.boolean.union(parts, engine='manifold')
print('bodies', len(out.split(only_watertight=False)), 'watertight', out.is_watertight, 'extents', np.round(out.extents, 1))
import shapely.geometry as sg
sec = base.section(plane_origin=[0, 0, 0.3], plane_normal=[0, 0, 1])
face = sg.Polygon(sec.discrete[0][:, :2]).buffer(0) if len(sec.discrete) == 1 else None
foot = sg.box(prongs.bounds[0][0], prongs.bounds[0][1], prongs.bounds[1][0], prongs.bounds[1][1])
print('prong footprint x %.1f..%.1f y %.1f..%.1f, inside the flat face: %s' % (foot.bounds[0], foot.bounds[2], foot.bounds[1], foot.bounds[3], (face.contains(foot) if face is not None else 'unknown')))
out.export(dst)
