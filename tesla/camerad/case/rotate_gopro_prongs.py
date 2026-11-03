#!/usr/bin/env python3
"""Turn the GoPro prongs ("ears") of a mount STL by 90 degrees about their own vertical axis, leaving the rest untouched.
Made for C920_GoPro_Mount.stl (prongs rise along +Y from a flat top at Y ~ 23.4). Usage:
  python3 rotate_gopro_prongs.py in.stl out.stl [split_y=23.4] [angle=90]
"""
import sys, math, numpy as np, trimesh
src, dst = sys.argv[1], sys.argv[2]
kv = dict(a.split('=') for a in sys.argv[3:])
split, ang = float(kv.get('split_y', 23.4)), float(kv.get('angle', 90))
m = trimesh.load(src)
big = 1000
def slab(y0, y1): return trimesh.creation.box(extents=[big, y1 - y0, big], transform=trimesh.transformations.translation_matrix([0, (y0 + y1) / 2, 0]))
base = trimesh.boolean.intersection([m, slab(-big, split)], engine='manifold')
prongs = trimesh.boolean.intersection([m, slab(split, big)], engine='manifold')
c = prongs.bounds.mean(0)                                   # rotate about the prongs' own centre line (parallel to Y)
R = trimesh.transformations.rotation_matrix(math.radians(ang), [0, 1, 0], point=[c[0], 0, c[2]])
prongs.apply_transform(R)
prongs.apply_translation([0, -0.4, 0])                      # sink 0.4 mm into the base so the two fuse
out = trimesh.boolean.union([base, prongs], engine='manifold')
print('bodies', len(out.split(only_watertight=False)), 'watertight', out.is_watertight, 'extents', np.round(out.extents, 1), 'prong footprint x %.1f..%.1f z %.1f..%.1f' % (prongs.bounds[0][0], prongs.bounds[1][0], prongs.bounds[0][2], prongs.bounds[1][2]))
out.export(dst)
