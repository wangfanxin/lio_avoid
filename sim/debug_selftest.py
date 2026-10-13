#!/usr/bin/env python3
"""Pin down which side of the self-test is wrong: print, for the first mismatching rays, what each
implementation thinks it hit.  A closed box means every ray must hit something, so a lower hit count
from one side is that side's bug."""
import math
import sys

import numpy as np

sys.path.insert(0, "/home/fanxin/lio_avoid/sim")
from raycast import World, load_pattern, cast, brute_force, rpy_to_R, make_mount_rotation

w = World.load("/home/fanxin/lio_avoid/worlds/pillars_dense.yaml")
print("world: %d pillars, floor %s, ceiling %s, walls %s" % (len(w.pillars), w.floor_z, w.ceiling_z,
                                                            w.walls))
rays, _ = load_pattern(None, 20000)
origin = np.array([1.97, -0.54, 2.15])
R = rpy_to_R(0.1, -0.2, 0.5) @ make_mount_rotation({"pitch": 45.0})

_, _, vec = cast(origin, R, rays, w, return_full=True)
bru = brute_force(origin, R, rays, w, limit=400)
bad = np.flatnonzero(np.isfinite(vec[:400]) != np.isfinite(bru[:400]))
print("mismatching rays in the 400-ray sample: %d" % len(bad))
print("  first few: %s" % bad[:8])

az, el = rays[:, 0], rays[:, 1]
d_body = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
d_all = d_body @ R.T


def trace(i):
    d = d_all[i]
    hits = []
    for name, zp in (("floor", w.floor_z), ("ceiling", w.ceiling_z)):
        if zp is None or abs(d[2]) < 1e-15:
            continue
        t = (zp - origin[2]) / d[2]
        if t > 0:
            hits.append((t, name))
    if w.walls:
        for axis, (lo, hi) in enumerate((w.walls["x"], w.walls["y"])):
            for plane, nm in ((lo, "wall%c_lo" % "xy"[axis]), (hi, "wall%c_hi" % "xy"[axis])):
                if abs(d[axis]) < 1e-15:
                    continue
                t = (plane - origin[axis]) / d[axis]
                if t > 0:
                    h = origin + t * d
                    other = 1 - axis
                    olo, ohi = (w.walls["x"] if other == 0 else w.walls["y"])
                    if olo <= h[other] <= ohi:
                        hits.append((t, nm))
    for k, (cx, cy, r, z0, z1) in enumerate(w.pillars):
        ox, oy = origin[0] - cx, origin[1] - cy
        a = d[0] ** 2 + d[1] ** 2
        if a < 1e-15:
            continue
        b = 2.0 * (ox * d[0] + oy * d[1])
        c = ox * ox + oy * oy - r * r
        disc = b * b - 4.0 * a * c
        if disc <= 0:
            continue
        sq = math.sqrt(disc)
        for t in ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)):
            if t > 0:
                hz = origin[2] + t * d[2]
                if z0 <= hz <= z1:
                    hits.append((t, "pillar%d" % k))
                    break
    hits.sort()
    return hits[:4]


for i in bad[:5]:
    print("  ray %4d az=%7.2f el=%7.2f  sensor-frame; body elevation %+6.2f deg"
          % (i, math.degrees(az[i]), math.degrees(el[i]),
             math.degrees(el[i]) - 45.0))
    print("      vector %.4f   brute %.4f" % (vec[i], bru[i]))
    for t, nm in trace(i):
        print("      candidate t=%.4f  %s" % (t, nm))
