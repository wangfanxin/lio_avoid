#!/usr/bin/env python3
"""Does the LIVE map contain the pillars the world file says are there?  Compare a recorded cloud with the
world's cylinders, one pillar at a time.

WHY (MEASURED 2026-10-14): Mission B's adapter withholds every planned path with
"pillar clearance 0.307 m < footprint 0.35 m", while the same scenario planned OFFLINE from a raycast cloud
holds 0.4978 m.  The planner can therefore keep the margin, so the difference has to be the map.  This
prints, per pillar: the number of recorded points on its surface, and the closest such point to its axis --
a pillar with no points is one the planner could route straight through.

usage: tools/map-vs-world.py --cloud cloud.csv --world worlds/forest_all.yaml
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import yaml


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--world", required=True)
    ap.add_argument("--scan", type=int, default=0, help="which recorded scan (0 = all)")
    args = ap.parse_args()

    t, p = [], []
    with open(args.cloud) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) != 5:
                continue
            if args.scan and int(f[0]) != args.scan:
                continue
            t.append(int(f[1]))
            p.append([float(f[2]), float(f[3]), float(f[4])])
    if not p:
        print("no points in %s" % args.cloud)
        return 1
    P = np.array(p)
    cfg = yaml.safe_load(open(args.world)) or {}
    pillars = cfg.get("pillars", []) or []
    print("cloud %s: %d points, %d scans, z %.2f..%.2f m" % (args.cloud, len(P), len(set(t)),
                                                             P[:, 2].min(), P[:, 2].max()))
    miss = []
    for i, q in enumerate(pillars):
        d = np.hypot(P[:, 0] - q["x"], P[:, 1] - q["y"])
        inz = (P[:, 2] >= q.get("z0", 0.0)) & (P[:, 2] <= q.get("z1", 4.2))
        dh = d[inz]
        near = int((np.abs(dh - q["r"]) < 0.10).sum())      # points within 10 cm of the surface
        if near < 20:
            miss.append((i, q["x"], q["y"], near, float(dh.min()) if len(dh) else float("nan")))
        print("   pillar %2d  (%+5.2f,%+5.2f) r=%.2f  points %6d  on-surface %5d  closest to axis %s"
              % (i, q["x"], q["y"], q["r"], len(dh), near,
                 "%.2f m" % dh.min() if len(dh) else "n/a"))
    print()
    if miss:
        print("PILLARS THE LIVE MAP BARELY HAS (%d of %d):" % (len(miss), len(pillars)))
        for i, x, y, near, dmin in miss:
            print("   pillar %2d at (%+5.2f,%+5.2f): %d on-surface points, closest to axis %.2f m"
                  % (i, x, y, near, dmin))
    else:
        print("every pillar has surface points in this cloud")
    return 0


if __name__ == "__main__":
    sys.exit(main())
