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
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json",
                    help="the L->W calibration.  THE CLOUD IS IN L AND THE WORLD FILE IS IN W: comparing"
                         " them without this reported points 0.18 m from the axis of a 0.30 m pillar, i.e."
                         " inside it (MEASURED 2026-10-14), and every conclusion drawn from that would"
                         " have been about the frame, not about the map.")
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
    # L -> W, the one conversion that makes the comparison mean anything
    try:
        import json
        c = json.load(open(args.calib_file))
        R, t = np.array(c["R_WL"]), np.array(c["t_WL"])
        P = (R @ P.T).T + t
        print("cloud transformed L->W with %s (yaw %+.3f deg)" % (args.calib_file, c.get("yaw_deg", 0.0)))
    except Exception as exc:                                   # noqa: BLE001
        print("WARNING: no L->W calibration applied (%s) -- the distances below are in the wrong frame"
              % exc)
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
