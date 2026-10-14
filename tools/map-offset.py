#!/usr/bin/env python3
"""How far is the MAP from the world file, as a rigid translation in the ground plane?

Replaces a per-pillar circle fit that could not work at this geometry: the pillars stand 2 m apart and a
1.5 m disc around one axis contains its neighbour and the floor, so the fits came back with radii 0.34-1.19 m
against a true 0.30 m and rms up to 0.32 m (MEASURED 2026-10-14).

This instead scores the whole cloud against the whole world: for a grid of candidate translations, sum the
distance from every cloud point near a pillar to the nearest pillar SURFACE.  The translation that minimises
it is the map's offset -- one vector, in the same frame the adapter judges clearances in, which is the
number the Mission B blockage is asking for.

usage: tools/map-offset.py --cloud cloud.csv --world worlds/forest_all.yaml [--calib-file ...]
"""
from __future__ import annotations

import argparse
import json
import sys

import numpy as np
import yaml


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--world", required=True)
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--span", type=float, default=0.80, help="search +- this far, m")
    ap.add_argument("--step", type=float, default=0.02)
    args = ap.parse_args()

    p = []
    with open(args.cloud) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) == 5:
                p.append([float(f[2]), float(f[3]), float(f[4])])
    if len(p) < 100:
        print("only %d points in %s" % (len(p), args.cloud))
        return 1
    P = np.array(p)
    try:
        c = json.load(open(args.calib_file))
        P = (np.array(c["R_WL"]) @ P.T).T + np.array(c["t_WL"])
        print("cloud %s: %d points, L->W with yaw %+.3f deg" % (args.cloud, len(P), c.get("yaw_deg", 0)))
    except Exception as exc:                                   # noqa: BLE001
        print("WARNING: no L->W calibration (%s)" % exc)
    cfg = yaml.safe_load(open(args.world)) or {}
    pil = np.array([[q["x"], q["y"], q.get("r", 0.3)] for q in cfg.get("pillars", [])])
    if not len(pil):
        print("no pillars in %s" % args.world)
        return 1

    # only the points that could be pillar surface: inside the pillars' z band and within 1 m of an axis
    inz = (P[:, 2] > 0.2) & (P[:, 2] < 4.2)
    Q = P[inz]
    d0 = np.min(np.hypot(Q[:, None, 0] - pil[None, :, 0], Q[:, None, 1] - pil[None, :, 1]), axis=1)
    Q = Q[d0 < 1.0]
    print("   %d of %d points are within 1 m of a pillar axis and in its z band" % (len(Q), len(P)))
    if len(Q) < 50:
        print("   too few to score")
        return 1

    def score(dx, dy):
        d = np.min(np.hypot(Q[:, None, 0] + dx - pil[None, :, 0],
                            Q[:, None, 1] + dy - pil[None, :, 1]) - pil[None, :, 2], axis=1)
        return float(np.mean(np.abs(d)))

    grid = np.arange(-args.span, args.span + 1e-9, args.step)
    best = None
    for dx in grid:
        for dy in grid:
            s = score(dx, dy)
            if best is None or s < best[0]:
                best = (s, dx, dy)
    s0 = score(0.0, 0.0)
    print("   mean |distance to nearest pillar surface|: %.4f m at zero offset, %.4f m at the best "
          "offset (%+.2f, %+.2f) m" % (s0, best[0], best[1], best[2]))
    print("   -> the MAP is offset from the world file by (%+.2f, %+.2f) m; applying the inverse makes the "
          "two agree to %.3f m on average" % (best[1], best[2], best[0]))
    # how much would the adapter's clearance check gain?
    clr0 = float(np.min(np.min(np.hypot(Q[:, None, 0] - pil[None, :, 0],
                                       Q[:, None, 1] - pil[None, :, 1]) - pil[None, :, 2], axis=1)))
    print("   closest cloud point to any pillar surface at zero offset: %.3f m (negative = inside a pillar)"
          % clr0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
