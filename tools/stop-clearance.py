#!/usr/bin/env python3
"""The owner's question, with numbers: can the rectangle's stops sit inside a pillar?
Plus: what did --min-traj-m 0.50 do to the accepted-plan count (AP5 flew 0.10)?

usage: python3 tools/stop-clearance.py [--mission rect|circle]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def load(name):
    spec = importlib.util.spec_from_file_location("m_" + name, os.path.join(ROOT, "tools", name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_b.yaml"))
    ap.add_argument("--mission", default="rect", choices=["rect", "circle"])
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    args = ap.parse_args()

    wc = load("world-circle.py")
    gen = load("world-rect.py") if args.mission == "rect" else wc
    stops = gen.commanded_points(args.world) if args.mission == "rect" else wc.commanded_points(args.world)
    clr = wc.world_clearance(args.world, stops)

    print("THE MISSION'S STOPS vs THE 0.20 m FOOTPRINT CONTRACT  (%s, %s)" % (args.mission, args.world))
    print("  (clearance = distance to the nearest TRUE pillar surface, the adapter's own function)")
    print()
    print("   #   stop (world_w)                 clearance   < 0.20 m?")
    for i, (p, c) in enumerate(zip(stops, clr)):
        print("  %2d   E %+7.4f  N %+7.4f  U %5.3f    %7.4f m   %s"
              % (i, p[0], p[1], p[2], c, "*** INSIDE THE CONTRACT ***" if c < 0.20 else "ok"))
    print()
    print("   minimum over the %d stops : %.4f m" % (len(stops), float(np.min(clr))))
    print("   stops closer than 0.20 m  : %d of %d" % (int((clr < 0.20).sum()), len(clr)))
    print("   stops closer than 0.30 m  : %d of %d" % (int((clr < 0.30).sum()), len(clr)))

    if args.mission == "rect":
        corners = gen.corners(args.world) if hasattr(gen, "corners") else None
        if corners is None:
            corners = [[4.0, -3.0, 2.0], [4.0, 3.0, 2.0], [-4.0, 3.0, 2.0], [-4.0, -3.0, 2.0]]
        cclr = wc.world_clearance(args.world, corners)
        print()
        print("  THE FOUR CORNERS THE OWNER SPECIFIED (x in [-4,+4], y in [-3,+3]), for comparison:")
        for p, c in zip(corners, cclr):
            print("     E %+7.4f  N %+7.4f   clearance %7.4f m" % (p[0], p[1], c))
        print("     minimum %.4f m, %d of %d closer than 0.20 m" % (float(np.min(cclr)),
                                                                    int((cclr < 0.20).sum()), len(cclr)))
        print()
        print("  WHERE THE COMMANDED STOPS DIFFER FROM THOSE CORNERS (the generator's clearance rule):")
        for i, (p, c) in enumerate(zip(stops, cclr)):
            d = min(float(np.linalg.norm(np.array(p[:2]) - np.array(q[:2]))) for q in corners)
            print("     stop %d is %.3f m from the nearest specified corner" % (i, d))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
