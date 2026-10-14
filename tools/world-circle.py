#!/usr/bin/env python3
"""Mission B's flight path -- the ONE place that turns the world file's circle into points.

The world file stores the circle as five numbers (`mission_b.circle`: centre, radius, altitude, number of
points, start angle).  Everything else -- the confirmation figure, the goal sequencer that feeds
ego-planner, and the metrics that judge the flight -- derives its point list from HERE, so the circle
drawn, the circle commanded and the circle scored cannot drift apart.  This is the same rule as mission
A's corners (tools/world-info.py): geometry is read once, from the world file.

The path a mission-B flight actually flies is

    pad (0, 0, z)  ->  circle point 0  ->  1  ->  ...  ->  n-1  ->  (back to 0)

i.e. a radial entry leg from the take-off point to the first point, then one closed turn.  The entry leg
is PART of the path and its pillar clearance is measured with the rest, because that is where a forest
this dense bites first: the pad sits in a pocket between four cells.

usage:
    tools/world-circle.py --world worlds/forest_b.yaml            # n lines: index, E, N, U
    tools/world-circle.py --world worlds/forest_b.yaml --json      # one object
    tools/world-circle.py --world worlds/forest_b.yaml --path      # pad + points + closing point
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import yaml


def read(world):
    """Return the circle as a dict, or None if this world has no mission-B circle."""
    cfg = yaml.safe_load(open(world)) or {}
    c = ((cfg.get("mission_b") or {}).get("circle")) or None
    if not c:
        return None
    centre = [float(v) for v in c.get("centre", [0.0, 0.0])]
    r = float(c["radius"])
    n = int(c.get("points", 24))
    alt = float(c.get("altitude", 2.0))
    start = math.radians(float(c.get("start_deg", 0.0)))
    ccw = str(c.get("direction", "ccw")).lower() != "cw"
    pts = []
    for i in range(n):
        a = start + (1.0 if ccw else -1.0) * 2.0 * math.pi * i / n
        pts.append([centre[0] + r * math.cos(a), centre[1] + r * math.sin(a), alt])
    return {"centre": centre, "radius": r, "altitude": alt, "n": n, "start_deg":
            float(c.get("start_deg", 0.0)), "direction": "ccw" if ccw else "cw", "points": pts}


def pad(world):
    cfg = yaml.safe_load(open(world)) or {}
    p = cfg.get("pad") or {}
    return [float(p.get("x", 0.0)), float(p.get("y", 0.0)), float((cfg.get("mission_b") or {})
                                                                  .get("takeoff_alt", 2.0))]


def path_points(world):
    """The whole flown path: the pad, the circle's points, and the closing point."""
    c = read(world)
    if not c:
        return None
    p = pad(world)
    return [p] + [list(q) for q in c["points"]] + [list(c["points"][0])]


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "forest_b.yaml"))
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--path", action="store_true",
                    help="print the whole path (pad + points + closing point) instead of the circle")
    args = ap.parse_args()

    if not os.path.exists(args.world):
        print("no world file at %s" % args.world, file=sys.stderr)
        return 1
    c = read(args.world)
    if not c:
        print("%s has no mission_b.circle" % args.world, file=sys.stderr)
        return 1
    if args.json:
        out = dict(c)
        out["pad"] = pad(args.world)
        out["path"] = path_points(args.world)
        out["world"] = os.path.abspath(args.world)
        print(json.dumps(out, indent=1))
        return 0
    pts = path_points(args.world) if args.path else c["points"]
    for i, q in enumerate(pts):
        print("%4d  E %+8.4f  N %+8.4f  U %+6.3f" % (i, q[0], q[1], q[2]))
    print("# %d points, centre (%.3f, %.3f), radius %.3f m, altitude %.3f m, %s, %.1f deg start"
          % (len(pts), c["centre"][0], c["centre"][1], c["radius"], c["altitude"], c["direction"],
             c["start_deg"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
