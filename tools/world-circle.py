#!/usr/bin/env python3
"""The world file's circle as points -- the ONE place that turns `mission_b.circle` into a list.

The world file stores the circle as five numbers (`mission_b.circle`: centre, radius, altitude, number of
points, start angle).  Everything else -- a confirmation figure, a metrics run, a station list -- derives
its point list from HERE, so the circle drawn, the circle commanded and the circle scored cannot drift
apart.  This is the same rule as mission A's corners (tools/world-info.py): geometry is read once, from
the world file.

STATUS: THE MISSION THAT USED THIS IS DROPPED.  Mission B (ego-planner, the r = 4 m circle in
`worlds/forest_b.yaml`) was dropped by the owner on 2026-10-15, so nothing in the working pipeline
(Mission A: the rectangle) calls this file.  It is KEPT because `tools/plot-run.py` imports it to label a
flight's path, because `mission_b.circle` is still the record of what was tried, and because the owner's
cleanup list named it.  `world_clearance` below no longer imports `planner/ego_adapter.py` -- that file
was deleted with the rest of the ego chain.

The path a mission-B flight actually flew is

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
            float(c.get("start_deg", 0.0)), "direction": "ccw" if ccw else "cw", "points": pts,
            "snap_need": float(c.get("snap_need", 0.0) or 0.0)}


def pad(world):
    cfg = yaml.safe_load(open(world)) or {}
    p = cfg.get("pad") or {}
    return [float(p.get("x", 0.0)), float(p.get("y", 0.0)), float((cfg.get("mission_b") or {})
                                                                  .get("takeoff_alt", 2.0))]


_WORLD_CACHE = {}


def world_clearance(world, pts):
    """Signed distance from each point to the nearest TRUE pillar surface, from the ONE definition.

    ONE DEFINITION, IMPORTED RATHER THAN RE-IMPLEMENTED.  This used to import
    `planner/ego_adapter.World.clearance`; the ego-planner line was dropped on 2026-10-15 and the adapter
    with it, so the same finite-cylinder rule (horizontal distance minus r inside the pillar's z band,
    the true CORNER distance above or below it) now lives in `tools/world-info.py::PillarWorld`, and the
    recorders import it from there too.  This is the rule the project's numbers are quoted against;
    `tools/metrics.py` measures C2 independently, on the flight's own truth.

    The loaded world is CACHED: MEASURED 2026-10-14, the first version re-parsed the yaml on EVERY point
    test, which made the 24-waypoint snap search take longer than a 600 s timeout.
    """
    import numpy as np
    key = os.path.abspath(world)
    if key not in _WORLD_CACHE:
        import importlib.util
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "world-info.py")
        spec = importlib.util.spec_from_file_location("world_info_wc", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _WORLD_CACHE[key] = mod.PillarWorld.load(key)
    return _WORLD_CACHE[key].clearance(np.atleast_2d(np.asarray(pts, dtype=float)))


def slide_points(world, need, step_deg=0.05, max_deg=90.0):
    """The commanded stops: a blocked one moves FORWARD along the circle to the next point that is clear.

    The user's rule (2026-10-14): *"如果这个点在圆柱里，那就找这个圆柱和路径的下一个交点，做我们的路径点"* --
    i.e. walk along the path in the direction of travel until the circle leaves that pillar, and command
    the exit point.  The stop keeps radius r (only its angle changes), every pillar stays in the world, and
    the reference is still "a circle of r = 4 m about the pad".  A goal inside a pillar is not an obstacle to
    avoid: it is an unreachable target (MEASURED 2026-10-14 -- commanding one made the path gate withhold 81
    of 93 paths and left the vehicle 0.7338 m short).

    `need` is the clearance the exit point must have, so the stop is not exactly ON the surface: it is the
    intersection of the circle with the pillar grown by `need`.

    History, so the rule that failed is not silently forgotten (all three measured on this world and in
    docs/STATUS.md row 52):
      * each blocked stop to its OWN nearest free spot (either direction): two neighbours converged, spacing
        fell from 1.047 m nominal to 0.035 m;
      * forward-only with an extra minimum gap: stops marched and left ONE 22.4 m gap (a single command
        covering 320 deg, which the dropped planner answered by cutting straight across the interior);
      * spacing the stops evenly in the circle's free arc length: correct spacing, but it moves every stop,
        including the ones that were already fine.
    This one moves ONLY the blocked stops, which is what the user asked for.
    """
    c = read(world)
    if not c:
        return None, []
    cx, cy = c["centre"]
    r, alt, n = c["radius"], c["altitude"], c["n"]
    sgn = 1.0 if c["direction"] == "ccw" else -1.0
    dth = math.radians(step_deg) * sgn
    steps = int(round(max_deg / step_deg))

    pts, moved = [], []
    prev_th = None
    for i in range(n):
        th0 = math.radians(c["start_deg"]) + sgn * 2.0 * math.pi * i / n
        found = None
        for k in range(steps + 1):
            th = th0 + k * dth
            p = [cx + r * math.cos(th), cy + r * math.sin(th), alt]
            if float(world_clearance(world, p)[0]) < need:
                continue
            found = (p, r * abs(math.radians(step_deg * k)), th)
            break
        if found is None:                     # no exit within max_deg of forward travel
            th = th0
            found = ([cx + r * math.cos(th), cy + r * math.sin(th), alt], float("nan"), th)
        pts.append(found[0])
        moved.append(found[1])
        prev_th = found[2]
    return pts, moved


def snapped_points(world, need):
    """Backwards-compatible name for `slide_points` (the first version moved goals off the circle)."""
    return slide_points(world, need)[0]


def commanded_points(world):
    """The waypoints the mission will actually command: the circle, with the slide rule applied.

    The requirement lives in the world file (`mission_b.circle.snap_need`) so that the figure, the goal
    sequencer and the metrics all derive the same list from the same place -- the same rule that puts
    mission A's corners in the world file.
    """
    c = read(world)
    if not c:
        return None
    if c["snap_need"] > 0:
        return slide_points(world, c["snap_need"])[0]
    return [list(q) for q in c["points"]]


def path_points(world):
    """The whole flown path: the pad, the commanded waypoints, and the closing one."""
    pts = commanded_points(world)
    if not pts:
        return None
    p = pad(world)
    return [p] + [list(q) for q in pts] + [list(pts[0])]


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "forest_b.yaml"))
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--snap-need", type=float, default=-1.0,
                    help="clearance a commanded waypoint must have; a blocked one SLIDES ALONG THE CIRCLE "
                         "to the nearest free spot (metres).  Default: the world file's "
                         "mission_b.circle.snap_need.  0 = command the circle verbatim")
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
    need = args.snap_need if args.snap_need >= 0 else c["snap_need"]
    before = [list(q) for q in c["points"]]
    moved = []
    if need > 0:
        pts, moved = slide_points(args.world, need)
        moved = [d for d in moved if d > 1e-9]
        print("# slid along the circle to clearance >= %.2f m%s: %d of %d waypoints moved, "
              "arc median %.3f / max %.3f m"
              % (need, "" if args.snap_need >= 0 else " (from the world file)", len(moved),
                 len(before), (sorted(moved)[len(moved) // 2] if moved else 0.0),
                 (max(moved) if moved else 0.0)), file=sys.stderr)
    else:
        pts = before
    c["points"] = pts
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
