#!/usr/bin/env python3
"""One place that reads the world file and hands out the geometry the flight uses.

WHY THIS EXISTS: `tools/run-mission.sh` used to carry the mission corners as a hard-coded string and pass
them to `tools/metrics.py` as the reference the corners are judged against.  A reference that is typed
twice can drift from the one that is flown -- and it did: the trajectory source projected the corners
into L with one calibration while the FSM projected them back with another, so the flown box sat 51 mm
from the intended rectangle while the metrics compared against the intended one (STATUS.md row 44).
Everything geometric now comes from the world file through this tool.

  --shell    prints shell assignments: WORLD, CORNERS, GEOFENCE_M, ALT
  --json     prints the same as JSON
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import yaml


def read(world):
    cfg = yaml.safe_load(open(world)) or {}
    mission = cfg.get("mission", {}) or {}
    corners = mission.get("corners", []) or []
    arena = (cfg.get("arena", {}) or {}).get("walls", {}) or {}
    xs = arena.get("x", [-5.0, 5.0])
    ys = arena.get("y", [-5.0, 5.0])
    # the FSM's geofence gate: inside the walls, with a SMALL margin.  It is there to catch a runner,
    # not to be tighter than the world the planner plans in.  MEASURED 2026-10-14: with 0.5 m the
    # FSM aborted a Mission B flight at N +5.52 m inside a 12 x 12 m arena (walls at +-6.0), i.e. the
    # gate fired on a legitimate trajectory 0.48 m inside the world.
    geo = min(max(abs(xs[0]), abs(xs[1])), max(abs(ys[0]), abs(ys[1])))  # the arena itself: the
    # planner refuses paths outside it, so this gate only has to catch a runner -- MEASURED
    # 2026-10-14: arena-0.2 aborted two Mission B flights at 5.84 and 5.88 m in a 6.0 m arena
    return {"corners": corners, "alt": float(mission.get("takeoff_alt", 2.0) or 2.0),
            "geofence_m": float(geo), "arena": {"x": xs, "y": ys}}


class PillarWorld:
    """The world file's pillars as finite cylinders, with the clearance EVERY tool must agree on.

    WHY THIS LIVES HERE, AND WHY IT IS ONE DEFINITION.  The signed distance to the nearest pillar surface
    used to be `planner/ego_adapter.World.clearance`, imported by path from the tools that needed it, so
    that a recorder and the gate that refused a planner's paths could not disagree.  The ego-planner line
    was dropped on 2026-10-15 and the adapter with it, so the same rule -- horizontal distance minus r
    inside the pillar's z band, and the true CORNER distance above or below it (a point over a pillar's
    cap is not 0 m from it) -- moved here, into the one file that reads the world file.

    `tools/metrics.py` measures C2 independently and on the FLIGHT's own truth (a horizontal band test,
    which is what "did the vehicle ever come closer than 0.20 m" needs and is deliberately not this
    function): C2 is the acceptance number, this one is what the recorders print next to a plan.
    """

    def __init__(self, cyl):
        self.cyl = cyl                      # (M,5): x, y, r, z0, z1

    @staticmethod
    def load(path):
        import numpy as np
        with open(path) as fh:
            d = yaml.safe_load(fh) or {}
        pl = d.get("pillars") or []
        arr = (np.array([[p["x"], p["y"], p["r"], p.get("z0", 0.0), p.get("z1", 4.0)] for p in pl],
                        dtype=float) if pl else np.zeros((0, 5)))
        return PillarWorld(arr)

    def clearance(self, P):
        import numpy as np
        P = np.atleast_2d(np.asarray(P, dtype=float))
        if self.cyl.shape[0] == 0:
            return np.full(len(P), np.inf)
        best = np.full(len(P), np.inf)
        for x, y, r, z0, z1 in self.cyl:
            dh = np.hypot(P[:, 0] - x, P[:, 1] - y)
            dz = np.maximum.reduce([z0 - P[:, 2], P[:, 2] - z1, np.zeros(len(P))])
            d = np.where(dz <= 0.0, dh - r, np.hypot(np.maximum(dh - r, 0.0), dz))
            best = np.minimum(best, d)
        return best


def load_world(path):
    """`PillarWorld.load`, by the name the recorders used when this came from the adapter."""
    return PillarWorld.load(path)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True)
    ap.add_argument("--shell", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()
    if not os.path.exists(args.world):
        print("no world file at %s" % args.world, file=sys.stderr)
        return 1
    d = read(args.world)
    # A world whose mission lies outside its own arena is a geometry error that a flight would only
    # reveal as a mysterious geofence abort (MEASURED 2026-10-14: a 12 m corridor world generated with
    # --cells 18 puts the arena walls at +-9 m and the last corner at 12.5 m).  Say it here instead.
    outside = [c for c in d["corners"] if abs(c[0]) > d["geofence_m"] or abs(c[1]) > d["geofence_m"]]
    if outside:
        print("WARNING: %d of %d corners lie outside the geofence (%.2f m): %s"
              % (len(outside), len(d["corners"]), d["geofence_m"],
                 " ".join("(%.2f,%.2f)" % (c[0], c[1]) for c in outside)), file=sys.stderr)
    if args.json:
        d["corners_outside_geofence"] = len(outside)
        print(json.dumps(d, indent=1))
        return 0
    if args.shell:
        print('CORNERS="%s"' % ";".join("%.4f,%.4f,%.4f" % (c[0], c[1], c[2]) for c in d["corners"]))
        print("GEOFENCE_M=%.2f" % d["geofence_m"])
        # the mission altitude is the corners' own; takeoff_alt is what the FCU climbs to first
        print("TAKEOFF_ALT=%.2f" % d["alt"])
        return 0
    print("%s: %d corners, geofence %.2f m, takeoff_alt %.2f m"
          % (args.world, len(d["corners"]), d["geofence_m"], d["alt"]))
    for c in d["corners"]:
        print("   E %+7.3f  N %+7.3f  U %+7.3f" % (c[0], c[1], c[2]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
