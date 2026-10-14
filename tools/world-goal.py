#!/usr/bin/env python3
"""Print mission B's goal from a world file as `E,N,U` in W, for tools/run-planner.sh.

The goal lives in the world file so there is one source of geometry, exactly as
`planner/rectangle_source.py` reads mission A's corners from the same file.  `z` is
`mission_b.takeoff_alt` unless `--z` says otherwise.

usage: tools/world-goal.py [--world worlds/forest_all.yaml] [--z 2.0]
"""
from __future__ import annotations

import argparse
import os
import sys

import yaml


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "forest_all.yaml"))
    ap.add_argument("--z", type=float, default=None)
    args = ap.parse_args()

    with open(args.world) as fh:
        cfg = yaml.safe_load(fh) or {}
    mb = cfg.get("mission_b") or {}
    g = mb.get("goal")
    if not g:
        print("%s has no mission_b.goal" % args.world, file=sys.stderr)
        return 1
    z = args.z if args.z is not None else float(mb.get("takeoff_alt", 2.0))
    print("%.4f,%.4f,%.4f" % (float(g[0]), float(g[1]), z))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
