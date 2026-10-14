#!/usr/bin/env python3
"""Did the aircraft ACTUALLY hover at each corner?  Measure it from the truth tap, not from the producer's log.

WHY: `rectangle_source.py` prints "HOLD at corner j: dwelling 5.0 s" whenever it has waited its own pacing
interval, which says nothing about where the AIRCRAFT was.  MEASURED (RECT3-hold5): the producer logged three
5 s holds while the truth track flew only pad -> c2 -> c3, i.e. the dwells happened wherever the aircraft
happened to be.  So the claim "hover 5 s at every corner" has to be measured from `tap.csv` (1200.5 Hz).

usage: tools/dwell-audit.py <run-dir-or-tap.csv> [--world worlds/pillars.yaml] [--radius 0.15] [--min-s 5.0]
"""
import argparse
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def corners_from_world(world):
    cfg = yaml.safe_load(open(world)) or {}
    m = cfg.get("mission") or cfg.get("mission_a") or {}
    sq = m.get("square")
    if not sq:
        raise SystemExit("%s has no mission.square" % world)
    x0, y0, x1, y1 = [float(v) for v in sq]
    # counter-clockwise, the order the producer uses
    return {"c1": (x0, y0), "c2": (x1, y0), "c3": (x1, y1), "c4": (x0, y1)}


def longest_run(mask, t):
    """Longest contiguous True span, in seconds."""
    best, start = 0.0, None
    for i, ok in enumerate(mask):
        if ok and start is None:
            start = t[i]
        elif not ok and start is not None:
            best = max(best, t[i] - start)
            start = None
    if start is not None:
        best = max(best, t[-1] - start)
    return best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="a run dir or a tap.csv")
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--radius", type=float, default=0.15, help="what counts as 'at the corner', m")
    ap.add_argument("--min-s", type=float, default=5.0, help="the dwell the mission asked for, s")
    args = ap.parse_args()

    tap = args.run if args.run.endswith(".csv") else os.path.join(args.run, "tap.csv")
    if not os.path.exists(tap):
        raise SystemExit("no tap.csv at %s" % tap)
    d = np.loadtxt(tap, delimiter=",", skiprows=1)
    t_us, pn, pe, pd = d[:, 0], d[:, 2], d[:, 3], d[:, 4]
    t = (t_us - t_us[0]) * 1e-6
    # the tap is NED-ish (pn = north, pe = east); the mission's square is authored in ENU (E, N)
    en = np.column_stack([pe, pn])
    print("### %s   (%d samples, %.1f s, radius %.2f m, asked %.1f s per corner)"
          % (os.path.basename(os.path.dirname(tap)), len(t), t[-1], args.radius, args.min_s))
    corners = corners_from_world(args.world)
    ok_n = 0
    for name, c in corners.items():
        dist = np.linalg.norm(en - np.asarray(c, dtype=float), axis=1)
        i_min = int(np.argmin(dist))
        dwell = longest_run(dist <= args.radius, t)
        visited = bool(dist[i_min] <= args.radius)
        ok = dwell >= args.min_s
        ok_n += 1 if ok else 0
        print("   %s (%+.2f,%+.2f): closest %.3f m at t=%5.1f s | longest dwell within %.2f m: %5.2f s  %s%s"
              % (name, c[0], c[1], dist[i_min], t[i_min], args.radius, dwell,
                 "HOLD-OK" if ok else "NO-HOLD", "" if visited else "  (never visited)"))
    print("   CORNERS HOVERED >= %.1f s: %d of %d" % (args.min_s, ok_n, len(corners)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
