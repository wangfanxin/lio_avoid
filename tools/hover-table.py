#!/usr/bin/env python3
"""At every corner HOVER, put the COMMAND, the FCU's own estimate and the truth side by side.

WHY: B1's box comes out 153 mm wider than the commanded 2.000 m in E on every clean sortie.  Before calling
that a flight defect, the mechanism has to be read: was the aircraft commanded to the corner and failed to
hold it, or is the command/estimate itself displaced?  The run's own setpoints.csv carries all three
quantities on the same sim clock (cmd = what the FSM streamed, W; fcu = the FCU's position estimate, F;
truth = tap.csv, W), so it is measured, not argued.

usage: tools/hover-table.py <run-dir> [--world worlds/pillars.yaml]
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--parked-v", type=float, default=0.03, help="'hovering' means slower than this, m/s")
    ap.add_argument("--near-m", type=float, default=0.30, help="...and this close to a corner, m")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.world)) or {}
    x0, y0, x1, y1 = [float(v) for v in cfg["mission"]["square"]]
    corners = [("c1", x0, y0), ("c2", x1, y0), ("c3", x1, y1), ("c4", x0, y1)]

    d = np.loadtxt(os.path.join(args.run, "tap.csv"), delimiter=",", skiprows=1)
    tt = d[:, 0].astype(np.int64)
    te, tn, tu = d[:, 3], d[:, 2], -d[:, 4]
    # speed from the tap itself
    v = np.hypot(np.gradient(te, tt * 1e-6), np.gradient(tn, tt * 1e-6))

    sp = np.loadtxt(os.path.join(args.run, "setpoints.csv"), delimiter=",", skiprows=1)
    st = sp[:, 1].astype(np.int64)
    scmd, sfcu = sp[:, 2:5], sp[:, 11:14]

    print("### %s" % os.path.basename(args.run.rstrip("/")))
    print("   hover = truth within %.2f m of the corner AND slower than %.2f m/s, runs >= 0.5 s"
          % (args.near_m, args.parked_v))
    print("   %-4s %8s %7s | %-34s | %-34s | %-34s"
          % ("", "t0..t1", "dur", "COMMAND (W) minus corner", "FCU estimate (F) minus corner",
             "TRUTH (W) minus corner"))
    for name, cx, cy in corners:
        dmin = np.hypot(te - cx, tn - cy)
        near = (dmin <= args.near_m) & (v <= args.parked_v)
        # contiguous runs
        runs, s = [], None
        for i, ok in enumerate(near):
            if ok and s is None:
                s = i
            elif not ok and s is not None:
                if tt[i - 1] - tt[s] >= 500_000:
                    runs.append((s, i - 1))
                s = None
        if s is not None and tt[-1] - tt[s] >= 500_000:
            runs.append((s, len(near) - 1))
        if not runs:
            print("   %-4s  NO HOVER FOUND (closest %.3f m)" % (name, dmin.min()))
            continue
        for (a, b) in runs:
            t0, t1 = tt[a], tt[b]
            m = (st >= t0) & (st <= t1)
            tc, tf = scmd[m].mean(axis=0), sfcu[m].mean(axis=0)
            th = np.array([te[a:b + 1].mean(), tn[a:b + 1].mean(), tu[a:b + 1].mean()])
            print("   %-4s %5.1f..%5.1f %6.1f s | cmd  E %+7.4f N %+7.4f | fcu  E %+7.4f N %+7.4f | "
                  "truth E %+7.4f N %+7.4f   (%d setpoints, %d tap samples)"
                  % (name, (t0 - tt[0]) * 1e-6, (t1 - tt[0]) * 1e-6, (t1 - t0) * 1e-6,
                     tc[0] - cx, tc[1] - cy, tf[0] - cx, tf[1] - cy, th[0] - cx, th[1] - cy,
                     int(m.sum()), b - a + 1))
            # the aircraft's own spread inside the hover, so a "hover" that is really a slow drift is visible
            print("        %-4s   truth spread in the hover: E %+7.4f..%+7.4f  N %+7.4f..%+7.4f  "
                  "U %+7.4f..%+7.4f (cmd U %+7.4f, fcu U %+7.4f)"
                  % ("", te[a:b + 1].min() - cx, te[a:b + 1].max() - cx,
                     tn[a:b + 1].min() - cy, tn[a:b + 1].max() - cy,
                     tu[a:b + 1].min() - 2.0, tu[a:b + 1].max() - 2.0, tc[2] - 2.0, tf[2] - 2.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
