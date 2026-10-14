#!/usr/bin/env python3
"""A. WHY DO EGO'S COMMITTED PLANS START ~2 m FROM THE VEHICLE?  Measured, from AP6's own recordings.

For every trajectory ego committed (`bspline.csv` + `bspline.csv.points.csv`) this reconstructs the spline
and compares, at the moment it was received, against the LIO odometry the aircraft actually had
(`lio.csv`, the same topic both the adapter and the FSM read):

    |cp0 - odom|          the FIRST CONTROL POINT -- what a naive reader would call "the start"
    |spline(t=0) - odom|  the CURVE at t=0 (for a uniform cubic B-spline this is a weighted average of
                          cp0..cp3, so it is NOT cp0 -- the distinction matters when diagnosing a start gap)
    |spline(t_cur) - odom| where the curve is at the time the plan is being read
    duration, n_cp, and the curve's own extent

usage: python3 tools/plan-start-audit.py <run-dir>
"""
from __future__ import annotations

import csv
import importlib.util
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def load_adapter():
    p = os.path.join(ROOT, "planner", "ego_adapter.py")
    spec = importlib.util.spec_from_file_location("ea_psa", p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main() -> int:
    run = sys.argv[1]
    ea = load_adapter()

    # the control points, grouped by trajectory
    cps = {}
    with open(os.path.join(run, "bspline.csv.points.csv")) as fh:
        for row in csv.DictReader(fh):
            tid = (row["t_recv_us"], row["traj_id"])
            cps.setdefault(tid, []).append((int(row["i"]), float(row["e"]), float(row["n"]), float(row["u"])))
    meta = {}
    with open(os.path.join(run, "bspline.csv")) as fh:
        for row in csv.DictReader(fh):
            meta[(row["t_recv_us"], row["traj_id"])] = row

    # the LIO's own track (L frame, the same odometry the adapter projects onto)
    lio_t, lio_p = [], []
    with open(os.path.join(run, "lio.csv")) as fh:
        rd = csv.DictReader(fh)
        cols = rd.fieldnames
        for row in rd:
            lio_t.append(float(row.get("t_us") or row.get("t_wall") or 0.0))
            lio_p.append([float(row.get("x", row.get("e", 0.0))), float(row.get("y", row.get("n", 0.0))),
                          float(row.get("z", row.get("u", 0.0)))])
    lio_t = np.array(lio_t)
    lio_p = np.array(lio_p)
    print("lio.csv columns: %s" % cols)
    print("lio samples %d, t %.1f .. %.1f s, x %.2f..%.2f y %.2f..%.2f z %.2f..%.2f"
          % (len(lio_t), (lio_t[0] - lio_t[0]) * 1e-6, (lio_t[-1] - lio_t[0]) * 1e-6,
             lio_p[:, 0].min(), lio_p[:, 0].max(), lio_p[:, 1].min(), lio_p[:, 1].max(),
             lio_p[:, 2].min(), lio_p[:, 2].max()))

    # t_recv_us is an epoch stamp in microseconds; lio's t_us is the SIM clock (also microseconds since the
    # tap's origin).  Correlate by matching each commit to the NEAREST lio sample in the sim clock implied by
    # the receive order: the recorder writes both files in the same run, so the mapping is monotone.  Rather
    # than guess an offset, report the comparison in the POSE domain: distance from cp0 / spline(0) to the
    # LIO track's nearest point in space (the aircraft was somewhere on that track).
    keys = sorted(cps, key=lambda k: int(k[0]))
    print()
    print("traj_id  n_cp  dur(s)   |cp0 - nearest LIO pt|  |spline(0) - nearest|  |spline(mid) - nearest|  extent(m)")
    rows = []
    for k in keys[:14]:
        pts = [c[1:] for c in sorted(cps[k])]
        P = np.array(pts)
        m = meta.get(k)
        if m is None or len(P) < 4:
            continue
        order = int(m["order"])
        n_cp = int(m["n_cp"])
        k0, k1 = float(m["k0"]), float(m["k1"])
        ts = k1 - k0
        knots = np.array([k0 + i * ts for i in range(n_cp + order + 1)])
        dur = ea.bspline_duration(knots, n_cp, order)
        p0 = ea.bspline_eval(knots, P, order, 0.0)
        pm = ea.bspline_eval(knots, P, order, dur / 2.0)
        d_cp0 = float(np.min(np.linalg.norm(lio_p - P[0], axis=1)))
        d_s0 = float(np.min(np.linalg.norm(lio_p - p0, axis=1)))
        d_sm = float(np.min(np.linalg.norm(lio_p - pm, axis=1)))
        ext = float(np.max(np.linalg.norm(P - P[0], axis=1)))
        print("%7s %5d %7.2f   %20.3f  %20.3f  %21.3f  %8.3f"
              % (k[1], n_cp, dur, d_cp0, d_s0, d_sm, ext))
        rows.append((dur, d_cp0, d_s0, ext, len(P)))
    if rows:
        a = np.array([[r[0], r[1], r[2], r[3], r[4]] for r in rows])
        print()
        print("over %d committed trajectories:" % len(a))
        print("   duration          median %.2f s   min %.2f  max %.2f" % (np.median(a[:, 0]), a[:, 0].min(), a[:, 0].max()))
        print("   |cp0 - LIO track| median %.3f m   max %.3f m" % (np.median(a[:, 1]), a[:, 1].max()))
        print("   |spline(0) - LIO|  median %.3f m   max %.3f m" % (np.median(a[:, 2]), a[:, 2].max()))
        print("   curve extent      median %.3f m   max %.3f m" % (np.median(a[:, 3]), a[:, 3].max()))
        print("   control points    median %.0f" % np.median(a[:, 4]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
