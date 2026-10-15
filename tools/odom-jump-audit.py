#!/usr/bin/env python3
"""IS THE OFFSET A LIO FRAME JUMP?  The two plan paths in ego BOTH pass `start_pt_ = odom_pos_; start_vel_ =
odom_vel_` to reboundReplan (planFromGlobalTraj sets them outright; planFromCurrentTraj overrides the
trajectory proxy with the odometry by patch 0004-H2).  So if the committed splines do not start at the
vehicle, either the pipeline loses the start point AFTER that, or "the vehicle" moved between the commit and
the reading -- and the only thing that can move a whole trajectory relative to the aircraft is a JUMP in the
odometry frame the plans are expressed in.

This measures the odometry the plans are built from (/lio/robo/odom, recorded as lio.csv) against the truth
(tap.csv, the same sim clock), looking for steps between CONSECUTIVE samples.  A relocalisation is a step of
decimetres to metres in one sample interval; a flying vehicle cannot do that.

usage: python3 tools/odom-jump-audit.py <run-dir>
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np


def read(path, cols):
    t, p = [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                t.append(float(row[cols[0]]))
                p.append([float(row[cols[1]]), float(row[cols[2]]), float(row[cols[3]])])
            except (KeyError, ValueError):
                continue
    return np.array(t), np.array(p)


def main() -> int:
    run = sys.argv[1]
    tl, pl = read(os.path.join(run, "lio.csv"), ("t_us", "x", "y", "z"))
    tt, pt = read(os.path.join(run, "tap.csv"), ("t_us", "pn", "pe", "pd"))
    print("run %s" % run)
    print("  lio.csv %d samples, %.1f s" % (len(tl), (tl[-1] - tl[0]) * 1e-6))
    print("  tap.csv %d samples, %.1f s" % (len(tt), (tt[-1] - tt[0]) * 1e-6))

    def steps(t, p, name, min_dt=1e-3):
        dt = np.diff(t) * 1e-6
        d = np.linalg.norm(np.diff(p, axis=0), axis=1)
        ok = dt > min_dt
        v = d[ok] / dt[ok]
        print("  %-8s step |dp|: median %.4f  p99 %.4f  MAX %.4f m   (implied speed max %.2f m/s)"
              % (name, np.median(d[ok]), np.percentile(d[ok], 99), d[ok].max(), v.max()))
        big = np.flatnonzero(ok)[np.argsort(d[ok])[-5:]][::-1]
        for i in big[:5]:
            print("        at t=%.2f s: |dp| %.4f m over %.4f s (%.2f m/s)"
                  % ((t[i] - t[0]) * 1e-6, d[i], dt[i], d[i] / dt[i]))
        return d[ok]

    dl = steps(tl, pl, "LIO")
    dt_ = steps(tt, pt, "truth")
    print()
    print("  total path length: LIO %.2f m, truth %.2f m" % (dl.sum(), dt_.sum()))
    # a jump is a step far outside the distribution of the other steps
    thr = max(10.0 * np.median(dl), 0.20)
    n_jump = int((dl > thr).sum())
    print("  LIO steps above %.3f m (10x the median, >= 0.20 m): %d" % (thr, n_jump))
    if n_jump:
        for i in np.flatnonzero(dl > thr)[:8]:
            print("        JUMP at lio t=%.2f s: %.3f m" % ((tl[i] - tl[0]) * 1e-6, dl[i]))
    print()
    print("  VERDICT: %s" % ("THE ODOMETRY JUMPS -- plans committed before a jump are expressed in the old "
                             "frame, which is what makes them 1-3 m from the aircraft / from the post-jump "
                             "track" if n_jump else
                             "no jump: the offset is NOT a frame jump, so the start point is lost or moved "
                             "inside the planner pipeline"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
