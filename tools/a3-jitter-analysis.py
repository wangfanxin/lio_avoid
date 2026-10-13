#!/usr/bin/env python3
"""What is the LIO's per-sample jitter, measured on the samples that were actually delivered?

WHY THIS FILE EXISTS (2026-10-13)
---------------------------------
A3 was written as "resample both streams onto a 200 Hz grid, take the second difference, divide by the
truth's".  Reading the recordings back shows that definition measures the interpolator, not the LIO:

    run                stream   delivered rate    native 2nd diff (mm)      on a 200 Hz grid (mm)
    missionA-verify5   LIO      33.4 Hz           see the table below
                       truth    1200 Hz           see the table below

The delivered odometry arrives at ~33 Hz with a jittery period, so `np.interp` onto a 200 Hz grid turns
it into a piecewise-linear sawtooth: every grid step that lands mid-segment gets a second difference of
curvature/6, and the result depends on the phase between the two clocks.  Interpolating a 1200 Hz
noiseless truth onto the same grid does the opposite -- it removes high-frequency content.  So the
"fair ratio" compared an inflated number against a deflated one.

The FCU's EKF never interpolates: it consumes each delivered sample when it arrives.  So the honest
measurement is the second difference of consecutive delivered samples, expressed per axis in mm at the
native interval, and the honest reference is the same vehicle trajectory sampled at the SAME instants.

    usage: tools/a3-jitter-analysis.py <run-dir> [...]
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from tools.metrics import read_tap, read_odom, match, fit_yaw_translation   # noqa: E402


def rate_stats(t):
    dt = np.diff(t) * 1e-6
    if len(dt) == 0:
        return 0.0, 0.0, 0.0, 0.0
    return (len(t) - 1) / max(t[-1] - t[0], 1) * 1e6, np.median(dt) * 1e3, dt.min() * 1e3, dt.max() * 1e3


def d2_native(t, p):
    """RMS of the second difference of consecutive samples, in mm per axis."""
    if len(t) < 3:
        return np.zeros(3)
    d2 = p[2:] - 2.0 * p[1:-1] + p[:-2]
    return np.sqrt(np.mean(d2 ** 2, axis=0)) * 1e3


def sample_at(t_src, p_src, t):
    return np.column_stack([np.interp(t, t_src, p_src[:, k]) for k in range(3)])


def main() -> int:
    runs = sys.argv[1:]
    if not runs:
        print(__doc__.strip().splitlines()[-1])
        return 2
    for run in runs:
        tap, lio = os.path.join(run, "tap.csv"), os.path.join(run, "lio.csv")
        if not (os.path.exists(tap) and os.path.exists(lio)):
            print("%s: missing recordings" % run)
            continue
        t_t, p_t = read_tap(tap)
        t_l, p_l = read_odom(lio)
        print("=" * 108)
        print("%s" % os.path.basename(run))
        # the same window A1/A2 use: the truth above 0.5 m
        above = p_t[:, 2] > 0.5
        t_tw, p_tw = t_t[above], p_t[above]
        tl0, tl1 = t_tw[0], t_tw[-1]
        m = (t_l >= tl0) & (t_l <= tl1)
        t_l, p_l = t_l[m], p_l[m]
        for name, t, p in (("truth", t_tw, p_tw), ("LIO", t_l, p_l)):
            r, md, mn, mx = rate_stats(t)
            print("  %-6s n=%6d  rate %6.1f Hz   dt median %6.2f ms  min %6.2f  max %6.2f"
                  % (name, len(t), r, md, mn, mx))
        # the same trajectory, sampled at the LIO's own instants: the sampling geometry matches
        p_t_at_l = sample_at(t_tw, p_tw, t_l)
        j_lio = d2_native(t_l, p_l)
        j_ref = d2_native(t_l, p_t_at_l)
        med = float(np.median(np.diff(t_l))) * 1e-6
        print("  native 2nd difference of consecutive samples, window = truth above 0.5 m, dt = %.1f ms" % (med * 1e3))
        print("    %-26s E %8.4f  N %8.4f  U %8.4f   mm/sample" % ("LIO", j_lio[0], j_lio[1], j_lio[2]))
        print("    %-26s E %8.4f  N %8.4f  U %8.4f   mm/sample" % ("truth at LIO instants", j_ref[0], j_ref[1], j_ref[2]))
        print("    %-26s E %8.3f  N %8.3f  U %8.3f   x the truth sampled the same way"
              % ("ratio (LIO / that)", j_lio[0] / max(j_ref[0], 1e-12), j_lio[1] / max(j_ref[1], 1e-12),
                 j_lio[2] / max(j_ref[2], 1e-12)))
        print("    %-26s E %8.4f  N %8.4f  U %8.4f   m/s^2 equivalent (jitter / dt^2)"
              % ("as acceleration", j_lio[0] * 1e-3 / med ** 2, j_lio[1] * 1e-3 / med ** 2, j_lio[2] * 1e-3 / med ** 2))
        print("    %-26s E %8.4f  N %8.4f  U %8.4f   m/s^2 of the same truth"
              % ("", j_ref[0] * 1e-3 / med ** 2, j_ref[1] * 1e-3 / med ** 2, j_ref[2] * 1e-3 / med ** 2))
        # the failure mode a jitter gate must catch: a single big step, not the RMS
        d1 = np.diff(p_l, axis=0)
        d2 = p_l[2:] - 2.0 * p_l[1:-1] + p_l[:-2]
        print("    worst single step %6.4f %6.4f %6.4f mm;  worst |2nd diff| %6.4f %6.4f %6.4f mm"
              % (*(np.abs(d1).max(axis=0) * 1e3), *(np.abs(d2).max(axis=0) * 1e3)))
        # WHERE the outliers are decides whether they are one event or a habit
        step_mm = np.linalg.norm(d1, axis=1) * 1e3
        big = np.flatnonzero(step_mm > 5.0)
        print("    steps > 5 mm: %d of %d samples (%.3f%%)" % (len(big), len(step_mm),
                                                               100.0 * len(big) / max(len(step_mm), 1)))
        for i in big[np.argsort(-step_mm[big])][:5]:
            print("       t=%.2f s (flight clock)  step %7.3f mm   at %s"
                  % ((t_l[i] - t_l[0]) * 1e-6, step_mm[i],
                     " ".join("%+7.2f" % x for x in d1[i] * 1e3)))
        # is the wander white, or concentrated at the lidar's 10 Hz corrections?
        phase = np.arange(len(d1)) % 20          # 200 Hz samples, 20 per 100 ms scan
        by = np.array([np.sqrt(np.mean((d2[phase[:len(d2)] == k] ** 2), axis=0)).mean() for k in range(20)])
        print("    2nd-difference rms by position within the 100 ms scan: min %.4f max %.4f mm "
              "(flat = white, spiky = scan-correction sawtooth)"
              % (by.min() * 1e3, by.max() * 1e3))
        # the typical step is the robust question: does the pose move like the vehicle, or like noise?
        d1t = np.diff(p_t_at_l, axis=0)
        for nm, dd in (("LIO", d1), ("truth", d1t)):
            q = np.percentile(np.abs(dd), [50, 90, 99, 99.9], axis=0) * 1e3
            print("    |step| mm %-6s p50 %6.3f %6.3f %6.3f  p90 %6.3f %6.3f %6.3f  "
                  "p99 %6.3f %6.3f %6.3f  p99.9 %6.3f %6.3f %6.3f"
                  % (nm, *q[0], *q[1], *q[2], *q[3]))
        # error to the truth, in W, after the same fit metrics.py uses
        ia, ib = match(t_tw, t_l)
        R, tr = fit_yaw_translation(p_l[ib], p_tw[ia])
        err = (R @ p_l[ib].T).T + tr - p_tw[ia]
        print("    error LIO-to-truth over this window: rms E %6.2f  N %6.2f  U %6.2f mm  (mean removed in U: %6.2f)"
              % (np.sqrt(np.mean(err[:, 0] ** 2)) * 1e3, np.sqrt(np.mean(err[:, 1] ** 2)) * 1e3,
                 np.sqrt(np.mean(err[:, 2] ** 2)) * 1e3, np.sqrt(np.mean((err[:, 2] - err[:, 2].mean()) ** 2)) * 1e3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
