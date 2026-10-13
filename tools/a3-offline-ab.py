#!/usr/bin/env python3
"""Is A3's vertical jitter real, or is it my own resampling?

A3 as written resamples both streams onto a 200 Hz grid before taking the second difference.  The LIO
publishes at ~199-207 Hz with a jittery period, so interpolating it onto an exact grid can add a
sawtooth of its own -- and that artefact would land in whatever channel has the least real jitter.

This computes the same quantity two ways from the SAME recordings, with no new flights:

  A) grid:      resample to 200 Hz, second difference              (what tools/metrics.py does now)
  B) own samples: only triplets whose two intervals are within 20% of each other, no interpolation;
                  the second difference is divided by dt^2 so the result is an acceleration and the two
                  streams are comparable at any rate.  The "mm per sample at 200 Hz" figure is then
                  a * (0.005 s)^2.

usage: tools/a3-offline-ab.py <run-dir> [...]
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames          # noqa: E402


def read_tap(path):
    t, p = [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 6:
                continue
            t.append(int(f[0]))
            p.append([float(f[2]), float(f[3]), float(f[4])])
    p = np.array(p, dtype=float)
    return np.array(t, dtype=np.int64), np.array([frames.ned_to_enu(x) for x in p])


def read_odom(path):
    t, p = [], []
    with open(path) as fh:
        fh.readline()
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 4:
                continue
            t.append(int(f[0]))
            p.append([float(f[1]), float(f[2]), float(f[3])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float)


def grid_jitter_mm(t, p, rate=200.0):
    if len(t) < 3:
        return np.zeros(3)
    grid = np.arange(t[0], t[-1], int(1e6 / rate))
    q = np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])
    d2 = q[2:] - 2.0 * q[1:-1] + q[:-2]
    return np.sqrt(np.mean(d2 ** 2, axis=0)) * 1e3


def own_sample_accel(t, p, tol=0.20):
    """RMS of the second difference divided by dt^2, using only well-behaved triplets."""
    if len(t) < 3:
        return np.zeros(3), 0
    dt = np.diff(t) * 1e-6
    keep = []
    d2 = []
    for i in range(len(t) - 2):
        d1, d2t = dt[i], dt[i + 1]
        if d1 <= 0 or d2t <= 0:
            continue
        if abs(d1 - d2t) / max(d1, d2t) > tol:
            continue
        keep.append(i)
        d2.append((p[i + 2] - 2.0 * p[i + 1] + p[i]) / (0.5 * (d1 + d2t)) ** 2)
    if not d2:
        return np.zeros(3), 0
    d2 = np.array(d2)
    return np.sqrt(np.mean(d2 ** 2, axis=0)), len(keep)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--rate", type=float, default=200.0)
    args = ap.parse_args()

    print("%-34s %-9s %-26s %-26s %s" % ("run", "stream", "A) grid, mm/sample",
                                         "B) own samples, mm/sample", "fair ratio (B)"))
    for run in args.runs:
        tap = os.path.join(run, "tap.csv")
        lio = os.path.join(run, "lio.csv")
        if not (os.path.exists(tap) and os.path.exists(lio)):
            print("%-34s (missing recordings)" % os.path.basename(run))
            continue
        t_t, p_t = read_tap(tap)
        t_l, p_l = read_lio(lio) if False else read_odom(lio)
        # only the flight: drop the long parked tail after landing by keeping samples above 0.5 m
        m = p_t[:, 2] > 0.5
        if m.any():
            t_t, p_t = t_t[m], p_t[m]
        tl0, tl1 = t_t[0], t_t[-1]
        ml = (t_l >= tl0) & (t_l <= tl1)
        t_l, p_l = t_l[ml], p_l[ml]

        ga = grid_jitter_mm(t_l, p_l, args.rate)
        gb = grid_jitter_mm(t_t, p_t, args.rate)
        aa, na = own_sample_accel(t_l, p_l)
        ab, nb = own_sample_accel(t_t, p_t)
        conv = (1.0 / args.rate) ** 2 * 1e3
        print("%-34s %-9s E %6.3f N %6.3f U %6.3f   E %6.3f N %6.3f U %6.3f   E %5.1f N %5.1f U %5.1f"
              % (os.path.basename(run), "LIO", ga[0], ga[1], ga[2],
                 aa[0] * conv, aa[1] * conv, aa[2] * conv,
                 aa[0] / max(ab[0], 1e-12), aa[1] / max(ab[1], 1e-12), aa[2] / max(ab[2], 1e-12)))
        print("%-34s %-9s E %6.3f N %6.3f U %6.3f   E %6.3f N %6.3f U %6.3f   (truth, n=%d)"
              % ("", "truth", gb[0], gb[1], gb[2], ab[0] * conv, ab[1] * conv, ab[2] * conv, nb))
        print("%-34s %-9s triplets used: LIO %d of %d, truth %d of %d"
              % ("", "", na, len(t_l), nb, len(t_t)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
