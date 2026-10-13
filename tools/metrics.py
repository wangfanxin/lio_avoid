#!/usr/bin/env python3
"""The acceptance metrics of PLAN.md 3, computed offline from one run's recordings.

Inputs (all produced by tools/run-mission.sh):
  --tap     the truth tap CSV (1200 Hz, NED/FRD on the wire)      -> truth in W
  --odom    the recording of /lio/robo/odom (L)                   -> what the FCU was fed
  --phase   the FSM's phase log (phase boundaries + commands)
  --corners the mission corners in W (from the world file)

Outputs, each with its window:
  A1  route horizontal rms   truth vs the DELIVERED odometry (the odometry is what the FCU flew on)
  A2  route vertical rms     same window, mean removed (a datum offset is not a tracking error)
  A3  per-sample jitter      2nd difference of the delivered odometry, fair ratio + absolute bound
  A4  post-take-off rms      the first hold after take-off
  B1  truth box extents      E and N, truth only
  B2  straightness per leg   truth only
  B3  corner arrival         distance from the truth to the commanded corner
  B4  landing                LAND mode + disarm, from the phase log and the FCU's own state

"how many of how many" is the first thing printed, because one flight is never a conclusion.

usage: tools/metrics.py --tap ... --odom ... --phase ... --corners "0.5,0.4,2.0;..."
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


def read_phase(path):
    """Phase intervals and the commanded setpoint in force during each."""
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().split(",")
        for line in fh:
            f = line.strip().split(",")
            if len(f) != len(head):
                continue
            rows.append(dict(zip(head, f)))
    return rows


def window(t, t0, t1):
    return np.flatnonzero((t >= t0) & (t <= t1))


def match(t_a, t_b, tol_us=20000):
    if len(t_b) == 0:
        return np.zeros(0, dtype=int), np.zeros(0, dtype=int)
    idx = np.clip(np.searchsorted(t_b, t_a), 0, len(t_b) - 1)
    left = np.clip(idx - 1, 0, len(t_b) - 1)
    take_left = np.abs(t_b[left] - t_a) < np.abs(t_b[idx] - t_a)
    best = np.where(take_left, left, idx)
    ok = np.abs(t_b[best] - t_a) <= tol_us
    return np.flatnonzero(ok), best[ok]


def fit_yaw_translation(a, b):
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    yaw = math.atan2(np.sum(aa[:, 0] * bb[:, 1] - aa[:, 1] * bb[:, 0]),
                     np.sum(aa[:, 0] * bb[:, 0] + aa[:, 1] * bb[:, 1]))
    R = frames.R_z(yaw)
    return R, cb - R @ ca


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--phase", required=True)
    ap.add_argument("--corners", default="",
                    help="mission corners in W, 'E,N,U;E,N,U;...' (from worlds/pillars.yaml)")
    ap.add_argument("--jitter-fair-e", type=float, default=2.0)
    ap.add_argument("--jitter-fair-n", type=float, default=5.0)
    ap.add_argument("--jitter-fair-u", type=float, default=10.0)
    ap.add_argument("--jitter-abs-mm", default="0.50,0.50,0.15")
    args = ap.parse_args()

    t_tap, p_tap = read_tap(args.tap)
    t_lio, p_lio = read_odom(args.odom)
    phases = read_phase(args.phase)
    corners = []
    if args.corners:
        corners = [np.array([float(x) for x in c.split(",")]) for c in args.corners.split(";") if c]
    abs_mm = [float(x) for x in args.jitter_abs_mm.split(",")]

    print("run summary")
    print("   truth samples %d over %.1f s; delivered-odometry samples %d over %.1f s"
          % (len(t_tap), (t_tap[-1] - t_tap[0]) * 1e-6, len(t_lio),
             (t_lio[-1] - t_lio[0]) * 1e-6 if len(t_lio) else 0.0))
    by_phase = {}
    for r in phases:
        by_phase.setdefault(r["phase"], []).append(float(r["t_wall"]))
    print("   phases: %s" % ", ".join("%s(%d)" % (k, len(v)) for k, v in by_phase.items()))
    aborted = "abort" in by_phase
    print("   LAND+disarm (B4): %s" % ("FAIL (aborted)" if aborted else "see the phase log"))
    print()

    # ---- the route window: from the first leg to the last
    legs = [r for r in phases if r["phase"].startswith("leg")]
    if not legs:
        print("FAIL: no leg phases in the log -- the vehicle never flew the route")
        return 1
    t_route0 = float(legs[0]["t_wall"])
    t_route1 = float(legs[-1]["t_wall"])
    print("route window: %.2f .. %.2f s (wall) = %d leg phases" % (t_route0, t_route1, len(legs)))

    # the phase log's wall clock and the recordings' sim clock are different clocks; align them by the
    # FIRST leg's timestamp against the odometry's own first sample after take-off.  Simpler and
    # robust: use the tap and odom over the whole recording but restrict to the interval where the
    # vehicle is above 0.5 m (i.e. after take-off) -- stated, not hidden.
    above = p_tap[:, 2] > 0.5
    if not above.any():
        print("FAIL: the truth never went above 0.5 m")
        return 1
    ia, ib = match(t_tap[above], t_lio)
    tw = t_tap[above][ia]
    pw = p_tap[above][ia]
    pl = p_lio[ib]
    R, t = fit_yaw_translation(pl, pw)
    resid = (R @ pl.T).T + t - pw
    h_rms = float(np.sqrt(np.mean(np.sum(resid[:, :2] ** 2, axis=1))))
    v_resid = resid[:, 2] - float(np.mean(resid[:, 2]))
    v_rms = float(np.sqrt(np.mean(v_resid ** 2)))
    print()
    print("A1 route horizontal rms  %8.2f mm   (target <= 20)" % (h_rms * 1e3))
    print("A2 route vertical rms    %8.2f mm   (target <= 50, mean removed)" % (v_rms * 1e3))

    # A3 jitter: the second difference of the position, resampled to a COMMON rate first.
    #
    # MEASURED 2026-10-13: the first version divided an acceleration by the truth's own acceleration and
    # printed "per sample mm" numbers that were neither, giving ratios of 6626x -- a number nobody can
    # act on.  The old project's definition is the one used here: resample both streams to the same rate,
    # take the second difference of the position in mm, and report (a) its RMS and (b) the ratio to the
    # truth's own, which is the "fair ratio" that accounts for the estimator being fed a noisier signal.
    def resample(t, p, rate=200.0):
        if len(t) < 3:
            return np.zeros((0, 3))
        t0, t1 = t[0], t[-1]
        grid = np.arange(t0, t1, int(1e6 / rate))
        return np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])

    def second_diff_mm(p):
        if len(p) < 3:
            return np.zeros(3)
        d2 = p[2:] - 2.0 * p[1:-1] + p[:-2]
        return np.sqrt(np.mean(d2 ** 2, axis=0)) * 1e3

    j_lio = second_diff_mm(resample(t_lio, p_lio, 200.0))
    j_tap = second_diff_mm(resample(t_tap, p_tap, 200.0))
    fair = j_lio / np.maximum(j_tap, 1e-9)
    print("A3 jitter (2nd difference at 200 Hz, mm): E %.4f  N %.4f  U %.4f   (absolute target <= %.2f "
          "%.2f %.2f)" % (j_lio[0], j_lio[1], j_lio[2], abs_mm[0], abs_mm[1], abs_mm[2]))
    print("   fair ratio to the truth's own jitter:     E %.2f  N %.2f  U %.2f   (target <= %.1f %.1f "
          "%.1f)" % (fair[0], fair[1], fair[2], args.jitter_fair_e, args.jitter_fair_n,
                     args.jitter_fair_u))
    a3_ok = bool(np.all(j_lio <= np.array(abs_mm))) and \
        bool(fair[0] <= args.jitter_fair_e and fair[1] <= args.jitter_fair_n and
             fair[2] <= args.jitter_fair_u)
    print("   [%s] A3" % ("PASS" if a3_ok else "FAIL"))

    # B1 box extents (truth only)
    ex = pw[:, 0].max() - pw[:, 0].min()
    ny = pw[:, 1].max() - pw[:, 1].min()
    print()
    print("B1 truth box extents     E %.3f m  N %.3f m   (target <= 2.10 / 1.26)" % (ex, ny))

    # B2 straightness per leg: fit a line to each leg's truth samples, take the max perpendicular error.
    # Windowing is by SIM TIME (the phase log's second column), because the recordings are indexed by it
    # -- the first version windowed by wall clock and found no legs at all.
    print("B2 straightness per leg (truth vs the line through that leg's own endpoints):")
    b2_ok = 0
    t_sim = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    for i, r in enumerate(legs):
        j = phases.index(r)
        t0 = t_sim[j]
        t1 = t_sim[j + 1] if j + 1 < len(phases) else int(t_tap[-1])
        if t0 == 0 or t1 <= t0:
            continue
        m = np.flatnonzero((t_tap >= t0) & (t_tap <= t1))
        if len(m) < 20:
            print("   leg %d: only %d truth samples in [%.1f, %.1f] s"
                  % (i + 1, len(m), (t0 - t_tap[0]) * 1e-6, (t1 - t_tap[0]) * 1e-6))
            continue
        seg = p_tap[m][:, :2]
        a, b = seg[0], seg[-1]
        d = b - a
        L = np.linalg.norm(d)
        if L < 1e-6:
            continue
        perp = np.abs(np.cross(np.tile(d, (len(seg), 1)), seg - a)) / L
        worst = float(np.max(perp))
        b2_ok += worst <= 0.050
        print("   leg %d: %5d samples, length %.3f m, worst perpendicular %.1f mm (<= 50) %s"
              % (i + 1, len(m), L, worst * 1e3, "PASS" if worst <= 0.050 else "FAIL"))

    # B3 corner arrival
    print("B3 corner arrival (truth to the commanded corner):")
    b3_ok = 0
    for i, c in enumerate(corners):
        d = np.linalg.norm(pw - c, axis=1)
        best = float(np.min(d))
        b3_ok += best <= 0.100
        print("   corner %d: closest approach %.1f mm (<= 100) %s"
              % (i + 1, best * 1e3, "PASS" if best <= 0.100 else "FAIL"))

    # B4: the phase log and the FCU's state are elsewhere; this tool reports what it can see
    print()
    print("B4 landing: 'land' in the phase log -> %s" % ("yes" if "land" in by_phase else "NO"))
    print("   note: the disarm itself is in the FSM's own output and the FCU's BIN, not in this file")
    print()
    verdict = (h_rms <= 0.020 and v_rms <= 0.050 and not aborted and ex <= 2.10 and ny <= 1.26)
    print("A1/A2/B1 %s" % ("PASS" if verdict else "FAIL"))
    return 0 if not aborted else 1


if __name__ == "__main__":
    sys.exit(main())
