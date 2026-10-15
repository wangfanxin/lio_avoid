#!/usr/bin/env python3
"""Does the LIO track the truth?  Offline, from two recordings, with the alignment choices PRINTED.

Inputs:
  --tap   the truth tap CSV (sim/truth_rx.py --csv), 1200 Hz, NED/FRD on the wire
  --odom  a recording of the LIO's body odometry (tools/record-odom.py), in L

Method, and why it is two fits rather than one:

  1. **4-DOF fit** (yaw + translation): this is the alignment the mission actually uses -- `Calib`
     gives L -> W as a constant yaw and an offset, and A1/A2 are measured in that alignment.  Its
     vertical residual is therefore the honest number for the mission.
  2. **6-DOF fit** (full rotation, Kabsch/SVD): if the whole LIO estimate is tilted by a small pitch
     relative to the truth, the 4-DOF fit hides it in the vertical residual while the 6-DOF fit
     removes it.  Round 7 measured exactly that: A2 31-37 mm with the 4-DOF fit collapsing to 1.7-2.2 mm
     once one rotation was allowed, i.e. a 2.35-2.61 deg pitch of the entire estimate.  Printing both
     numbers, and the fitted rotation's axis, is what makes the difference attributable rather than
     mysterious.

usage: tools/lio-vs-truth.py --tap tap.csv --odom lio.csv [--window T0,T1]
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid/src")
from common import frames          # noqa: E402


def read_tap(path):
    t, p, q = [], [], []
    with open(path) as fh:
        if not fh.readline().startswith("t_us,"):
            raise SystemExit("%s is not a tap csv" % path)
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 14:
                continue
            t.append(int(f[0]))
            p.append([float(f[2]), float(f[3]), float(f[4])])          # NED
            q.append([float(f[7]), float(f[8]), float(f[9]), float(f[10])])   # w,x,y,z body FRD->NED
    t = np.array(t, dtype=np.int64)
    p_w = np.array([frames.ned_to_enu(x) for x in p])
    q_w = np.array([frames.wire_to_world([0, 0, 0], [0, 0, 0], qq)[2] for qq in q])
    return t, p_w, q_w


def read_odom(path):
    t, p, q = [], [], []
    with open(path) as fh:
        head = fh.readline()
        if not head.startswith("t_us,"):
            raise SystemExit("%s is not an odom csv" % path)
        for line in fh:
            f = line.strip().split(",")
            if len(f) < 14:
                continue
            t.append(int(f[0]))
            p.append([float(f[1]), float(f[2]), float(f[3])])
            q.append([float(f[4]), float(f[5]), float(f[6]), float(f[7])])
    return np.array(t, dtype=np.int64), np.array(p), np.array(q)


def match(t_a, t_b, tol_us=20000):
    """Nearest-sample pairing; returns index arrays into a and b."""
    idx = np.clip(np.searchsorted(t_b, t_a), 0, len(t_b) - 1)
    left = np.clip(idx - 1, 0, len(t_b) - 1)
    take_left = np.abs(t_b[left] - t_a) < np.abs(t_b[idx] - t_a)
    best = np.where(take_left, left, idx)
    ok = np.abs(t_b[best] - t_a) <= tol_us
    return np.flatnonzero(ok), best[ok]


def fit_yaw_translation(a, b):
    """Find yaw and translation taking L points `a` onto W points `b` (both (N,3))."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    # yaw from the horizontal cross-covariance
    num = np.sum(aa[:, 0] * bb[:, 1] - aa[:, 1] * bb[:, 0])
    den = np.sum(aa[:, 0] * bb[:, 0] + aa[:, 1] * bb[:, 1])
    yaw = math.atan2(num, den)
    R = frames.R_z(yaw)
    t = cb - R @ ca
    return R, t, yaw


def fit_full_rotation(a, b):
    """Kabsch: the rotation (no reflection) taking a onto b, plus the translation."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    H = aa.T @ bb
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    R = Vt.T @ D @ U.T
    t = cb - R @ ca
    return R, t


def rot_report(R):
    """(roll, pitch, yaw) of a rotation matrix, degrees, in the W/FLU sense."""
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    axis = None
    # the 6-DOF fit's excess over the 4-DOF one is what round 7 saw as a "tilt of the whole estimate"
    c = (R[0, 0] + R[1, 1] + R[2, 2] - 1.0) / 2.0
    ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    if ang > 1e-9:
        ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        n = np.linalg.norm(ax)
        if n > 1e-12:
            axis = ax / n
    return roll, pitch, yaw, ang, axis


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--tap", required=True)
    ap.add_argument("--odom", required=True)
    ap.add_argument("--tol-us", type=int, default=20000)
    ap.add_argument("--window", default=None, help="T0,T1 in seconds of the tap's time base")
    ap.add_argument("--max-h-rms-mm", type=float, default=20.0)
    ap.add_argument("--max-v-rms-mm", type=float, default=50.0)
    ap.add_argument("--max-drift-mm", type=float, default=50.0,
                    help="parked case: allowed LIO excursion from its own mean, mm")
    ap.add_argument("--write-calib", default=None,
                    help="write the fitted L->F transform (R_WL, t_WL) as json for the FSM.  Only "
                         "meaningful when the trajectory identifies a rotation: it refuses otherwise, "
                         "because a parked or straight-line window would silently hand the FSM a yaw "
                         "that is 90 or 180 degrees wrong")
    args = ap.parse_args()

    t_tap, p_tap, q_tap = read_tap(args.tap)
    t_lio, p_lio, q_lio = read_odom(args.odom)
    print("tap samples : %d (%.3f s)" % (len(t_tap), (t_tap[-1] - t_tap[0]) * 1e-6))
    print("lio samples : %d (%.3f s)" % (len(t_lio), (t_lio[-1] - t_lio[0]) * 1e-6))
    if len(t_lio) < 10:
        print("FAIL: not enough LIO samples")
        return 1

    if args.window:
        t0, t1 = (float(x) * 1e6 + t_tap[0] for x in args.window.split(","))
        keep = (t_tap >= t0) & (t_tap <= t1) & (t_lio >= t0) & (t_lio <= t1)
        p_tap, t_tap = p_tap[keep], t_tap[keep]
        p_lio, t_lio = p_lio[keep], t_lio[keep]
        print("window      : %.3f .. %.3f s -> %d tap, %d lio samples"
              % (args.window.split(",")[0], args.window.split(",")[1], len(t_tap), len(t_lio)))

    ia, ib = match(t_tap, t_lio, args.tol_us)
    print("matched     : %d pairs within %.0f ms" % (len(ia), args.tol_us / 1000.0))
    if len(ia) < 50:
        print("FAIL: too few matched samples -- are both recordings from the same run?")
        return 1
    w = p_tap[ia]          # truth in W
    l = p_lio[ib]          # LIO in L

    # ---- what can this recording identify?  THE TRUTH decides, not the estimate.
    #
    # MEASURED 2026-10-13: an earlier version measured the trajectory's shape on the LIO's own path.  A
    # shove test that moved the vehicle not at all (the default ground behaviour zeroes velocity and
    # gyro on the ground) therefore looked "shaped" -- because the LIO DRIFTED 2.7 m -- and the tool
    # happily reported a yaw of -29 deg and a 7.6 deg "tilt" from a stationary vehicle.  An estimator's
    # own drift is not information about what the data can identify.
    c_t = w - w.mean(axis=0)
    # singular values of an (N,3) cloud are sqrt(N) times the per-axis spread: normalise them, or a
    # 0.2 m path prints as "16.4 m" (MEASURED 2026-10-13 -- the arena check below is what caught it)
    sv_t = np.linalg.svd(c_t, compute_uv=False) / math.sqrt(max(1, len(c_t)))
    truth_extent = float(sv_t[0])
    truth_h_extent = float(np.max(np.linalg.norm(c_t[:, :2], axis=1)))

    if truth_h_extent < 0.010 and float(np.max(np.abs(c_t[:, 2]))) < 0.010:
        # PARKED: the truth is a point, so this recording measures DRIFT, not tracking
        lc = l - l.mean(axis=0)
        max_exc = float(np.max(np.linalg.norm(lc, axis=1)))
        rms = float(np.sqrt(np.mean(np.sum(lc ** 2, axis=1))))
        span = l.max(axis=0) - l.min(axis=0)
        dur = (t_tap[ia][-1] - t_tap[ia][0]) * 1e-6
        print()
        print("PARKED case: the truth did not move (horizontal extent %.4f m over %.1f s)"
              % (truth_h_extent, dur))
        print("   LIO drift: max excursion %.2f mm, rms %.2f mm" % (max_exc * 1e3, rms * 1e3))
        print("   LIO span : E %.2f mm  N %.2f mm  U %.2f mm" % tuple(span * 1e3))
        print("   no yaw or tilt is reported: a stationary truth cannot identify either")
        ok = (max_exc * 1e3) <= args.max_drift_mm
        print()
        print("[%s] drift max excursion %.2f mm (<= %.1f)" % ("PASS" if ok else "FAIL", max_exc * 1e3,
                                                               args.max_drift_mm))
        print("GATE: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1

    R4, t4, yaw = fit_yaw_translation(l, w)
    r4 = (R4 @ l.T).T + t4 - w
    h_rms4 = float(np.sqrt(np.mean(np.sum(r4[:, :2] ** 2, axis=1))))
    v_rms4 = float(np.sqrt(np.mean(r4[:, 2] ** 2)))
    # did the truth stay inside the mapped arena?  A shove test that slides the model through a wall
    # (SITL has no walls) leaves the map and diverges the LIO -- measured 109 m of travel once, with
    # 14.6 m of "error" that says nothing about the estimator.
    out_of_arena = float(np.max(np.abs(w[:, :2]))) if len(w) else 0.0
    if out_of_arena > 4.5:
        print()
        print("INVALID RUN: the truth left the arena (max |x|,|y| = %.2f m, the walls are at 4.5 m)."
              % out_of_arena)
        print("   SITL has no wall collisions, so the vehicle slid out of the mapped world; the LIO's")
        print("   error here measures the test, not the estimator.")
        return 1
    print()
    print("4-DOF fit (yaw + translation -- the alignment the mission uses)")
    print("   yaw        %+.3f deg   t = [%+.4f %+.4f %+.4f] m" % (math.degrees(yaw), t4[0], t4[1],
                                                                   t4[2]))
    print("   horizontal rms  %7.2f mm" % (h_rms4 * 1e3))
    print("   vertical   rms  %7.2f mm  (mean %+7.2f mm removed)" % (v_rms4 * 1e3,
                                                                     float(np.mean(r4[:, 2])) * 1e3))

    # ---- is the trajectory SHAPED enough to identify a rotation at all?  (from the TRUTH, normalized)
    c = w - w.mean(axis=0)
    sv = np.linalg.svd(c, compute_uv=False) / math.sqrt(max(1, len(c)))
    extent = sv[0] if sv[0] > 0 else 1.0
    shape2 = float(sv[1] / extent)      # 0 = a straight line, ~1 = a plane with equal axes
    shape3 = float(sv[2] / extent)      # 0 = planar, >0 = genuinely 3D
    print("   truth trajectory: spread %.3f %.3f %.3f m (per-axis sigma), max excursion %.3f m"
          % (sv[0], sv[1], sv[2], float(np.max(np.linalg.norm(c, axis=1)))))
    print("                     shape2 %.3f, shape3 %.3f" % (shape2, shape3))
    identifiable_yaw = shape2 > 0.10
    identifiable_tilt = shape2 > 0.20
    print("   yaw identifiable: %s   tilt identifiable: %s"
          % (identifiable_yaw, identifiable_tilt))
    if not identifiable_yaw:
        print("   NOTE: the path is a line, so its yaw is only determined up to 180 deg")

    R6, t6 = fit_full_rotation(l, w)
    r6 = (R6 @ l.T).T + t6 - w
    rms6 = float(np.sqrt(np.mean(np.sum(r6 ** 2, axis=1))))
    roll, pitch, yaw6, ang, axis = rot_report(R6 @ R4.T)     # the EXCESS rotation over the 4-DOF fit
    print()
    print("6-DOF fit (full rotation, Kabsch)")
    if identifiable_tilt:
        print("   excess rotation over the 4-DOF fit: %.3f deg about [%s]" % (
            ang, ", ".join("%+.3f" % x for x in axis) if axis is not None else "n/a"))
        print("   that excess as rpy: roll %+.3f, pitch %+.3f, yaw %+.3f deg" % (roll, pitch, yaw6))
        print("   3-DOF rms       %7.2f mm   (round 7 measured 5.0-6.4 mm with the older code)" % (
            rms6 * 1e3))
    else:
        print("   NOT REPORTED: this trajectory cannot identify a rotation (shape2 %.3f <= 0.20)." % shape2)
        print("   The fit returned %.3f deg about [%s], which is the unconstrained axis, not a tilt."
              % (ang, ", ".join("%+.3f" % x for x in axis) if axis is not None else "n/a"))
        print("   A tilt measurement needs a path with real 2-D extent -- the mission square has it.")

    ok = (h_rms4 * 1e3 <= args.max_h_rms_mm) and (v_rms4 * 1e3 <= args.max_v_rms_mm)
    if args.write_calib:
        if not identifiable_yaw:
            print()
            print("REFUSING to write a calibration: this trajectory cannot identify a rotation "
                  "(shape2 %.3f <= 0.10).  A parked or straight-line window would give the FSM a yaw "
                  "that is 90 or 180 degrees wrong, and the aircraft would fly the mission rotated -- "
                  "which is exactly what a 90 deg error did on 2026-10-13." % shape2)
            return 1
        import json
        with open(args.write_calib, "w") as fh:
            json.dump({"R_WL": R4.tolist(), "t_WL": t4.tolist(),
                       "yaw_deg": math.degrees(math.atan2(R4[1, 0], R4[0, 0])),
                       "n_samples": int(len(ia)), "rms_m": float(np.sqrt(np.mean(np.sum(r4 ** 2,
                                                                                        axis=1)))),
                       "source": os.path.basename(args.tap), "shape2": shape2}, fh, indent=1)
        print()
        print("wrote %s: L->F yaw %+.3f deg, t = [%+.4f %+.4f %+.4f] m"
              % (args.write_calib, math.degrees(math.atan2(R4[1, 0], R4[0, 0])), t4[0], t4[1], t4[2]))
    print()
    print("[%s] horizontal rms %.2f mm (<= %.1f)" % ("PASS" if h_rms4 * 1e3 <= args.max_h_rms_mm
                                                      else "FAIL", h_rms4 * 1e3, args.max_h_rms_mm))
    print("[%s] vertical rms   %.2f mm (<= %.1f)" % ("PASS" if v_rms4 * 1e3 <= args.max_v_rms_mm
                                                      else "FAIL", v_rms4 * 1e3, args.max_v_rms_mm))
    print("GATE: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
