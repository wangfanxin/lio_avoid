#!/usr/bin/env python3
"""Cross-check the truth tap against the firmware's own record of the same state.

The tap is our code and the metrics will be built on it, so it must be shown to agree with an
independent channel.  ArduPilot logs `SIM2` at loop rate, from the simulator's own state, with its
own clock (`AP_HAL::micros64()`), so the comparison also answers "are the tap's timestamps the same
clock the autopilot uses?" -- which matters for every TF lookup and every metric window.

  tools/bin-vs-tap.py --csv <tap.csv> --bin <flight.BIN> [--tol-mm 1.0] [--time-tol-ms 1.0]

Prints the sample counts, the clock offset, the per-axis position differences and a PASS/FAIL.
Exit status is the verdict, so it can be used as a gate.
"""
from __future__ import annotations

import argparse
import math
import sys

import numpy as np


def read_tap(path):
    t, p = [], []
    with open(path) as fh:
        header = fh.readline()
        if not header.startswith("t_us,"):
            raise SystemExit("unexpected tap csv header: %r" % header.strip())
        for line in fh:
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            t.append(int(parts[0]))
            p.append([float(parts[2]), float(parts[3]), float(parts[4])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float)


def read_sim2(path, wait_sec=5.0):
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(path)
    t, p, v = [], [], []
    while True:
        msg = m.recv_match(type="SIM2")
        if msg is None:
            break
        r = msg.to_dict()
        t.append(int(r["TimeUS"]))
        p.append([float(r["PN"]), float(r["PE"]), float(r["PD"])])
        v.append([float(r["VN"]), float(r["VE"]), float(r["VD"])])
    return np.array(t, dtype=np.int64), np.array(p, dtype=float), np.array(v, dtype=float)


def nearest_match(t_query, t_ref):
    """For each t_query, the index of the nearest t_ref, plus the absolute time difference."""
    idx = np.searchsorted(t_ref, t_query)
    idx = np.clip(idx, 1, len(t_ref) - 1)
    left = t_ref[idx - 1]
    right = t_ref[idx]
    choose_left = np.abs(t_query - left) <= np.abs(right - t_query)
    best = np.where(choose_left, idx - 1, idx)
    return best, np.abs(t_query - t_ref[best])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--bin", required=True)
    ap.add_argument("--tol-mm", type=float, default=1.0)
    ap.add_argument("--bias-tol-ms", type=float, default=0.2,
                    help="tolerance on the MEDIAN clock offset (bias)")
    ap.add_argument("--drift-tol-ppm", type=float, default=100.0,
                    help="tolerance on the clock slope, parts per million")
    args = ap.parse_args()

    t_tap, p_tap = read_tap(args.csv)
    t_sim, p_sim, v_sim = read_sim2(args.bin)
    print("tap samples : %d  (%.3f s)" % (len(t_tap), (t_tap[-1] - t_tap[0]) * 1e-6))
    print("SIM2 samples: %d  (%.3f s, %.1f Hz)" % (
        len(t_sim), (t_sim[-1] - t_sim[0]) * 1e-6,
        len(t_sim) / max(1e-9, (t_sim[-1] - t_sim[0]) * 1e-6)))
    if len(t_tap) < 10 or len(t_sim) < 10:
        print("not enough samples to compare")
        return 1

    idx, dt_us = nearest_match(t_tap, t_sim)
    keep = dt_us <= 3000          # only compare where the two streams really overlap in time
    n_keep = int(keep.sum())
    # microseconds -> milliseconds, once (an earlier version multiplied by 1000 as well and printed
    # a 2.5 ms period as "2499.0000 ms")
    sim2_period_ms = float(np.median(np.diff(t_sim))) * 1e-3 if len(t_sim) > 1 else 0.0
    print("matched     : %d of %d tap samples within 3 ms of a SIM2 sample" % (n_keep, len(t_tap)))
    if n_keep < 0.5 * len(t_tap):
        print("FAIL: the two streams barely overlap -- wrong BIN, or the tap ran at another time")
        return 1

    signed_dt_ms = (t_tap[keep] - t_sim[idx[keep]]) * 1e-3
    d = (p_tap[keep] - p_sim[idx[keep]]) * 1000.0        # mm
    rms = np.sqrt(np.mean(d * d, axis=0))
    mx = np.max(np.abs(d), axis=0)

    # Clock: the MEDIAN offset is the bias (each stream's own sampling granularity cancels), and the
    # slope of tap time against SIM2 time is the drift.  Max |offset| is NOT a clock test here: SIM2
    # is logged at the scheduler rate (400 Hz measured), so nearest-neighbour differences must spread
    # by about half that period -- 2.5 ms measured, which says nothing about the clock.
    bias_ms = float(np.median(signed_dt_ms))
    spread_ms = float(np.ptp(signed_dt_ms))
    slope = float(np.polyfit(t_sim[idx[keep]], t_tap[keep], 1)[0])
    drift_ppm = (slope - 1.0) * 1e6

    print("clock bias  : median %+.4f ms   (spread %.4f ms, SIM2 period %.4f ms -- the spread is "
          "sampling granularity, not skew)" % (bias_ms, spread_ms, sim2_period_ms))
    print("clock drift : %+.2f ppm (slope of tap time against SIM2 time)" % drift_ppm)
    print("position difference (tap - SIM2), mm:")
    print("   rms  N %8.4f  E %8.4f  D %8.4f" % tuple(rms))
    print("   max  N %8.4f  E %8.4f  D %8.4f" % tuple(mx))
    print("   mean N %8.4f  E %8.4f  D %8.4f" % tuple(np.mean(d, axis=0)))

    ok_pos = float(np.max(mx)) <= args.tol_mm
    ok_bias = abs(bias_ms) <= args.bias_tol_ms
    ok_drift = abs(drift_ppm) <= args.drift_tol_ppm
    print()
    print("[%s] position max %.4f mm (<= %.2f mm)" % ("PASS" if ok_pos else "FAIL",
                                                      float(np.max(mx)), args.tol_mm))
    print("[%s] clock bias %+.4f ms (<= %.2f ms)" % ("PASS" if ok_bias else "FAIL", bias_ms,
                                                     args.bias_tol_ms))
    print("[%s] clock drift %+.2f ppm (<= %.0f ppm)" % ("PASS" if ok_drift else "FAIL", drift_ppm,
                                                        args.drift_tol_ppm))
    print("GATE: %s" % ("PASS" if (ok_pos and ok_bias and ok_drift) else "FAIL"))
    return 0 if (ok_pos and ok_bias and ok_drift) else 1


if __name__ == "__main__":
    sys.exit(main())
