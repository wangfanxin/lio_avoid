#!/usr/bin/env python3
"""How stale is what the flight controller is steering on?

A position loop that oscillates is usually a loop whose bandwidth is too high for its own delay, so the
delay has to be a number, not a suspicion.  This measures, against the truth:

  * the lag of the FCU's POSITION estimate (XKF1 PN/PE) and of its VELOCITY (VN/VE),
  * the age of the velocity the FSM feeds it -- `bridge` sends the LIO's velocity from `/lio/odom`, which
    is published once per lidar scan (10 Hz), while the position comes from `/lio/robo/odom` at 200 Hz,
  * how often that velocity actually changes (a staircase is not a measurement).

usage: tools/bin-lag.py <flight.BIN> --tap <tap.csv>
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from tools.metrics import read_tap        # noqa: E402


def xkf(path):
    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(path)
    t, p, v = [], [], []
    while True:
        m = mlog.recv_match(type="XKF1", blocking=False)
        if m is None:
            break
        t.append(int(m.TimeUS))
        p.append([m.PN, m.PE])
        v.append([m.VN, m.VE])
    return np.array(t, dtype=np.int64), np.array(p), np.array(v)


def best_lag(t_e, y, t_t, x, lo=-300_000, hi=300_000, step=2000):
    best, bl = None, 0
    for lag in range(lo, hi + 1, step):
        ref = np.interp(t_e + lag, t_t, x)
        r = float(np.sqrt(np.mean((y - ref) ** 2)))
        if best is None or r < best:
            best, bl = r, lag
    return bl * 1e-6, best


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--tap", required=True)
    ap.add_argument("--lo", type=float, default=-0.30)
    ap.add_argument("--hi", type=float, default=0.30)
    args = ap.parse_args()

    t_tap, p_tap = read_tap(args.tap)
    t_ned, p_ned = read_tap(args.tap, wire=True)
    # the truth's NED velocity, differenced from the 1200 Hz position (noiseless, so this is exact enough)
    v_n = np.gradient(p_ned[:, 0], t_ned * 1e-6)
    v_e = np.gradient(p_ned[:, 1], t_ned * 1e-6)
    t_e, p_e, v_e_fcu = xkf(args.bin)
    print("%d XKF1 samples over %.1f s" % (len(t_e), (t_e[-1] - t_e[0]) * 1e-6))
    m = (t_ned >= t_e[0] + int(args.lo * 1e6)) & (t_ned <= t_e[-1] + int(args.hi * 1e6))
    for nm, y, x in (("position N", p_e[:, 0], p_ned[m, 0]),
                     ("position E", p_e[:, 1], p_ned[m, 1]),
                     ("velocity N", v_e_fcu[:, 0], v_n[m]),
                     ("velocity E", v_e_fcu[:, 1], v_e[m])):
        # remove the datum before comparing positions
        y0 = y - y.mean()
        x0 = x - x.mean()
        lag, r = best_lag(t_e, y0, t_ned[m], x0, int(args.lo * 1e6), int(args.hi * 1e6),
                          2000 if "pos" in nm else 2000)
        print("   %-11s best lag %+7.1f ms   residual rms %7.2f %s   (zero-lag rms %7.2f)"
              % (nm, lag * 1e3, r * (1e3 if "pos" in nm else 1e3),
                 "mm" if "pos" in nm else "mm/s",
                 float(np.sqrt(np.mean((y0 - np.interp(t_e, t_ned[m], x0)) ** 2))) * 1e3))
    # the velocity staircase: how often does the FCU's velocity estimate actually change?
    dv = np.linalg.norm(np.diff(v_e_fcu, axis=0), axis=1)
    big = dv > 1e-4
    print("   the FCU's own velocity estimate changes on %.1f %% of its 10 Hz samples (%d of %d)"
          % (100.0 * big.mean(), int(big.sum()), len(big)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
