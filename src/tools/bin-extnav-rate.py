#!/usr/bin/env python3
"""What the FCU was ACTUALLY fed, measured from the FCU's own log rather than from our intent.

The FSM streams to the FCU from its own loop (GATES["rate_hz"], 50 Hz), sampling the newest LIO message
each tick, so the rate the FCU sees is not the LIO's output rate and not the recorder's rate.  The only
witness that cannot be argued with is the flight log: count the ODOMETRY (331) messages the firmware
received and report their rate and spacing.

MEASURED 2026-10-13 (missionA-imu200a): the LIO published /lio/robo/odom at 200.1 Hz, this tool reports
the ODOMETRY rate the FCU received -- the two are different numbers and only this one describes the EKF's
input.

usage: tools/bin-extnav-rate.py <flight.BIN> [--name ODOMETRY]
"""
from __future__ import annotations

import argparse
import sys

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--name", default="ODOMETRY", help="MAVLink message name (331 = ODOMETRY)")
    ap.add_argument("--window", default="", help="T0,T1 seconds of log time, optional")
    args = ap.parse_args()

    from pymavlink import mavutil
    mlog = mavutil.mavlink_connection(args.bin)
    name = args.name
    t = []
    while True:
        m = mlog.recv_match(type=name, blocking=False)
        if m is None:
            break
        stamp = getattr(m, "time_usec", None)
        if stamp is not None:
            t.append(int(stamp))
    if not t:
        print("no %s messages in %s" % (name, args.bin))
        return 1
    t = np.array(t, dtype=np.int64)
    if args.window:
        a, b = [float(x) for x in args.window.split(",")]
        t0 = t[0] + int(a * 1e6)
        t1 = t[0] + int(b * 1e6)
        t = t[(t >= t0) & (t <= t1)]
        if len(t) < 2:
            print("window %s leaves %d samples" % (args.window, len(t)))
            return 1
    dt = np.diff(t) * 1e-6
    span = (t[-1] - t[0]) * 1e6
    print("%s in %s" % (name, args.bin))
    print("   messages %d over %.1f s -> %.1f Hz" % (len(t), span * 1e-6, (len(t) - 1) / (span * 1e-6)))
    print("   dt median %.3f ms  min %.3f  max %.3f  (missing if the max is a multiple of the median)"
          % (np.median(dt) * 1e3, dt.min() * 1e3, dt.max() * 1e3))
    print("   gaps > 2x median: %d" % int((dt > 2.0 * np.median(dt)).sum()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
