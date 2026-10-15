#!/usr/bin/env python3
"""Did the flight controller RECEIVE the vision we sent, and did it use it?

Prints, once a second: how many VISP/VISV rows the FCU logged, the vision position it logged, the
truth (SIM2) and its own fused estimate (XKF1), all in NED.  Three different causes look identical from
the outside ("the aircraft drifted while the FCU thought it was fine"):

  * VISP empty                 -> the messages never arrived (bridge, port, or rate)
  * VISP != SIM2               -> they arrived but carry the wrong numbers (frame conversion)
  * VISP == SIM2 but XKF1 lags -> they arrived correctly and the EKF is not using them (gates, noise)

usage: tools/bin-vision-check.py <flight.BIN>
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--every", type=float, default=1.0)
    args = ap.parse_args()

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.bin)
    bucket = defaultdict(lambda: defaultdict(list))
    while True:
        msg = m.recv_match(type=["VISP", "VISV", "SIM2", "XKF1", "NKF1"])
        if msg is None:
            break
        t = msg.get_type()
        us = getattr(msg, "TimeUS", None)
        if us is None:
            continue
        k = int(us * 1e-6 / args.every)
        bucket[k][t].append(msg)

    print("%8s %5s %-24s %-24s %-24s %s" % ("t (s)", "nVISP", "vision position (NED)",
                                            "truth SIM2 (NED)", "EKF XKF1 position (NED)",
                                            "vision PErr/Ign/Q"))
    for k in sorted(bucket):
        b = bucket[k]
        visp = b.get("VISP", [])
        visv = b.get("VISV", [])
        sim2 = b.get("SIM2", [])
        xkf1 = b.get("XKF1", [])
        mid = visp[len(visp) // 2] if visp else None
        vis_pos = ("%+7.3f %+7.3f %+7.3f" % (mid.PX, mid.PY, mid.PZ)) if mid else "--"
        # PErr is the position innovation the EKF saw, Ign says it threw the sample away, Q is the
        # quality we send (100).  Without these three columns "the aircraft drifted" has three possible
        # causes that look identical from outside.
        extra = ("%7.3f %2d %3d / visv %7.3f %2d" % (mid.PErr, mid.Ign, mid.Q,
                                                     visv[len(visv) // 2].VErr if visv else 0.0,
                                                     visv[len(visv) // 2].Ign if visv else 0)) \
            if mid else "--"
        sim_pos = ("%+7.3f %+7.3f %+7.3f" % (sim2[-1].PN, sim2[-1].PE, sim2[-1].PD)) if sim2 else "--"
        ekf_pos = ("%+7.3f %+7.3f %+7.3f" % (xkf1[-1].PN, xkf1[-1].PE, xkf1[-1].PD)) if xkf1 else "--"
        print("%8.1f %5d %-24s %-24s %-24s %s" % (k * args.every, len(visp), vis_pos, sim_pos,
                                                  ekf_pos, extra))
    return 0


if __name__ == "__main__":
    sys.exit(main())
