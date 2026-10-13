#!/usr/bin/env python3
"""Triage a flight log: why did the mode change, and what did the FCU say about it?

Reads a dataflash BIN and prints, in time order, every mode change, every statustext, and the
EKF/position summary around them.  Written because "the vehicle drifted and landed" has several
possible causes (EKF failsafe, GUIDED timeout, pilot input, battery) and the log names the one that
happened rather than the one I guessed.

usage: tools/bin-triage.py <flight.BIN> [--window T0,T1]
"""
from __future__ import annotations

import argparse
import sys
from collections import defaultdict


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bin")
    ap.add_argument("--window", default=None, help="T0,T1 seconds of log time")
    ap.add_argument("--only", default="MODE,MSG,ARM,EV",
                    help="message types to show (comma separated); the rest are counted only")
    ap.add_argument("--series", default="XKF1,SIM2,CTUN",
                    help="types to show as a decimated time series at 0.5 Hz")
    args = ap.parse_args()

    show = {s.strip() for s in args.only.split(",") if s.strip()}
    series = {s.strip() for s in args.series.split(",") if s.strip()}

    from pymavlink import mavutil
    m = mavutil.mavlink_connection(args.bin)
    events = []
    counts = defaultdict(int)
    first_us = None
    t0 = t1 = None
    if args.window:
        a, b = (float(x) for x in args.window.split(","))
        t0, t1 = a * 1e6, b * 1e6

    while True:
        msg = m.recv_match()
        if msg is None:
            break
        t = msg.get_type()
        counts[t] += 1
        us = getattr(msg, "TimeUS", None)
        if us is None:
            continue
        if first_us is None:
            first_us = us
        if t0 is not None and not (t0 <= us <= t1):
            continue
        sec = us * 1e-6
        if t in show:
            if t == "MODE":
                events.append((sec, "MODE     -> mode %d" % msg.Mode))
            elif t == "MSG":
                events.append((sec, "MSG      %s" % msg.Message))
            elif t == "EV":
                events.append((sec, "EV       id=%d" % msg.Id))
            elif t == "ARM":
                events.append((sec, "ARM      %s" % ("armed" if msg.ArmState else "DISARMED")))
        elif t in series and int(sec * 2) % 10 == 0:
            if t == "XKF1":
                events.append((sec, "XKF1     rpy %+7.2f %+7.2f %+7.2f | P %+7.3f %+7.3f %+7.3f | "
                                     "V %+6.2f %+6.2f %+6.2f"
                              % (msg.Roll, msg.Pitch, msg.Yaw, msg.PN, msg.PE, msg.PD,
                                 msg.VN, msg.VE, msg.VD)))
            elif t == "SIM2":
                events.append((sec, "SIM2     P %+7.3f %+7.3f %+7.3f (truth)"
                              % (msg.PN, msg.PE, msg.PD)))
            elif t == "CTUN":
                events.append((sec, "CTUN     alt %+7.3f dalt %+6.2f thr %4.0f%%"
                              % (msg.Alt, msg.DAlt, msg.ThO * 100.0)))

    print("messages in the log: %s" % dict(sorted(counts.items(), key=lambda kv: -kv[1])[:16]))
    print()
    # decimate by TIME, not by a counter: SIM2 is logged at the model rate (400 Hz), so a modulo test
    # let ~200 rows per step through and buried every event
    last_printed = defaultdict(lambda: -1e9)
    for sec, text in sorted(events):
        t = text.split()[0]
        if t in ("MODE", "MSG", "ARM", "EV") or sec - last_printed[t] >= 0.5:
            last_printed[t] = sec
            print("[%8.2f] %s" % (sec, text))
    return 0


if __name__ == "__main__":
    sys.exit(main())
