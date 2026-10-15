#!/usr/bin/env python3
"""Measure the ACTUAL rate of every stream in one run, from that run's own files."""
import csv
import os
import sys

run = sys.argv[1]
print(f"run: {run}")
print(f"{'stream':<38}{'rows':>9}{'span [s]':>10}{'rate [Hz]':>11}")


def csv_rate(path, tcol_us, label, skip_header=True):
    if not os.path.exists(path):
        print(f"{label:<38}{'--':>9}{'missing':>10}")
        return
    n = 0
    t0 = t1 = None
    with open(path, newline="", encoding="utf-8", errors="replace") as fh:
        r = csv.reader(fh)
        head = next(r, None)
        idx = 0
        if head and tcol_us in head:
            idx = head.index(tcol_us)
        elif head:
            pass
        for row in r:
            try:
                t = float(row[idx])
            except (ValueError, IndexError):
                continue
            if t0 is None:
                t0 = t
            t1 = t
            n += 1
    if n < 2:
        print(f"{label:<38}{n:>9}{'--':>10}")
        return
    span = (t1 - t0) / 1e6
    print(f"{label:<38}{n:>9}{span:>10.2f}{n/span:>11.2f}")


csv_rate(f"{run}/tap.csv", "t_us", "truth tap (UDP -> CSV, NED 1200 Hz)")
csv_rate(f"{run}/lio.csv", "t_us", "/lio/robo/odom -> lio.csv (L, zero twist)")
csv_rate(f"{run}/traj.csv", "t_us", "/planner/traj -> traj.csv (L, Path)")
csv_rate(f"{run}/setpoints.csv", "t_us", "setpoint 84 -> setpoints.csv (F)")
csv_rate(f"{run}/cloud.csv", "t_us", "/lio/cloud_world -> cloud.csv")

# the inbound external-nav records as the FCU logged them
try:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(f"{run}/flight.BIN")
    counts = {}
    t0 = t1 = None
    while True:
        msg = m.recv_match(type=["VISP", "VISV"])
        if msg is None:
            break
        d = msg.to_dict()
        k = d["mavpackettype"]
        counts[k] = counts.get(k, 0) + 1
        if t0 is None:
            t0 = d["TimeUS"]
        t1 = d["TimeUS"]
    span = (t1 - t0) / 1e6
    for k, n in counts.items():
        print(f"{('FCU log ' + k + ' (331 as fused)'):<38}{n:>9}{span:>10.2f}{n/span:>11.2f}")
except Exception as exc:  # pragma: no cover
    print("FCU log rate: could not read:", exc)
