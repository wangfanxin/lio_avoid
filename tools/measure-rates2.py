#!/usr/bin/env python3
"""Two rates that need more than (rows / span): the setpoint loop's period and the cloud's scan rate."""
import csv
import statistics
import sys

run = sys.argv[1]

# setpoints.csv carries t_wall in SECONDS; the loop rate is the median inter-row interval
ts = []
with open(f"{run}/setpoints.csv", newline="") as fh:
    r = csv.DictReader(fh)
    for row in r:
        try:
            ts.append(float(row["t_wall"]))
        except (KeyError, ValueError):
            pass
dts = sorted(b - a for a, b in zip(ts, ts[1:]) if b > a)
if dts:
    med = statistics.median(dts)
    print(f"setpoint 84: {len(ts)} rows, median period {med*1000:.2f} ms -> {1/med:.1f} Hz  "
          f"(p10 {dts[int(0.1*len(dts))]*1000:.2f} ms, p90 {dts[int(0.9*len(dts))]*1000:.2f} ms)")

# cloud.csv is one row per POINT with the scan's stamp repeated: count DISTINCT scans
scans = {}
with open(f"{run}/cloud.csv", newline="") as fh:
    r = csv.DictReader(fh)
    for row in r:
        try:
            scans[int(row["scan"])] = float(row["t_us"])
        except (KeyError, ValueError):
            pass
if len(scans) > 1:
    t = sorted(scans.values())
    span = (t[-1] - t[0]) / 1e6
    print(f"/lio/cloud_world: {len(scans)} distinct scans over {span:.2f} s -> {len(scans)/span:.2f} Hz")
