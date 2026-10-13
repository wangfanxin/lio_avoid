#!/usr/bin/env python3
"""Aggregate a batch of missions into the acceptance table, with its denominators.

Every line is reported as "passed in X of N", because a number without its denominator is not a result
(the rule this project has followed since the first round).  It reads the per-flight `*.metrics.txt`
files that tools/run-mission.sh writes, so nothing is re-derived here: the aggregation and the
measurement are separate steps, and the raw per-flight files stay on disk as the evidence.

usage: tools/batch-summary.py <glob-or-dir> [--glob 'missionA-phaseA-*-flight*.metrics.txt']
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import statistics as st
import sys


def parse(path):
    txt = open(path).read()
    r = {"file": os.path.basename(path)}
    m = re.search(r"A1 route horizontal rms\s+([\d.]+) mm", txt)
    r["A1"] = float(m.group(1)) if m else None
    m = re.search(r"A2 route vertical rms\s+([\d.]+) mm", txt)
    r["A2"] = float(m.group(1)) if m else None
    # A3 was redefined 2026-10-13 (see tools/metrics.py): the interpolated "fair ratio" is gone, the
    # numbers that remain are the delivered-stream wander, its outlier tail and the delivery rate.
    m = re.search(r"wander\s+rms 2nd difference\s+E\s+([\d.]+) N\s+([\d.]+) U\s+([\d.]+) mm", txt)
    r["A3_abs"] = [float(x) for x in m.groups()] if m else None
    m = re.search(r"outliers p99 single step\s+E\s+([\d.]+) N\s+([\d.]+) U\s+([\d.]+) mm", txt)
    r["A3_p99"] = [float(x) for x in m.groups()] if m else None
    m = re.search(r"delivery rate ([\d.]+) Hz", txt)
    r["A3_rate"] = float(m.group(1)) if m else None
    m = re.search(r"\[(PASS|FAIL)\] A3", txt)
    r["A3_pass"] = (m.group(1) == "PASS") if m else None
    m = re.search(r"\[(PASS|FAIL)\] A4", txt)
    r["A4_pass"] = (m.group(1) == "PASS") if m else None
    m = re.search(r"horizontal rms\s+([\d.]+) mm\s+\(<= 50, mean removed\)", txt)
    r["A4"] = float(m.group(1)) if m else None
    m = re.search(r"B1 truth box extents over the route \([^)]*\): E ([\d.]+) m\s+N ([\d.]+) m", txt)
    r["B1"] = [float(x) for x in m.groups()] if m else None
    m = re.search(r"against the legacy bound 1\.26 on N: (PASS|FAIL)", txt)
    r["B1_legacy"] = (m.group(1) == "PASS") if m else None
    legs = re.findall(r"leg \d+:.*?(PASS|FAIL)", txt)
    r["B2_legs"] = legs
    corners = re.findall(r"corner \d+: closest horizontal approach [\d.]+ mm \(<= 100\) (PASS|FAIL)", txt)
    r["B3_corners"] = corners
    m = re.search(r"B4 landing: 'land' in the phase log -> (\w+)", txt)
    r["B4"] = (m.group(1) == "yes") if m else None
    m = re.search(r"phases: ([^\n]+)", txt)
    r["phases"] = m.group(1) if m else ""
    r["completed"] = "done" in (r["phases"] or "")
    return r


def count(values, ok):
    passed = sum(1 for v in values if v is not None and ok(v))
    have = sum(1 for v in values if v is not None)
    return passed, have


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("where", help="a directory or a glob")
    ap.add_argument("--glob", default="*-flight*.metrics.txt")
    args = ap.parse_args()
    pattern = os.path.join(args.where, args.glob) if os.path.isdir(args.where) else args.where
    files = sorted(glob.glob(pattern))
    if not files:
        print("no metrics files matched %s" % pattern)
        return 1

    rows = [parse(f) for f in files]
    n = len(rows)
    print("acceptance table, aggregated from %d flights" % n)
    print("(the per-flight files are the evidence; this file only counts them)")
    print()

    done = [r for r in rows if r["completed"]]
    print("flights completed (FSM reached 'done')   %d of %d" % (len(done), n))

    a1 = [r["A1"] for r in rows]
    a2 = [r["A2"] for r in rows]
    print("A1 route horizontal rms  <= 20 mm        %d of %d   %s"
          % (*count(a1, lambda v: v <= 20.0), _stats(a1, "mm")))
    print("A2 route vertical rms    <= 50 mm        %d of %d   %s"
          % (*count(a2, lambda v: v <= 50.0), _stats(a2, "mm")))

    a3 = [r["A3_pass"] for r in rows]
    print("A3 continuity (delivery + wander + outliers) %d of %d   worst wander E %.3f N %.3f U %.3f mm"
          % (sum(1 for v in a3 if v), n,
             max((r["A3_abs"][0] for r in rows if r["A3_abs"]), default=float("nan")),
             max((r["A3_abs"][1] for r in rows if r["A3_abs"]), default=float("nan")),
             max((r["A3_abs"][2] for r in rows if r["A3_abs"]), default=float("nan"))))
    a4 = [r["A4"] for r in rows]
    print("A4 FCU estimate vs truth  <= 50 mm       %d of %d   %s"
          % (*count(a4, lambda v: v <= 50.0), _stats(a4, "mm")))

    b1e = [r["B1"][0] for r in rows if r["B1"]]
    b1n = [r["B1"][1] for r in rows if r["B1"]]
    print("B1 route box E <= 2.10, N <= 1.30 m      %d of %d   %s / %s"
          % (sum(1 for r in rows if r["B1"] and r["B1"][0] <= 2.10 and r["B1"][1] <= 1.30), n,
             _stats(b1e, "m"), _stats(b1n, "m")))
    print("   (against the legacy N <= 1.26 bound   %d of %d)"
          % (sum(1 for r in rows if r["B1_legacy"]), n))

    b2 = [len([x for x in r["B2_legs"] if x == "PASS"]) for r in rows]
    b2n = [len(r["B2_legs"]) for r in rows]
    print("B2 straightness <= 50 mm per leg         %d of %d legs   (%s of %s across flights)"
          % (sum(b2), sum(b2n), "/".join(str(x) for x in b2), "/".join(str(x) for x in b2n)))

    b3 = [len([x for x in r["B3_corners"] if x == "PASS"]) for r in rows]
    b3n = [len(r["B3_corners"]) for r in rows]
    print("B3 corner arrival <= 100 mm              %d of %d corners (%s of %s across flights)"
          % (sum(b3), sum(b3n), "/".join(str(x) for x in b3), "/".join(str(x) for x in b3n)))

    print("B4 LAND + disarm                         %d of %d"
          % (sum(1 for r in rows if r["B4"]), n))
    print()
    print("raw per flight:")
    for r in rows:
        print("   %-46s A1 %-7s A2 %-7s A4 %-7s B1 %-16s B2 %s/%s B3 %s/%s"
              % (r["file"], r["A1"], r["A2"], r["A4"],
                 "%.3f/%.3f" % tuple(r["B1"]) if r["B1"] else "--",
                 len([x for x in r["B2_legs"] if x == "PASS"]), len(r["B2_legs"]),
                 len([x for x in r["B3_corners"] if x == "PASS"]), len(r["B3_corners"])))
    return 0


def _stats(v, unit):
    v = [x for x in v if x is not None]
    if not v:
        return "(no data)"
    return "min %.3f / median %.3f / max %.3f %s" % (min(v), st.median(v), max(v), unit)


if __name__ == "__main__":
    sys.exit(main())
