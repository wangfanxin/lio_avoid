#!/usr/bin/env python3
"""Did the COMMAND land on the intended corner, and did the aircraft land on the command?

Both questions matter and they are different.  In `phase.csv` every row is written when a phase STARTS,
so the row for `legN` (N >= 2) carries, at the same instant:
   cmd_*    the point that was commanded during legN  = the corner the aircraft was sent to
   truth_*  where the aircraft really was              = its arrival at that corner
so the two errors are, per corner:
   command error = cmd - intended_corner (from worlds/pillars.yaml)
   arrival error = truth - cmd           (the controller's own accuracy)
MEASURED 2026-10-13: the command error was 51 mm in N (a calibration mismatch inside our own chain),
which moved the whole flown box; the arrival error was 8-26 mm.

usage: _stage/command-vs-intended.py <glob-of-phase.csv> [...]
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from tools.metrics import read_phase          # noqa: E402

# worlds/pillars.yaml, mission.corners -- the intended geometry, in W
INTENDED = [(0.5, 0.4), (2.5, 0.4), (2.5, 1.6), (0.5, 1.6)]


def main() -> int:
    pats = sys.argv[1:] or ["/home/fanxin/lio_avoid/logs/missionA-*/phase.csv"]
    files = []
    for p in pats:
        files.extend(sorted(glob.glob(p)))
    cmd_err, arr_err = {i: [] for i in range(4)}, {i: [] for i in range(4)}
    for f in files:
        ph = read_phase(f)
        legs = [r for r in ph if r["phase"].startswith("leg")]
        for i, r in enumerate(legs):
            if i == 0 or i - 1 >= len(INTENDED):
                continue                       # leg1 is the approach from the pad
            c = INTENDED[i - 1]
            cmd = np.array([float(r["cmd_x"]), float(r["cmd_y"])])
            tru = np.array([float(r["truth_x"]), float(r["truth_y"])])
            cmd_err[i - 1].append(cmd - np.array(c))
            arr_err[i - 1].append(tru - cmd)
    print("%d phase logs" % len(files))
    print("   corner   command minus intended (E,N)      truth minus command (E,N)")
    for i in range(4):
        if not cmd_err[i]:
            continue
        c = np.array(cmd_err[i])
        a = np.array(arr_err[i])
        print("   c%d      %+7.1f %+7.1f mm  (sd %4.1f %4.1f)   %+7.1f %+7.1f mm  (sd %4.1f %4.1f)"
              % (i + 1, c[:, 0].mean() * 1e3, c[:, 1].mean() * 1e3, c[:, 0].std() * 1e3,
                 c[:, 1].std() * 1e3, a[:, 0].mean() * 1e3, a[:, 1].mean() * 1e3, a[:, 0].std() * 1e3,
                 a[:, 1].std() * 1e3))
    allc = np.array([v for i in range(4) for v in cmd_err[i]])
    alla = np.array([v for i in range(4) for v in arr_err[i]])
    if len(allc):
        print("   mean over corners: command error E %+.1f N %+.1f mm   |%.1f| mm;  "
              "arrival error E %+.1f N %+.1f mm   |%.1f| mm"
              % (allc[:, 0].mean() * 1e3, allc[:, 1].mean() * 1e3,
                 np.linalg.norm(allc.mean(axis=0)) * 1e3, alla[:, 0].mean() * 1e3,
                 alla[:, 1].mean() * 1e3, np.linalg.norm(alla.mean(axis=0)) * 1e3))
        print("   worst command error %.1f mm, worst arrival error %.1f mm"
              % (np.abs(allc).max() * 1e3, np.linalg.norm(alla, axis=1).max() * 1e3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
