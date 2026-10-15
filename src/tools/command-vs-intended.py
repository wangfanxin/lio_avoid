#!/usr/bin/env python3
"""Did the COMMAND land on the intended corner, and did the aircraft land on the command?

Both questions matter and they are different.  In `phase.csv` every row is written when a phase STARTS, so
the row for `legN` carries, at the same instant, a commanded point and the truth beside it:

   command error = cmd - intended_corner (from worlds/pillars.yaml, in W)
   arrival error = truth - cmd           (the controller's own accuracy)
MEASURED 2026-10-13: the command error was 51 mm in N (a calibration mismatch inside our own chain),
which moved the whole flown box; the arrival error was 8-26 mm.

WHICH ROW CARRIES WHICH CORNER CHANGED IN F1, AND THIS TOOL NOW DETECTS IT INSTEAD OF ASSUMING IT.
`fsm/mission_fsm.py::set_phase` writes `self.cmd` -- the command in force when the phase begins, **in W**.
It is `bridge.send_cmd`'s return value (`frames.l_to_wire(cmd_l.p, calib)`): on RECT24's leg2 row it reads
(2.5027, 0.4021, 1.9872), which IS corner 2 in W; in L that same point would be (0.406, -2.500).  What F1
changed is WHEN the row is written relative to the command:

  PRE-F1   the FSM called `set_phase("legN")` BEFORE its first pump sent the new setpoint, so the row's
           `cmd` is still the corner just reached: legs[i] holds c(i) and the truth in that SAME row is the
           arrival at c(i).                                              (leg2 -> c1, truth at c1)
  POST-F1  the phase comes from the stream's `trajectory_id`, which advances the moment the PLAN BOX
           switches its target, so the row's `cmd` is already the corner being flown TO: legs[i] holds
           c(i+1) and the truth beside it is the arrival at c(i).        (leg2 -> c2, truth at c1)

Both conventions describe the same flight and the same phase WINDOW -- `tools/metrics.py` windows the route
with these rows and is unaffected, and C1 reads the LAST row (cmd and truth both at the final corner).  What
broke was this tool: it assumed the pre-F1 convention and, on a post-F1 run, compared corner k's command
against corner k-1's intended point.  L and W differ by ~90 deg of yaw, so what came out was metre-scale
(worst 2002.7 mm -- c2's command (2.5, 0.4) read against c1's intended (0.5, 0.4)) and it looked like a
flight defect.  A tool that prints 2002 mm as an arrival error is worse than no tool in a project whose
method is "the number in the file is the evidence".

SO: this tool evaluates BOTH conventions on the file it is given, uses the one that fits, SAYS WHICH, and
**refuses to report a number** if neither fits within `--max-err-mm` -- naming the frame/order hypothesis
instead.

usage: tools/command-vs-intended.py <glob-of-phase.csv> [...] [--max-err-mm 100]
"""
from __future__ import annotations

import argparse
import glob
import sys

import numpy as np

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid/src")
from tools.metrics import read_phase          # noqa: E402

# worlds/pillars.yaml, mission.corners -- the intended geometry, in W
INTENDED = [(0.5, 0.4), (2.5, 0.4), (2.5, 1.6), (0.5, 1.6)]
PRE_F1 = "pre-F1: the row's cmd is the corner REACHED, truth from the same row"
POST_F1 = "post-F1: the row's cmd is the corner flown TO, truth from the NEXT row"


def rows_for(legs, convention):
    """[(corner_index, cmd_xy, truth_xy)] under one convention -- one entry per corner arrival."""
    out = []
    for i, r in enumerate(legs):
        if i == 0:
            continue                                    # leg1 is the approach from the pad
        cmd = np.array([float(r["cmd_x"]), float(r["cmd_y"])])
        if convention == "post":
            if i + 1 >= len(legs):
                continue
            nxt = legs[i + 1]
            tru = np.array([float(nxt["truth_x"]), float(nxt["truth_y"])])
            out.append((i % len(INTENDED), cmd, tru))
        else:
            if i - 1 >= len(INTENDED):
                continue
            tru = np.array([float(r["truth_x"]), float(r["truth_y"])])
            out.append((i - 1, cmd, tru))
    return out


def pooled(files, convention):
    """(cmd_err, arr_err, per-corner dict) under one convention."""
    cmd_err, arr_err, per = [], [], {k: ([], []) for k in range(len(INTENDED))}
    for f in files:
        legs = [r for r in read_phase(f) if r["phase"].startswith("leg")]
        for k, cmd, tru in rows_for(legs, convention):
            cmd_err.append(cmd - np.array(INTENDED[k]))
            arr_err.append(tru - cmd)
            per[k][0].append(cmd - np.array(INTENDED[k]))
            per[k][1].append(tru - cmd)
    c = np.array(cmd_err).reshape(-1, 2)
    a = np.array(arr_err).reshape(-1, 2)
    return c, a, per


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("pats", nargs="*", default=[])
    ap.add_argument("--max-err-mm", type=float, default=100.0,
                    help="a command error larger than this means the CONVENTION is wrong, not the flight")
    args = ap.parse_args()
    pats = args.pats or ["/home/fanxin/lio_avoid/logs/missionA-*/phase.csv"]
    files = []
    for p in pats:
        files.extend(sorted(glob.glob(p)))
    if not files:
        print("no phase logs matched %s" % pats, file=sys.stderr)
        return 2

    fits = {}
    for conv in ("pre", "post"):
        c, a, per = pooled(files, conv)
        fits[conv] = (c, a, per, float(np.abs(c).max()) * 1e3 if len(c) else float("inf"))
    print("%d phase log(s)" % len(files))
    for conv, label in (("pre", PRE_F1), ("post", POST_F1)):
        print("   convention %-4s worst command error %9.1f mm   %s" % (conv, fits[conv][3], label))
    conv = "pre" if fits["pre"][3] <= fits["post"][3] else "post"
    cmd_err, arr_err, per, worst = fits[conv]
    print("   USING '%s' (worst command error %.1f mm; the other convention would report %.1f mm)"
          % (conv, worst, fits["post" if conv == "pre" else "pre"][3]))
    if worst > args.max_err_mm:
        print()
        print("   REFUSING TO REPORT A NUMBER: neither convention puts the command within %.0f mm of the "
              "intended corner (best: %.1f mm under '%s').  A metre-scale 'command error' is a FRAME or an "
              "ORDER mistake in this tool, not a flight defect -- the candidates are: the phase row's cmd_* "
              "is in L rather than W, or the leg-to-corner mapping is off by one.  Nothing is printed as an "
              "error until that is settled." % (args.max_err_mm, worst, conv))
        return 1

    print("   corner   command minus intended (E,N)      truth minus command (E,N)")
    for k in range(len(INTENDED)):
        c = np.array(per[k][0]).reshape(-1, 2)
        a = np.array(per[k][1]).reshape(-1, 2)
        if not len(c):
            continue
        print("   c%d      %+7.1f %+7.1f mm  (sd %4.1f %4.1f)   %+7.1f %+7.1f mm  (sd %4.1f %4.1f)"
              % (k + 1, c[:, 0].mean() * 1e3, c[:, 1].mean() * 1e3, c[:, 0].std() * 1e3,
                 c[:, 1].std() * 1e3, a[:, 0].mean() * 1e3, a[:, 1].mean() * 1e3, a[:, 0].std() * 1e3,
                 a[:, 1].std() * 1e3))
    print("   mean over corners: command error E %+.1f N %+.1f mm   |%.1f| mm;  "
          "arrival error E %+.1f N %+.1f mm   |%.1f| mm"
          % (cmd_err[:, 0].mean() * 1e3, cmd_err[:, 1].mean() * 1e3,
             np.linalg.norm(cmd_err.mean(axis=0)) * 1e3, arr_err[:, 0].mean() * 1e3,
             arr_err[:, 1].mean() * 1e3, np.linalg.norm(arr_err.mean(axis=0)) * 1e3))
    print("   worst command error %.1f mm, worst arrival error %.1f mm"
          % (np.abs(cmd_err).max() * 1e3, np.linalg.norm(arr_err, axis=1).max() * 1e3))
    return 0


if __name__ == "__main__":
    sys.exit(main())
