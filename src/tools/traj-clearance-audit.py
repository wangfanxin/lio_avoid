#!/usr/bin/env python3
"""Does `traj.csv`'s `clr_m` mean what its name says?  Recompute it in W and say which frame it was in.

WHY THIS EXISTS (2026-10-15).  `tools/record-traj.py` handed the L-frame path straight to the W-frame
pillar clearance, so `logs/missionA-RECT6-clean-square/traj.csv` carries
`clr_m = -0.297153` for a plan whose real clearance is `+0.600 m` -- the world's own designed worst case,
and the number the same recording gives as soon as the L -> W conversion the FSM and the producer both use
is applied.  A recorder column that is 0.9 m wrong is worse than no column, so this tool is the check that
it cannot come back:

  * it reads the recorded `clr_m` (BEFORE) and recomputes the clearance from the recorded POINTS (AFTER),
    converting them to W **only if the file says they are in L** (a `frame` column since the fix; no column
    = an older recording = L);
  * it prints the calibration it used, and one point before/after, so the conversion is visible;
  * it compares the AFTER value with `metrics.txt`'s own C2 (the flight's minimum pillar-surface distance,
    measured on the truth inside tools/metrics.py) and states the tolerance;
  * on a NEW recording -- where the recorder already converted -- BEFORE and AFTER must agree exactly, which
    is what makes this an audit of the live path and not just of the arithmetic.

usage: tools/traj-clearance-audit.py <run-dir> [--world W] [--calib C] [--tol-m 0.15] [--all]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
#: the PROJECT root, one level above the code: worlds/, params/, docs/, frozen/, rviz/ live
#: there, not next to the code (2026-10-16: every code directory moved under src/).
PROJ = os.path.abspath(os.path.join(ROOT, ".."))
sys.path.insert(0, ROOT)


def load_world(path):
    spec = importlib.util.spec_from_file_location("world_info_clr", os.path.join(HERE, "world-info.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.PillarWorld.load(path)


def load_calib(path):
    from common import frames
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float)), d


def metrics_c2(run):
    """`metrics.txt`'s C2 line: (minimum pillar-surface distance, n_inside, n_samples, foot)."""
    p = os.path.join(run, "metrics.txt")
    if not os.path.exists(p):
        return None
    txt = open(p).read()
    m = re.search(r"minimum pillar-surface distance ([-\d.]+) m;\s+(\d+) of (\d+) samples inside the "
                  r"([\d.]+) m footprint", txt)
    if not m:
        return None
    return float(m.group(1)), int(m.group(2)), int(m.group(3)), float(m.group(4))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="a run dir (with traj.csv and traj.csv.points.csv)")
    ap.add_argument("--world", default=os.path.join(PROJ, "worlds", "pillars.yaml"))
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--tol-m", type=float, default=0.15,
                    help="how far the plan's own clearance and the FLIGHT's minimum may legitimately "
                         "differ (m).  They differ only by how far the aircraft left the plan, which the "
                         "same run's B2 measures per leg; RECT6's worst is 0.1246 m, hence 0.15")
    ap.add_argument("--all", action="store_true", help="print every distinct message, not the first 5")
    args = ap.parse_args()

    tj = os.path.join(args.run, "traj.csv")
    pts = tj + ".points.csv"
    for p in (tj, pts):
        if not os.path.exists(p):
            print("no %s -- was tools/record-traj.py running?" % p, file=sys.stderr)
            return 1
    hdr = np.genfromtxt(tj, delimiter=",", names=True, dtype=None, encoding="utf-8")
    P = np.genfromtxt(pts, delimiter=",", names=True, dtype=None, encoding="utf-8")
    hdr = np.atleast_1d(hdr)
    P = np.atleast_1d(P)
    if "frame" in P.dtype.names:
        seen = sorted({str(x) for x in np.atleast_1d(P["frame"])})
        frame = seen[0] if len(seen) == 1 else "MIXED(%s)" % ",".join(seen)
    else:
        frame = "L"
    calib, cdict = load_calib(args.calib)
    world = load_world(args.world) if os.path.exists(args.world) else None
    print("run        : %s" % args.run)
    print("recording  : %d messages, %d point rows, points recorded in frame %s%s"
          % (len(hdr), len(P), frame,
             "  (no `frame` column: written before the 2026-10-15 fix, so L)" if frame == "L" and
             "frame" not in P.dtype.names else ""))
    print("calibration: %s" % args.calib)
    print("             yaw %+.4f deg, t_WL = [%+.5f %+.5f %+.5f] m  (%s)"
          % (cdict.get("yaw_deg", float("nan")), calib.t_WL[0], calib.t_WL[1], calib.t_WL[2],
             cdict.get("source", "?")))
    print("world      : %s (%s)" % (args.world, "loaded" if world is not None else "MISSING -> no audit"))
    if world is None:
        return 1

    conv = frame != "W"
    print()
    print("the conversion applied to the recorded points:  p_W = R_WL @ p_recorded + t_WL   %s"
          % ("(APPLIED -- the file says L)" if conv else "(NOT applied -- the file already says W)"))
    t_us = P["t_us"].astype(np.int64)
    rows = np.column_stack([P["e"], P["n"], P["u"]]).astype(float)
    ex = 0
    for tm in np.unique(t_us)[:1]:
        j = np.flatnonzero(t_us == tm)[ex]
        q = rows[j]
        qw = (calib.R_WL @ q) + calib.t_WL if conv else q
        print("   message %d, point %d: recorded (E %+.5f N %+.5f U %+.5f) -> (E %+.5f N %+.5f U %+.5f)"
              % (tm, int(P["i"][j]), q[0], q[1], q[2], qw[0], qw[1], qw[2]))

    uniq = np.unique(t_us)
    print()
    print("%-20s %5s %9s %12s %12s %10s" % ("message t_us", "n", "arc_m", "clr_m BEFORE", "clr_m AFTER",
                                            "delta"))
    worst_before, worst_after = np.inf, np.inf
    for tm in uniq:
        sel = t_us == tm
        q = rows[sel]
        qw = (calib.R_WL @ q.T).T + calib.t_WL if conv else q
        after = float(world.clearance(qw).min())
        before = float(hdr["clr_m"][np.flatnonzero(hdr["t_us"].astype(np.int64) == tm)][0])
        worst_before = min(worst_before, before)
        worst_after = min(worst_after, after)
        if len(uniq) <= 5 or args.all or tm in uniq[:5]:
            print("%-20d %5d %9.3f %12.4f %12.4f %10.4f" % (tm, int(sel.sum()),
                                                            float(hdr["arc_m"][
                                                                np.flatnonzero(hdr["t_us"].astype(np.int64)
                                                                               == tm)][0]),
                                                            before, after, after - before))
    print("worst over %d distinct messages: BEFORE %+.4f m, AFTER %+.4f m"
          % (len(uniq), worst_before, worst_after))

    c2 = metrics_c2(args.run)
    problems = []
    print()
    if conv:
        print("the RECORDED column is the L-frame value and the recomputed one is the W-frame value: this is")
        print("an OLD recording, and AFTER is what the column should have said.")
    else:
        print("this recording was written AFTER the fix: the recorder already converted, so BEFORE and AFTER")
        print("must be identical.  worst |BEFORE-AFTER| = %.6f m"
              % max(abs(worst_before - worst_after), 0.0))
        if abs(worst_before - worst_after) > 1e-6:
            problems.append("the recorder's own clr_m (%.6f) and the audit's recomputation (%.6f) disagree "
                            "on a W-frame recording" % (worst_before, worst_after))
    if c2 is None:
        print("no metrics.txt in the run dir: the flight's own C2 was not available to compare against")
    else:
        cmin, n_inside, n_samp, foot = c2
        d = worst_after - cmin
        print("metrics.txt C2 (the FLIGHT's own minimum pillar-surface distance, from the truth): %.3f m; "
              "%d of %d samples inside the %.2f m footprint" % (cmin, n_inside, n_samp, foot))
        print("the PLAN's own minimum clearance after conversion: %.4f m  ->  difference %+.4f m  "
              "(tolerance %.2f m)" % (worst_after, d, args.tol_m))
        if abs(d) > args.tol_m:
            problems.append("the plan's clearance %.4f m and the flight's %.3f m differ by %.4f m, more than "
                            "the %.2f m tolerance" % (worst_after, cmin, d, args.tol_m))
        if d > 0.0:
            print("   the sign is the expected one: the PLAN is clearer than the FLIGHT, because the "
                  "aircraft can only leave the plan, and this run's own B2 measures how far (RECT6 leg 4: "
                  "0.1246 m worst perpendicular).  A plan that sat exactly on the flight's minimum would "
                  "mean the aircraft tracked it perfectly.")
        else:
            problems.append("the plan (%.4f m) is TIGHTER than the flight's own minimum (%.3f m), which "
                            "cannot happen unless the aircraft left the plan -- or the two numbers are in "
                            "different frames" % (worst_after, cmin))
    print()
    if problems:
        print("CLEARANCE-COLUMN AUDIT FAILED (%d problems):" % len(problems))
        for p in problems:
            print("   - %s" % p)
        return 1
    print("CLEARANCE-COLUMN AUDIT PASSED: clr_m means what its name says (clearance to the world file's "
          "pillars, in W), and it agrees with the flight's own C2 to within %.2f m." % args.tol_m)
    return 0


if __name__ == "__main__":
    sys.exit(main())
