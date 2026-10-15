#!/usr/bin/env python3
"""B1's excursion, per CORNER VISIT: the turnaround overshoot separated from the steady-state offset.

WHY: B1 prints one number for the whole route ("E 2.154 m against a 2.10 bound [FAIL]"), and one number
cannot say what the aircraft DID.  The refusal to raise the bound needs the mechanism, so this tool splits
the extent into the two things it could be:

  * a TURNAROUND OVERSHOOT -- arriving at speed, crossing the corner, swinging past the command and
    coming back.  Signature: the |offset| peaks shortly after arrival and then DECAYS; the peak is
    proportional to the arrival speed, and the settled offset afterwards is small.
  * a STEADY-STATE OFFSET -- a constant bias that no amount of waiting removes.  Signature: the offset
    after the transient is the same size as the peak.

It reports, per corner visit: the arrival time and the peak speed within 1 m of the corner (the approach
speed the FSM's corner tolerance has to absorb), the closest approach, the largest offset from the
COMMANDED corner on each axis with its sign, the settled offset after --settle-s, the half-period of the
resulting oscillation, and the 1-2-3 mm figure the hover finally holds.  Every number comes from the run's
own tap.csv -- the same truth, and the same E/N/U conversion, that tools/metrics.py judges B1 with.

usage: tools/corner-excursion.py <run-dir> [<run-dir> ...] [--world worlds/pillars.yaml] [--dwell-s 8.0]
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_tap(path):
    """t_us, and the truth in W as (E, N, U) -- the same conversion tools/metrics.py does."""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t = d[:, 0].astype(np.int64)
    return t, np.column_stack([d[:, 3], d[:, 2], -d[:, 4]])


def visits(t, p, corner, near_m, gap_s):
    """Contiguous spans within near_m of one corner, SPLIT where the aircraft is away for longer than gap_s.

    A corner can be reached twice in one sortie (the tour closes on corner 1, and the FSM's own final
    approach touches it again), and those are two different visits with two different arrival speeds --
    merging them would average the very thing this tool exists to separate.
    """
    d = np.hypot(p[:, 0] - corner[0], p[:, 1] - corner[1])
    inside = np.flatnonzero(d <= near_m)
    if len(inside) == 0:
        return []
    out, start, prev = [], inside[0], inside[0]
    for i in inside[1:]:
        if (t[i] - t[prev]) * 1e-6 > gap_s:
            out.append((start, prev))
            start = i
        prev = i
    out.append((start, prev))
    return [(a, b) for a, b in out if (t[b] - t[a]) * 1e-6 >= 0.30]


def report(run, corners, near_m, gap_s, settle_s, v_rush_m, dwell_s):
    tap = os.path.join(run, "tap.csv")
    if not os.path.exists(tap):
        print("### %s: no tap.csv" % run)
        return
    t, p = read_tap(tap)
    ts = (t - t[0]) * 1e-6
    dt = np.diff(t) * 1e-6
    v = np.hypot(np.diff(p[:, 0]) / dt, np.diff(p[:, 1]) / dt)
    # the axis each corner sits on, and WHICH WAY is "past the command box": c1 and c3 sit on the N walls
    # (N = y0 and N = y1), c2 and c4 on the E walls (E = x1 and E = x0).  An excursion "past the corner" is
    # a departure from the commanded rectangle across the wall that corner sits on -- that is the overshoot
    # B1's box measures, and it is the axis the trajectory's own excursion number is quoted on.
    axes = [("N", -1, lambda q, k=1: q[:, k]), ("E", +1, lambda q, k=0: q[:, k]),
            ("N", +1, lambda q, k=1: q[:, k]), ("E", -1, lambda q, k=0: q[:, k])]
    print("### %s" % os.path.basename(run.rstrip("/")))
    print("   %d tap samples over %.1f s;  visit = truth within %.2f m of the corner (split on a gap "
          "above %.1f s)" % (len(t), ts[-1], near_m, gap_s))
    print("   past-wall = across the wall this corner sits on, measured from the COMMANDED corner.  The "
          "classification")
    print("   window starts at the CLOSEST APPROACH -- the turnaround -- and runs for %.1f s or to the end "
          "of the visit," % dwell_s)
    print("   whichever is first; taking the LAST QUARTER OF THE VISIT instead measures the DEPARTURE, and "
          "with a dwell")
    print("   that follows the departure the ratio reads 1.00 for a transient that had already decayed.  "
          "'x' is the")
    print("   surviving fraction at the end of that window: near 0 = a transient, near 1 = an offset that "
          "waiting does")
    print("   not remove.  A visit with no overshoot at all prints '--'.")
    print("   %-3s %8s %7s %8s | %-8s %-7s %-6s | %-15s | %s"
          % ("cor", "t_arrive", "v_peak", "closest", "past-wall", "at", "since", "PEAK (surviving x)",
             "back to 10 mm"))
    for k, c in enumerate(corners, start=1):
        aname, asign, get = axes[k - 1]
        for (a, b) in visits(t, p, c, near_m, gap_s):
            pre = np.flatnonzero((ts[:a] >= ts[a] - 6.0) &
                                 (np.hypot(p[:a, 0] - c[0], p[:a, 1] - c[1]) <= 1.0))
            vpk = float(v[pre].max()) if len(pre) else float("nan")
            dv = np.column_stack([p[a:b + 1, 0] - c[0], p[a:b + 1, 1] - c[1]])
            dist = np.hypot(dv[:, 0], dv[:, 1])
            jc = int(np.argmin(dist))
            over = get(dv, 0) * asign                       # > 0 means outside the wall
            m = int(np.argmax(over))
            # THE CLASSIFICATION WINDOW, and why it is not "the last quarter of the visit": see the header.
            hi = min(int(np.searchsorted(ts[a:b + 1], ts[a + jc] + dwell_s)), len(over) - 1)
            hi = max(hi, m)
            late_end = float(over[hi])
            # the half-period: from the peak back to within 10 mm of the command
            near_cmd = np.flatnonzero(np.hypot(dv[m:, 0], dv[m:, 1]) <= 0.010)
            half = ("%5.2f s" % (ts[a + m + near_cmd[0]] - ts[a + m])) if len(near_cmd) else "  none "
            # A visit whose largest value on this axis is still on the COMMAND side has no overshoot to
            # classify at all (corner 1's stopping point), and quoting a ratio of two non-excursions would
            # invent one.  The absolute end value is always printed beside the peak, so a ratio that is
            # large only because the peak is tiny is visible as such.
            late_txt = ("x%5.2f" % (late_end / float(over[m]))) if over[m] > 0.001 else "  --  "
            print("   c%-2d %7.2f %6.2f %7.4f | %s %+6.4f %6.2f s %5.1f s | %5.1f mm -> %+5.1f mm %s | %s"
                  % (k, ts[a], vpk, float(dist[jc]), aname, float(over[m]), ts[a + m], ts[a + m] - ts[a],
                     float(over[m]) * 1e3, late_end * 1e3, late_txt, half))
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--near-m", type=float, default=0.5, help="a corner visit is truth within this, m")
    ap.add_argument("--gap-s", type=float, default=3.0, help="away this long = a new visit")
    ap.add_argument("--settle-s", type=float, default=3.0, help="'settled' starts this long after arrival")
    ap.add_argument("--dwell-s", type=float, default=8.0,
                    help="the classification window, measured from the closest approach (the turnaround)")
    ap.add_argument("--v-rush", type=float, default=0.5, help="speed above which the approach is 'at speed'")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.world)) or {}
    x0, y0, x1, y1 = [float(v) for v in cfg["mission"]["square"]]
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    for r in args.runs:
        report(r, corners, args.near_m, args.gap_s, args.settle_s, args.v_rush, args.dwell_s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
