#!/usr/bin/env python3
"""WHY a corner's dwell stopped: print the aircraft's distance to one corner, and its speed, over time.

The dwell a sortie is judged by is the LONGEST contiguous span within --radius of the corner, so a 4.96 s
reading against a 5.0 s requirement is a statement about a GAP: the aircraft left the radius mid-hold and
came back.  This tool prints where it went and how fast, so the gap has a mechanism instead of a guess.

usage: tools/corner-dwell-trace.py <run-dir> <c1|c2|c3|c4|c5> [--radius 0.15] [--world ...]
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: the PROJECT root, one level above the code: worlds/, params/, docs/, frozen/, rviz/ live
#: there, not next to the code (2026-10-16: every code directory moved under src/).
PROJ = os.path.abspath(os.path.join(ROOT, ".."))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("corner")
    ap.add_argument("--radius", type=float, default=0.15)
    ap.add_argument("--world", default=os.path.join(PROJ, "worlds", "pillars.yaml"))
    ap.add_argument("--step", type=float, default=0.1)
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.world)) or {}
    x0, y0, x1, y1 = [float(v) for v in cfg["mission"]["square"]]
    cs = {"c1": (x0, y0), "c2": (x1, y0), "c3": (x1, y1), "c4": (x0, y1), "c5": (x0, y0)}
    c = np.array(cs[args.corner], dtype=float)

    d = np.loadtxt(os.path.join(args.run, "tap.csv"), delimiter=",", skiprows=1)
    t = (d[:, 0] - d[0, 0]) * 1e-6
    e, n, u = d[:, 3], d[:, 2], -d[:, 4]
    v = np.hypot(np.gradient(e, t), np.gradient(n, t))
    dist = np.hypot(e - c[0], n - c[1])
    near = dist <= args.radius
    # the spans inside the radius
    spans, s = [], None
    for i, ok in enumerate(near):
        if ok and s is None:
            s = i
        elif not ok and s is not None:
            spans.append((s, i - 1))
            s = None
    if s is not None:
        spans.append((s, len(near) - 1))
    spans = [(a, b) for a, b in spans if t[b] - t[a] >= 0.20]
    print("### %s  corner %s = (%+.3f, %+.3f), radius %.2f m" % (os.path.basename(args.run.rstrip("/")),
                                                                 args.corner, c[0], c[1], args.radius))
    print("   spans inside the radius (>= 0.2 s):")
    for a, b in spans:
        print("      t %7.2f .. %7.2f = %5.2f s   closest %.4f m   U %.3f..%.3f" %
              (t[a], t[b], t[b] - t[a], dist[a:b + 1].min(), u[a:b + 1].min(), u[a:b + 1].max()))
    # the gaps between them
    for (a1, b1), (a2, b2) in zip(spans, spans[1:]):
        gap = np.arange(b1, a2 + 1)
        i_far = int(gap[np.argmax(dist[gap])])
        print("   GAP t %7.2f .. %7.2f = %5.2f s outside the radius: reached %.4f m (%.0f mm outside) at "
              "t=%.2f s, U %.3f m, |v| %.3f m/s there, and the aircraft passed the corner at %.3f m/s"
              % (t[b1], t[a2], t[a2] - t[b1], dist[i_far], (dist[i_far] - args.radius) * 1e3, t[i_far],
                 u[i_far], v[i_far], v[b1:a2 + 1].max()))
    # the trace around the longest span
    if spans:
        a, b = max(spans, key=lambda ab: t[ab[1]] - t[ab[0]])
        lo, hi = max(t[a] - 2.0, 0.0), min(t[b] + 0.5, t[-1])
        print("   around the longest span (%.2f s, t %.2f..%.2f):" % (t[b] - t[a], t[a], t[b]))
        x = lo
        while x <= hi:
            i = int(np.argmin(np.abs(t - x)))
            print("      t %7.2f  d %6.4f m  E %+7.4f N %+7.4f U %+7.4f  |v| %5.3f  %s"
                  % (x, dist[i], e[i], n[i], u[i], v[i], "IN" if near[i] else "out"))
            x += args.step
    return 0


if __name__ == "__main__":
    sys.exit(main())
