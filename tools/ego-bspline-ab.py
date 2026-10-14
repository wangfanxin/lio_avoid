#!/usr/bin/env python3
"""A/B the adapter's B-spline evaluator against the planner's own C++.

`planner/ego_adapter.py` re-implements `ego_planner::UniformBspline::evaluateDeBoorT` in Python,
because the adapter is the one file allowed to know the planner's message type and a Python node
cannot link libbspline_opt.  A transliteration that is only checked against the source it was copied
from proves nothing, so this tool compiles `bspline_opt/src/uniform_bspline.cpp` as it stands in the
pinned ros2_version checkout and compares the two on:

  * random splines of order 2, 3 and 4 with the uniform knot vector `u_i = (i-p)*ts` (the shape the
    FSM actually publishes -- see ego_replan_fsm.cpp, `bspline.order = 3`),
  * the control points that `UniformBspline::parameterizeToBspline` produces, and
  * the knots and control points of a REAL Bspline message, if one was captured with
    `planner/ego_adapter.py --dump-bspline`.

usage: tools/ego-bspline-ab.py [--dump FILE] [--out FILE]
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ROOT, "planner"))

from ego_adapter import bspline_eval, bspline_duration, parameterize_to_bspline   # noqa: E402

EGO = os.environ.get("EGO_TREE", "/home/fanxin/lio_avoid/src/ego-planner-swarm")


def build_ref():
    src = os.path.join(EGO, "src", "planner", "bspline_opt", "src", "uniform_bspline.cpp")
    inc = os.path.join(EGO, "src", "planner", "bspline_opt", "include")
    out = os.path.join(tempfile.gettempdir(), "ego-bspline-ref")
    if not os.path.exists(src):
        raise SystemExit("no ego-planner checkout at %s (set EGO_TREE)" % EGO)
    cmd = ["g++", "-O2", "-std=c++17", "-I", inc, "-I", "/usr/include/eigen3",
           os.path.join(HERE, "bspline-ref.cpp"), src, "-o", out]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        raise SystemExit("compiling the C++ reference failed")
    return out


def run_ref(exe, order, knots, pts, ts):
    d = tempfile.mkdtemp(prefix="bspline-ab-")
    fin = os.path.join(d, "in.txt")
    ft = os.path.join(d, "t.txt")
    with open(fin, "w") as fh:
        fh.write("%d %d %d\n" % (order, len(pts), len(ts)))
        fh.write(" ".join("%.17g" % v for v in knots) + "\n")
        for p in pts:
            fh.write("%.17g %.17g %.17g\n" % (p[0], p[1], p[2]))
    with open(ft, "w") as fh:
        fh.write("\n".join("%.17g" % t for t in ts) + "\n")
    r = subprocess.run([exe, fin, ft], capture_output=True, text=True)
    if r.returncode:
        print(r.stdout, r.stderr)
        raise SystemExit("the C++ reference failed")
    return np.array([[float(v) for v in line.split()] for line in r.stdout.strip().splitlines()])


def case(name, order, pts, knots, n_eval=257, results=None):
    dur = bspline_duration(knots, len(pts), order)
    ts = np.linspace(0.0, dur, n_eval)
    cpp = run_ref(EXE, order, knots, pts, ts)
    py = np.array([bspline_eval(knots, pts, order, float(t)) for t in ts])
    err = float(np.linalg.norm(cpp - py, axis=1).max())
    results.append((name, order, len(pts), dur, err))
    print("  %-46s order %d, %3d ctrl, T = %6.3f s   max |C++ - Python| = %.3e m"
          % (name, order, len(pts), dur, err))
    return err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default="", help="a Bspline captured by ego_adapter --dump-bspline")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    global EXE
    EXE = build_ref()
    print("ego-planner C++ reference built from %s" % EGO)
    rng = np.random.default_rng(20261014)
    results = []
    worst = 0.0

    for order in (2, 3, 4):
        for k, ts in ((4, 0.2), (9, 0.15)):
            pts = np.cumsum(rng.normal(0.0, 0.6, size=(k + order, 3)), axis=0)
            u = np.array([(i - order) * ts for i in range(len(pts) + order + 1)])
            worst = max(worst, case("random order %d, ts=%.2f" % (order, ts), order, pts, u, results=results))

    K = 9
    wps = np.cumsum(rng.normal(0.0, 0.5, size=(K, 3)), axis=0)
    der = np.array([[0.3, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    pts = parameterize_to_bspline(0.2, wps, der)
    u = np.array([(i - 3) * 0.2 for i in range(len(pts) + 4)])
    worst = max(worst, case("parameterizeToBspline output", 3, pts, u, results=results))

    if args.dump and os.path.exists(args.dump):
        d = json.load(open(args.dump))
        pts = np.array(d["pos_pts"], dtype=float)
        u = np.array(d["knots"], dtype=float)
        worst = max(worst, case("a REAL Bspline from the planner", int(d["order"]), pts, u,
                                n_eval=701, results=results))
    else:
        print("  (no --dump: skipping the real-message case)")

    print()
    print("worst |C++ - Python| over %d cases: %.3e m" % (len(results), worst))
    ok = worst < 1e-9
    print("VERDICT %s (bound 1e-9 m)" % ("PASS" if ok else "FAIL"))

    if args.out:
        with open(args.out, "w") as fh:
            fh.write("ego_adapter's Python B-spline evaluator vs the planner's own C++\n")
            fh.write("C++ source: %s/src/planner/bspline_opt/src/uniform_bspline.cpp\n" % EGO)
            fh.write("revision  : %s\n" % subprocess.run(
                ["git", "-C", EGO, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip())
            fh.write("\n%-46s %5s %6s %10s %14s\n" % ("case", "order", "n_ctrl", "T [s]", "max err [m]"))
            for name, order, n, dur, err in results:
                fh.write("%-46s %5d %6d %10.3f %14.3e\n" % (name, order, n, dur, err))
            fh.write("\nworst %.3e m over %d cases\nVERDICT %s\n"
                     % (worst, len(results), "PASS" if ok else "FAIL"))
        print("written to %s" % args.out)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
