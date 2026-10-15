#!/usr/bin/env python3
"""DOES THE B-SPLINE SELFTEST HAVE TEETH?  Nine deliberate defects, and every one must be caught.

WHY THIS EXISTS.  `planner/bspline.py` says its evaluator is "verified against an INDEPENDENT Cox-de Boor
basis recursion".  A cross-check that cannot fail is not a check -- it is a second copy of the same belief.
This file attacks the claim instead of restating it: it applies NINE single-token mutations, each one a
mistake a real port of `UniformBspline::evaluateDeBoorT` plausibly makes, writes each mutated module to a
scratch file, and runs THAT module's own `_selftest()` as a subprocess.  A mutation that still passes is a
SURVIVOR, and a survivor is a hole in the selftest -- reported as a failure of this test, never hidden.

The control matters as much as the mutations: the UNMUTATED source must pass, from the same scratch
directory, or nothing below is meaningful.

WHAT EACH MUTATION IS (name -> the defect it injects):

    clamp/no-clamp          drop the domain clamp: evaluate past the end instead of holding the endpoint
    clamp/wrong-end         clamp to u[m-p-1] instead of u[m-p]: the domain loses its last span
    span/inclusive          `u[k+1] <= ub` instead of `<`: the span search runs one knot too far
    levels/one-fewer        one fewer de Boor corner-cutting level: a degree-(p-1) answer to a degree-p
    alpha/other-knot        the alpha numerator uses a neighbouring knot: a plausible transcription slip
    deriv/knot-offset       the derivative's knot differences are taken one index early
    deriv/no-degree         the degree factor p is dropped from the derivative control points
    deriv/knots-not-cut     the derivative's knot vector is not reduced (`u` instead of `u[1:-1]`)
    duration/wrong-domain   getTimeSum taken as u[m] - u[0] instead of u[m-p] - u[p]

usage: tools/test-bspline-mutations.py [--keep]     (exit 0 = every mutation is caught)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))
SRC = os.path.join(CODE, "planner", "bspline.py")

#: (name, the exact text to replace, what to replace it with).  Each `old` must occur EXACTLY once, so a
#: refactor that moves these lines makes this test fail loudly rather than silently stop mutating anything.
MUTATIONS = (
    ("clamp/no-clamp",
     "ub = min(max(u[p], t + u[p]), u[m - p])",
     "ub = t + u[p]"),
    ("clamp/wrong-end",
     "ub = min(max(u[p], t + u[p]), u[m - p])",
     "ub = min(max(u[p], t + u[p]), u[m - p - 1])"),
    ("span/inclusive",
     "while u[k + 1] < ub:",
     "while u[k + 1] <= ub:"),
    ("levels/one-fewer",
     "for i in range(p, r - 1, -1):",
     "for i in range(p - 1, r - 1, -1):"),
    ("alpha/other-knot",
     "alpha = 0.0 if den == 0.0 else (ub - u[i + k - p]) / den",
     "alpha = 0.0 if den == 0.0 else (ub - u[i + 1 + k - r]) / den"),
    ("deriv/knot-offset",
     "den = u[p + 1:p + 1 + n] - u[1:1 + n]",
     "den = u[p:p + n] - u[0:n]"),
    ("deriv/no-degree",
     "Q[ok] = (float(p) / den[ok])[:, None] * num[ok]",
     "Q[ok] = num[ok] / den[ok][:, None]"),
    ("deriv/knots-not-cut",
     "return u[1:-1], Q",
     "return u, Q"),
    ("duration/wrong-domain",
     "return float(u[m - int(order)] - u[int(order)])",
     "return float(u[m] - u[0])"),
)


def run_one(source: str, workdir: str, tag: str):
    """Write the module and run its own selftest as a subprocess.  Returns (rc, the last line of output)."""
    path = os.path.join(workdir, "bspline_%s.py" % tag.replace("/", "_"))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(source)
    r = subprocess.run([sys.executable, path], capture_output=True, text=True, timeout=120)
    lines = (r.stdout + r.stderr).strip().splitlines()
    return r.returncode, (lines[-1] if lines else "")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory for inspection")
    args = ap.parse_args()

    src = open(SRC, encoding="utf-8").read()
    print("the file under attack: %s" % os.path.relpath(SRC, CODE))
    print("%d mutation(s), each must make the selftest FAIL\n" % len(MUTATIONS))

    bad = []
    with tempfile.TemporaryDirectory(prefix="bspline-mut-") as work:
        rc, last = run_one(src, work, "control")
        print("CONTROL  unmutated source")
        print("   rc=%d  %s" % (rc, last[:90]))
        if rc != 0:
            bad.append("the control failed: the unmutated source does not pass its own selftest (%s)" % last)
            print("\nBSPLINE MUTATION TEST FAILED: the control is broken, nothing below means anything")
            return 1
        print()

        for i, (name, old, new) in enumerate(MUTATIONS, 1):
            n = src.count(old)
            if n != 1:
                print("M%-2d %-22s NOT APPLIED: its anchor text occurs %d times, not once" % (i, name, n))
                bad.append("%s: the anchor text occurs %d times (the file moved under this test)" % (name, n))
                continue
            mrc, mlast = run_one(src.replace(old, new), work, name)
            caught = mrc != 0
            print("M%-2d %-22s rc=%-3d %s" % (i, name, mrc, "CAUGHT" if caught else "*** SURVIVED ***"))
            print("      %s" % (mlast[:120] or "(no output)"))
            if not caught:
                bad.append("%s SURVIVED: the selftest still passes with this defect in the evaluator" % name)

        if args.keep:
            keep = os.path.join(os.getcwd(), "bspline-mutations")
            subprocess.run(["cp", "-r", work, keep], check=False)
            print("\nscratch kept at %s" % keep)

    print()
    caught_n = len(MUTATIONS) - len([b for b in bad if "SURVIVED" in b or "anchor" in b])
    if bad:
        print("BSPLINE MUTATION TEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("BSPLINE MUTATION TEST PASSED: %d of %d defects caught, and the unmutated source passes"
          % (caught_n, len(MUTATIONS)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
