#!/usr/bin/env python3
"""THE ONE LINT FINDING THAT MATTERS: an UNDEFINED NAME, which is a crash on a path no test walks.

WHY THIS EXISTS.  `planner/ego_source.py`'s ROS half used `DurabilityPolicy` without importing it.  Its
offline `--selftest` passed -- that selftest has no ROS in it and never touches the node -- and so did all
seventeen gates, because none of them starts a ROS node.  The defect surfaced only when the adapter was first
put in a live loop: `NameError: name 'DurabilityPolicy' is not defined`, on a stack brought up for the
purpose (`_stage/ego-adapter-parked.sh`).  That is the expensive shape of bug: correct everywhere it is
tested, fatal where it is used.

WHAT IT CHECKS.  pyflakes over every tracked `.py` under the code root, and it FAILS on **undefined names**
only.  The rest of pyflakes' output -- unused imports, assigned-but-unused locals -- is reported as a COUNT
and never as a failure: those 50 findings are cosmetic, and a gate that fails on cosmetics gets disabled the
first time it is inconvenient.  Vendored trees (`lio-fork-local`, `quadrotor_msgs`) are excluded: their
findings are not ours to fix.

IF PYFLAKES IS ABSENT the check is SKIPPED LOUDLY rather than passed quietly -- a gate that goes green when
its tool is missing is worse than no gate -- and the exit status says so.

usage: tools/test-lint.py [--all]      (--all also lists the non-fatal findings)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))          # <repo>/src
PROJ = os.path.abspath(os.path.join(CODE, ".."))          # <repo>
VENDORED = ("src/lio-fork-local", "src/quadrotor_msgs")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--all", action="store_true", help="also print the non-fatal findings")
    args = ap.parse_args()

    try:
        subprocess.run([sys.executable, "-m", "pyflakes", "--version"], capture_output=True, check=True)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("LINT GATE SKIPPED: pyflakes is not installed in this interpreter (%s)." % sys.executable)
        print("                  `python3 -m pip install pyflakes` makes this gate real; until then it "
              "proves nothing, and it says so instead of passing quietly.")
        return 2

    tracked = subprocess.run(["git", "-C", PROJ, "ls-files", "src"], capture_output=True, text=True)
    files = [f for f in tracked.stdout.split("\n")
             if f.endswith(".py") and not any(v in f for v in VENDORED)]
    print("the code root: %s" % CODE)
    print("files linted : %d (vendored trees excluded: %s)" % (len(files), ", ".join(VENDORED)))

    r = subprocess.run([sys.executable, "-m", "pyflakes"] + files, capture_output=True, text=True)
    lines = [l for l in (r.stdout + r.stderr).splitlines() if l.strip()]
    undefined = [l for l in lines if "undefined name" in l]
    other = [l for l in lines if l not in undefined]

    print("undefined names : %d   (FATAL -- each one is a NameError waiting for its code path)" % len(undefined))
    for l in undefined:
        print("   *** %s" % l)
    print("other findings  : %d   (not fatal: unused imports and unused locals)" % len(other))
    if args.all:
        for l in other:
            print("   %s" % l)

    if undefined:
        print()
        print("LINT GATE FAILED: %d undefined name(s).  This is the defect class that passes every offline "
              "test and dies in a live loop." % len(undefined))
        return 1
    print()
    print("LINT GATE PASSED: no undefined names in the code root's %d files" % len(files))
    return 0


if __name__ == "__main__":
    sys.exit(main())
