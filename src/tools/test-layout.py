#!/usr/bin/env python3
"""THE LAYOUT GATE: the code lives under `src/`, the data does not, and every tool still starts.

WHY THIS EXISTS.  2026-10-16: every code directory moved under `src/` (bridge, common, lio, planner, sim,
supervisor, tools, patches, quadrotor_msgs) while the DATA stayed at the top (`docs/`, `frozen/`, `params/`,
`rviz/`, `worlds/`, `firmware/`), because the frozen manifest pins those paths by SHA-256.  That introduced a
distinction nothing was checking: a tool needs TWO roots -- the CODE root (what it imports) and the PROJECT
root (what it reads) -- and getting one of them wrong is silent until a specific tool is run.  `frozen-check`
was exactly that casualty: it found its manifest through the project root and then resolved every entry
against the code root, and reported "53 missing".

WHAT IT CHECKS, all offline and in seconds:
  1. the two roots resolve, and the directories that are supposed to be on each side are there;
  2. every python module under `src/` imports (with a stated allowlist for modules that read `sys.argv` at
     import time -- a pre-existing property, not a layout error);
  3. every tool answers `--help` (allowlist for the two that treat `argv[1]` as data before argparse);
  4. no tracked CODE file still refers to the pre-move layout (`tools/x.py` instead of `src/tools/x.py`,
     `$REPO/params`, a `sys.path.insert` of the repository root, ...);
  5. `frozen-check.py` and `docs-budget.py` run and pass -- they are the two tools whose whole job is to read
     the PROJECT side, so they are the ones a layout mistake breaks first.

usage: tools/test-layout.py           (exit 0 = the layout is consistent)
"""
from __future__ import annotations

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))          # <repo>/src
PROJ = os.path.abspath(os.path.join(CODE, ".."))          # <repo>

#: modules that read sys.argv at import time, so importing them from a test cannot work.  Each is stated
#: with why, and the list is checked for staleness below (an entry that no longer fails is reported).
ARGV_AT_IMPORT = {
    "tools.measure-rates": "reads sys.argv[1] as a run dir at module level",
    "tools.measure-rates2": "reads sys.argv[1] as a run dir at module level",
}
#: tools whose argv[1] is DATA, so `--help` is read as a filename/port instead of a flag.
NO_HELP = {
    "bin-params.py": "argv[1] is the flight BIN and the first thing it does is open it",
    "measure-rates.py": "argv[1] is a run dir",
    "measure-rates2.py": "argv[1] is a run dir",
}
VENDORED = ("lio-fork-local", "quadrotor_msgs", "patches", "__pycache__")


def walk_modules():
    for root, dirs, files in os.walk(CODE):
        dirs[:] = [d for d in dirs if d not in VENDORED]
        for f in sorted(files):
            if not f.endswith(".py"):
                continue
            rel = os.path.relpath(os.path.join(root, f), CODE)
            mod = rel[:-3].replace("/", ".")
            if mod.endswith(".__init__"):
                mod = mod[:-9]
            yield mod, os.path.join(root, f)


def main() -> int:
    bad, notes = [], []

    # ---- 1. the two roots
    print("code root    %s" % CODE)
    print("project root %s" % PROJ)
    for d in ("bridge", "common", "lio", "planner", "sim", "supervisor", "tools", "patches"):
        if not os.path.isdir(os.path.join(CODE, d)):
            bad.append("code directory missing: src/%s" % d)
    for d in ("docs", "frozen", "params", "rviz", "worlds", "firmware"):
        if not os.path.isdir(os.path.join(PROJ, d)):
            bad.append("project directory missing: %s/" % d)
    for d in ("tools", "planner", "bridge", "sim", "supervisor", "common"):
        if os.path.exists(os.path.join(PROJ, d)):
            bad.append("a code directory is still at the top level: %s/ (it belongs under src/)" % d)
    print("   %d code dirs and 6 project dirs checked" % 8)

    # ---- 2. every module imports
    sys.path.insert(0, CODE)
    import contextlib
    import importlib
    import io
    failed = 0
    seen_allow = set()
    for mod, path in walk_modules():
        try:
            # some modules print at import time; that is not what this gate is about
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                importlib.import_module(mod)
        except SystemExit:
            pass
        except BaseException as exc:                      # noqa: BLE001 -- report, do not crash
            if mod in ARGV_AT_IMPORT:
                seen_allow.add(mod)
                continue
            failed += 1
            bad.append("import %s failed: %s: %s" % (mod, type(exc).__name__, str(exc)[:80]))
    print("   modules imported: %d, failures: %d, allowlisted: %d"
          % (sum(1 for _ in walk_modules()), failed, len(seen_allow)))
    for mod in sorted(set(ARGV_AT_IMPORT) - seen_allow):
        notes.append("allowlist entry %s no longer fails to import -- remove it" % mod)

    # ---- 3. every tool answers --help
    #     (NOT this file: running it is not answering --help, it would recurse)
    SELF = os.path.basename(os.path.abspath(__file__))
    tools = sorted(f for f in os.listdir(HERE) if f.endswith(".py") and f != SELF)
    helped = 0
    for t in tools:
        if t in NO_HELP:
            continue
        try:
            r = subprocess.run([sys.executable, os.path.join(HERE, t), "--help"],
                               capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            bad.append("--help TIMED OUT for %s (120 s) -- does it start doing work instead of parsing?" % t)
            continue
        if r.returncode not in (0, 2):
            bad.append("--help failed for %s (rc=%s): %s"
                       % (t, r.returncode, (r.stderr.strip().splitlines() or [""])[-1][:80]))
        else:
            helped += 1
    print("   tools answering --help: %d of %d (%d allowlisted, self excluded)"
          % (helped, len(tools), len(NO_HELP)))

    # ---- 4. no tracked code file still names the old layout
    #     (NOT this file: it contains those patterns as regexes, so it would match itself)
    SELF_REL = os.path.relpath(os.path.abspath(__file__), PROJ)
    OLD = [r'sys\.path\.insert\(0, "/mnt/d/distill_self/lio_avoid"\)',
           r'\$REPO/(params|worlds|docs|frozen|rviz)',
           r'os\.path\.join\(ROOT, "(worlds|docs|frozen|params|rviz)"',
           r'"os\.path\.join\(HERE, "\.\."\)']
    tracked = subprocess.run(["git", "-C", PROJ, "ls-files", "src"], capture_output=True, text=True)
    files = [f for f in tracked.stdout.split("\n")
             if f.endswith((".py", ".sh")) and not any(v in f for v in VENDORED) and f != SELF_REL]
    stale = []
    for f in files:
        try:
            s = open(os.path.join(PROJ, f), encoding="utf-8").read()
        except OSError:
            continue
        for pat in OLD:
            if re.search(pat, s):
                stale.append("%s matches %s" % (f, pat))
    print("   code files scanned for the old layout: %d, stale: %d" % (len(files), len(stale)))
    bad += stale

    # ---- 5. the two project-side tools
    for t in ("frozen-check.py", "docs-budget.py"):
        r = subprocess.run([sys.executable, os.path.join(HERE, t)], capture_output=True, text=True, timeout=300)
        line = (r.stdout.strip().splitlines() or [""])[-1]
        print("   %-18s rc=%s  %s" % (t, r.returncode, line[:70]))
        if r.returncode != 0:
            bad.append("%s exited %s" % (t, r.returncode))

    print()
    for n in notes:
        print("NOTE: %s" % n)
    if bad:
        print("LAYOUT TEST FAILED (%d problem(s)):" % len(bad))
        for b in bad[:15]:
            print("   - %s" % b)
        return 1
    print("LAYOUT TEST PASSED: the code root, the project root and every tool agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
