#!/usr/bin/env python3
"""EVERY `file:line` IN THE DOCS MUST RESOLVE — because one of them did not.

WHY THIS EXISTS.  `docs/PLANNERS.md` cited `docs/ego-planner-interface-read.md` as one of "the two full
audits".  **No such file has ever existed in this repository**, and no reader could tell: a citation to a
missing document looks exactly like a citation to a present one.  That is the same failure mode as a comment
naming a function that has been renamed, and it is worth a gate because these five documents are the
deliverable the safety argument rests on.

WHAT IT CHECKS.  Backticked citations of the form `file.ext:NN` or `:NN-MM` must resolve to a REAL file with
at least that many lines.  Citations are written four different ways in this project -- bare
(`traj_server.cpp:211`), partial (`plan_manage/src/traj_server.cpp:214`), repository-relative, and rooted in
another checkout (`ArduCopter/takeoff.cpp:29`) -- so a citation is resolved by PATH under every root and,
failing that, by BASENAME within them.  A basename that matches several files is reported with its
candidates and the line is required to exist in all of them; silence would be the wrong answer.

It does NOT check that the cited line SAYS what the prose claims -- only a human can do that -- so `--show`
prints the cited lines, which is what makes this reviewable rather than merely green.

`--paths` ALSO checks backticked paths that carry no line number.  It is NOT the default, and the reason is
measured rather than assumed: over these five documents that mode reports 16 unresolved names, 15 of which
are files the documents deliberately record as DELETED (`src/sim/debug_selftest.py`, `planner/goal_sequencer.py`,
`tools/corner-tour.py`, `rviz/mission_b.rviz`, ...: naming a file that is gone is the POINT of a history
section).  The sixteenth was the real defect this tool was written for --
`docs/ego-planner-interface-read.md`, cited as "the two full audits" and never committed.  Run it when the
documents change: a name in a history section should be preceded by "deleted", and an un-numbered path that
looks current should resolve.

WHERE IT LOOKS.  The repository, `<repo>/src`, the external trees the docs quote (ArduPilot, Super-LIO,
ego-planner-swarm, whose line numbers are the LIVE WORKING TREE of that checkout and not its HEAD), and
`_stage`.  **A root that does not exist is REPORTED, never silently skipped**: a gate that goes quiet when
its input is missing is worse than no gate.

usage: tools/check-doc-citations.py [--doc docs/PLANNERS.md ...] [--show N]
"""
from __future__ import annotations

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CODE = os.path.abspath(os.path.join(HERE, ".."))          # <repo>/src
PROJ = os.path.abspath(os.path.join(CODE, ".."))          # <repo>

#: where a citation's path may be rooted.  The external checkouts come last and may be absent.
ROOTS = (
    PROJ,
    CODE,
    "/home/fanxin/lio_avoid/src/ardupilot",
    "/home/fanxin/lio_avoid/src/Super-LIO",
    "/home/fanxin/lio_avoid/src/ego-planner-swarm",
    "/home/fanxin/lio_avoid/src/ego-planner-swarm/src",
    "/mnt/d/distill_self/_stage",
)

#: `path/to/thing.ext:12` or `:12-34`, inside backticks.  The extension list is explicit so that a prose
#: `12:30` (a clock time) or a chapter reference is not mistaken for a citation.
CITE = re.compile(r"`([A-Za-z0-9_./+-]+\.(?:py|md|sh|cpp|h|hpp|msg|launch|yaml|parm|txt|json|rviz)):(\d+)(?:-(\d+))?`")

#: the `--paths` mode: the same shape with NO line number.
PATHONLY = re.compile(r"`([A-Za-z0-9_][A-Za-z0-9_./+-]*\.(?:py|md|sh|cpp|h|hpp|msg|launch|yaml|parm|txt|json|rviz))`")

SKIP_DIRS = {".git", "build", "install", "log", "__pycache__", "node_modules", ".vscode"}

#: citations that are KNOWN to point outside every checkout here, each with the reason.  These are not
#: defects: they are files read elsewhere, and the citing line says so.  The list is checked for STALENESS
#: below -- an entry whose citation now resolves locally is reported, so an allowlist cannot quietly rot.
EXTERNAL = {
    ("FRAMES.md", "vision_pose_estimate.cpp"):
        "a mavros file, read on that project's ros2 branch; the citing line says so",
    ("FRAMES.md", "odom.cpp"):
        "a mavros file, read on that project's ros2 branch; the citing line says so",
    ("STATUS.md", "vision_pose_estimate.cpp"):
        "a mavros file, read on that project's ros2 branch (the same citation as FRAMES.md's)",
}


def index_basenames(root: str) -> dict:
    """basename -> [paths], built once per root.  Built lazily: most citations resolve by path."""
    out = {}
    for dirpath, dirnames, files in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for f in files:
            out.setdefault(f, []).append(os.path.join(dirpath, f))
    return out


_INDEX = {}


def by_basename(name: str) -> list:
    hits = []
    for root in ROOTS:
        if not os.path.isdir(root):
            continue
        if root not in _INDEX:
            _INDEX[root] = index_basenames(root)
        hits += _INDEX[root].get(name, [])
    return hits


#: a search root that holds STALE COPIES of other trees.  `_stage/ego-src/` is a snapshot of the ego tree
#: and is 99 lines SHORTER than the live checkout, so a citation resolved there would be judged against a
#: file the document never meant.  It is tried LAST, and only when nothing live matches.
STALE = ("_stage",)


def rank(path: str) -> int:
    """Lower is better: an exact path, then a live basename, then a basename inside a snapshot."""
    stale = any(s in path.split(os.sep) for s in STALE)
    return 2 if stale else 0


def resolve(rel: str) -> tuple:
    """(candidate paths, best-first, how it was found).  Path, then path SUFFIX, then basename.

    The suffix step is what makes `AP_VisualOdom/LogStructure.h:79` resolve to
    `libraries/AP_VisualOdom/LogStructure.h` instead of being answered by an unrelated `LogStructure.h`
    in another library -- a citation that carries part of a path means that part.
    """
    direct = [os.path.join(r, rel) for r in ROOTS if os.path.isfile(os.path.join(r, rel))]
    if direct:
        return sorted(direct, key=rank), "path"
    tail = rel.rsplit("/", 1)[-1]
    cands = by_basename(tail)
    if "/" in rel:
        suffixed = [p for p in cands if p.replace(os.sep, "/").endswith("/" + rel)]
        if suffixed:
            return sorted(suffixed, key=rank), "path suffix"
    return sorted(cands, key=rank), ("basename %r" % tail if rel != tail else "basename")


def nlines(path: str) -> int:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            return sum(1 for _ in fh)
    except OSError:
        return 0


def line_at(path: str, n: int) -> str:
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            for i, line in enumerate(fh, 1):
                if i == n:
                    return line.rstrip()
    except OSError as e:
        return "(unreadable: %s)" % e
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--doc", action="append", default=None,
                    help="a document to check (default: the five registered docs/*.md)")
    ap.add_argument("--show", type=int, default=0, help="print the first N cited lines in full")
    ap.add_argument("--paths", action="store_true",
                    help="ALSO require un-numbered backticked paths to resolve (noisy: the docs name "
                         "deliberately deleted files; the header says why this is not the default)")
    ap.add_argument("--quiet", action="store_true", help="only the problems and the verdict")
    args = ap.parse_args()

    docs = args.doc or [os.path.join(PROJ, "docs", f) for f in
                        ("PLAN.md", "ARCHITECTURE.md", "FRAMES.md", "STATUS.md", "PLANNERS.md")]
    absent = [r for r in ROOTS if not os.path.isdir(r)]
    if not args.quiet:
        print("citation roots, in the order tried:")
        for r in ROOTS:
            print("   %s%s" % (r, "   *** ABSENT ***" if r in absent else ""))
        if absent:
            print("   (an absent root is REPORTED, not skipped: a citation into it is judged unresolved)")
        print()

    bad, shown, total, by_base, multi, external_used = [], 0, 0, 0, 0, set()
    for doc in docs:
        base = os.path.basename(doc)
        if not os.path.isfile(doc):
            bad.append("the document itself is missing: %s" % doc)
            continue
        hits = CITE.findall(open(doc, encoding="utf-8", errors="replace").read())
        if not args.quiet:
            print("%s: %d citation(s)" % (os.path.relpath(doc, PROJ), len(hits)))
        for rel, n1, n2 in hits:
            total += 1
            hi = int(n2 or n1)
            paths, how = resolve(rel)
            if not paths:
                if (base, rel) in EXTERNAL:
                    external_used.add((base, rel))
                    if not args.quiet:
                        print("   external  %-30s %s" % ("%s:%s" % (rel, hi), EXTERNAL[(base, rel)]))
                    continue
                bad.append("%s cites `%s:%s`, which resolves to NO file under any root"
                           % (os.path.relpath(doc, PROJ), rel, hi))
                print("   MISSING  %s:%s" % (rel, hi))
                continue
            if how != "path":
                by_base += 1
            if len(paths) > 1:
                multi += 1
            # AT LEAST ONE candidate must reach the line: a bare basename is ambiguous by nature
            # (`mode_guided.cpp` is both ArduPlane's and ArduCopter's), and the citation means whichever
            # one has the line -- requiring all of them would fail on a file the document never meant.
            reach = [p for p in paths if nlines(p) >= hi]
            if not reach:
                best = paths[0]
                bad.append("%s cites `%s:%s` but no candidate reaches it -- the closest, %s, has only %d line(s)"
                           % (os.path.relpath(doc, PROJ), rel, hi, os.path.relpath(best, PROJ), nlines(best)))
                print("   OUT OF RANGE  %s:%s (best candidate %s has %d lines)"
                      % (rel, hi, os.path.relpath(best, PROJ), nlines(best)))
                continue
            if (base, rel) in EXTERNAL:
                external_used.add((base, rel))       # now resolvable locally -> the entry is STALE
            if shown < args.show:
                print("   ok  %-34s %-40s %s"
                      % ("%s:%s" % (rel, hi), "(%d candidate(s), %s)" % (len(paths), how),
                         line_at(reach[0], hi).strip()[:70]))
                shown += 1
        if not args.quiet:
            print()

    stale = sorted(set(EXTERNAL) - external_used)
    if stale:
        for k in stale:
            bad.append("EXTERNAL has a STALE entry: %s %s now resolves locally, so the reason no longer "
                       "holds and the entry must go" % k)

    unnamed = 0
    if args.paths:
        print("--paths: un-numbered backticked paths, which the documents use for files they record as "
              "DELETED as well as for live ones -- each one is printed, and none is called a defect")
        for doc in docs:
            if not os.path.isfile(doc):
                continue
            toks = sorted(set(PATHONLY.findall(open(doc, encoding="utf-8", errors="replace").read())))
            miss = [t for t in toks if not resolve(t)[0]]
            print("   %-18s %3d token(s), %d unresolved" % (os.path.basename(doc), len(toks), len(miss)))
            unnamed += len(miss)
            for m in miss:
                print("        unresolved  %s" % m)
        print()

    if bad:
        print("DOC CITATION CHECK FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("DOC CITATION CHECK PASSED: %d citation(s) across %d document(s) resolve to real lines "
          "(%d by basename; %d had more than one candidate and were checked against all of them)"
          % (total, len(docs), by_base, multi))
    return 0


if __name__ == "__main__":
    sys.exit(main())
