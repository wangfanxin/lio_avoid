#!/usr/bin/env python3
"""Enforce the documentation budget: at most 5 markdown documents in docs/.

The user's rule (2026-10-13): the total number of documents may never exceed 5; if it does, merge and
clean up.  A rule that is only written down decays, so this is the executable form: run it before every
commit, and let it fail the run.

Counted:  docs/*.md  (any depth below docs/, so a stray docs/notes/foo.md counts too).
Ignored:  every non-markdown artefact (figures, csv, metrics dumps) -- they are evidence, not documents.

Usage:
    python3 tools/docs-budget.py            # report and exit 0/1
    python3 tools/docs-budget.py --list     # same, always exit 0
"""
from __future__ import annotations

import argparse
import os
import sys

LIMIT = 5
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
DOCS = os.path.join(ROOT, "docs")

# The four that exist, and the one free slot.  Named here so a new file has to be a deliberate act:
# adding a name to this list is the same commit as writing the file, and it must push the count to 5.
EXPECTED = {
    "PLAN.md": "goal, decisions, acceptance, order, rules",
    "ARCHITECTURE.md": "components, dataflow, topics, TF, transport, safety",
    "FRAMES.md": "coordinate conventions, wire facts, where conversions live, tests",
    "STATUS.md": "current state, evidence, next steps",
}


def find_docs() -> list[str]:
    out = []
    for dirpath, _dirnames, filenames in os.walk(DOCS):
        for fn in filenames:
            if fn.lower().endswith(".md"):
                out.append(os.path.relpath(os.path.join(dirpath, fn), DOCS).replace("\\", "/"))
    return sorted(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", action="store_true", help="always exit 0")
    args = ap.parse_args()

    found = find_docs()
    print("documentation budget: %d of %d used" % (len(found), LIMIT))
    for name in found:
        note = EXPECTED.get(name, "UNREGISTERED -- add it to EXPECTED in this file, or merge it away")
        print("  %-24s %s" % (name, note))
    for name, note in EXPECTED.items():
        if name not in found:
            print("  %-24s MISSING (expected: %s)" % (name, note))

    problems = []
    if len(found) > LIMIT:
        problems.append("%d documents > %d allowed" % (len(found), LIMIT))
    unregistered = [n for n in found if n not in EXPECTED]
    if unregistered:
        problems.append("unregistered: %s" % ", ".join(unregistered))
    missing = [n for n in EXPECTED if n not in found]
    if missing:
        problems.append("missing: %s" % ", ".join(missing))

    if problems:
        print("\nBUDGET VIOLATION: " + "; ".join(problems))
        print("Merge the content into one of the four owners (see PLAN.md section 6) and delete the file.")
        return 0 if args.list else 1
    print("\nOK: %d of %d used, all registered." % (len(found), LIMIT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
