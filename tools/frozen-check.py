#!/usr/bin/env python3
"""Check that the FROZEN mission-A artifacts are still byte-for-byte what they were when frozen.

The user's instruction (2026-10-14): "mission A之前的任务和rviz的地图都保存好，冻结，不搞了" -- mission A,
everything flown before it, and its RViz view are finished: keep them, freeze them, stop working on them.
A freeze that is only a sentence in a document is not a freeze: someone (me, in the next round) edits
`worlds/forest_all.yaml`, every mission-A number in docs/STATUS.md quietly becomes a number about a
different world, and nothing anywhere fails.  So the frozen set is listed with its SHA-256 in
`frozen/mission_a.manifest`, and this tool recomputes them.

    tools/frozen-check.py                 # verify; exit 1 and name every file that moved
    tools/frozen-check.py --update        # re-stamp the listed files (a deliberate act, and it prints
                                          # what changed, so it cannot be silent)
    tools/frozen-check.py --update <path> # re-stamp and ADD a path to the frozen set

The manifest is plain text:  <sha256>  <path>  [size]  -- comments start with #.
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
DEFAULT = os.path.join(ROOT, "frozen", "mission_a.manifest")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def read_manifest(path):
    """Return (entries, header): entries is a list of [sha, relpath, size], header the comment lines."""
    entries, header = [], []
    with open(path) as fh:
        for line in fh:
            s = line.rstrip("\n")
            if not s.strip():
                continue
            if s.lstrip().startswith("#"):
                header.append(s)
                continue
            parts = s.split()
            if len(parts) < 2:
                print("manifest line is not '<sha>  <path>': %r" % s, file=sys.stderr)
                continue
            entries.append([parts[0], parts[1], parts[2] if len(parts) > 2 else ""])
    return entries, header


def write_manifest(path, entries, header):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        for h in header:
            fh.write(h + "\n")
        for sha, rel, size in sorted(entries, key=lambda e: e[1]):
            fh.write("%s  %s  %s\n" % (sha, rel, size))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default=DEFAULT)
    ap.add_argument("--update", nargs="*", default=None,
                    help="re-stamp every listed file, and add the paths given")
    args = ap.parse_args()

    if not os.path.exists(args.manifest):
        print("no manifest at %s (create it with --update <paths>)" % args.manifest, file=sys.stderr)
        return 1
    entries, header = read_manifest(args.manifest)
    if not entries and args.update is None:
        print("the manifest lists no files", file=sys.stderr)
        return 1

    if args.update is None:
        bad, missing, ok = [], [], 0
        for sha, rel, size in entries:
            p = os.path.join(ROOT, rel)
            if not os.path.exists(p):
                missing.append(rel)
                continue
            now = sha256(p)
            if now != sha:
                bad.append((rel, sha[:12], now[:12]))
            else:
                ok += 1
        print("frozen set: %d of %d files unchanged" % (ok, len(entries)))
        for rel in missing:
            print("  MISSING  %s" % rel)
        for rel, was, now in bad:
            print("  CHANGED  %s  (%s -> %s)" % (rel, was, now))
        if bad or missing:
            print("\nFROZEN SET BROKEN: %d changed, %d missing.  Mission A is frozen: restore the file, or "
                  "re-stamp deliberately with --update (which invalidates the mission-A numbers in "
                  "docs/STATUS.md unless they are re-measured)." % (len(bad), len(missing)))
            return 1
        print("OK: every frozen mission-A artifact is byte-for-byte unchanged.")
        return 0

    # --update: re-stamp, printing every change so it cannot be silent
    known = {rel: (sha, size) for sha, rel, size in entries}
    for extra in args.update:
        rel = os.path.relpath(os.path.abspath(extra), ROOT).replace("\\", "/")
        if rel not in known:
            known[rel] = (None, "")
    new_entries = []
    for rel, (old_sha, _old_size) in sorted(known.items()):
        p = os.path.join(ROOT, rel)
        if not os.path.exists(p):
            print("  DROPPED  %s (gone)" % rel)
            continue
        now = sha256(p)
        size = os.path.getsize(p)
        if old_sha is None:
            print("  ADDED    %s  %s  (%d bytes)" % (rel, now[:12], size))
        elif old_sha != now:
            print("  RE-STAMPED %s  %s -> %s" % (rel, old_sha[:12], now[:12]))
        new_entries.append([now, rel, str(size)])
    write_manifest(args.manifest, new_entries, header)
    print("wrote %s with %d files" % (args.manifest, len(new_entries)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
