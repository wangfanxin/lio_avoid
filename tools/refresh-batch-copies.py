#!/usr/bin/env python3
"""Refresh the batch's per-flight metrics copies from the run directories they were copied out of.

A batch writes `logs/missionA-<prefix>-<stamp>-flightNN.metrics.txt` by copying
`logs/missionA-<prefix>-NN-<stamp>/metrics.txt` at the end of each flight.  When the metric definitions
change, the run directories are regenerated (tools/... offline) and these copies must follow, or the
aggregate silently mixes two versions of the metric -- which is exactly what it did once.
"""
from __future__ import annotations

import glob
import os
import re
import shutil
import sys

BASE = "/home/fanxin/lio_avoid/logs"
pat = re.compile(r"missionA-(?P<prefix>.+)-(?P<stamp>\d{8}_\d{6})-flight(?P<n>\d+)\.metrics\.txt$")
n = 0
missing = []
for f in sorted(glob.glob(os.path.join(BASE, "missionA-*-flight*.metrics.txt"))):
    m = pat.search(os.path.basename(f))
    if not m:
        continue
    run = os.path.join(BASE, "missionA-%s-%s-%s" % (m.group("prefix"), m.group("n"), m.group("stamp")))
    src = os.path.join(run, "metrics.txt")
    if not os.path.exists(src):
        missing.append(run)
        continue
    shutil.copyfile(src, f)
    n += 1
print("refreshed %d batch copies" % n)
if missing:
    print("no run directory for %d of them:" % len(missing))
    for x in missing[:10]:
        print("   %s" % x)
sys.exit(0)
