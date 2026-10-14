#!/usr/bin/env python3
"""One sortie, scored against the mission's own gates -- every number with its denominator.

The gates come from the run's own `metrics.txt` (computed by tools/metrics.py from the tap, the odometry,
the phase log and the FCU's BIN), and the three things metrics.txt cannot see are added here:

  * the SEQUENCER's tally: how many of how many stops were REACHED / MISSED (from sequencer.out)
  * the PLANNER's health: how long ego went without committing a new trajectory (from bspline.csv, which
    tools/record-bspline.py writes), and whether the planner or the adapter ever announced a stall
  * what the plan stream looked like: how many distinct plans the FSM was offered

usage: tools/sortie-score.py <run-dir> [...]
"""
from __future__ import annotations

import os
import re
import sys


def line(path, pat):
    if not os.path.exists(path):
        return None
    for l in open(path, errors="replace"):
        if re.search(pat, l):
            return l.rstrip()
    return None


def seq_tally(run):
    p = os.path.join(run, "sequencer.out")
    if not os.path.exists(p):
        return None
    txt = open(p, errors="replace").read()
    reached = len(re.findall(r"stop (\d+)/(\d+) REACHED", txt))
    missed = re.findall(r"stop (\d+)/(\d+) MISSED", txt)
    total = re.findall(r"stop (\d+)/(\d+) ", txt)
    n_total = int(total[-1][1]) if total else 0
    last = line(p, r"sequence complete")
    return {"reached": reached, "missed": len(missed), "total": n_total, "last": (last or "").strip()}


def planner_health(run):
    out = {"commits": 0, "gap_med": float("nan"), "gap_max": float("nan"), "commits_after": 0}
    p = os.path.join(run, "bspline.csv")
    if os.path.exists(p):
        first = {}
        with open(p) as fh:
            fh.readline()
            for l in fh:
                f = l.split(",")
                if len(f) < 5:
                    continue
                t, tid = int(f[0]), int(f[3])
                first.setdefault(tid, t)
        ts = sorted(first.values())
        out["commits"] = len(ts)
        if len(ts) > 1:
            gaps = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
            gaps.sort()
            out["gap_med"] = gaps[len(gaps) // 2]
            out["gap_max"] = gaps[-1]
    log = os.path.join(run, "planner.log")
    if os.path.exists(log):
        txt = open(log, errors="replace").read()
        out["ego_stall_lines"] = len(re.findall(r"PLANNER-STALL", txt))
        out["adapter_stalls"] = len(re.findall(r"PLANNER STALLED", txt))
        out["withheld"] = len(re.findall(r"PATH WITHHELD", txt))
    return out


def score(run):
    name = os.path.basename(run.rstrip("/"))
    m = os.path.join(run, "metrics.txt")
    if not os.path.exists(m):
        print("== %s: no metrics.txt ==\n" % name)
        return
    txt = open(m, errors="replace").read()
    get = lambda pat: (re.search(pat, txt).group(0).strip() if re.search(pat, txt) else "n/a")   # noqa: E731
    c2 = re.search(r"C2 obstacle clearance.*?minimum pillar-surface distance (-?[\d.]+) m;\s+(\d+) of (\d+) samples inside the 0.20 m footprint\s+\[(\w+)\]", txt, re.S)
    a1 = re.search(r"A1 route horizontal rms\s+([\d.]+) mm", txt)
    a2 = re.search(r"A2 route vertical rms\s+([\d.]+) mm", txt)
    a4 = re.search(r"A4 FCU estimate vs truth.*?horizontal rms\s+([\d.]+) mm", txt, re.S)
    c1 = re.search(r"C1 final point: the aircraft stopped ([\d.]+) m from the last commanded point \(([^)]*)\)", txt)
    b4 = re.search(r"B4 landing: 'land' in the phase log -> (\w+)", txt)
    a3 = re.search(r"\[(PASS|FAIL)\] A3", txt)
    ab = re.search(r"aborted (\w+)", txt)
    seq = seq_tally(run)
    ph = planner_health(run)
    print("== %s ==" % name)
    if seq:
        print("   mission    : %d of %d stops REACHED, %d MISSED; %s"
              % (seq["reached"], seq["total"], seq["missed"], seq["last"] or "(no 'sequence complete' line)"))
    print("   A1 %s mm (<=20)   A2 %s mm (<=50)   A4 %s mm (<=50)   A3 %s   aborted %s"
          % (a1.group(1) if a1 else "n/a", a2.group(1) if a2 else "n/a", a4.group(1) if a4 else "n/a",
             a3.group(1) if a3 else "n/a", ab.group(1) if ab else "n/a"))
    if c1:
        print("   C1 %s m from the last commanded point %s   B4 landed: %s"
              % (c1.group(1), c1.group(2), b4.group(1) if b4 else "n/a"))
    if c2:
        inside = int(c2.group(2))
        print("   C2 min clearance %s m   %d of %d samples inside the 0.20 m footprint = %.2f %%   [%s]   (0 inside a pillar is the hard line)"
              % (c2.group(1), inside, int(c2.group(3)), 100.0 * inside / max(1, int(c2.group(3))), c2.group(4)))
    print("   planner    : %d committed trajectories; gap between commits median %.2f s, max %.2f s; "
          "ego stall lines %d, adapter stalls %d, withheld %d"
          % (ph.get("commits", 0), ph.get("gap_med", float("nan")), ph.get("gap_max", float("nan")),
             ph.get("ego_stall_lines", 0), ph.get("adapter_stalls", 0), ph.get("withheld", 0)))
    print()


if __name__ == "__main__":
    runs = sys.argv[1:]
    if not runs:
        print(__doc__)
        sys.exit(2)
    for r in runs:
        score(r if os.path.isabs(r) else os.path.join("/home/fanxin/lio_avoid/logs", r))
