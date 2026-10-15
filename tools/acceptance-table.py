#!/usr/bin/env python3
"""The acceptance table, computed -- one row per sortie, every cell with its denominator.

WHY THIS EXISTS.  The owner's acceptance is a set of numbers that must be reported WITH denominators
("4 of 4 corners", "0 of N samples inside 0.20 m", "N of M sorties"), and F3 has to report a RATIO over
several sorties.  Hand-assembling that from logs is how a project ends up quoting a number nobody can
reproduce.  This tool reads each run's own files, calls the SAME judge the owner's criterion names
(`tools/dwell-audit.py` for the hover, `metrics.txt` for the rest), and refuses to invent a cell it
cannot read.

usage: tools/acceptance-table.py <run dir> [<run dir> ...] [--airborne-m 0.5] [--figure out.png]
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)

ROW_KEYS = {
    "A1": r"A1 route horizontal rms\s+([0-9.]+) mm",
    "A2": r"A2 route vertical rms\s+([0-9.]+) mm",
    "A4": r"A4 FCU estimate.*?horizontal rms\s+([0-9.]+) mm",
    "C1": r"C1 final point: the aircraft stopped ([0-9.]+) m",
    "C2m": r"minimum pillar-surface distance ([0-9.]+) m",
    "C2inside": r"(\d+) of (\d+) samples inside the ([0-9.]+) m footprint",
    "B1E": r"B1 truth box extents over the route \([0-9.]+ s\): E ([0-9.]+) m\s+N ([0-9.]+) m",
}


def read_metrics(run):
    path = os.path.join(run, "metrics.txt")
    if not os.path.exists(path):
        return None
    txt = open(path, encoding="utf-8", errors="replace").read()
    got = {}
    for k, pat in ROW_KEYS.items():
        m = re.search(pat, txt, re.S)
        if not m:
            continue
        got[k] = m.groups() if m.lastindex and m.lastindex > 1 else m.group(1)
    got["B2_pass"], got["B2_total"] = 0, 0
    for m in re.finditer(r"leg \d+:.*?PASS", txt):
        got["B2_pass"] += 1
    got["B2_total"] = len(re.findall(r"leg \d+:\s+\d+ samples", txt))
    got["B3_pass"] = len(re.findall(r"corner \d+: closest horizontal approach.*?PASS", txt))
    got["B3_total"] = len(re.findall(r"corner \d+: closest horizontal approach", txt))
    got["B4"] = "yes" if re.search(r"B4 landing: 'land' in the phase log -> yes", txt) else "NO"
    return got


def read_dwell(run, airborne_m):
    """The owner's judge, called -- not reimplemented."""
    cmd = [sys.executable, os.path.join(HERE, "dwell-audit.py"), run, "--radius", "0.15"]
    if airborne_m is not None:
        cmd += ["--airborne-m", str(airborne_m)]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    n_ok, n_all = None, None
    m = re.search(r"CORNERS HOVERED >= ([0-9.]+) s:\s+(\d+) of (\d+)", out)
    if m:
        n_ok, n_all = int(m.group(2)), int(m.group(3))
    spans = {}
    for m in re.finditer(r"(c\d+) \(\+[0-9.]+,([+-][0-9.]+)\): closest [0-9.]+ m at t=\s*([0-9.]+) s \| "
                         r"longest dwell within 0.15 m:\s*([0-9.]+) s\s+(\S+)", out):
        spans[m.group(1)] = (float(m.group(4)), m.group(5))
    return n_ok, n_all, spans


def read_faults(run):
    n = 0
    for f, pat in (("fsm.log", r"ABORT"), ("planner.log", r"NEVER ARRIVED"), ("traj.log", r"NEVER ARRIVED"),
                   ("fsm.log", r"EKF Failsafe|EKF failsafe")):
        p = os.path.join(run, f)
        if os.path.exists(p):
            n += len(re.findall(pat, open(p, encoding="utf-8", errors="replace").read()))
    return n


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--airborne-m", type=float, default=0.5,
                    help="the stricter judged quantity (0 disables: pass 0)")
    ap.add_argument("--figure", default="")
    args = ap.parse_args()

    runs = []
    for r in args.runs:
        runs.extend(sorted(glob.glob(r)) if any(c in r for c in "*?[") else [r])

    air = None if args.airborne_m == 0 else args.airborne_m
    table, ok_sorties = [], 0
    print(f"{'sortie':<26}{'hover':>7}{'corner visits':>15}{'A1 mm':>7}{'A2 mm':>7}{'A4 mm':>7}"
          f"{'B2':>7}{'C2 min m':>10}{'inside 0.20':>13}{'B4':>5}{'faults':>7}")
    for run in runs:
        name = os.path.basename(run.rstrip("/")).replace("missionA-", "")
        mt = read_metrics(run)
        n_ok, n_all, spans = read_dwell(run, air)
        faults = read_faults(run)
        if mt is None or n_ok is None:
            print(f"{name:<26}  (incomplete: metrics={mt is not None}, dwell={n_ok is not None})")
            continue
        if n_ok == n_all:
            ok_sorties += 1
        c2 = mt.get("C2inside", ("?", "?", "?"))
        b1e = mt.get("B1E", ("?", "?"))
        b2 = f"{mt.get('B2_pass', 0)} of {mt.get('B2_total', 0)}"
        print(f"{name:<26}{f'{n_ok} of {n_all}':>7}{n_all:>15}"
              f"{mt.get('A1','?'):>7}{mt.get('A2','?'):>7}{mt.get('A4','?'):>7}"
              f"{b2:>7}"
              f"{mt.get('C2m','?'):>10}{f'{c2[0]} of {c2[1]}':>13}{mt['B4']:>5}{faults:>7}")
        table.append((name, n_ok, n_all, spans, mt, faults))

    if table:
        tot_ok = sum(t[1] for t in table)
        tot_all = sum(t[2] for t in table)
        print(f"\nPOOLED: {ok_sorties} of {len(table)} sorties with 4 of 4 corners hovered >= 5.0 s "
              f"(strict, airborne > {air} m);  {tot_ok} of {tot_all} corner visits >= 5.0 s")
        for name, _n, _a, spans, _mt, _f in table:
            worst = min(spans.items(), key=lambda kv: kv[1][0]) if spans else None
            if worst:
                print(f"   {name:<24} tightest corner: {worst[0]} {worst[1][0]:.2f} s ({worst[1][1]})")

    if args.figure and table:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
        for _c in ("/mnt/c/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/simhei.ttf"):
            if os.path.exists(_c):
                font_manager.fontManager.addfont(_c)
                plt.rcParams["font.sans-serif"] = [font_manager.FontProperties(fname=_c).get_name()]
                break
        plt.rcParams["axes.unicode_minus"] = False
        names = [t[0] for t in table]
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(14.5, 5.4))
        w = 0.8 / max(1, len(table[0][3]))
        for i, n in enumerate(names):
            spans = table[i][3]
            for j, (c, (v, verdict)) in enumerate(sorted(spans.items())):
                a1.bar(i + (j - len(spans) / 2) * w, v, w * 0.9,
                       color="#2e9e5b" if v >= 5.0 else "#d9822b")
                a1.text(i + (j - len(spans) / 2) * w, v + 0.08, f"{v:.1f}", ha="center", fontsize=7.5,
                        rotation=90)
        a1.axhline(5.0, color="#b03030", ls="--", lw=1.4)
        a1.set_xticks(range(len(names))); a1.set_xticklabels(names, fontsize=10, rotation=12)
        a1.set_ylabel("每个角点最长连续驻留 [s]", fontsize=11)
        a1.set_title("每个角点的悬停（虚线 = 要求的 5.0 s）", fontsize=12)
        a1.grid(axis="y", alpha=0.3)
        inside = [float(t[4]["C2inside"][0]) for t in table]
        a2.bar(range(len(names)), inside, 0.5, color="#2f6db5")
        for i, v in enumerate(inside):
            a2.text(i, v + max(inside) * 0.02 if max(inside) else 0.02, f"{int(v)}", ha="center", fontsize=10)
        a2.set_xticks(range(len(names))); a2.set_xticklabels(names, fontsize=10, rotation=12)
        a2.set_ylabel("进入 0.20 m 足印的采样数", fontsize=11)
        a2.set_title("安全闸门：全部必须为 0", fontsize=12)
        a2.grid(axis="y", alpha=0.3)
        fig.suptitle("验收表（由 tools/acceptance-table.py 从各架次自己的文件算出）", fontsize=14)
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(args.figure, dpi=140)
        print("wrote", args.figure)
    return 0


if __name__ == "__main__":
    sys.exit(main())
