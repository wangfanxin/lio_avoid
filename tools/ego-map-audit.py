#!/usr/bin/env python3
"""那根被穿过的柱子，在 ego 自己的占据栅格里有吗？

输入（同一次架次录下的三样东西）：
  planner.log      拒收原文，现在带 traj start_time（epoch）与"第几个点"
  bspline.csv(.points.csv)  ego 提交的每条轨迹的控制点（L 系）
  ego_map.csv       ego 自己发布的 grid_map/occupancy_inflate（L 系，每帧一批占据格）
  世界文件 + calib 把柱子换到 L 系，用于和地图对账

输出：被穿最深的柱子（W/L 位置、穿深），以及 ego 地图里该柱子表面附近的占据格数量与覆盖比例。

usage: ego-map-audit.py <run-name>
"""
from __future__ import annotations

import os
import re
import sys

import numpy as np
import yaml

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames                                  # noqa: E402

L = "/home/fanxin/lio_avoid/logs"
W = "/mnt/d/distill_self/lio_avoid/worlds/forest_b.yaml"
CALIB = "/mnt/d/distill_self/_stage/calib.json"


def load_calib():
    import json
    d = json.load(open(CALIB))
    R = np.array(d.get("R_WL") or d.get("R"), float)
    t = np.array(d.get("t_WL") or d.get("t"), float)
    return R, t


def main(name):
    run = os.path.join(L, name)
    w = yaml.safe_load(open(W))
    pil = w.get("pillars") or w.get("obstacles") or []
    P = np.array([[q["x"], q["y"]] for q in pil])
    r = float(pil[0].get("radius", 0.4))
    R_WL, t_WL = load_calib()

    # (1) the refused trajectories, from the log's new message
    refusals = []
    for line in open(os.path.join(run, "planner.log"), errors="replace"):
        m = re.search(r"pillar clearance (-?[\d.]+) m < footprint [\d.]+ m \(at point (\d+), traj ([\d.]+),", line)
        if m:
            refusals.append((float(m.group(1)), int(m.group(2)), float(m.group(3))))
    print("== %s: %d refusals with a trajectory id ==" % (name, len(refusals)))
    if not refusals:
        print("   (none -- did this run use the instrumented adapter?)\n")
        return

    # (2) the control points of those trajectories (L)
    cp = {}
    with open(os.path.join(run, "bspline.csv.points.csv")) as fh:
        fh.readline()
        for l in fh:
            f = l.split(",")
            if len(f) < 6:
                continue
            cp.setdefault(int(f[1]), []).append((int(f[2]), float(f[3]), float(f[4]), float(f[5])))
    # the message's traj start_time is an epoch float; the csv keys traj_id (int) -- match by proximity
    starts = {}
    with open(os.path.join(run, "bspline.csv")) as fh:
        hdr = fh.readline().strip().split(",")
        for l in fh:
            f = l.strip().split(",")
            if len(f) < len(hdr):
                continue
            d = dict(zip(hdr, f))
            starts[int(d["traj_id"])] = float(d["start_us"]) * 1e-6

    # (3) for each refused trajectory: its worst offending point, in L, and which pillar it is inside
    print("   %-22s %-8s %-26s %s" % ("traj (epoch)", "refusals", "worst control point (L)", "the pillar it is inside (W)"))
    worst = None
    seen = set()
    for clr, pt, tid in refusals:
        if tid in seen:
            continue
        seen.add(tid)
        cand = [k for k, v in starts.items() if abs(v - tid) < 0.02]
        if not cand:
            continue
        pts = np.array([[x, y, z] for _, x, y, z in sorted(cp.get(cand[0], []))])
        if len(pts) < 2:
            continue
        lon = pts - t_WL
        pw = (R_WL @ lon.T).T + t_WL                        # L -> W (R_WL is the L->W rotation by construction)
        d = np.hypot(pw[None, :, 0] - P[:, None, 0], pw[None, :, 1] - P[:, None, 1]) - r
        j = int(np.argmin(d))
        pi, pj = np.unravel_index(j, d.shape)        # d is (pillar, control point)
        inside = d[pi, pj]
        print("   %-22.3f %-8s L[%+.2f %+.2f %+.2f]      #%d at W[%+.2f %+.2f]  plan is %+.3f m inside"
              % (tid, "%d<%.2f" % (len(refusals), clr), pw[pj, 0], pw[pj, 1], pw[pj, 2], pi, P[pi, 0], P[pi, 1], inside))
        if worst is None or inside < worst[0]:
            worst = (inside, pi, pw[pj].copy())

    if worst is None:
        print("   (could not match any refusal to a recorded trajectory)\n")
        return
    depth, pj, pw_pt = worst
    print("   -> the worst penetration is %+.3f m into pillar #%d at W[%+.2f %+.2f]" % (depth, pj, P[pj, 0], P[pj, 1]))

    # (4) ego's own map: are there occupied cells on that pillar's shell?
    #     the shell in L is a circle around the pillar's centre, radius r
    cw = np.array([P[pj, 0], P[pj, 1]])
    cl = R_WL.T @ (np.r_[cw, 0.0] - t_WL) + t_WL
    shell = []
    with open(os.path.join(run, "ego_map.csv")) as fh:
        fh.readline()
        for l in fh:
            f = l.split(",")
            if len(f) < 5:
                continue
            x, y, z = float(f[2]), float(f[3]), float(f[4])
            if abs(z - cl[2]) > 2.0:
                continue
            dd = abs(np.hypot(x - cl[0], y - cl[1]) - r)
            if dd < 0.15:
                shell.append((int(f[0]), dd))
    scans = sorted({s for s, _ in shell})
    print("   ego's own map (grid_map/occupancy_inflate): %d occupied cells within 0.15 m of that pillar's shell, "
          "in %d of the recorded scans" % (len(shell), len(scans)))
    if shell:
        print("      -> the pillar IS in ego's map (in %d scans: %s%s)"
              % (len(scans), scans[:6], " ..." if len(scans) > 6 else ""))
    else:
        print("      -> the pillar is NOT in ego's map at all: the planner flew through a pillar its grid never marked")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "missionA-O1-mapdiag")
