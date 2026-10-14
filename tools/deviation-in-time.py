#!/usr/bin/env python3
"""WHERE is the vehicle not on its plan?  Time-resolve the deviation that breaks C2.

The state of the question (2026-10-14, docs/metrics/phaseB-c2-requirement.txt): on the completed 12-stop
sortie B9 the published plan holds exactly 0.200 m, and the truth still goes 0.342 m inside a pillar --
and raising the plan's clearance does NOT help (the fraction of truth samples below the contract stays
~21 % even where the plan is 0.5-0.75 m clear).  So the deviation itself is the whole story, and this tool
says WHEN it happens, because the fix is different for each answer:

  * during a trajectory  -> the follower lags or cuts corners (a control problem);
  * between trajectories -> the vehicle is holding while the plan has moved on (a sequencing problem);
  * at a plan SWITCH     -> the first fraction of a second of a new plan is a step (a switching problem);
  * at take-off/landing  -> the truth is off-plan for reasons no planner controls.

usage: tools/deviation-in-time.py --run logs/missionA-B9-circle-12 --world worlds/forest_b.yaml
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from tools.metrics import read_tap, read_phase     # noqa: E402


def load_calib(path):
    from common import frames
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))


def clarity(pil, P):
    out = np.full(len(P), np.inf)
    for x, y, r in pil:
        out = np.minimum(out, np.hypot(P[:, 0] - x, P[:, 1] - y) - r)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_b.yaml"))
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--fresh-s", type=float, default=0.5)
    args = ap.parse_args()

    calib = load_calib(args.calib)
    w = yaml.safe_load(open(args.world))
    pil = np.array([[q["x"], q["y"], q["r"]] for q in w["pillars"]])
    P = np.genfromtxt(args.run + "/traj.csv.points.csv", delimiter=",", names=True, dtype=None,
                      encoding="utf-8")
    t_msg = P["t_us"].astype(np.int64)
    pw = (calib.R_WL @ np.vstack([P["e"], P["n"], P["u"]])).T + calib.t_WL
    uniq = np.unique(t_msg)
    seg = {int(t): pw[t_msg == t] for t in uniq}
    t_uniq_s = uniq * 1e-6

    t_tap, p_w = read_tap(os.path.join(args.run, "tap.csv"))
    t_tap_s = t_tap.astype(np.int64) * 1e-6
    ph = read_phase(os.path.join(args.run, "phase.csv"))
    m = np.flatnonzero(p_w[:, 2] > 0.5)
    tu = np.arange(t_tap_s[m][0], t_tap_s[m][-1], 0.01)
    pu = np.column_stack([np.interp(tu, t_tap_s[m], p_w[m, k]) for k in range(3)])
    starts = np.array([seg[int(t)][0] for t in uniq])
    c0 = float(t_uniq_s[0] - tu[0])
    best, off = None, c0
    for cand in np.arange(c0 - 60, c0 + 60, 0.02):
        j = np.clip(np.searchsorted(tu, t_uniq_s - cand), 0, len(tu) - 1)
        r = float(np.median(np.hypot(pu[j, 0] - starts[:, 0], pu[j, 1] - starts[:, 1])))
        if best is None or r < best:
            best, off = r, cand
    print("clock offset %+.3f s (median |truth - path start| %.3f m)" % (off, best))

    # per-airborne-sample: the plan in force (nearest in time, must be fresh), lateral deviation, clearances
    tt = (t_tap_s + off) * 1e6
    idx = np.clip(np.searchsorted(uniq, tt), 0, len(uniq) - 1)
    rows_all = np.flatnonzero(p_w[:, 2] > 0.5)
    lat = np.full(len(p_w), np.nan)
    clr_pl = np.full(len(p_w), np.nan)
    age = np.full(len(p_w), np.nan)
    for k, t in enumerate(uniq):
        rr0 = rows_all[idx[rows_all] == k]
        if not len(rr0):
            continue
        a0 = np.abs(t_tap_s[rr0] + off - t * 1e-6)
        keep = a0 <= args.fresh_s
        rr, a = rr0[keep], a0[keep]           # the mask must be applied to BOTH, or the shapes diverge
        if not len(rr):
            continue
        q = seg[int(t)]
        d = np.hypot(q[None, :, 0] - p_w[rr, 0][:, None], q[None, :, 1] - p_w[rr, 1][:, None])
        j = np.argmin(d, axis=1)
        lat[rr] = d[np.arange(len(rr)), j]
        clr_pl[rr] = np.array([float(np.min(np.hypot(pil[:, 0] - q[jj, 0], pil[:, 1] - q[jj, 1])
                                          - pil[:, 2])) for jj in j])
        age[rr] = a
    ok = np.flatnonzero(np.isfinite(lat))
    clr_tr = clarity(pil, p_w)
    print("joined %d airborne samples with a fresh plan" % len(ok))

    # ---- by phase ---------------------------------------------------------------------------------
    names = [p["phase"] for p in ph]
    t_ph = np.array([float(p["t_sim_us"]) * 1e-6 for p in ph])
    print()
    print("%-14s %8s %8s %8s %8s %8s" % ("phase", "samples", "lat p50", "lat p95", "clr_tr p05", "below .2"))
    bounds = list(zip(t_ph, list(t_ph[1:]) + [t_tap_s[-1]]))
    for (t0, t1), nm in zip(bounds, names):
        sel = ok[(t_tap_s[ok] >= t0) & (t_tap_s[ok] < t1)]
        if len(sel) < 20:
            continue
        print("%-14s %8d %8.3f %8.3f %8.3f %8.1f%%"
              % (nm, len(sel), np.median(lat[sel]), np.percentile(lat[sel], 95),
                 np.percentile(clr_tr[sel], 5), 100.0 * (clr_tr[sel] < 0.20).mean()))

    # ---- how the deviation grows with the AGE of the plan -----------------------------------------
    print()
    print("deviation vs how old the plan is (a lag ALONG the path cannot change the distance to the curve):")
    for lo, hi in ((0.0, 0.05), (0.05, 0.10), (0.10, 0.20), (0.20, 0.30), (0.30, 0.50)):
        k = ok[(age[ok] >= lo) & (age[ok] < hi)]
        if len(k) < 50:
            continue
        print("   age [%.2f,%.2f) s: n %6d  lat p50 %.3f  p95 %.3f  max %.3f  below 0.20 m: %.1f %%"
              % (lo, hi, len(k), np.median(lat[k]), np.percentile(lat[k], 95), lat[k].max(),
                 100.0 * (clr_tr[k] < 0.20).mean()))

    # ---- right after a plan SWITCH -----------------------------------------------------------------
    switch_t = t_uniq_s - off                      # sim times at which a new plan appeared
    print()
    print("deviation in the first moments after a NEW plan appears:")
    for lo, hi in ((0.0, 0.3), (0.3, 0.6), (0.6, 1.0), (1.0, 2.0), (2.0, 99.0)):
        sel = []
        for t in switch_t:
            sel.extend(np.flatnonzero((t_tap_s >= t + lo) & (t_tap_s < t + hi) & np.isfinite(lat)))
        sel = np.array(sel, dtype=int)
        if len(sel) < 50:
            continue
        print("   +%.1f-%.1f s after a switch: n %6d  lat p50 %.3f  p95 %.3f  below 0.20 m: %.1f %%"
              % (lo, hi, len(sel), np.median(lat[sel]), np.percentile(lat[sel], 95),
                 100.0 * (clr_tr[sel] < 0.20).mean()))

    # ---- the worst windows -------------------------------------------------------------------------
    print()
    print("the ten worst 1 s windows of the whole flight (truth clearance):")
    sec = np.floor(t_tap_s - t_tap_s[0]).astype(int)
    worst = []
    for s in np.unique(sec[ok]):
        sel = ok[sec[ok] == s]
        if len(sel) < 100:
            continue
        worst.append((float(clr_tr[sel].min()), int(s), float(np.median(lat[sel])),
                      float(clr_pl[sel].min())))
    worst.sort()
    for c, s, l, cp in worst[:10]:
        nm = [n for (t0, t1), n in zip(bounds, names) if t0 <= t_tap_s[0] + s < t1]
        print("   t %+4d s  truth clearance %+.3f m  (plan clearance %+.3f, deviation p50 %.3f)  phase %s"
              % (s, c, cp, l, nm[0] if nm else "?"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
