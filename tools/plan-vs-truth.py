#!/usr/bin/env python3
"""Compare what was PUBLISHED with what was FLOWN: the plan's clearance against the truth's.

The question this answers (2026-10-14): C2 measured the TRUTH 55.9 % inside the 0.20 m contract and 1.6 %
inside a pillar on a flight where the adapter had published only verified-clear paths.  Both can be true
only if the vehicle does not follow the plan, and there was no record of the plans to check.  With
`tools/record-traj.py` recording them, this tool says which, in numbers:

  * |truth - plan| horizontally and vertically, for the plan that was in force at each instant;
  * the plan's own minimum clearance next to the truth's;
  * the clearance of the plan AT THE TRUTH's worst moment -- the one number that separates "the plan was
    too tight" from "the vehicle left the plan";
  * how many plan switches there were and how far the vehicle was from the new plan when it switched
    (a switch is where corner-cutting happens).

Clock: the trajectory stamps are wall-clock and the tap is sim/sim-clock, so the offset is FITTED the same
way A4 fits it (minimise the vertical mismatch while airborne) and reported, not assumed.

usage:
    tools/plan-vs-truth.py --run logs/missionA-B2b-circle-12 --world worlds/forest_b.yaml
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
from tools.metrics import read_tap            # noqa: E402


def load_calib(path):
    """The SAME calibration object the FSM and the adapter use.

    Imported as a MODULE, not by file path: `common/frames.py` defines a dataclass, and a dataclass created
    from a module that importlib never registered in `sys.modules` dies with
    `AttributeError: 'NoneType' object has no attribute '__dict__'` inside dataclasses itself.  (MEASURED
    2026-10-14: that is exactly how the first version of this tool and the first version of the plot's plan
    overlay both failed, the second one silently falling back to drawing plans in the L frame.)
    """
    from common import frames
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))


def clarity(pil, p):
    return np.min(np.hypot(pil[:, 0] - p[0], pil[:, 1] - p[1]) - pil[:, 2])


def clarity_many(pil, P):
    out = np.full(len(P), np.inf)
    for x, y, r in pil:
        out = np.minimum(out, np.hypot(P[:, 0] - x, P[:, 1] - y) - r)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_b.yaml"))
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    tj = os.path.join(args.run, "traj.csv")
    pts = tj + ".points.csv"
    if not (os.path.exists(tj) and os.path.exists(pts)):
        print("no traj.csv(+.points.csv) in %s -- was tools/record-traj.py running?" % args.run,
              file=sys.stderr)
        return 1
    try:
        hdr = np.genfromtxt(tj, delimiter=",", names=True, dtype=None, encoding="utf-8")
        P = np.genfromtxt(pts, delimiter=",", names=True, dtype=None, encoding="utf-8")
    except Exception as exc:
        print("could not read the recording: %s" % exc, file=sys.stderr)
        return 1
    if hdr.ndim == 0:
        hdr = hdr.reshape(1)
    if P.ndim == 0:
        P = P.reshape(1)
    if len(hdr) == 0 or len(P) == 0:
        print("the recording is empty: no /planner/traj message arrived during the run", file=sys.stderr)
        return 1
    calib = load_calib(args.calib)
    world = yaml.safe_load(open(args.world))
    pil = np.array([[q["x"], q["y"], q["r"]] for q in world["pillars"]])

    t_tap, p_w = read_tap(os.path.join(args.run, "tap.csv"))
    t_tap_s = t_tap.astype(np.int64) * 1e-6
    # the plan is in L; the tap is in W.  Convert the PLAN to W with the FSM's own calibration -- comparing
    # in two frames would be comparing nothing.
    t_msg = P["t_us"].astype(np.int64)
    lm = np.vstack([P["e"], P["n"], P["u"]])
    pw = (calib.R_WL @ lm).T + calib.t_WL
    uniq = np.unique(t_msg)
    seg = {int(tm): pw[t_msg == tm] for tm in uniq}
    t_msg_s = uniq * 1e-6                              # ONE time per message, not per point (P is per point)

    # CLOCK.  The trajectory stamps are EPOCH seconds and the tap's are SITL sim seconds, so what is needed
    # is one constant offset -- and it is FITTED on the property the interface itself guarantees, not on a
    # guess: PLAN.md 4.1 says a published path must start within 0.5 m of the vehicle state.  So the offset
    # that minimises the median |truth(t) - path_start| over all messages is the right one; a search on the
    # vertical alone (the first version) walked into its own +-40 s bound and reported a 5.5 m lateral
    # deviation that was pure clock error.
    starts = np.array([seg[int(tm)][0] for tm in uniq])
    m = np.flatnonzero(p_w[:, 2] > 0.5)
    if len(m) < 50:
        print("the truth never got above 0.5 m -- nothing to compare", file=sys.stderr)
        return 1
    # the truth as a function of time, uniformly resampled so a candidate offset can be evaluated with one
    # interpolation instead of a search through 100 k samples per candidate
    tu = np.arange(t_tap_s[m][0], t_tap_s[m][-1], 0.01)
    pu = np.column_stack([np.interp(tu, t_tap_s[m], p_w[m, k]) for k in range(3)])
    c0 = float(t_msg_s[0] - tu[0])                     # both recordings start with the run
    best, off = None, c0
    for cand in np.arange(c0 - 120.0, c0 + 120.0, 0.02):
        tt = t_msg_s - cand
        j = np.clip(np.searchsorted(tu, tt), 0, len(tu) - 1)
        d = np.hypot(pu[j, 0] - starts[:, 0], pu[j, 1] - starts[:, 1])
        r = float(np.median(d))
        if best is None or r < best:
            best, off = r, cand
    print("clock: fitted offset %+.3f s (epoch stamps - sim stamps); median |truth - path start| at that "
          "offset %.3f m over %d plans -- the interface's own 0.5 m obligation is what it is fitted on"
          % (off, best, len(uniq)))
    if best > 1.0:
        print("  WARNING: the best fit is %.3f m, so the alignment is not convincing" % best)

    # join: each airborne truth sample -> the plan message nearest in time, then the nearest point on it
    tt = (t_tap_s + off) * 1e6
    idx = np.clip(np.searchsorted(uniq, tt), 0, len(uniq) - 1)
    lat = np.full(len(tt), np.nan)
    ver = np.full(len(tt), np.nan)
    clr_pl = np.full(len(tt), np.nan)
    order = np.argsort(idx, kind="stable")
    bounds = np.searchsorted(idx[order], np.arange(len(uniq) + 1))
    n_switch = 0
    prev = None
    for k in range(len(uniq)):
        rows = order[bounds[k]:bounds[k + 1]]
        if not len(rows):
            continue
        q = seg[int(uniq[k])]
        d = np.hypot(q[None, :, 0] - p_w[rows, 0][:, None], q[None, :, 1] - p_w[rows, 1][:, None])
        j = np.argmin(d, axis=1)
        lat[rows] = d[np.arange(len(rows)), j]
        ver[rows] = np.abs(q[j, 2] - p_w[rows, 2])
        clr_pl[rows] = np.array([clarity(pil, q[jj]) for jj in j])
        prev = k
    air = np.flatnonzero((p_w[:, 2] > 0.3) & np.isfinite(lat))
    clr_true = clarity_many(pil, p_w)
    print()
    print("plan vs truth: %d joined samples, %d truth samples airborne, %d published Path messages, "
          "%d distinct plans" % (len(air), len(m), len(t_msg), len(uniq)))
    print("  lateral  |truth - plan|   median %.3f m, p95 %.3f m, max %.3f m"
          % (np.median(lat[air]), np.percentile(lat[air], 95), lat[air].max()))
    print("  vertical |truth - plan|   median %.3f m, p95 %.3f m, max %.3f m"
          % (np.median(ver[air]), np.percentile(ver[air], 95), ver[air].max()))
    print("  the PLAN's own clearance: worst published %.3f m over %d messages (contract 0.20)"
          % (hdr["clr_m"].min(), len(hdr)))
    print("  the TRUTH's clearance   : min %.3f m; %.1f %% of airborne samples below 0.20 m, "
          "%.2f %% inside a pillar"
          % (clr_true[air].min(), 100.0 * (clr_true[air] < 0.20).mean(),
             100.0 * (clr_true[air] < 0.0).mean()))
    w = air[int(np.argmin(clr_true[air]))]
    print("  at the truth's worst sample (t %.1f s, E %+.3f N %+.3f U %.3f): the plan in force was "
          "%.3f m from the truth and %.3f m clear of the pillars"
          % (t_tap_s[w] - t_tap_s[0], p_w[w, 0], p_w[w, 1], p_w[w, 2], lat[w], clr_pl[w]))
    # how often was the truth below the contract while its plan was above it?
    both = (clr_true[air] < 0.20) & (clr_pl[air] >= 0.20)
    print("  of the samples that break the contract, %.1f %% were following a plan that HELD it "
          "(so the vehicle, not the plan, is what went below)" % (100.0 * both.sum() / max((clr_true[air] < 0.20).sum(), 1)))
    if args.out:
        np.savetxt(args.out, np.column_stack([t_tap_s[air] - t_tap_s[0], lat[air], ver[air],
                                              clr_pl[air], clr_true[air]]), delimiter=",",
                   header="t_s,lat_m,ver_m,clr_plan_m,clr_true_m", comments="")
        print("  wrote %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
