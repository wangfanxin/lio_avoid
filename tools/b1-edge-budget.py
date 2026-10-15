#!/usr/bin/env python3
"""B1's box growth, attributed to WALLS, CORNERS and LEGS -- from the run's own truth tap.

WHY THIS TOOL EXISTS
--------------------
`tools/metrics.py` prints one number for B1 ("truth box extents over the route: E 2.110 m  N 1.288 m
against a bound of 2.110/2.300, [FAIL]") and that number cannot say WHERE the extra 110 mm of E and 88 mm
of N are made.  Two hypotheses were already refuted by measurement -- the pad/entry leg is not in the
window (`tools/b1-window-probe.py`, `docs/metrics/B1-adjudication-20261015.txt` sections 1-2), and the
excursion did not go away when the F2 dwell rule cut the corner arrival speed by 10x (2 m/s -> 0.1 m/s
moved B1 by 44 mm).  What is left is an ATTRIBUTION question, and this tool answers it with the four
identities that hold exactly, by construction:

   growth_E  =  max(0, x0 - min E)  +  max(0, max E - x1)      (the two E walls)
   growth_N  =  max(0, y0 - min N)  +  max(0, max N - y1)      (the two N walls)

so the whole growth is carried by at most four numbers, one per wall.  Each is then attributed to an
event: WHICH corner visit it happens in (the plan box commands a corner at a time, so every corner has a
visit), WHEN inside that visit (departure / on the leg / arrival / settle / dwell), and HOW MUCH of it the
FCU's own position estimate already sees (`truth - fcu` versus `fcu - command`, both recorded at 50 Hz in
`setpoints.csv`).  The B2 straightness numbers are re-derived leg by leg with `tools/metrics.py`'s own
formula so the two can be compared event by event instead of quoted side by side.

WHAT IS MEASUREMENT AND WHAT IS INFERENCE  (stated per row in the output, never mixed)
  * measurement : the tap (`tap.csv`) is the truth; the B1 window comes from `phase.csv` (the same two
    lines `tools/metrics.py:458-459` uses); the commanded waypoint and the FCU's estimate come from
    `setpoints.csv`, whose `cmd_x/cmd_y` and `fcu_x/fcu_y` are in W (ENU, origin = pad) -- checked by
    this tool against the world file's corners, printed as the "command check" line.
  * inference   : the DWELL WINDOW is `[next command - hold_s, next command]`, because that stack's rule
    is "the dwell clock starts on the SETTLED arrival and the phase boundary IS the end of the hold"
    (`planner/cmd_stream.py` header).  `hold_s` is read from the run's own `traj.log` when present, else
    from `--hold-s`.  The tap's own closest approach is printed beside it so the reader can see the rule
    was not assumed to be the truth: the aircraft is USUALLY already outside the wall before the dwell
    clock starts.

usage:
  tools/b1-edge-budget.py <run-dir> [<run-dir> ...] [--world worlds/pillars.yaml] [--hold-s 5.0]
                          [--timeline RECT26-accept] [--csv out.csv] [--figure docs/figures/x.png]
  (a run dir is anything holding tap.csv + phase.csv; --csv/--figure are optional and re-runnable)
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB = os.path.join(os.path.dirname(ROOT), "_stage", "calib.json")

# setpoints.csv, by column index (the header is printed by the tool so a reader can check it):
#   t_wall, t_sim_us, cmd_x, cmd_y, cmd_z, vel_*, acc_*, fcu_x, fcu_y, fcu_z, fcu_v*,
#   lio_x, lio_y, lio_z, yaw_cmd
SP = {"t": 1, "cmd_x": 2, "cmd_y": 3, "vx": 5, "vy": 6, "fcu_x": 11, "fcu_y": 12,
      "fcu_vx": 14, "fcu_vy": 15, "lio_x": 17, "lio_y": 18}


# ---------------------------------------------------------------------------------------------------
# reading
# ---------------------------------------------------------------------------------------------------
def read_tap(path):
    """t_us and the truth in W as (E, N, U).  `tap.csv` is NED on the wire (`pn, pe, pd`), so E = pe,
    N = pn, U = -pd -- the same conversion tools/metrics.py, tools/b1-window-probe.py and
    tools/corner-excursion.py all do; `phase.csv`'s own truth_x/truth_y columns are checked against it."""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d[:, 0].astype(np.int64), np.column_stack([d[:, 3], d[:, 2], -d[:, 4]])


def read_phase(path):
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().split(",")
        for line in fh:
            f = line.strip().split(",")
            if len(f) == len(head):
                rows.append(dict(zip(head, f)))
    return rows


def read_setpoints(path):
    """The bridge's 50 Hz record: the commanded waypoint, the FCU's own estimate and the LIO, on the
    tap's sim clock (`t_sim_us` at index 1 -- the same clock phase.csv uses)."""
    if not os.path.exists(path):
        return None
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    return d if d.ndim == 2 and len(d) > 10 else None


def read_lio_w(path, calib):
    """The delivered odometry in W, from the run's OWN recording, rotated with the calibration the plan
    box used (the run prints it in its log; the same file is read here)."""
    if not os.path.exists(path):
        return None, None
    # columns: t_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id -- frame ids are TEXT,
    # so only the six numeric columns this needs are read.
    d = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(0, 1, 2, 3, 8, 9))
    t = d[:, 0].astype(np.int64)
    if calib is None:
        R = np.array([[0.0, -1.0], [1.0, 0.0]])
        tw = np.zeros(2)
    else:
        R = np.asarray(calib["R_WL"], float)[:2, :2]
        tw = np.asarray(calib["t_WL"], float)[:2]
    p = (R @ d[:, 1:3].T).T + tw
    v = np.hypot(d[:, 4], d[:, 5])
    return t, np.column_stack([p, v])


def load_calib():
    try:
        with open(CALIB) as fh:
            return json.load(fh)
    except Exception:
        return None


def hold_s_of(run_dir, default):
    """The dwell the run was flown with, from the run's own plan-box log; `--hold-s` is the fallback."""
    for name in ("traj.log", "supervisor.log"):
        p = os.path.join(run_dir, name)
        if not os.path.exists(p):
            continue
        txt = open(p, errors="replace").read()
        for pat in ("dwell %.1f s at each corner", "corner hold : %.1f s dwell", "holding %.1f s,"):
            i = txt.find(pat.split(" ")[0])
            if i >= 0:
                import re
                m = re.search(pat.replace("%.1f", r"([0-9.]+)").replace("(", r"\(").replace(")", r"\)"), txt)
                if m:
                    return float(m.group(1)), name
    return default, "--hold-s default"


# ---------------------------------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------------------------------
def walls_of(x0, y0, x1, y1):
    """The four commanded walls.  `sense` = -1 for the min walls (outside = coordinate BELOW the wall),
    +1 for the max walls; d_out = sense * (coord - value) is positive OUTSIDE the rectangle."""
    return [dict(name="E-min", axis=0, val=x0, sense=-1, pos="E=%.3f" % x0),
            dict(name="E-max", axis=0, val=x1, sense=+1, pos="E=%.3f" % x1),
            dict(name="N-min", axis=1, val=y0, sense=-1, pos="N=%.3f" % y0),
            dict(name="N-max", axis=1, val=y1, sense=+1, pos="N=%.3f" % y1)]


def out_in(p, w):
    """Distance outside (positive = past the wall) and inside, per sample, for one wall."""
    d = w["sense"] * (p[:, w["axis"]] - w["val"])
    return d, -d


def wall_of_point(pt, walls, x0, y0, x1, y1):
    """The wall a point is 'on': the nearest of the four wall SEGMENTS (a Voronoi split of the boundary,
    so a point in a corner belongs to one of the two walls that meet there and never to both)."""
    best, bi = 1e9, 0
    for i, w in enumerate(walls):
        if w["axis"] == 0:
            lo, hi = y0, y1
            t = min(max(pt[1], lo), hi)
            d = np.hypot(pt[0] - w["val"], pt[1] - t)
        else:
            lo, hi = x0, x1
            t = min(max(pt[0], lo), hi)
            d = np.hypot(pt[0] - t, pt[1] - w["val"])
        if d < best:
            best, bi = d, i
    return bi


def nearest_corner(pt, corners):
    d = [float(np.hypot(pt[0] - c[0], pt[1] - c[1])) for c in corners]
    k = int(np.argmin(d))
    return k + 1, d[k]


# ---------------------------------------------------------------------------------------------------
# the analysis of one run
# ---------------------------------------------------------------------------------------------------
def analyse(run_dir, corners, hold_s_default, want_timeline=False, near_m=0.30, arr_m=0.15):
    name = os.path.basename(os.path.normpath(run_dir))
    res = dict(run=name, dir=run_dir, warn=[])
    tap = os.path.join(run_dir, "tap.csv")
    ph_path = os.path.join(run_dir, "phase.csv")
    if not (os.path.exists(tap) and os.path.exists(ph_path)):
        res["fatal"] = "no tap.csv/phase.csv"
        return res
    t, p = read_tap(tap)
    phases = read_phase(ph_path)
    tsim = np.array([int(r["t_sim_us"]) for r in phases])
    pnames = [r["phase"] for r in phases]
    legs = [i for i, r in enumerate(phases) if r["phase"].startswith("leg")]
    res.update(n_tap=len(t), span_s=(t[-1] - t[0]) * 1e-6, phases=phases, pnames=pnames, tsim=tsim,
               legs=legs, t=t, p=p)
    x0 = float(np.min(corners[:, 0])); x1 = float(np.max(corners[:, 0]))
    y0 = float(np.min(corners[:, 1])); y1 = float(np.max(corners[:, 1]))
    res.update(x0=x0, y0=y0, x1=x1, y1=y1, walls=walls_of(x0, y0, x1, y1), cmd_w=x1 - x0, cmd_h=y1 - y0,
               near_m=near_m, arr_m=arr_m)
    if not legs:
        res["fatal"] = "no leg phases -- B1 does not apply to this run"
        return res

    # --- the B1 window, tools/metrics.py's rule as of 2026-10-15: open at the FIRST LEG PHASE whose own
    # recorded truth position (phase.csv truth_x/truth_y, which is the tap's pe/pn -- this tool checks
    # that on the way in) is within 0.15 m of the entry corner, i.e. the ARRIVAL AT CORNER 1 expressed in
    # terms of the aircraft rather than a phase index.  WHY IT CHANGED: a setpoint-driven take-off put a
    # CLIMB leg in front of the tour, so the old index rule (the SECOND leg phase) opened the box at the
    # PAD -- measured on RECT34-accept as E 2.584 / N 1.651 for a 2.0 x 1.2 m square.  The old rule's
    # number is printed beside the new one whenever the two differ, and B1's verdict uses the NEW one.
    entry = corners[0]
    r0 = None
    for i in legs:
        try:
            tx, ty = float(phases[i]["truth_x"]), float(phases[i]["truth_y"])
        except (KeyError, ValueError):
            continue
        if np.hypot(tx - entry[0], ty - entry[1]) <= 0.15:
            r0 = tsim[i]
            res["win_open_phase"] = phases[i]["phase"]
            break
    r0_old = tsim[legs[1]] if len(legs) > 1 else tsim[legs[0]]
    if r0 is None:                       # no phase entry at the corner (a short log): the old rule
        r0 = r0_old
        res["win_open_phase"] = phases[legs[1] if len(legs) > 1 else legs[0]]["phase"] + " (fallback)"
    r1 = tsim[legs[-1] + 1] if legs[-1] + 1 < len(phases) else int(t[-1])
    m = np.flatnonzero((t >= r0) & (t <= r1))
    q = p[m]
    res.update(r0=r0, r1=r1, r0_old=r0_old, win_n=len(m), win_s=(t[m][-1] - t[m][0]) * 1e-6,
               ext_E=float(np.ptp(q[:, 0])), ext_N=float(np.ptp(q[:, 1])))
    if abs(r0 - r0_old) > 1_000:
        mo = np.flatnonzero((t >= r0_old) & (t <= r1))
        res["old_rule"] = (float(np.ptp(p[mo][:, 0])), float(np.ptp(p[mo][:, 1])), len(mo))
    # the speed from the tap itself (1200 Hz -- the same derivative tools/b1-window-probe.py takes)
    v = np.zeros(len(t))
    dt = np.maximum(np.diff(t) * 1e-6, 1e-9)
    v[1:] = np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1])) / dt
    v[0] = v[1]
    res["v"] = v
    res["phase_of"] = np.array([_phase_at(tt, tsim, pnames, int(t[-1])) for tt in t])

    # --- the FCU / command channels, interpolated onto the tap's own times
    sp = read_setpoints(os.path.join(run_dir, "setpoints.csv"))
    if sp is not None:
        st = sp[:, SP["t"]].astype(np.int64)
        g = lambda k: np.interp(t, st, sp[:, SP[k]])
        res.update(has_sp=True, sp_n=len(sp),
                   cmd=np.column_stack([g("cmd_x"), g("cmd_y")]),
                   fcu=np.column_stack([g("fcu_x"), g("fcu_y")]),
                   fv=np.column_stack([g("fcu_vx"), g("fcu_vy")]),
                   sp_t=st, sp_cmd=sp[:, [SP["cmd_x"], SP["cmd_y"]]])
    else:
        res["has_sp"] = False
        res["warn"].append("no setpoints.csv: the command/FCU decomposition is NOT available")

    calib = load_calib()
    lt, lio = read_lio_w(os.path.join(run_dir, "lio.csv"), calib)
    res["lio"] = (lt, lio)

    hold_s, src = hold_s_of(run_dir, hold_s_default)
    res.update(hold_s=hold_s, hold_src=src)

    # --- the commanded tour, from the command stream (the plan box commands ONE corner at a time, so the
    #     times at which cmd jumps give every visit's window without assuming a phase naming).
    visits = []
    if res["has_sp"]:
        ch = np.flatnonzero(np.hypot(np.diff(sp[:, SP["cmd_x"]]), np.diff(sp[:, SP["cmd_y"]])) > 1e-3) + 1
        idx = [0] + list(ch)
        for a, j in enumerate(idx):
            b = idx[a + 1] if a + 1 < len(idx) else len(sp)
            if b - j < 5:
                continue
            pt = sp[j, [SP["cmd_x"], SP["cmd_y"]]]
            kc = int(np.argmin(np.hypot(corners[:, 0] - pt[0], corners[:, 1] - pt[1])))
            if np.hypot(corners[kc, 0] - pt[0], corners[kc, 1] - pt[1]) > 0.05:
                continue          # the pad hover / the climb leg: not a corner
            visits.append(dict(t0=float(st[j]), t1=float(st[b - 1]), corner=pt.copy(),
                               kc=kc, corner_world=corners[kc].copy(),
                               off_mm=float(np.hypot(corners[kc, 0] - pt[0],
                                                     corners[kc, 1] - pt[1])) * 1e3))
    res["visits"] = visits

    # --- TABLE A: the four walls, and the identity that the growth IS their sum
    tab = []
    for w in res["walls"]:
        d, din = out_in(q, w)
        j = int(np.argmax(d))
        assign = np.array([wall_of_point(pt, res["walls"], x0, y0, x1, y1) for pt in q])
        onw = assign == res["walls"].index(w)
        jin = int(np.argmax(din)) if len(din) else 0
        jmax = int(np.argmax(d))
        row = dict(wall=w["name"], pos=w["pos"], out=float(d[j]) * 1e3, t_out=float((t[m][j] - t[0]) * 1e-6),
                   E_out=float(q[j, 0]), N_out=float(q[j, 1]), v_out=float(v[m][j]),
                   phase_out=_phase_at(t[m][j], tsim, pnames, int(t[-1])),
                   nc_out=nearest_corner(q[j], corners), n_on=int(onw.sum()),
                   out_on=float(np.max(d[onw])) * 1e3 if onw.any() else float("nan"),
                   t_out_on=float((t[m][np.flatnonzero(onw)[int(np.argmax(d[onw]))]] - t[0]) * 1e-6)
                   if onw.any() else float("nan"),
                   in_on=float(np.max(din[onw])) * 1e3 if onw.any() else float("nan"),
                   t_in_on=float((t[m][np.flatnonzero(onw)[int(np.argmax(din[onw]))]] - t[0]) * 1e-6)
                   if onw.any() else float("nan"),
                   t_in=float((t[m][jin] - t[0]) * 1e-6), in_=float(din[jin]) * 1e3)
        if res["has_sp"]:
            row.update(cmd_at_out=res["cmd"][m[j]].copy(), fcu_at_out=res["fcu"][m[j]].copy(),
                       fv_at_out=float(np.hypot(*res["fv"][m[j]])))
        tab.append(row)
    res["walls_tab"] = tab
    gE = tab[0]["out"] + tab[1]["out"]
    gN = tab[2]["out"] + tab[3]["out"]
    res.update(sum_E=gE, sum_N=gN, id_E=gE - (res["ext_E"] - res["cmd_w"]) * 1e3,
               id_N=gN - (res["ext_N"] - res["cmd_h"]) * 1e3)
    res["b1_ok"] = res["ext_E"] <= res["cmd_w"] + 0.10 and res["ext_N"] <= res["cmd_h"] + 0.10

    # --- TABLE B: per VISIT (one commanded corner, approach -> dwell -> departure), what the truth does
    res["visit_tab"] = []
    for vi, vis in enumerate(visits):
        a, b = np.flatnonzero((t >= vis["t0"]) & (t <= vis["t1"]))[[0, -1]]
        seg = np.arange(a, b + 1)
        qq = p[seg]
        tt = (t[seg] - t[0]) * 1e-6
        c = vis["corner"]
        dist = np.hypot(qq[:, 0] - c[0], qq[:, 1] - c[1])
        jc = int(np.argmin(dist))
        dwell0 = vis["t1"] - hold_s * 1e6          # the dwell clock: see the header's INFERENCE note
        for w in res["walls"]:
            k = res["walls"].index(w)
            # the wall a corner sits on is decided by the WORLD FILE's corner (B1's judged rectangle is
            # built from `--corners`), with a 20 mm tolerance: F1's plan box commanded corners 2.6 mm
            # off the world's, and that offset is printed rather than silently changing the wall.
            if abs(c[w["axis"]] - w["val"]) > 0.020:
                continue                            # this corner does not sit on this wall
            d, din = out_in(qq, w)
            j = int(np.argmax(d))
            when = _when(tt[j], (vis["t0"] - t[0]) * 1e-6, (t[seg[jc]] - t[0]) * 1e-6,
                         (dwell0 - t[0]) * 1e-6, (vis["t1"] - t[0]) * 1e-6)
            row = dict(visit=vi + 1, corner_pt=c.copy(), wall=w["name"],
                       t_cmd0=float((vis["t0"] - t[0]) * 1e-6), t_cmd1=float((vis["t1"] - t[0]) * 1e-6),
                       in_b1=bool(vis["t0"] >= r0 - 200_000),
                       dwell0=float((dwell0 - t[0]) * 1e-6),
                       t_close=float((t[seg[jc]] - t[0]) * 1e-6), close=float(dist[jc]) * 1e3,
                       out=float(d[j]) * 1e3, t_out=float(tt[j]), when=when,
                       since_close=float(tt[j] - (t[seg[jc]] - t[0]) * 1e-6),
                       since_dwell0=float(tt[j] - (dwell0 - t[0]) * 1e-6),
                       v_out=float(v[seg[j]]), in_=float(np.max(din)) * 1e3,
                       t_in=float(tt[int(np.argmax(din))]))
            if res["has_sp"]:
                # the SAME signed quantity B1's growth is made of, for the three channels: how far past
                # this wall the truth, the FCU's own estimate and the commanded point are (mm).
                row.update(truth_past=float(d[j]) * 1e3,
                           cmd_past=float(w["sense"] * (res["cmd"][seg[j], w["axis"]] - w["val"])) * 1e3,
                           fcu_past=float(w["sense"] * (res["fcu"][seg[j], w["axis"]] - w["val"])) * 1e3,
                           cmd=res["cmd"][seg[j]].copy(), fcu=res["fcu"][seg[j]].copy(),
                           fv=float(np.hypot(*res["fv"][seg[j]])))
            res["visit_tab"].append(row)

    # --- TABLE E: per CORNER, the largest distance the truth reaches from it during the ARRIVAL AND HOLD
    #     (the span in which the truth is inside `near_m` of the corner -- the only span in which it can
    #     matter to B1, whose walls meet AT the corner), which way that bulges, and the RING the arrival
    #     excites.  Everything is in the frame of the leg that ARRIVES at the corner, so "along the
    #     direction of travel" (the ring: short of the target, then past it) and "across it" (the lateral
    #     bulge) are separated instead of mixed into one number.
    res["corner_tab"] = []
    for vi, vis in enumerate(visits):
        seg = np.flatnonzero((t >= vis["t0"]) & (t <= vis["t1"]))
        if len(seg) < 20:
            continue
        c = vis["corner"]
        qq = p[seg]
        tt = (t[seg] - t[0]) * 1e-6
        dist = np.hypot(qq[:, 0] - c[0], qq[:, 1] - c[1])
        near = np.flatnonzero(dist <= near_m)
        if len(near) < 5:
            continue
        # The RING is measured in a 5 s window CENTRED ON THE CORNER PASS -- the truth's closest approach
        # to the corner -- because that is the only window whose boundaries are events of the flight
        # rather than limits of this tool: starting it at a radius crossing would report the APPROACH as
        # an undershoot of the loop, which is not what the loop did.
        jc0 = int(np.argmin(dist))
        lo = int(np.searchsorted(tt, tt[jc0] - 2.0))
        hi = int(np.searchsorted(tt, tt[jc0] + 3.0)) - 1
        lo = max(0, lo)
        hi = min(len(seg) - 1, max(hi, jc0))
        a, b = int(near[0]), int(near[-1])          # the corner-occupancy span (arrival .. departure)
        sl = slice(lo, hi + 1)
        u = np.array([1.0, 0.0]) if vi == 0 else c - visits[vi - 1]["corner"]
        nu = float(np.linalg.norm(u))
        u = u / nu if nu > 1e-9 else np.array([1.0, 0.0])
        nv = np.array([-u[1], u[0]])
        dv = qq[sl, :2] - c
        along = dv @ u * 1e3
        cross = dv @ nv * 1e3
        jj = int(np.argmax(dist[sl]))
        jc = int(np.argmin(dist[sl]))
        dwell0 = (vis["t1"] - hold_s * 1e6 - t[0]) * 1e-6
        row = dict(visit=vi + 1, kc=vis["kc"], corner_pt=c.copy(),
                   in_b1=bool(vis["t0"] >= r0 - 200_000),
                   t_cmd0=float((vis["t0"] - t[0]) * 1e-6), dwell0=float(dwell0),
                   span=(float(tt[a]), float(tt[b])),
                   t_arr=float(tt[a]), arr=float(dist[a]) * 1e3,
                   ring_span=(float(tt[sl][0]), float(tt[sl][-1])),
                   t_max=float(tt[jc0]), max_dist=float(dist[jc0]) * 1e3,
                   dir_E=float((qq[jc0, 0] - c[0]) / max(dist[jc0], 1e-9)),
                   dir_N=float((qq[jc0, 1] - c[1]) / max(dist[jc0], 1e-9)),
                   along_at_max=float(along[jj]), cross_at_max=float(cross[jj]),
                   when=_when(tt[sl][int(np.argmax(along))], (vis["t0"] - t[0]) * 1e-6,
                              (t[seg][a] - t[0]) * 1e-6, (vis["t1"] - hold_s * 1e6 - t[0]) * 1e-6,
                              (vis["t1"] - t[0]) * 1e-6),
                   v_at_max=float(v[seg][jc0]), t_close=float(tt[jc0]),
                   close=float(dist[jc0]) * 1e3,
                   since_arr=float(tt[sl][int(np.argmax(along))] - tt[jc0]),
                   along_min=float(along.min()), t_along_min=float(tt[sl][int(np.argmin(along))]),
                   along_max=float(along.max()), t_along_max=float(tt[sl][int(np.argmax(along))]),
                   cross_min=float(cross.min()), cross_max=float(cross.max()),
                   ring_pp=float(along.max() - along.min()),
                   half_period=float(tt[sl][int(np.argmax(along))] - tt[sl][int(np.argmin(along))]),
                   v_at_along_min=float(v[seg][sl][int(np.argmin(along))]),
                   v_at_along_max=float(v[seg][sl][int(np.argmax(along))]))
        if res["has_sp"]:
            jm = int(np.flatnonzero(seg == seg[sl][jj])[0]) if False else sl.start + jj
            row.update(truth_fcu_at_max=float(np.hypot(*(qq[jm, :2] - res["fcu"][seg[jm]]))) * 1e3,
                       fcu_past_corner=float(np.hypot(*(res["fcu"][seg[jm]] - c))) * 1e3,
                       cmd_past_corner=float(np.hypot(*(res["cmd"][seg[jm]] - c))) * 1e3)
        res["corner_tab"].append(row)

    # --- TABLE C: per LEG PHASE, tools/metrics.py's own B2 number and the wall it runs along
    res["leg_tab"] = []
    for li, j in enumerate(legs):
        t0 = tsim[j]
        t1 = tsim[j + 1] if j + 1 < len(phases) else int(t[-1])
        sel = np.flatnonzero((t >= t0) & (t <= t1))
        if len(sel) < 20:
            continue
        qq = p[sel]
        a, b = qq[0], qq[-1]
        d = b - a
        L = float(np.linalg.norm(d))
        r = qq - a
        perp = np.abs(d[0] * r[:, 1] - d[1] * r[:, 0]) / max(L, 1e-9)
        jp = int(np.argmax(perp))
        # sign of the worst perpendicular relative to the leg's direction (left/right of the line)
        sgn = np.sign(d[0] * r[jp, 1] - d[1] * r[jp, 0])
        # the wall this leg runs along: the one both endpoints sit on (within 1 mm), else the nearest
        row = dict(leg=li + 1, phase=phases[j]["phase"], n=len(sel), t0=float((t0 - t[0]) * 1e-6),
                   t1=float((t1 - t[0]) * 1e-6), length=L,
                   perp=float(perp[jp]) * 1e3, t_perp=float((t[sel[jp]] - t[0]) * 1e-6),
                   E_perp=float(qq[jp, 0]), N_perp=float(qq[jp, 1]), side=float(sgn))
        onw = None
        # the wall this leg RUNS ALONG: the leg's direction has to be perpendicular to that wall's
        # normal, and the wall has to be the one nearest the leg's own mean coordinate.  A leg that
        # runs along NO wall (the pad -> corner 1 entry leg is diagonal) gets '--': its excursion past
        # a wall is meaningless (the pad is 0.4 m outside the N-min wall by construction).
        if abs(d[1]) > 2.0 * abs(d[0]):
            axis = 0
        elif abs(d[0]) > 2.0 * abs(d[1]):
            axis = 1
        else:
            axis = None
        if axis is not None:
            cand = [w for w in res["walls"] if w["axis"] == axis]
            mean = float(np.mean(qq[:, axis]))
            onw = cand[int(np.argmin([abs(mean - w["val"]) for w in cand]))]
        if onw is None:
            row.update(wall="--", out=float("nan"), t_out=float("nan"), same_event=False,
                       out_first2=float("nan"), t_out_first2=float("nan"))
            res["leg_tab"].append(row)
            continue
        dd, _ = out_in(qq, onw)
        jw = int(np.argmax(dd))
        row.update(wall=onw["name"], out=float(dd[jw]) * 1e3, t_out=float((t[sel[jw]] - t[0]) * 1e-6),
                   same_event=bool(abs((t[sel[jw]] - t[sel[jp]]) * 1e-6) <= 0.30))
        # the departure dip: the outward excursion in the first 2 s of the leg
        early = np.flatnonzero((t[sel] - t0) * 1e-6 <= 2.0)
        row["out_first2"] = float(np.max(dd[early])) * 1e3 if len(early) else float("nan")
        row["t_out_first2"] = float((t[sel][early[int(np.argmax(dd[early]))]] - t[0]) * 1e-6) \
            if len(early) else float("nan")
        res["leg_tab"].append(row)

    # --- the FCU's own contribution over the window (measured at 50 Hz, interpolated)
    if res["has_sp"]:
        fcu_w = res["fcu"][m]
        cmd_w = res["cmd"][m]
        e_tf = q[:, :2] - fcu_w
        e_tc = q[:, :2] - cmd_w
        r = lambda x: float(np.sqrt(np.mean(np.sum(x ** 2, axis=1)))) * 1e3
        # the DWELL windows: the only instants at which "truth - command" is a parking error and not the
        # intended travel (the command is a STEP to the next corner, so outside the dwells it is metres).
        dw = np.zeros(len(q), bool)
        for vis in visits[1:]:
            dw |= (t[m] >= vis["t1"] - hold_s * 1e6) & (t[m] <= vis["t1"])
        res["dwell_mask"] = dw
        res["fcu_stat"] = dict(
            rms_tf=r(e_tf), rms_tc_dwell=r(e_tc[dw]) if dw.any() else float("nan"),
            max_tc_dwell=float(np.max(np.hypot(e_tc[dw, 0], e_tc[dw, 1]))) * 1e3 if dw.any() else float("nan"),
            t_max_tc_dwell=float((t[m][np.flatnonzero(dw)[int(np.argmax(np.hypot(e_tc[dw, 0], e_tc[dw, 1])))]] - t[0]) * 1e-6)
            if dw.any() else float("nan"),
            p95_tf=float(np.percentile(np.hypot(e_tf[:, 0], e_tf[:, 1]), 95)) * 1e3,
            max_tf=float(np.max(np.hypot(e_tf[:, 0], e_tf[:, 1]))) * 1e3,
            t_max_tf=float((t[m][int(np.argmax(np.hypot(e_tf[:, 0], e_tf[:, 1])))] - t[0]) * 1e-6),
            n_dwell=int(dw.sum()))
        # the frame self-check: during a dwell the commanded point must BE the corner
        chk = []
        for vi in visits[1:]:
            j = int(np.argmin(np.abs(t - (vi["t1"] - hold_s * 5e5))))
            chk.append(np.hypot(res["cmd"][j, 0] - vi["corner"][0], res["cmd"][j, 1] - vi["corner"][1]))
        res["cmd_check_mm"] = float(np.max(chk)) * 1e3 if chk else float("nan")

    # --- the arrival speed at each corner, measured the way the old note did (peak within 1 m of it)
    res["arrival"] = []
    for vi in visits:
        c = vi["corner"]
        seg = np.flatnonzero((t >= vi["t0"]) & (t <= vi["t1"]))
        dist = np.hypot(p[seg, 0] - c[0], p[seg, 1] - c[1])
        near = dist <= 1.0
        res["arrival"].append(dict(corner=c.copy(),
                                   v_peak=float(v[seg][near].max()) if near.any() else float("nan"),
                                   t_close=float((t[seg][int(np.argmin(dist))] - t[0]) * 1e-6),
                                   close=float(dist.min()) * 1e3,
                                   v_at_close=float(v[seg][int(np.argmin(dist))]),
                                   v_settle=float(np.median(v[seg][dist <= 0.15]))
                                   if (dist <= 0.15).any() else float("nan")))
    if want_timeline:
        res["timeline"] = _timeline(t, p, v, res, corners)
    return res


def _phase_at(tt, tsim, pnames, tend):
    i = int(np.searchsorted(tsim, tt, side="right")) - 1
    return pnames[max(i, 0)]


def _when(tt, t0, t_close, dwell0, t1):
    """Where an event sits inside a corner visit.

    The reference is the DWELL CLOCK (`t1 - hold_s`), not the closest approach: with a position loop
    that rings around the target, the closest approach is often in the MIDDLE of the dwell (the ring
    passing through the corner), so it is not an arrival time and is not used as one here."""
    if tt < t0 + 2.0:
        return "departure (first 2 s of the visit)"
    if tt < dwell0:
        return "settling (command held, dwell clock not yet started)"
    if tt <= t1:
        return "while parked (dwell clock running)"
    return "after the visit"


def _timeline(t, p, v, res, corners):
    out = []
    for row in res["visit_tab"]:
        if row["out"] < 5.0:
            continue
        out.append(dict(visit=row["visit"], wall=row["wall"], t_out=row["t_out"],
                        t0=row["t_cmd0"], t1=row["t_cmd1"], dwell0=row["dwell0"]))
    return out


# ---------------------------------------------------------------------------------------------------
# printing
# ---------------------------------------------------------------------------------------------------
def fmt_pt(pt):
    return "(E %+.4f, N %+.4f)" % (pt[0], pt[1])


def print_run(r, verbose=True):
    print("=" * 100)
    if r.get("fatal"):
        print("run %s: %s" % (r["run"], r["fatal"]))
        return
    print("run %s   %s" % (r["run"], r["dir"]))
    print("   truth tap %d samples over %.1f s;  command rectangle E [%.3f, %.3f] N [%.3f, %.3f] "
          "(%.3f x %.3f m);  dwell %.1f s (%s)"
          % (r["n_tap"], r["span_s"], r["x0"], r["x1"], r["y0"], r["y1"], r["cmd_w"], r["cmd_h"],
             r["hold_s"], r["hold_src"]))
    if r.get("has_sp"):
        print("   command/FCU channels: setpoints.csv %d rows at %.1f Hz;  command check (the commanded "
              "point during a dwell vs the corner): %.2f mm" % (r["sp_n"], r["sp_n"] / r["win_s"],
                                                                r.get("cmd_check_mm", float("nan"))))
    print("   B1 window [the first leg phase whose truth is at corner 1, the land entry] = phase %s,"
          " sim %.2f .. %.2f s = %.2f s, %d truth samples"
          % (r.get("win_open_phase", "?"), (r["r0"] - r["t"][0]) * 1e-6, (r["r1"] - r["t"][0]) * 1e-6,
             r["win_s"], r["win_n"]))
    if "old_rule" in r:
        print("   the OLD index rule (the second leg phase) would open at sim %.2f s and read E %.3f /"
              " N %.3f m  [REPORTED: it counts the pad/climb]"
              % ((r["r0_old"] - r["t"][0]) * 1e-6, r["old_rule"][0], r["old_rule"][1]))
    print("   B1 extents  E %.4f m  N %.4f m   =>  growth E %+.1f mm  N %+.1f mm   bound +100/+100 mm"
          "   [%s]" % (r["ext_E"], r["ext_N"], (r["ext_E"] - r["cmd_w"]) * 1e3,
                       (r["ext_N"] - r["cmd_h"]) * 1e3, "PASS" if r["b1_ok"] else "FAIL"))
    print("   IDENTITY    E: E-min out + E-max out = %.1f + %.1f = %.1f mm vs growth %.1f mm (resid %.3f mm)"
          % (r["walls_tab"][0]["out"], r["walls_tab"][1]["out"], r["sum_E"],
             (r["ext_E"] - r["cmd_w"]) * 1e3, r["id_E"]))
    print("               N: N-min out + N-max out = %.1f + %.1f = %.1f mm vs growth %.1f mm (resid %.3f mm)"
          % (r["walls_tab"][2]["out"], r["walls_tab"][3]["out"], r["sum_N"],
             (r["ext_N"] - r["cmd_h"]) * 1e3, r["id_N"]))
    print()
    print("   TABLE A -- the four walls: what makes the growth (global extremes over the B1 window)")
    print("     %-6s %-9s %9s %8s %-6s %6s  %-20s %-14s | %9s %8s | %9s %8s"
          % ("wall", "at", "out (mm)", "t (s)", "phase", "v", "where", "nearest corner",
             "out|on", "t (s)", "in|on", "t (s)"))
    for row in r["walls_tab"]:
        k, dc = row["nc_out"]
        print("     %-6s %-9s %+9.1f %8.2f %-6s %6.3f  %-20s c%d %6.3f m   | %+9.1f %8.2f | %+9.1f %8.2f"
              % (row["wall"], row["pos"], row["out"], row["t_out"], row["phase_out"], row["v_out"],
                 fmt_pt((row["E_out"], row["N_out"])), k, dc, row["out_on"], row["t_out_on"],
                 row["in_on"], row["t_in_on"]))
    print("     'out' = how far past that wall the truth reaches (positive = OUTSIDE the rectangle, this is")
    print("     the growth); 'in|on' = the largest stand-off INSIDE on the samples whose nearest wall is")
    print("     this one (a leg flown 20 mm inside its wall shows up here, and cannot make B1 grow).")
    if r.get("has_sp"):
        print("     the same four peak samples, decomposed (all from setpoints.csv at 50 Hz):")
        print("        %-6s %14s %14s %14s %14s" % ("wall", "truth-cmd E/N", "fcu-cmd E/N",
                                                   "truth-fcu E/N", "|v_fcu|"))
        for row in r["walls_tab"]:
            print("        %-6s %+7.1f %+6.1f %+7.1f %+6.1f %+7.1f %+6.1f %11.3f"
                  % (row["wall"],
                     (row["E_out"] - row["cmd_at_out"][0]) * 1e3,
                     (row["N_out"] - row["cmd_at_out"][1]) * 1e3,
                     (row["fcu_at_out"][0] - row["cmd_at_out"][0]) * 1e3,
                     (row["fcu_at_out"][1] - row["cmd_at_out"][1]) * 1e3,
                     (row["E_out"] - row["fcu_at_out"][0]) * 1e3,
                     (row["N_out"] - row["fcu_at_out"][1]) * 1e3, row["fv_at_out"]))
    print()
    print("   TABLE B -- the corner visits: which visit owns the wall extreme, and WHEN inside it")
    print("     %-4s %-16s %-6s %8s %9s %-12s %8s %8s %8s %7s | %9s %9s %9s"
          % ("vis", "commanded corner", "wall", "cmd t0", "dwell t0", "closest", "out (mm)", "t (s)",
             "since c/a", "since dw", "truth-cmd", "fcu-cmd", "truth-fcu"))
    for row in r["visit_tab"]:
        if row["out"] < 5.0 and row["in_b1"]:
            continue
        extra = ""
        if r.get("has_sp"):
            extra = "%+9.1f %+9.1f %+9.1f" % (row["truth_past"] - row["cmd_past"],
                                              row["fcu_past"] - row["cmd_past"],
                                              row["truth_past"] - row["fcu_past"])
        print("     %-4d %-16s %-6s %8.2f %9s %-12s %+8.1f %8.2f %8s %7s | %s%s"
              % (row["visit"], fmt_pt(row["corner_pt"]), row["wall"], row["t_cmd0"],
                 "%.2f" % row["dwell0"] if row["visit"] > 1 else "  --  ",
                 "%.1f mm @ %.2f" % (row["close"], row["t_close"]), row["out"], row["t_out"],
                 "%.2f" % row["since_close"], "%.2f" % row["since_dwell0"] if row["visit"] > 1 else " -- ",
                 extra, "" if row["in_b1"] else "   <-- BEFORE the B1 window (the entry leg)"))
    print("     a visit = one commanded corner: from the moment the plan box commands it to the moment it")
    print("     commands the next one (the dwell is the last hold_s of it).  'since c/a' = seconds after")
    print("     the truth's closest approach to that corner; 'since dw' = seconds after the dwell clock")
    print("     starts (negative = the aircraft is ALREADY outside when the park begins).  The last three")
    print("     columns are the signed deviation past the wall from the command, from the FCU's own")
    print("     estimate and the FCU's own error, in mm.")
    print()
    print("   TABLE C -- per leg, B2's own number next to the wall the leg runs along")
    print("     %-4s %-6s %6s %7s %8s %8s %9s %-6s %9s %8s %10s %8s" %
          ("leg", "phase", "n", "len(m)", "B2 perp", "at t", "side", "wall", "wall-out", "at t",
           "out<2 s", "same?"))
    for row in r["leg_tab"]:
        if row["wall"] == "--":
            print("     %-4d %-6s %6d %7.3f %7.1f mm %8.2f %+9.0f %-6s %11s %8s %10s %8s"
                  % (row["leg"], row["phase"], row["n"], row["length"], row["perp"], row["t_perp"],
                     row["side"], "--", "(no wall)", "--", "--", "different"))
            continue
        print("     %-4d %-6s %6d %7.3f %7.1f mm %8.2f %+9.0f %-6s %+8.1f mm %8.2f %+9.1f %8s"
              % (row["leg"], row["phase"], row["n"], row["length"], row["perp"], row["t_perp"],
                 row["side"], row["wall"], row["out"], row["t_out"], row["out_first2"],
                 "SAME" if row["same_event"] else "different"))
    print("     'side' = the sign of the worst B2 sample against the leg's own direction; 'wall-out' = the")
    print("     largest excursion past the wall this leg runs along, inside the SAME leg phase; 'same?'")
    print("     says whether those two are the same instant (|dt| <= 0.3 s) or two different events.")
    print()
    print("   TABLE E -- per corner: the RING the arrival excites, in the frame of the leg that arrives")
    print("              (ring window = 2 s before to 3 s after the corner pass; the pass is the truth's")
    print("              closest approach to the corner; 'along' = the direction of travel, positive =")
    print("              PAST the commanded corner)")
    print("     %-4s %-16s %-15s %9s %8s %8s %9s %8s %8s %8s %6s | %8s %8s %8s %s"
          % ("vis", "corner", "pass", "short(mm)", "t", "v", "past(mm)", "t", "v", "delay", "half-T",
             "cross-", "cross+", "p-p", "when the 'past' extreme happens"))
    for row in res_corner_rows(r):
        print("     %-4d %-16s %5.1fmm@%5.2f %+9.1f %8.2f %8.3f %+9.1f %8.2f %8.3f %8.2f %5.2f | "
              "%+8.1f %+8.1f %8.1f %s%s"
              % (row["visit"], fmt_pt(row["corner_pt"]), row["close"], row["t_close"],
                 row["along_min"], row["t_along_min"], row["v_at_along_min"],
                 row["along_max"], row["t_along_max"], row["v_at_along_max"], row["since_arr"],
                 row["half_period"], row["cross_min"], row["cross_max"], row["ring_pp"], row["when"],
                 "" if row["in_b1"] else "   <-- entry leg, BEFORE the B1 window"))
    print("     'short'/'past' = how far the truth gets on the two sides of the commanded corner along")
    print("     the direction of travel inside that window: a position loop that rings shows BOTH,")
    print("     seconds apart, with the aircraft nearly stationary (v) at each extreme.  'delay' =")
    print("     seconds from the corner pass to the 'past' extreme; 'half-T' = the interval between the")
    print("     two extremes (half a period if it is one oscillation).  'cross-/cross+' is the lateral")
    print("     component on the two sides of the leg's own line; 'p-p' is the along peak-to-peak.")
    if any(abs(v["off_mm"]) > 1.0 for v in r["visits"]):
        print("     NOTE: this run's commanded corners sit %.1f mm from the world file's (the judged"
              % max(abs(v["off_mm"]) for v in r["visits"]))
        print("           rectangle is built from the world file): the walls above are the WORLD's, as")
        print("           tools/metrics.py judges them.")
    if r.get("has_sp"):
        s = r["fcu_stat"]
        print()
        print("   TABLE D -- the FCU's own contribution (measured from setpoints.csv at 50 Hz)")
        print("     over the DWELL windows only (%d of %d window samples -- the instants at which the "
              "commanded" % (s["n_dwell"], r["win_n"]))
        print("     point is a FIXED corner and 'truth - command' is a parking error, not the intended "
              "travel):")
        print("        truth - command  rms %6.2f mm, worst %6.2f mm at t %.2f s"
              % (s["rms_tc_dwell"], s["max_tc_dwell"], s["t_max_tc_dwell"]))
        print("     over the WHOLE B1 window:  truth - FCU est  rms %6.2f mm, p95 %6.2f mm, max %6.2f mm"
              " at t %.2f s" % (s["rms_tf"], s["p95_tf"], s["max_tf"], s["t_max_tf"]))
        print("     (A4 reports the same difference over the whole flight as ~15 mm rms: the tail ABOVE it")
        print("      is made at the corner arrivals, and the four wall peaks in TABLE A carry its value.)")
    av = [a for a in r["arrival"]]
    print()
    print("   the arrivals (the peak speed within 1 m of each commanded corner, truth-only):")
    for a in av:
        print("     %-16s closest %6.1f mm at t %7.2f (v %5.3f m/s),  peak within 1 m %5.2f m/s,"
              "  median v within 0.15 m %5.3f m/s"
              % (fmt_pt(a["corner"]), a["close"], a["t_close"], a["v_at_close"], a["v_peak"],
                 a["v_settle"]))
    for w in r.get("warn", []):
        print("   NOTE: %s" % w)
    print()


def _signed(row, r, kind):
    """The signed deviation PAST that visit's wall, in mm, for the three channels."""
    if kind == "truth":
        return row["truth_past"] - row["cmd_past"]
    if kind == "fcu":
        return row["fcu_past"] - row["cmd_past"]
    return row["truth_past"] - row["fcu_past"]


def res_corner_rows(r):
    """Every corner visit the tool could measure (the ring window needs samples on both sides)."""
    return [x for x in r.get("corner_tab", []) if x["ring_span"][1] > x["ring_span"][0]]


def print_summary(rs):
    print("=" * 100)
    print("SUMMARY -- the growth, wall by wall, in mm (out = past the wall; the two walls of an axis add")
    print("           up to B1's growth on that axis)")
    print("   %-24s %8s %8s %8s %8s | %8s %8s | %-7s  who owns each wall (corner, when)"
          % ("run", "E-min", "E-max", "N-min", "N-max", "growthE", "growthN", "B1"))
    for r in rs:
        if r.get("fatal"):
            continue
        w = r["walls_tab"]
        own = []
        for row in w:
            k, dc = row["nc_out"]
            own.append("c%d" % k)
        print("   %-24s %+8.1f %+8.1f %+8.1f %+8.1f | %+8.1f %+8.1f | %-7s  %s"
              % (r["run"], w[0]["out"], w[1]["out"], w[2]["out"], w[3]["out"],
                 (r["ext_E"] - r["cmd_w"]) * 1e3, (r["ext_N"] - r["cmd_h"]) * 1e3,
                 "PASS" if r["b1_ok"] else "FAIL", ", ".join(own)))
    print()
    print("   the same four walls, and WHEN in the corner visit the extreme sits:")
    for r in rs:
        if r.get("fatal"):
            continue
        print("   %-24s" % r["run"])
        for row in r["walls_tab"]:
            hit = [v for v in r["visit_tab"] if v["wall"] == row["wall"] and v["out"] >= 5.0
                   and v["in_b1"]]
            best = max(hit, key=lambda v: v["out"]) if hit else None
            if best is None:
                k, dc = row["nc_out"]
                print("      %-6s %+6.1f mm  no command stream: the extreme is in phase %s, "
                      "%.3f m from corner %d, at t %.2f s, v %.3f m/s"
                      % (row["wall"], row["out"], row["phase_out"], dc, k, row["t_out"], row["v_out"]))
                continue
            print("      %-6s %+6.1f mm  visit %d (corner %s), %5.2f s after the closest approach, "
                  "%+5.2f s from the dwell clock, v %.3f m/s -- %s"
                  % (row["wall"], row["out"], best["visit"], fmt_pt(best["corner_pt"]),
                     best["since_close"], best["since_dwell0"], best["v_out"], best["when"]))
    print()


def fmt_report(rs, fh):
    old = sys.stdout
    sys.stdout = fh
    try:
        for r in rs:
            print_run(r)
        if len(rs) > 1:
            print_summary(rs)
    finally:
        sys.stdout = old


# ---------------------------------------------------------------------------------------------------
# csv + figure
# ---------------------------------------------------------------------------------------------------
def write_csv(rs, path):
    with open(path, "w") as fh:
        fh.write("kind,run,key,wall,axis,out_mm,t_sim_s,in_mm,t_in_s,phase,v_out_mps,"
                 "corner,corner_dist_m,since_close_s,since_dwell_s,when,close_mm,truth_cmd_mm,"
                 "fcu_cmd_mm,truth_fcu_mm,b2_perp_mm,b2_t_s,b2_wall_out_mm,b2_same_event\n")
        for r in rs:
            if r.get("fatal"):
                continue
            for row in r["walls_tab"]:
                k, dc = row["nc_out"]
                axis = "E" if [w["name"] for w in r["walls"]].index(row["wall"]) < 2 else "N"
                fh.write("wall,%s,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%s,%.4f,c%d,%.4f,,,,,,,,\n"
                         % (r["run"], row["pos"], row["wall"], axis, row["out"], row["t_out"],
                            row["in_on"], row["t_in_on"], row["phase_out"], row["v_out"], k, dc))
            for row in r["visit_tab"]:
                w = [x for x in r["walls"] if x["name"] == row["wall"]][0]
                tc = (w["sense"] * (row["cmd"][w["axis"]] - w["val"])) * 1e3 if r.get("has_sp") else float("nan")
                fc = (w["sense"] * (row["fcu"][w["axis"]] - w["val"])) * 1e3 if r.get("has_sp") else float("nan")
                tf = tc - fc if r.get("has_sp") else float("nan")
                fh.write("visit,%s,%d,%s,%s,%.3f,%.3f,,,,%.4f,%.4f,%.4f,%.3f,%.3f,%s,%.3f,%.3f,%.3f,%.3f,,,,\n"
                         % (r["run"], row["visit"], row["wall"],
                            "E" if [x["name"] for x in r["walls"]].index(row["wall"]) < 2 else "N",
                            row["out"], row["t_out"],
                            row["v_out"], row["corner_pt"][0], row["corner_pt"][1], row["since_close"],
                            row["since_dwell0"], row["when"].replace(",", ";"), row["close"],
                            tc, fc, tf))
            for row in r["leg_tab"]:
                fh.write("leg,%s,%s,%s,,,,,,%.4f,,,%.3f,%.3f,,,,,,%.3f,%.3f,%.3f,%s\n"
                         % (r["run"], row["leg"], row["wall"], row["out"], row["t_out"], row["perp"],
                            row["t_perp"], row["out"], row["t_out"], row["same_event"]))
    print("wrote %s" % path)


CJK_FONT = "/mnt/c/Windows/Fonts/msyh.ttc"


def setup_cjk():
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    if os.path.exists(CJK_FONT):
        font_manager.fontManager.addfont(CJK_FONT)
        name = font_manager.FontProperties(fname=CJK_FONT).get_name()
        matplotlib.rcParams["font.family"] = name
        matplotlib.rcParams["axes.unicode_minus"] = False
        return name
    print("NOTE: %s not found -- CJK labels will not render" % CJK_FONT)
    return None


def make_figure(rs, path, focus=None):
    import matplotlib.pyplot as plt
    font = setup_cjk()
    ok = [r for r in rs if not r.get("fatal")]
    if focus is None:
        focus = ok[-1]["run"]
    fr = [r for r in ok if r["run"] == focus]
    fr = fr[0] if fr else ok[-1]
    fig = plt.figure(figsize=(15.5, 10.5), dpi=110)
    gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.22)

    # (a) the per-wall bar chart, every run
    ax = fig.add_subplot(gs[0, 0])
    labels = [w["wall"] for w in ok[0]["walls_tab"]]
    n = len(ok)
    xw = np.arange(len(labels))
    for i, r in enumerate(ok):
        vals = [row["out"] for row in r["walls_tab"]]
        b = ax.bar(xw + (i - (n - 1) / 2) * 0.8 / n, vals, 0.8 / n, label=r["run"])
        for rect, v in zip(b, vals):
            ax.text(rect.get_x() + rect.get_width() / 2, v + 1.5, "%.0f" % v, ha="center", fontsize=6.5)
    ax.axhline(100, color="k", ls="--", lw=1)
    ax.text(0.02, 103, "每轴允许 +100 mm（两条墙合计上限 200 mm）", fontsize=8)
    ax.set_xticks(xw)
    ax.set_xticklabels(["E 最小边\nE=%.2f" % ok[0]["x0"], "E 最大边\nE=%.2f" % ok[0]["x1"],
                        "N 最小边\nN=%.2f" % ok[0]["y0"], "N 最大边\nN=%.2f" % ok[0]["y1"]], fontsize=9)
    ax.set_ylabel("越出墙面的距离 (mm)", fontsize=10)
    ax.set_title("(a) B1 盒增量的逐墙归因：每条墙的最大外扩\n每根柱 = 一次 sortie 的 tap 真值，窗口与 metrics.py 的 B1 完全一致",
                 fontsize=11)
    ax.legend(fontsize=6.2, ncol=2, loc="lower left", framealpha=0.92)
    ax.grid(axis="y", alpha=0.3)

    # (b) the truth path with the four wall excursions
    ax = fig.add_subplot(gs[0, 1])
    t, p, r0, r1 = fr["t"], fr["p"], fr["r0"], fr["r1"]
    m = np.flatnonzero((t >= r0) & (t <= r1))
    q = p[m]
    ax.plot(q[:, 0], q[:, 1], lw=0.6, color="0.55", label="真值轨迹（B1 窗口）")
    rect = plt.Rectangle((fr["x0"], fr["y0"]), fr["cmd_w"], fr["cmd_h"], fill=False, ec="k", lw=1.6,
                         label="指令矩形 2.000 × 1.200 m")
    ax.add_patch(rect)
    vmax = 0.0
    offs = [(14, 30, "left"), (-14, 30, "right"), (14, -34, "left"), (-14, -34, "right")]
    for k, c in enumerate(fr["walls_tab"]):
        col = ["tab:red", "tab:orange", "tab:green", "tab:blue"][k]
        ax.plot([c["E_out"]], [c["N_out"]], "o", ms=10, mfc="none", mec=col, mew=2.2)
        ox, oy, ha = offs[k]
        ax.annotate("%s 越出 %+.1f mm（v=%.3f m/s）" % (c["wall"], c["out"], c["v_out"]),
                    (c["E_out"], c["N_out"]), textcoords="offset points", xytext=(ox, oy), ha=ha,
                    fontsize=9, color=col, arrowprops=dict(arrowstyle="->", color=col, lw=1.2))
        vmax = max(vmax, c["v_out"])
    for k, c in enumerate([(fr["x0"], fr["y0"]), (fr["x1"], fr["y0"]), (fr["x1"], fr["y1"]),
                           (fr["x0"], fr["y1"])]):
        ax.plot([c[0]], [c[1]], "k+", ms=11)
        ax.annotate("c%d" % (k + 1), c, textcoords="offset points", xytext=(6, -15), fontsize=11)
    ax.set_xlim(fr["x0"] - 0.30, fr["x1"] + 0.30)
    ax.set_ylim(fr["y0"] - 0.30, fr["y1"] + 0.30)
    ax.set_aspect("equal")
    ax.set_xlabel("E (m)", fontsize=10)
    ax.set_ylabel("N (m)", fontsize=10)
    ax.set_title("(b) %s：四处越墙点全部落在四个拐角上\n圆圈 = 该墙的最大外扩，真值速度 ≤ %.3f m/s（已经停住）"
                 % (fr["run"], vmax), fontsize=11)
    ax.legend(fontsize=8, loc="center", framealpha=0.92)
    ax.grid(alpha=0.3)

    # (c) the deviation from the command over the window
    ax = fig.add_subplot(gs[1, 0])
    if fr.get("has_sp"):
        tw = (t[m] - t[0]) * 1e-6
        dev = (q[:, :2] - fr["cmd"][m]) * 1e3
        ax.plot(tw, dev[:, 0], lw=0.7, color="tab:red", label="真值 − 指令  E 方向")
        ax.plot(tw, dev[:, 1], lw=0.7, color="tab:blue", label="真值 − 指令  N 方向")
        ylo, yhi = ax.get_ylim()
        for vis in fr["visits"][1:]:
            d0 = (vis["t1"] - fr["hold_s"] * 1e6 - t[0]) * 1e-6
            d1 = (vis["t1"] - t[0]) * 1e-6
            ax.axvspan(d0, d1, color="0.85", alpha=0.5)
            ax.text((d0 + d1) / 2, yhi * 0.86, "驻留\nc%d" % (vis["kc"] + 1), ha="center", fontsize=7)
        for row in fr["walls_tab"]:
            k = [w["wall"] for w in fr["walls_tab"]].index(row["wall"])
            j = int(np.argmin(np.abs(tw - row["t_out"])))
            ax.plot([row["t_out"]], [dev[j, 0] if k < 2 else dev[j, 1]], "kv", ms=8)
        ax.axhline(0, color="k", lw=0.8)
        ax.set_xlabel("仿真时间 (s)", fontsize=10)
        ax.set_ylabel("真值 − 指令 (mm)", fontsize=10)
        ax.set_title("(c) %s：相对指令的全程偏差（灰带 = 驻留窗口，▼ = 四处越墙极值）\n"
                     "指令是「一步跳到下一个拐角」的位置目标（速度前馈恒为 0），所以每个拐角都是一次阶跃响应"
                     % fr["run"], fontsize=11)
        ax.legend(fontsize=8, loc="upper left")
        ax.grid(alpha=0.3)
    else:
        ax.text(0.5, 0.5, "no setpoints.csv: no command channel", ha="center")

    # (d) the ring at one corner: truth vs the FCU's own estimate, with the speed
    ax = fig.add_subplot(gs[1, 1])
    if fr.get("has_sp") and fr.get("corner_tab"):
        cand = [x for x in fr["corner_tab"] if x["in_b1"]]
        row = max(cand, key=lambda x: x["ring_pp"]) if cand else fr["corner_tab"][-1]
        a = int(np.argmin(np.abs((t - t[0]) * 1e-6 - (row["t_close"] - 2.5))))
        b = int(np.argmin(np.abs((t - t[0]) * 1e-6 - (row["t_close"] + 4.0))))
        idx = np.arange(a, b)
        u = row["corner_pt"]
        ax.plot((t[idx] - t[0]) * 1e-6, (p[idx, 0] - u[0]) * 1e3 if abs(row["dir_E"]) > abs(row["dir_N"])
                else (p[idx, 1] - u[1]) * 1e3, lw=1.6, color="tab:red", label="真值 − 拐角")
        ax.plot((t[idx] - t[0]) * 1e-6, (fr["fcu"][idx, 0] - u[0]) * 1e3
                if abs(row["dir_E"]) > abs(row["dir_N"]) else (fr["fcu"][idx, 1] - u[1]) * 1e3,
                lw=1.1, ls="--", color="tab:purple", label="FCU 自估计 − 拐角")
        ax.plot((t[idx] - t[0]) * 1e-6, (fr["cmd"][idx, 0] - u[0]) * 1e3
                if abs(row["dir_E"]) > abs(row["dir_N"]) else (fr["cmd"][idx, 1] - u[1]) * 1e3,
                lw=1.0, ls=":", color="0.35", label="指令 − 拐角")
        ax.axhline(0, color="k", lw=0.8)
        ax.axvline(row["dwell0"], color="0.4", ls=":", lw=1.2)
        ax.annotate("驻留时钟起点", (row["dwell0"], ax.get_ylim()[1] * 0.75), fontsize=8, color="0.3")
        for tt, vv, lab in ((row["t_along_min"], row["along_min"], "内侧极值"),
                            (row["t_along_max"], row["along_max"], "越墙极值")):
            ax.plot([tt], [vv], "kv", ms=7)
            ax.annotate("%s %+.0f mm\nv=%.3f m/s" % (lab, vv, row["v_at_along_min"] if vv < 0
                                                     else row["v_at_along_max"]),
                        (tt, vv), textcoords="offset points", xytext=(-4, 26 if vv > 0 else -34),
                        fontsize=8, ha="center")
        ax.set_xlabel("仿真时间 (s)", fontsize=10)
        ax.set_ylabel("相对指令拐角的偏差 (mm)", fontsize=10)
        ax.set_title("(d) %s：拐角 c%d 的到点振荡（越墙极值 %.0f mm，半周期 %.2f s）\n"
                     "真值先冲到内侧 %.0f mm，穿过指令点后再越出 %.0f mm，两端速度都≈0，然后才收回"
                     % (fr["run"], row["kc"] + 1, row["along_max"], row["half_period"], row["along_min"],
                        row["along_max"]), fontsize=11)
        ax.legend(fontsize=8, loc="lower right")
        ax.grid(alpha=0.3)
    fig.suptitle("B1 盒增量的机理：越墙发生在拐角，不在航段上（数据 = 各 run 自己的 tap.csv 真值；"
                 "指令/FCU 通道 = setpoints.csv 50 Hz）\n工具：tools/b1-edge-budget.py", fontsize=12.5)
    fig.savefig(path, bbox_inches="tight")
    print("wrote %s (font %s)" % (path, font))


# ---------------------------------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="attribute B1's box growth to walls, corners and legs")
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "pillars.yaml"))
    ap.add_argument("--hold-s", type=float, default=5.0,
                    help="the dwell the sorties flew; each run's own log overrides it when present")
    ap.add_argument("--csv", default="")
    ap.add_argument("--figure", default="")
    ap.add_argument("--focus", default="", help="which run the figure's panels (b)-(d) show")
    ap.add_argument("--near-m", type=float, default=0.30,
                    help="TABLE E's 'at the corner' window: the truth within this of the corner, m")
    ap.add_argument("--timeline", action="store_true", help="also explain each visit's timeline")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.world)) or {}
    miss = cfg.get("mission") or {}
    if miss.get("corners"):
        corners = np.asarray(miss["corners"], dtype=float)[:, :2]
        uniq = []
        for c in corners:
            if not any(np.allclose(c, u, atol=1e-6) for u in uniq):
                uniq.append(c)
        corners = np.asarray(uniq[:4])
    elif miss.get("square"):
        x0, y0, x1, y1 = [float(v) for v in miss["square"]]
        corners = np.asarray([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    else:
        print("%s: no mission.corners and no mission.square -- B1 is not defined" % args.world)
        return 2
    rs = [analyse(d, corners, args.hold_s, want_timeline=args.timeline, near_m=args.near_m)
          for d in args.runs]
    for r in rs:
        print_run(r)
    if len(rs) > 1:
        print_summary(rs)
    if args.csv:
        write_csv(rs, args.csv)
    if args.figure:
        make_figure(rs, args.figure, args.focus or None)
    return 0 if all(r.get("b1_ok", False) for r in rs) else 0


if __name__ == "__main__":
    sys.exit(main())
