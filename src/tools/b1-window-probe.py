#!/usr/bin/env python3
"""B1's window, MEASURED rather than assumed: exactly which truth samples does the row cover, and where
do the E/N extents come from?

WHY: every clean sortie prints "B1 truth box extents over the route: E 2.139-2.155 m, N 1.279-1.287 m
(command 2.000 / 1.200; target <= 2.10 / 1.30) [FAIL]".  The hypothesis under test is that the window
begins at the PAD, so the pad -> corner-1 entry leg is inside the box and the 154 mm of E overshoot is the
pad's own lateral offset.  That is a statement about (1) which samples the window covers and (2) where the
extremes sit, so both are printed here, from the run's own tap.csv and phase.csv.

usage: tools/b1-window-probe.py <run-dir> [<run-dir> ...] [--world worlds/pillars.yaml]
"""
import argparse
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: the PROJECT root, one level above the code: worlds/, params/, docs/, frozen/, rviz/ live
#: there, not next to the code (2026-10-16: every code directory moved under src/).
PROJ = os.path.abspath(os.path.join(ROOT, ".."))


def read_tap(path):
    """t_us, and the truth in W as (E, N, U) -- the same conversion tools/metrics.py does."""
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    t = d[:, 0].astype(np.int64)
    pn, pe, pd = d[:, 2], d[:, 3], d[:, 4]
    return t, np.column_stack([pe, pn, -pd])


def read_phase(path):
    rows = []
    with open(path) as fh:
        head = fh.readline().strip().split(",")
        for line in fh:
            f = line.strip().split(",")
            if len(f) != len(head):
                continue
            rows.append(dict(zip(head, f)))
    return rows


def square_of(world):
    cfg = yaml.safe_load(open(world)) or {}
    m = cfg.get("mission") or cfg.get("mission_a") or {}
    sq = m.get("square")
    x0, y0, x1, y1 = [float(v) for v in sq]
    return x0, y0, x1, y1


def fmt_extents(t, p, m, label, corners, extra=""):
    """Print one extent set: sample count, span, E and N extents, and WHERE each extreme is."""
    n = int(m.sum())
    if n == 0:
        print("   %-40s %6d samples   (EMPTY)" % (label, n))
        return None
    q, tt = p[m], t[m]
    span = (tt[-1] - tt[0]) * 1e-6
    ex = q[:, 0].max() - q[:, 0].min()
    ny = q[:, 1].max() - q[:, 1].min()
    print("   %-40s %6d samples over %5.1f s   E %6.3f .. %6.3f = %6.3f   N %6.3f .. %6.3f = %6.3f%s"
          % (label, n, span, q[:, 0].min(), q[:, 0].max(), ex, q[:, 1].min(), q[:, 1].max(), ny, extra))
    # where the four extremes are, and how far that point is from the nearest corner of the mission
    for axis, name in enumerate("EN"):
        for which, idx in (("min", int(np.argmin(q[:, axis]))), ("max", int(np.argmax(q[:, axis])))):
            pt = q[idx]
            dc = min(float(np.linalg.norm(pt[:2] - np.asarray(c[:2], float))) for c in corners)
            # speed at that sample, from the tap itself
            t0 = max(idx - 12, 0)
            t1 = min(idx + 12, len(q) - 1)
            if t1 > t0:
                v = np.linalg.norm(q[t1][:2] - q[t0][:2]) / ((tt[t1] - tt[t0]) * 1e-6)
            else:
                v = float("nan")
            print("        %s %s = %+7.3f at t = %6.2f s of the recording, U %+5.2f m, %5.3f m from the "
                  "nearest commanded corner, |v| = %.3f m/s%s"
                  % (name, which, pt[axis], (tt[idx] - t[0]) * 1e-6, pt[2], dc, v,
                     "" if dc < 0.20 else "   <-- NOT near a corner"))
    return ex, ny


def probe(run, world):
    tap = os.path.join(run, "tap.csv")
    phase = os.path.join(run, "phase.csv")
    if not os.path.exists(tap) or not os.path.exists(phase):
        print("### %s: no tap.csv/phase.csv" % run)
        return
    t, p = read_tap(tap)
    ph = read_phase(phase)
    x0, y0, x1, y1 = square_of(world)
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    legs = [r for r in ph if r["phase"].startswith("leg")]
    ts = np.array([int(r.get("t_sim_us", 0)) for r in ph])
    li = [ph.index(r) for r in legs]

    print("### %s" % os.path.basename(run.rstrip("/")))
    print("   tap: %d samples, %.1f s;  command square E[%.3f %.3f] N[%.3f %.3f] (%.3f x %.3f)"
          % (len(t), (t[-1] - t[0]) * 1e-6, x0, x1, y0, y1, x1 - x0, y1 - y0))
    print("   phase log (sim t, s, relative to the tap's first sample):")
    for i, r in enumerate(ph):
        print("      %-10s %7.2f .. %7.2f s   cmd (%s, %s)  truth (%s, %s, %s)"
              % (r["phase"], (ts[i] - t[0]) * 1e-6,
                 ((ts[i + 1] - t[0]) * 1e-6 if i + 1 < len(ph) else (t[-1] - t[0]) * 1e-6),
                 r.get("cmd_x", "?"), r.get("cmd_y", "?"),
                 r.get("truth_x", "?"), r.get("truth_y", "?"), r.get("truth_z", "?")))
    if not li:
        print("   no leg phases -- not a rectangle sortie")
        return

    # --- the window tools/metrics.py ACTUALLY uses for B1 (verbatim from the code)
    rt0 = ts[li[1]] if len(li) > 1 else ts[li[0]]
    rt1 = ts[li[-1] + 1] if li[-1] + 1 < len(ph) else int(t[-1])
    m_b1 = (t >= rt0) & (t <= rt1)
    # --- the pad -> corner-1 entry leg: the leg phase BEFORE the B1 window (leg1 the FSM flies from the pad)
    m_entry = (t >= ts[li[0]]) & (t < rt0)
    # --- the whole airborne recording (the pre-2026-10-13 window, "every truth sample above 0.5 m")
    m_air = p[:, 2] > 0.5
    # --- (b) the four corner visits only
    dmin = np.min(np.stack([np.linalg.norm(p[:, :2] - np.asarray(c, float), axis=1) for c in corners]), axis=0)
    m_c15 = m_air & (dmin <= 0.15)
    m_c40 = m_air & (dmin <= 0.40)
    # --- (c) everything after the aircraft first reaches the rectangle's own bounding box
    inb = (p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1)
    first = int(np.flatnonzero(inb & m_air)[0]) if (inb & m_air).any() else None
    m_after = np.zeros(len(t), bool)
    if first is not None:
        m_after[first:] = True
        m_after &= m_air

    print("   B1's window in tools/metrics.py: [leg2 entry, land entry] = [%.2f, %.2f] s of the recording"
          % ((rt0 - t[0]) * 1e-6, (rt1 - t[0]) * 1e-6))
    print("   the extent sets, from this run's own tap.csv truth (E/N/U in W):")
    res = {}
    res["(a) B1 window as coded"] = fmt_extents(t, p, m_b1, "(a) B1 window as coded", corners)
    fmt_extents(t, p, m_entry, "    the leg1 phase BEFORE it (pad->c1)", corners)
    res["(b) corner visits, r<=0.15"] = fmt_extents(t, p, m_c15, "(b) corner visits, r <= 0.15 m", corners)
    res["(b') corner visits, r<=0.40"] = fmt_extents(t, p, m_c40, "(b') corner visits, r <= 0.40 m", corners)
    if first is not None:
        res["(c) after first bbox entry"] = fmt_extents(
            t, p, m_after, "(c) after the first bbox entry (t=%.2f s)" % ((t[first] - t[0]) * 1e-6), corners)
    res["(d) whole airborne recording"] = fmt_extents(t, p, m_air, "(d) whole airborne recording", corners)
    print("   verdict against the command box + 0.10 (E <= %.3f, N <= %.3f):"
          % (x1 - x0 + 0.10, y1 - y0 + 0.10))
    for k, v in res.items():
        if v is None:
            continue
        print("      %-32s E %.3f %s   N %.3f %s" % (k, v[0], "PASS" if v[0] <= x1 - x0 + 0.10 else "FAIL",
                                                    v[1], "PASS" if v[1] <= y1 - y0 + 0.10 else "FAIL"))
    print()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--world", default=os.path.join(PROJ, "worlds", "pillars.yaml"))
    args = ap.parse_args()
    for r in args.runs:
        probe(r, args.world)
    return 0


if __name__ == "__main__":
    sys.exit(main())
