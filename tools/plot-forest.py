#!/usr/bin/env python3
"""One map figure for a mission: the world's pillars, the commanded path, and who is in the way.

Drawn from the world file alone, so the picture cannot disagree with what the simulator renders or the
planner plans in:
  * the arena and every pillar as its true circle (radius from the file);
  * mission A's square, if the world has one -- the frozen worlds do, worlds/forest_b.yaml does not;
  * mission B's path: the radial entry leg from the pad, then the closed circle (tools/world-circle.py);
  * every pillar whose SURFACE is closer to that path than the vehicle's half footprint, in red, and the
    smallest clearance the path has anywhere, marked on the zoomed panel;
  * optionally, the truth trajectory of any run directories given, one thin line each.

The numbers in the title are measured here from the geometry, not copied from the generator: "dense" and
"in the way" have to be quantities.

usage:
    tools/plot-forest.py --world worlds/forest_b.yaml --out fig.png [--runs logs/...] [--zoom]
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from tools.metrics import read_tap          # noqa: E402

import matplotlib                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt               # noqa: E402
from matplotlib.patches import Circle, Rectangle   # noqa: E402

FOOTPRINT = 0.20        # the clearance contract, m -- the number PLAN.md 4.1 and the adapter gate on
INFLATION = 0.10        # ego-planner's own obstacle inflation, m (params/ego_planner_humble.yaml)


def load_circle_module():
    """tools/world-circle.py is a script with a hyphen in its name, so import it by path."""
    path = os.path.join(HERE, "world-circle.py")
    spec = importlib.util.spec_from_file_location("world_circle", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def seg_point_distance(p, a, b):
    """Distance from point p to segment ab, in the plane."""
    p = np.asarray(p, float)
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    d = b - a
    L2 = float(d @ d)
    t = 0.0 if L2 == 0 else float(np.clip((p - a) @ d / L2, 0.0, 1.0))
    return float(np.hypot(*(p - (a + t * d))))


def path_clearances(pillars, path):
    """Per pillar: (distance to the path polyline) - (pillar radius), and the closest point.

    Planar on purpose: the pillars are vertical cylinders from the floor up, and a mission-B path is flown
    at a constant altitude inside their height, so the horizontal distance IS the clearance.
    """
    flat = [(float(p[0]), float(p[1])) for p in path]
    out = []
    for q in pillars:
        best, bp = 1e9, None
        for k in range(len(flat) - 1):
            a, b = flat[k], flat[k + 1]
            d = seg_point_distance((q["x"], q["y"]), a, b)
            if d < best:
                # the foot of the perpendicular, for the drawing
                a2, b2 = np.asarray(a, float), np.asarray(b, float)
                dd = b2 - a2
                L2 = float(dd @ dd)
                t = 0.0 if L2 == 0 else float(np.clip((np.array([q["x"], q["y"]]) - a2) @ dd / L2, 0, 1))
                best, bp = d, a2 + t * dd
        out.append((best - q.get("r", 0.3), bp))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(HERE, "..", "worlds", "forest_b.yaml"))
    ap.add_argument("--out", default=os.path.join(HERE, "..", "docs", "figures", "map.png"))
    ap.add_argument("--runs", nargs="*", default=[])
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--footprint", type=float, default=FOOTPRINT)
    ap.add_argument("--inflation", type=float, default=INFLATION,
                    help="ego-planner's own obstacle inflation, m: drawn as a dashed ring and used to "
                         "report the free channel the PLANNER sees (corridor - 2 x inflation).  Set 0 to "
                         "hide it")
    ap.add_argument("--zoom", type=float, default=0.0,
                    help="half-width of the zoomed panel, m; 0 = draw the whole arena only")
    ap.add_argument("--title", default="")
    ax = ap.parse_args()

    wc = load_circle_module()
    cfg = yaml.safe_load(open(ax.world)) or {}
    pillars = cfg.get("pillars", []) or []
    arena = (cfg.get("arena", {}) or {}).get("walls", {}) or {}
    ax_x, ax_y = arena.get("x", [-6, 6]), arena.get("y", [-6, 6])
    sq = (cfg.get("mission", {}) or {}).get("square")
    circle = wc.read(ax.world)
    path = wc.path_points(ax.world) if circle else None
    pad = cfg.get("pad") or {}

    runs = []
    for pat in ax.runs:
        runs.extend(sorted(glob.glob(pat)))

    # ---- measured numbers -------------------------------------------------------------------------
    n = len(pillars)
    # The corridor that decides whether a vehicle can pass between two pillars is the NEAREST-NEIGHBOUR
    # surface gap, per pillar -- not the median over all pairs, which for 256 pillars is 11 m of empty
    # forest and describes nothing (MEASURED: this line read "median 11.375 m" until it was fixed).
    nn = []
    for i in range(n):
        best = 1e9
        for j in range(n):
            if i != j:
                best = min(best, float(np.hypot(pillars[i]["x"] - pillars[j]["x"],
                                                pillars[i]["y"] - pillars[j]["y"]))
                           - pillars[i].get("r", 0.3) - pillars[j].get("r", 0.3))
        nn.append(best)
    gap_txt = ("corridor between neighbouring pillars: min %.3f m, median %.3f m;  the %.2f m contract "
               "needs >= %.2f m to pass (slack %+.3f m);  ego sees a free channel of %.3f m"
               % (min(nn), float(np.median(nn)), ax.footprint, 2 * ax.footprint,
                  min(nn) - 2 * ax.footprint, min(nn) - 2 * ax.inflation)) if nn \
        else "no pillars"
    clr = path_clearances(pillars, path) if path else []
    viol = [(pillars[i], c) for i, (c, _) in enumerate(clr) if c < ax.footprint]
    minc = min((c for c, _ in clr), default=float("nan"))
    min_i = int(np.argmin([c for c, _ in clr])) if clr else -1
    # THE WAYPOINTS ARE WHAT THE MISSION COMMANDS, one at a time, and a waypoint inside a pillar is not a
    # clearance problem -- it is an impossible goal.  MEASURED 2026-10-14 on the world as specified:
    # 13 of the 24 circle points were INSIDE a pillar while the path's own minimum was only -0.271 m.
    wp_min, wp_bad, wp_in = float("nan"), 0, 0
    if circle:
        wp = np.array(circle["points"], dtype=float)
        wc_ = np.full(len(wp), np.inf)
        for q in pillars:
            wc_ = np.minimum(wc_, np.hypot(wp[:, 0] - q["x"], wp[:, 1] - q["y"]) - q.get("r", 0.3))
        wp_min, wp_bad, wp_in = float(wc_.min()), int((wc_ < ax.footprint).sum()), int((wc_ < 0).sum())
        if path:
            print("waypoints: min clearance %+.3f m, %d of %d below the %.2f m contract, %d inside a pillar"
                  % (wp_min, wp_bad, len(wp), ax.footprint, wp_in))
    if path:
        print("path: %d points, min clearance %+.3f m, %d of %d pillars within the %.2f m footprint"
              % (len(path), minc, len(viol), n, ax.footprint))
    print("%s: %d pillars, %s" % (os.path.basename(ax.world), n, gap_txt))

    # ---- figure ------------------------------------------------------------------------------------
    if ax.zoom:
        fig, axs = plt.subplots(1, 2, figsize=(19.0, 9.4))
        axes = list(axs)
    else:
        fig, a0 = plt.subplots(figsize=(10.5, 9.6))
        axes = [a0]

    for k, a in enumerate(axes):
        a.add_patch(Rectangle((ax_x[0], ax_y[0]), ax_x[1] - ax_x[0], ax_y[1] - ax_y[0], fill=False,
                              edgecolor="0.35", lw=1.0, ls="--",
                              label="arena %.1f x %.1f m" % (ax_x[1] - ax_x[0], ax_y[1] - ax_y[0])))
        for q in pillars:
            a.add_patch(Circle((q["x"], q["y"]), q.get("r", 0.3), facecolor="0.66", edgecolor="0.32",
                               lw=0.5, zorder=2))
        if ax.inflation > 0:
            # WHAT THE PLANNER SEES: ego-planner grows every occupied cell by its own inflation.  If two
            # neighbouring rings touch, that corridor is SOLID to the planner whatever the vehicle could
            # physically do, so this ring is the honest answer to "can ego get through here at all?".
            for q in pillars:
                a.add_patch(Circle((q["x"], q["y"]), q.get("r", 0.3) + ax.inflation, fill=False,
                                   edgecolor="#00a0a0", lw=0.4, ls=(0, (2, 2)), alpha=0.9, zorder=1))
            a.plot([], [], ls=(0, (2, 2)), color="#00a0a0", lw=0.8,
                   label="ego's inflation +%.2f m (its map)" % ax.inflation)
        for q, c in viol:
            a.add_patch(Circle((q["x"], q["y"]), q.get("r", 0.3) + ax.footprint, facecolor="#d02020",
                               alpha=0.16, edgecolor="none", zorder=1))
            a.add_patch(Circle((q["x"], q["y"]), q.get("r", 0.3), facecolor="#e06060",
                               edgecolor="#a01010", lw=0.8, zorder=3))
        if viol:
            a.plot([], [], "o", mfc="#e06060", mec="#a01010", ms=7,
                   label="%d of %d pillars closer to the path than %.2f m" % (len(viol), n, ax.footprint))
        if sq:
            a.add_patch(Rectangle((sq[0], sq[1]), sq[2] - sq[0], sq[3] - sq[1], fill=False,
                                  edgecolor="#1040c0", lw=1.2, ls=":",
                                  label="mission A square (frozen)"))
        if path:
            a.plot([path[0][0], path[1][0]], [path[0][1], path[1][1]], color="#1040c0", lw=1.6, ls="--",
                   zorder=5, label="entry leg from the pad")
            cx = [p[0] for p in path[1:]] + [path[1][0]]
            cy = [p[1] for p in path[1:]] + [path[1][1]]
            a.plot(cx, cy, color="#1040c0", lw=1.8, zorder=5, label="mission B path")
            # direction arrows, so clockwise/anticlockwise is visible and not inferred
            for t in (0.08, 0.33, 0.58, 0.83):
                i = int(t * (len(path) - 2)) + 1
                a.annotate("", xy=(path[i + 1][0], path[i + 1][1]), xytext=(path[i][0], path[i][1]),
                           arrowprops=dict(arrowstyle="-|>", color="#1040c0", lw=1.6), zorder=6)
        a.plot([pad.get("x", 0.0)], [pad.get("y", 0.0)], "g^", ms=11, zorder=7, label="pad (0, 0)")
        if circle:
            a.plot([circle["centre"][0]], [circle["centre"][1]], "k+", ms=12, mew=1.2, zorder=7)
        if min_i >= 0:
            a.plot([pillars[min_i]["x"]], [pillars[min_i]["y"]], "kv", ms=8, zorder=7,
                   label="nearest pillar to the path: %+.3f m" % minc)
        for d in runs:
            tap = os.path.join(d, "tap.csv")
            if not os.path.exists(tap):
                continue
            _, p = read_tap(tap)
            m = p[:, 2] > 0.3
            if m.sum() >= 100:
                a.plot(p[m][:, 0], p[m][:, 1], lw=0.9, alpha=0.8, color="#ff8000", zorder=6)
        a.set_aspect("equal")
        if k == 0 or not ax.zoom:
            a.set_xlim(ax_x[0] - 0.4, ax_x[1] + 0.4)
            a.set_ylim(ax_y[0] - 0.4, ax_y[1] + 0.4)
            a.set_title("the whole arena (%.0f x %.0f m)" % (ax_x[1] - ax_x[0], ax_y[1] - ax_y[0])
                        if ax.zoom else "")
        else:
            a.set_xlim(-ax.zoom, ax.zoom)
            a.set_ylim(-ax.zoom, ax.zoom)
            a.set_title("zoom: x, y in [%+.1f, %+.1f] m -- %.3f m per pixel at 130 dpi"
                        % (-ax.zoom, ax.zoom, (2 * ax.zoom) / (9.4 * 130)))
        a.grid(alpha=0.25)
        a.set_xlabel("E [m]")
        a.set_ylabel("N [m]")
        a.legend(loc="upper left", fontsize=8.5)

    lines = []
    if ax.title:
        lines.append(ax.title)
    lines.append("%s -- %d pillars of r = %.2f m in a %.0f x %.0f m arena"
                 % (os.path.basename(ax.world), n, (pillars[0].get("r", 0.3) if pillars else 0),
                    ax_x[1] - ax_x[0], ax_y[1] - ax_y[0]))
    lines.append(gap_txt)
    if path:
        lines.append("commanded path: pad -> %d points around a r = %.2f m circle (%.1f m of travel); "
                     "clearance min %+.3f m, %d of %d pillars inside the %.2f m footprint"
                     % (circle["n"], circle["radius"], 2 * np.pi * circle["radius"] + circle["radius"],
                        minc, len(viol), n, ax.footprint))
        lines.append("commanded WAYPOINTS: min clearance %+.3f m, %d of %d below the %.2f m contract, "
                     "%d INSIDE a pillar" % (wp_min, wp_bad, circle["n"], ax.footprint, wp_in))
    if runs:
        lines.append("%d run directories given" % len(runs))
    fig.suptitle("\n".join(lines), fontsize=9.5, y=0.995, va="top")
    fig.subplots_adjust(top=0.955 - 0.035 * len(lines))
    os.makedirs(os.path.dirname(os.path.abspath(ax.out)), exist_ok=True)
    fig.savefig(ax.out, dpi=130)
    plt.close(fig)
    print("wrote %s" % ax.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
