#!/usr/bin/env python3
"""Draw what the vehicle actually did: a top view, and position against time.

Inputs are the recordings of one run (no simulator needed): `tap.csv` is the truth at 1200 Hz, `lio.csv`
is the odometry the FCU's feed was sampled from (200 Hz, in L), `phase.csv` is the FSM's own timeline with
the commanded point in force during each phase.  The LIO's path is drawn in W through the same
calibration the FSM flew with, so both curves are in one frame and any visible gap between them is the
LIO's error, not a drawing choice.

usage: tools/plot-run.py <run-dir> [--calib _stage/calib.json] [--out DIR] [--tag NAME]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames                       # noqa: E402
from tools.metrics import read_tap, read_odom, read_phase    # noqa: E402
# `tools/world-info.py` cannot be imported by name (hyphen), and the world file is plain YAML anyway: the
# corners, the pillars, the arena and the square are read here directly.

import matplotlib                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt               # noqa: E402
from matplotlib.patches import Circle, Rectangle             # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--bin", default="", help="the FCU's flight log: draws its own EKF estimate too")
    ap.add_argument("--world", default="/mnt/d/distill_self/lio_avoid/worlds/pillars.yaml",
                    help="the world file: the corners, the pillars and therefore the plot's SCALE come "
                         "from here, so a 6 x 4 m mission in a 12 x 12 m forest is drawn as such instead "
                         "of inside a 2 x 1.2 m box (the plot used to hard-code both)")
    ap.add_argument("--out", default="")
    ap.add_argument("--tag", default="")
    ap.add_argument("--goals", default="",
                    help="mark these goals on the top view, 'E,N;E,N;...' -- for a mission whose stops are "
                         "not the world's own circle (e.g. pad -> (-5,-5), hover, -> (6,6))")
    args = ap.parse_args()
    tag = args.tag or os.path.basename(args.run.rstrip("/")).replace("missionA-", "")
    out = args.out or os.path.dirname(os.path.abspath(args.run))

    t_t, p_t = read_tap(os.path.join(args.run, "tap.csv"))
    t_l, p_l = read_odom(os.path.join(args.run, "lio.csv"))
    phases = read_phase(os.path.join(args.run, "phase.csv"))
    with open(args.calib) as fh:
        c = json.load(fh)
    R, tr = np.array(c["R_WL"]), np.array(c["t_WL"])
    p_lw = (R @ p_l.T).T + tr                     # the LIO's own path, in W
    wcfg = yaml.safe_load(open(args.world)) if os.path.exists(args.world) else {}
    corners = wcfg.get("mission", {}).get("corners", []) or []
    # A world with NO commanded rectangle must not be drawn as if it had one: MEASURED 2026-10-14, the first
    # sortie in the circle world was plotted with a fallback 2.0 x 1.2 m square and a "Mission A top view"
    # title, which is a picture of a mission that was not flown.  The circle generator that used to be read
    # here (`tools/world-circle.py`) was deleted on 2026-10-15 with the dropped ego-planner line, so a world
    # without `mission.corners` now simply draws no mission geometry -- which is the honest picture.
    has_square = bool(corners)
    CORNERS = np.array([[c[0], c[1]] for c in corners]) if has_square else np.zeros((0, 2))
    pillars = wcfg.get("pillars", []) or []
    sq = wcfg.get("mission", {}).get("square")
    arena = (wcfg.get("arena", {}) or {}).get("walls", {}) or {}
    ax_x, ax_y = arena.get("x", [-5.0, 5.0]), arena.get("y", [-5.0, 5.0])

    # EVERY PATH THE TRAJECTORY SOURCE PUBLISHED, in full, drawn over the truth -- the user asked for
    # exactly this ("每次ego发布新的path，你整条存下来画在图里，这样我才知道跟踪效果"), and the rule outlived the
    # planner it was written for: for Mission A the source is planner/rectangle_source.py.  The recorder
    # stores every point of every Path message; a source that republishes the same trajectory is grouped by
    # the trajectory's own start stamp (t0_us) -- one line per distinct PLAN, not per message.
    plans = []
    _tp = os.path.join(args.run, "traj.csv")
    _pp = _tp + ".points.csv"

    def _load_calib():
        """The L->W calibration this sortie flew with: anything recorded in L is drawn through it."""
        import json as _json
        sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        from common import frames as _frames
        with open(args.calib) as _fh:
            _d = _json.load(_fh)
        return _frames.Calib(R_WL=np.array(_d["R_WL"], dtype=float),
                             t_WL=np.array(_d["t_WL"], dtype=float))

    if os.path.exists(_tp) and os.path.exists(_pp):
        try:
            ph = np.genfromtxt(_tp, delimiter=",", names=True, dtype=None, encoding="utf-8")
            pp = np.genfromtxt(_pp, delimiter=",", names=True, dtype=None, encoding="utf-8")
            if ph.ndim == 0:
                ph = ph.reshape(1)
            if pp.ndim == 0:
                pp = pp.reshape(1)
            calib = None
            try:
                # import as a MODULE: a dataclass built from a module that is not in sys.modules raises
                # inside dataclasses itself (MEASURED 2026-10-14), and the except below then hid it by
                # drawing the plans in the wrong frame.
                calib = _load_calib()
            except Exception as _exc:
                print("   (no calibration: %s -- plans would be drawn in L, so they are NOT drawn)"
                      % _exc)
                plans = []
            # SINCE 2026-10-15 `tools/record-traj.py` writes the points in W and records that per row, so a
            # fresh recording needs no calibration to be drawn and must NOT be converted again; a file
            # without the column is an old one and its points are in L.
            pframe = str(np.atleast_1d(pp["frame"])[0]) if "frame" in pp.dtype.names else "L"
            # WHICH STAMP IN `points.csv` IDENTIFIES A PLAN -- MEASURED 2026-10-15, and it is the whole
            # reason this overlay vanished from the F2 figures ("6451 Path messages, 0 distinct plans
            # drawn").  `tools/record-traj.py` changed shape with F1: for the old `nav_msgs/Path` source
            # `points.csv.t_us` was the stamp of the Path MESSAGE, so the join key was the freshest message
            # stamp of that plan; for the `PositionCommand` source the same column holds the stamp at which
            # that leg's target was FIRST commanded, i.e. exactly `traj.csv.t0_us` (one polyline per leg,
            # 101 resampled points, written in W).  Join on `t0_us` and fall back to the message stamp, so
            # an old run still draws.
            for t0 in np.unique(ph["t0_us"]):
                sel = ph["t_us"][ph["t0_us"] == t0]
                tm = int(sel[-1] if len(sel) else t0)          # the freshest message of this plan
                rows = pp[pp["t_us"] == t0]
                if not len(rows):
                    rows = pp[pp["t_us"] == tm]
                if not len(rows):
                    continue
                q = np.column_stack([rows["e"], rows["n"], rows["u"]]).astype(float)
                if pframe == "W":
                    pass
                elif calib is not None:
                    q = (calib.R_WL @ q.T).T + calib.t_WL
                else:
                    continue
                plans.append((float(t0) * 1e-6, q))
            print("   plans: %d Path messages, %d distinct plans drawn (points in %s)"
                  % (len(ph), len(plans), pframe))
        except Exception as exc:
            print("   could not read the plan recording: %s" % exc)

    # THE PLAN IS A SEQUENCE OF WAYPOINTS, NOT A CONTINUOUS PATH -- and that is a property of the PLAN box,
    # not of the drawing: `planner/cmd_stream.py` publishes ONE target at a time with velocity and
    # acceleration ZERO (`sample()` docstring), so the straight lines between targets and the whole speed
    # profile come from the FCU's own position controller.  The markers below are those targets, in the
    # order they were commanded, converted L->W with the sortie's own calibration; the polylines above are
    # how the recorder renders the same targets.
    wps = []
    _cp = os.path.join(args.run, "traj.csv.cmd.csv")
    if os.path.exists(_cp):
        try:
            cc = np.genfromtxt(_cp, delimiter=",", names=True, dtype=None, encoding="utf-8")
            if cc.ndim == 0:
                cc = cc.reshape(1)
            for r in cc:
                q = np.array([float(r["x_l"]), float(r["y_l"]), float(r["z_l"])])
                if not len(wps) or np.linalg.norm(q - wps[-1][0]) > 1e-9:
                    wps.append((q, int(r["trajectory_id"])))
            calib_c = None
            try:
                calib_c = _load_calib()
            except Exception as _exc:
                print("   (no calibration: %s -- the commanded waypoints are in L, so they are NOT drawn)"
                      % _exc)
            if calib_c is not None and wps:
                wps = [((calib_c.R_WL @ q) + calib_c.t_WL, tid) for q, tid in wps]
            print("   commanded waypoints: %d distinct targets in %d messages%s"
                  % (len(wps), len(cc), "" if calib_c is not None else " (NOT converted, not drawn)"))
            for i, (q, tid) in enumerate(wps[:12]):
                print("      w%-2d traj_id=%-2d  E %+8.4f  N %+8.4f  U %+8.4f" % (i + 1, tid, q[0], q[1], q[2]))
        except Exception as exc:
            print("   could not read the command recording: %s" % exc)

    # the FCU's own estimate (XKF1), clock-aligned to the truth by the take-off ramp -- the same trick A4
    # uses.  This is the curve that answers "did the EKF keep what the LIO gave it".
    ekf = None
    if args.bin and os.path.exists(args.bin):
        from tools.metrics import read_ekf_pos
        t_e, p_e = read_ekf_pos(args.bin)
        p_ned = np.array([x for x in p_t])        # only for shape; the tap above is ENU
        t_ned, p_ned = read_tap(os.path.join(args.run, "tap.csv"), wire=True)
        best, lag = None, 0
        for lg in range(-3_000_000, 3_000_001, 5000):
            prof = np.interp(t_e + lg, t_ned, p_ned[:, 2])
            r = float(np.sqrt(np.mean((p_e[:, 2] - prof) ** 2)))
            if best is None or r < best:
                best, lag = r, lg
        t_e = t_e + lag
        ekf = np.column_stack([p_e[:, 1], p_e[:, 0], -p_e[:, 2]])
        print("   EKF curve: clock lag %+.2f s, vertical alignment residual %.1f mm" % (lag * 1e-6,
                                                                                        best * 1e3))

    sim_t = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    names = [r["phase"] for r in phases]
    t0 = t_t[0]

    # ---------------------------------------------------------------- (1) top view
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.2))
    for ax, zoom in ((a1, False), (a2, True)):
        # every published plan first, so the truth and the LIO sit ON TOP of it
        if plans:
            import matplotlib.cm as cm
            import matplotlib.colors as mcolors
            t0s = np.array([t for t, _ in plans])
            lo, hi = float(t0s.min()), float(t0s.max())
            norm = mcolors.Normalize(vmin=lo, vmax=hi if hi > lo else lo + 1.0)
            for t0, q in plans:
                ax.plot(q[:, 0], q[:, 1], lw=0.9, alpha=0.75,
                        color=cm.viridis(norm(t0)), zorder=2)
            sm = cm.ScalarMappable(norm=norm, cmap="viridis")
            cb = fig.colorbar(sm, ax=ax, fraction=0.035, pad=0.02)
            cb.set_label("plan start time [s since epoch]", fontsize=8)
            ax.plot([], [], lw=1.2, color=cm.viridis(0.5),
                    label="published paths (%d plans, in full)" % len(plans))
        ax.plot(p_t[:, 0], p_t[:, 1], lw=1.6, color="#1f4fd8", label="truth (1200 Hz)", zorder=5)
        # the targets themselves, on top of the truth: "the plan is these points, and only these points"
        if wps and wps[0][0][2] > -1e6:
            ax.plot([q[0] for q, _ in wps], [q[1] for q, _ in wps], ls="none", marker="o", mfc="none",
                    mec="#7a1010", ms=7, mew=1.0, zorder=6,
                    label="commanded waypoints (%d, in order)" % len(wps))
            for i, (q, _tid) in enumerate(wps, start=1):
                ax.annotate("w%d" % i, (q[0], q[1]), textcoords="offset points", xytext=(7, -11),
                            fontsize=8, color="#7a1010", zorder=7)
        ax.plot(p_lw[:, 0], p_lw[:, 1], lw=0.9, color="#e8791a", alpha=0.9,
                label="LIO odometry (200 Hz, L->W)")
        if ekf is not None:
            ax.plot(ekf[:, 0], ekf[:, 1], lw=1.3, color="#2a9d3f", alpha=0.9,
                    label="the FCU's own EKF (XKF1)")
        if has_square:
            ax.plot(np.append(CORNERS[:, 0], CORNERS[0, 0]), np.append(CORNERS[:, 1], CORNERS[0, 1]),
                    "--", color="#c02020", lw=1.2, label="commanded rectangle")
            ax.plot(CORNERS[:, 0], CORNERS[:, 1], "s", color="#c02020", ms=6)
            for i, (x, y) in enumerate(CORNERS, start=1):
                ax.annotate("c%d" % i, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=9,
                            color="#c02020")
        ax.plot([0], [0], "k^", ms=8)
        ax.annotate("pad", (0, 0), textcoords="offset points", xytext=(6, -12), fontsize=9)
        # the mission's own goals, when they are not the world's circle
        if args.goals:
            g = [[float(v) for v in s.split(",")] for s in args.goals.split(";") if s.strip()]
            ax.plot([q[0] for q in g], [q[1] for q in g], "o", mfc="#00c000", mec="#006000", ms=8,
                    zorder=9, label="mission goals (%d)" % len(g))
            for i, q in enumerate(g):
                ax.annotate("g%d" % i, (q[0], q[1]), textcoords="offset points", xytext=(6, 5), fontsize=9,
                            color="#006000")
        ax.set_xlabel("E [m]")
        ax.set_ylabel("N [m]")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        # the pillars and the arena, straight from the world file
        for q in pillars:
            ax.add_patch(Circle((q["x"], q["y"]), q.get("r", 0.3), facecolor="0.72",
                                edgecolor="0.45", lw=0.5, zorder=0))
        ax.add_patch(Rectangle((ax_x[0], ax_y[0]), ax_x[1] - ax_x[0], ax_y[1] - ax_y[0], fill=False,
                               edgecolor="0.55", ls=":", lw=0.8, zorder=0))
        if zoom and sq:
            # the ZOOM follows the mission square, whatever size it is
            cx = (sq[0] + sq[2]) / 2.0
            cy = (sq[1] + sq[3]) / 2.0
            half = max(sq[2] - sq[0], sq[3] - sq[1]) / 2.0 + 0.8
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half, cy + half)
            ax.set_title("zoom: the %.1f x %.1f m mission square (truth vs LIO vs EKF)"
                         % (sq[2] - sq[0], sq[3] - sq[1]))
        elif zoom:
            ax.set_title("zoom: the mission square")
        else:
            # the whole flight, with the arena as the frame
            ax.set_xlim(ax_x[0], ax_x[1])
            ax.set_ylim(ax_y[0], ax_y[1])
            ax.set_title("whole flight: pad, take-off, route, landing (%d pillars drawn)" % len(pillars))
            ax.legend(loc="upper left", fontsize=8)
    fig.suptitle("%s top view -- %s" % ("Mission A", tag))
    fig.tight_layout()
    f1 = os.path.join(out, "traj-xy-%s.png" % tag)
    fig.savefig(f1, dpi=130)
    plt.close(fig)

    # ---------------------------------------------------------------- (2) position vs time
    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for ax, k, nm in zip(axs, range(3), ("E", "N", "U")):
        ax.plot((t_t - t0) * 1e-6, p_t[:, k], lw=1.4, color="#1f4fd8", label="truth")
        ax.plot((t_l - t0) * 1e-6, p_lw[:, k], lw=0.9, color="#e8791a", alpha=0.9, label="LIO")
        if ekf is not None:
            ax.plot((t_e - t0) * 1e-6, ekf[:, k], lw=1.2, color="#2a9d3f", alpha=0.9,
                    label="FCU EKF")
        for s, n in zip(sim_t, names):
            if s > 0:
                ax.axvline((s - t0) * 1e-6, color="0.75", lw=0.6)
        ax.set_ylabel("%s [m]" % nm)
        ax.grid(alpha=0.3)
        if k == 0:
            ax.legend(loc="upper right", fontsize=9)
            for s, n in zip(sim_t, names):
                if s > 0:
                    ax.annotate(n, ((s - t0) * 1e-6, ax.get_ylim()[0]), fontsize=7, rotation=90,
                                color="0.45", va="bottom")
    # the commanded z per phase, as the FSM logged it
    cz = [(float(r["cmd_z"]), int(r.get("t_sim_us", 0))) for r in phases if int(r.get("t_sim_us", 0))]
    if cz:
        axs[2].step([(s - t0) * 1e-6 for _, s in cz], [v for v, _ in cz], where="post", color="#c02020",
                    ls="--", lw=1.0, label="commanded z")
        axs[2].legend(loc="upper right", fontsize=9)
    axs[2].set_xlabel("time from recording start [s]")
    fig.suptitle("%s position vs time -- %s" % ("Mission A", tag))
    fig.tight_layout()
    f2 = os.path.join(out, "traj-xyzt-%s.png" % tag)
    fig.savefig(f2, dpi=130)
    plt.close(fig)

    print("wrote %s\n      %s" % (f1, f2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
