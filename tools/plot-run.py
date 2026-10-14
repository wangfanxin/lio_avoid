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
    CORNERS = np.array([[c[0], c[1]] for c in corners]) if corners else \
        np.array([[0.5, 0.4], [2.5, 0.4], [2.5, 1.6], [0.5, 1.6]])
    pillars = wcfg.get("pillars", []) or []
    sq = wcfg.get("mission", {}).get("square")
    arena = (wcfg.get("arena", {}) or {}).get("walls", {}) or {}
    ax_x, ax_y = arena.get("x", [-5.0, 5.0]), arena.get("y", [-5.0, 5.0])

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
        ax.plot(p_t[:, 0], p_t[:, 1], lw=1.6, color="#1f4fd8", label="truth (1200 Hz)")
        ax.plot(p_lw[:, 0], p_lw[:, 1], lw=0.9, color="#e8791a", alpha=0.9,
                label="LIO odometry (200 Hz, L->W)")
        if ekf is not None:
            ax.plot(ekf[:, 0], ekf[:, 1], lw=1.3, color="#2a9d3f", alpha=0.9,
                    label="the FCU's own EKF (XKF1)")
        ax.plot(np.append(CORNERS[:, 0], CORNERS[0, 0]), np.append(CORNERS[:, 1], CORNERS[0, 1]),
                "--", color="#c02020", lw=1.2, label="commanded rectangle")
        ax.plot(CORNERS[:, 0], CORNERS[:, 1], "s", color="#c02020", ms=6)
        for i, (x, y) in enumerate(CORNERS, start=1):
            ax.annotate("c%d" % i, (x, y), textcoords="offset points", xytext=(6, 4), fontsize=9,
                        color="#c02020")
        ax.plot([0], [0], "k^", ms=8)
        ax.annotate("pad", (0, 0), textcoords="offset points", xytext=(6, -12), fontsize=9)
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
    fig.suptitle("Mission A top view -- %s" % tag)
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
    fig.suptitle("Mission A position vs time -- %s" % tag)
    fig.tight_layout()
    f2 = os.path.join(out, "traj-xyzt-%s.png" % tag)
    fig.savefig(f2, dpi=130)
    plt.close(fig)

    print("wrote %s\n      %s" % (f1, f2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
