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

sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
from common import frames                       # noqa: E402
from tools.metrics import read_tap, read_odom, read_phase    # noqa: E402

import matplotlib                            # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt               # noqa: E402

CORNERS = np.array([[0.5, 0.4], [2.5, 0.4], [2.5, 1.6], [0.5, 1.6]])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--calib", default="/mnt/d/distill_self/_stage/calib.json")
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

    sim_t = np.array([int(r.get("t_sim_us", 0)) for r in phases])
    names = [r["phase"] for r in phases]
    t0 = t_t[0]

    # ---------------------------------------------------------------- (1) top view
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.2))
    for ax, zoom in ((a1, False), (a2, True)):
        ax.plot(p_t[:, 0], p_t[:, 1], lw=1.6, color="#1f4fd8", label="truth (1200 Hz)")
        ax.plot(p_lw[:, 0], p_lw[:, 1], lw=0.9, color="#e8791a", alpha=0.9,
                label="LIO odometry (200 Hz, L->W)")
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
        if zoom:
            ax.set_title("zoom: the rectangle (truth vs LIO)")
            ax.set_xlim(-0.35, 2.95)
            ax.set_ylim(-0.25, 2.05)
        else:
            ax.set_title("whole flight: pad, take-off, rectangle, landing")
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
