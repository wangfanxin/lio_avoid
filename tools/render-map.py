#!/usr/bin/env python3
"""Render the accumulated cloud and the world file into one figure -- the eye's version of the gate.

Reads the npz that `tools/pillar-fit.py --dump` writes (or any cloud in W) and draws a top view and a
side view with the declared pillars on top, so a frame or mount mistake is visible, not just numeric.

usage: tools/render-map.py --npz logs/s4-pillar-*/fit_right.npz --out docs/figures/s4_map.png
"""
from __future__ import annotations

import argparse

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--title", default="simulated MID-360 clouds in W, with worlds/pillars.yaml")
    ap.add_argument("--max-points", type=int, default=120000)
    args = ap.parse_args()

    d = np.load(args.npz)
    pts = d["pts"]
    pillars = d["pillars"]
    if len(pts) > args.max_points:
        idx = np.random.default_rng(0).choice(len(pts), args.max_points, replace=False)
        pts = pts[idx]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(1, 2, figsize=(14, 6.2))
    ax[0].scatter(pts[:, 0], pts[:, 1], s=0.25, c=pts[:, 2], cmap="viridis", linewidths=0)
    for cx, cy, r, _, _ in pillars:
        ax[0].add_patch(plt.Circle((cx, cy), r, color="k", fill=False, lw=1.0))
    ax[0].plot(0, 0, "r*", ms=15, label="pad / sensor")
    ax[0].set_aspect("equal")
    ax[0].set_xlabel("East (m)")
    ax[0].set_ylabel("North (m)")
    ax[0].set_title("top view: %d points, black = declared pillars" % len(pts))
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.25)

    ax[1].scatter(pts[:, 0], pts[:, 2], s=0.25, c=pts[:, 1], cmap="magma", linewidths=0)
    ax[1].set_xlabel("East (m)")
    ax[1].set_ylabel("Up (m)")
    ax[1].set_title("front view (colour = North)")
    ax[1].grid(alpha=0.25)

    fig.suptitle(args.title)
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("wrote %s (%d points drawn of %d)" % (args.out, len(pts), len(d["pts"])))
    if "fit" in d and len(d["fit"]):
        err = d["fit"][:, 2]
        print("fitted pillars in this dump: %d, worst centre error %.4f m" % (len(err), err.max()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
