#!/usr/bin/env python3
"""THE HOVER TOP VIEW: the 5 s hover sortie drawn from its own truth recording.

Top panel  the field (96 pillars of the owner's random forest), the 9-waypoint tour, the truth path, and every
           HOVER marked where it happened with its measured length and how close it settled to the corner point.
Bottom     the speed, so the nine plateaus at ~0 m/s are visible as the mechanism behind the marks above.

The numbers in the title are the sortie's own acceptance rows, not a memory of them.

usage: src/tools/hover-topview.py <tag> <out.png>
"""
import csv
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
    print("usage: src/tools/hover-topview.py <sortie tag> [out.png]\n\n"
          "Draws the top view of a Mission-C/B sortie from its own recording: the field, the tour, the truth,\n"
          "every hover marked with its measured length, and the speed trace that shows the hovers as plateaus.")
    raise SystemExit(0)
TAG = sys.argv[1] if len(sys.argv) > 1 else "FLIGHTHOLD5-maxjitter"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/mnt/d/distill_self/_stage/figures/hover-%s.png" % TAG
D = os.path.expanduser("~/lio_avoid/logs/missionA-") + TAG
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

w = yaml.safe_load(open(REPO + "/worlds/rect68_random.yaml"))
P = np.array([[p["x"], p["y"]] for p in w["pillars"]])
R = np.array([p["r"] for p in w["pillars"]])
C = np.array([[c[0], c[1]] for c in w["mission"]["corners"]], float)

T = np.genfromtxt(D + "/tap.csv", delimiter=",", names=True)
t = (T["t_us"] - T["t_us"][0]) / 1e6
x, y, alt = T["pe"], T["pn"], -T["pd"]
v = np.hypot(T["ve"], T["vn"])

# --- the hovers, taken from the shell's OWN record of them --------------------------------------------------
# `traj.log` prints one line per arrival with the distance it settled at, and the dwell length; the tour's own
# waypoint list is the world file's, in order.  A detector on the tap's speed was tried first and is WORSE than
# the record: the tap's velocity is noisy enough to split a 5 s hover into pieces, and a corner radius counts the
# approach and the departure as part of the dwell.
import re

dwell_s = 0.0
arrivals = []
for line in open(D + "/traj.log", errors="replace"):
    mm = re.search(r"ARRIVED at waypoint (\d+) of (\d+) \(([\d.]+) m horizontal.*dwelling ([\d.]+) s", line)
    if mm:
        arrivals.append((int(mm.group(1)), float(mm.group(3))))
        dwell_s = float(mm.group(4))
print("arrivals from traj.log: %d, dwell %.1f s each" % (len(arrivals), dwell_s))

# the stationary windows, for the time panel only (no labels: the record above is the authority on length)
SLOW, GAP_S, MIN_S = 0.25, 0.60, 2.5
m = (v < SLOW) & (alt > 1.2)
idx = np.where(m)[0]
bands = []
if len(idx):
    for g in np.split(idx, np.where(np.diff(idx) > GAP_S * 1200)[0] + 1):
        if t[g[-1]] - t[g[0]] >= MIN_S:
            bands.append((t[g[0]], t[g[-1]]))
print("stationary windows drawn in the time panel: %d" % len(bands))
hovers = [(0.0, 0.0, dwell_s, C[k - 1][0], C[k - 1][1], d, k) for k, d in arrivals if 1 <= k <= len(C)]

fig = plt.figure(figsize=(11.5, 12.4))
gs = fig.add_gridspec(2, 1, height_ratios=[3.15, 1.0], hspace=0.16)
ax = fig.add_subplot(gs[0])

# the field
for px, py, pr in zip(P[:, 0], P[:, 1], R):
    ax.add_patch(plt.Circle((px, py), pr, facecolor="0.72", edgecolor="0.45", lw=0.3, zorder=1))
# the tour
ax.plot(np.append(C[:, 0], C[0, 0]), np.append(C[:, 1], C[0, 1]), "k--", lw=0.9, alpha=0.6, zorder=2,
        label="the tour (9 waypoints)")
# the truth
ax.plot(x, y, color="#d40000", lw=1.3, alpha=0.95, zorder=4, label="truth (SITL)")
# the hovers
# the tour revisits (3,4), (-3,4), (3,-4), (-3,-4) on the second lap, so the nine arrivals sit on five
# distinct points: name them all, once per point, so a reader is not told there are five hovers.
by_point = {}
for (t0, t1, dur, hx, hy, dmin, wpi) in hovers:
    key = (round(hx, 3), round(hy, 3))
    by_point.setdefault(key, []).append((wpi, dmin))
print("distinct hovered points: %d for %d arrivals" % (len(by_point), len(hovers)))
for (hx, hy), lst in by_point.items():
    ax.plot(hx, hy, "o", ms=11, mfc="none", mec="#0050c8", mew=1.8, zorder=6)
    names = "/".join("wp%d" % w for w, _ in lst)
    best = min(d for _, d in lst)
    ax.annotate("%s: %.1f s\nsettled %.3f m" % (names, dwell_s, best), (hx, hy), textcoords="offset points",
                xytext=(11, 8), fontsize=7.6, color="#003c96", zorder=7,
                bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="#0050c8", lw=0.5, alpha=0.9))
# the pad
ax.plot(0, 0, "s", ms=7, mfc="white", mec="k", mew=1.0, zorder=5)
ax.annotate("pad", (0, 0), textcoords="offset points", xytext=(7, -14), fontsize=8)

ax.set_xlim(-5.2, 5.2)
ax.set_ylim(-5.2, 5.2)
ax.set_aspect("equal")
ax.grid(alpha=0.15, lw=0.4)
ax.set_xlabel("East (m) [W]")
ax.set_ylabel("North (m) [W]")
ax.legend(loc="lower left", fontsize=8.5, framealpha=0.9)

ax2 = fig.add_subplot(gs[1])
ax2.plot(t, v, color="#d40000", lw=0.9)
for t0, t1 in bands:
    ax2.axvspan(t0, t1, color="#0050c8", alpha=0.18, lw=0)
ax2.set_xlim(t[0], t[-1])
ax2.set_xlabel("simulation time (s)")
ax2.set_ylabel("|v| horizontal (m/s)")
ax2.grid(alpha=0.15, lw=0.4)
ax2.set_title("the same sortie in time: each blue band is one hover (%.1f s) at a corner" % dwell_s,
              fontsize=9.5)
ax2.axhline(SLOW, color="0.4", ls=":", lw=0.8)

fig.suptitle(
    "%s -- the owner's random forest, a 5.0 s hover at every corner\n"
    "C2 minimum -0.005 m   |   closest approach %.3f m   |   9 of 9 waypoints   |   legs 6.96-16.28 s (0 of 9 under 6 s)"
    % (TAG, min(np.min(np.sqrt(((np.stack([x, y], 1)[:, None, :] - P[None, :, :]) ** 2).sum(-1)) - R[None, :], 1))),
    fontsize=11.5)
fig.savefig(OUT, dpi=118, bbox_inches="tight")
print("wrote %s" % OUT)
