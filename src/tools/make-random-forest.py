#!/usr/bin/env python3
"""THE FOREST THE OWNER SPECIFIED (2026-10-17), verbatim:

    "我希望柱子还是随机生成的，不要重复棋盘。你把柱子林改成：每1*1m的方格，随机生成一根0.25m半径的
     柱子。中心起飞区域不要柱子。切记，这是我要的柱子林形状。"

So:
  * 1 x 1 m cells, integer boundaries, 10 x 10 = the same +-5 m arena;
  * **one pillar per cell, radius 0.25 m, at a RANDOM position inside its cell** -- jitter, not a lattice
    of centres, and NO checkerboard: every cell outside the reserved area gets its pillar;
  * the take-off area in the CENTRE is empty (nothing within 0.8 m of the pad's surface);
  * z 0 -> 10 m, seed 7.

OWNER'S SECOND INSTRUCTION (2026-10-17, same day): "随机量尽可能搞大，柱子更随机，不要人为设置这么小的上限".
So the jitter is now MAXIMAL: the wanted surface gap `min_gap` is **0.0 m**, which makes the jitter budget the
whole cell,
    lim = (cell - 2r - min_gap)/2 = (1.00 - 0.50 - 0.00)/2 = **0.25 m**
i.e. a pillar may stand ANYWHERE in its cell.  What is left is only the non-overlap rule (`hypot < 2r` is
rejected), and with lim = 0.25 m the worst possible pair of adjacent cells is exactly 0.5 m apart between
centres = just touching, so in practice nothing is ever rejected and the measured minimum gap is whatever
the draws give (reported in the audit, and it is the number that matters to the flight).

WHY IT IS NOT A CHECKERBOARD THIS TIME, in one line: the old field's `min_gap 0.8 m` needed 1.4 m between
centres, which a 1.0 m pitch cannot give, so every other cell was dropped and only one parity class survived.
Here 2r + gap = 0.50 m < 1.0 m, so adjacent cells are always admissible and the whole lattice stands.

Writes worlds/rect68_random.yaml (the tour block is copied unchanged from worlds/rect68.yaml) and prints the
audit: count, jitter, the real minimum gap, whether adjacent cells are occupied (the anti-checkerboard
test), and what the tour's clearance becomes on this field.

usage: src/tools/make-random-forest.py           (no arguments; the parameters below ARE the specification)
"""
import os
import sys

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "src"))
from sim.make_world import generate  # noqa: E402

CELLS = 10
CELL = 1.0
RADIUS = 0.25
SEED = 7
MIN_GAP = 0.0           # OWNER: "随机量尽可能搞大" -- no artificial gap is asked for, only non-overlap; the
                        # jitter budget is then the whole cell: (1.0 - 0.5 - 0.0)/2 = 0.25 m
PAD_CLEAR = 0.80       # the take-off area: no pillar SURFACE within 0.8 m of the pad
Z0, Z1 = 0.0, 10.0

base = yaml.safe_load(open(os.path.join(REPO, "worlds", "rect68.yaml")))
pillars, n_clear, n_pad, n_gap = generate(
    CELLS, CELL, RADIUS, SEED, MIN_GAP, None, PAD_CLEAR,
)
P = np.array([[x, y] for x, y, _ in pillars])
C = np.array([[c[0], c[1]] for c in base["mission"]["corners"]], dtype=float)

# ------------------------------------------------------------------ the audit
d = np.sqrt(((P[:, None, :] - P[None, :, :]) ** 2).sum(-1))
np.fill_diagonal(d, 1e9)
gap = d.min() - 2 * RADIUS


def clr_of(pts):
    return float(np.min(np.sqrt(((pts[:, None, :] - P[None, :, :]) ** 2).sum(-1)) - RADIUS))


legs = []
for k in range(len(C) - 1):
    a, b = C[k], C[k + 1]
    ss = np.linspace(0.0, 1.0, 600)[:, None]
    legs.append(clr_of(a[None, :] + ss * (b - a)[None, :]))
corners = [clr_of(c[None, :]) for c in C]

# jitter, measured: how far is each pillar from its cell centre?
centres = P - np.clip(P - np.round((P - 0.5) / CELL) * CELL - 0.5, -1, 1)
res = P - (np.floor(P / CELL) * CELL + CELL / 2.0)
jitter = np.abs(res).max(axis=1)

# the anti-checkerboard test on CELL INDICES (coordinates are jittered, so comparing positions would be wrong):
# if every cell outside the reserved area is occupied, then each pillar's in-grid neighbours must be too.
cell_of = np.floor((P + CELLS * CELL / 2.0) / CELL).astype(int)
occ = set(map(tuple, cell_of))
nbr_ok = 0
nbr_test = 0
for i, j in occ:
    for di, dj in ((1, 0), (0, 1)):
        ni, nj = i + di, j + dj
        if 0 <= ni < CELLS and 0 <= nj < CELLS:
            nbr_test += 1
            nbr_ok += (ni, nj) in occ
print("  occupied cells         %d of %d (the pad's clearing removes the rest)"
      % (len(occ), CELLS * CELLS))
print("RANDOM FOREST -- %dx%d cells of %.1f m, r %.2f m, min surface gap %.2f m, seed %d"
      % (CELLS, CELLS, CELL, RADIUS, MIN_GAP, SEED))
print("  pillars                %d   (dropped: %d for the take-off area, %d for the pad, %d by the gap rule)"
      % (len(pillars), n_clear, n_pad, n_gap))
print("  min surface gap        %.3f m (rule %.2f)" % (gap, MIN_GAP))
print("  jitter                 max %.3f m, mean %.3f m (budget %.3f m) -- a random position, not a lattice"
      % (jitter.max(), jitter.mean(), (CELL - 2 * RADIUS - MIN_GAP) / 2))
print("  adjacent CELL pairs occupied: %d of %d  (a checkerboard would be 0)" % (nbr_ok, nbr_test))
print("  the tour on THIS field: legs %s" % " ".join("%+.3f" % v for v in legs))
print("                          worst leg %+.3f m, worst corner %+.3f m"
      % (min(legs), min(corners)))
print("  (the field of record: legs +0.200, corners +0.407; the 0.5 m fine field: +0.150 / +0.254)")

head = """\
# GENERATED by src/tools/make-random-forest.py (2026-10-17) from the owner's own specification:
#   "我希望柱子还是随机生成的,不要重复棋盘。你把柱子林改成:每1*1m的方格,随机生成一根0.25m半径的柱子。
#    中心起飞区域不要柱子。切记,这是我要的柱子林形状。"
# Re-run it rather than editing the list:  python3 src/tools/make-random-forest.py
#
# THE RULE (src/sim/make_world.py:generate -- the same generator as every world here):
#   * {cells} x {cells} grid of {cell} m cells with integer boundaries, +-5 m arena;
#   * **one pillar per cell, radius {radius} m, at a RANDOM position inside its cell** -- the jitter budget is
#     (cell - 2r - min_gap)/2 = {lim} m, i.e. THE WHOLE HALF-CELL (owner: "随机量尽可能搞大"), so the pillar may
#     stand anywhere in it and the field is not a lattice;
#   * no checkerboard: 2r + min_gap = {reach} m between centres is well inside the {cell} m pitch, so EVERY cell
#     outside the reserved area stands ({n} pillars) and the gap rule never fires;
#   * the take-off area in the CENTRE is empty: no pillar surface within {pad} m of the pad;
#   * z {z0} -> {z1} m (no over-the-top).
#
# Audit (computed): {n} pillars, min surface gap {gap:.3f} m, jitter max {jit:.3f} m mean {jitm:.3f} m,
# {nok} of {nt} adjacent-cell pairs occupied.  The tour on THIS field: worst leg {wl:+.3f} m, worst corner
# {wc:+.3f} m from a pillar surface.  The mission block is COPIED from worlds/rect68.yaml.
""".format(cells=CELLS, cell=CELL, radius=RADIUS, lim=(CELL - 2 * RADIUS - MIN_GAP) / 2,
           reach=2 * RADIUS + MIN_GAP, n=len(pillars), pad=PAD_CLEAR, z0=Z0, z1=Z1, gaprule=MIN_GAP,
           gap=gap, jit=jitter.max(), jitm=jitter.mean(), nok=nbr_ok, nt=nbr_test,
           wl=min(legs), wc=min(corners))

body = {"pillars": [{"x": round(x, 3), "y": round(y, 3), "r": RADIUS, "z0": Z0, "z1": Z1}
                    for x, y, _ in pillars]}
for k in ("spawn", "pad", "arena", "sensor", "imu"):
    body[k] = base[k]
mission = dict(base["mission"])
mission["name"] = "rect68_random"
mission["note"] = ("MISSION B on the RANDOM forest the owner specified 2026-10-17: 1 x 1 m cells, one pillar "
                   "of r = 0.25 m at a random position in each, the take-off area in the centre empty.  "
                   "Same 9-waypoint, two-lap 6 x 8 m rectangle as worlds/rect68.yaml; %d pillars." % len(pillars))
body["mission"] = mission
body["mission_b"] = base["mission_b"]
body["fake_truth"] = base["fake_truth"]
body["meta"] = {"note": "RANDOM forest variant of worlds/rect68.yaml, generated by _stage/make-random-forest.py; "
                        "pillars differ, everything else is the same world."}

out = os.path.join(REPO, "worlds", "rect68_random.yaml")
with open(out, "w", encoding="utf-8") as f:
    f.write(head)
    yaml.safe_dump(body, f, sort_keys=False, default_flow_style=None, width=100)
print("wrote %s" % out)

w = yaml.safe_load(open(out))
P2 = np.array([[p["x"], p["y"]] for p in w["pillars"]])
assert len(P2) == len(P) and np.allclose(P2, np.round(P, 3))   # the file stores 3 decimals
assert np.allclose(np.array([[c[0], c[1]] for c in w["mission"]["corners"]], dtype=float), C)
print("re-read: %d pillars, same coordinates, same 9 waypoints -- OK" % len(P2))
