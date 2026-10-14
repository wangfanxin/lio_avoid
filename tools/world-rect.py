#!/usr/bin/env python3
"""Mission B's flight path -- the ONE place that turns the world frame's RECTANGLE into points.

The owner's change (verbatim intent, 2026-10-15): *"别飞圆了，以后圆改，x(-4,4)，y(-3,3)的矩形"* -- stop
flying the circle; the mission is a RECTANGLE in the world frame (`world_w`), x from -4 to +4 and y from
-3 to +3.  This file is `tools/world-circle.py`'s replacement, and it keeps that tool's contract exactly:
the point list commanded, the figure drawn and the flight scored all derive from HERE, so they cannot
drift apart (the same rule as mission A's corners, tools/world-info.py).  Geometry is read once, from the
world file.

THE RECTANGLE
    bounds        x in [-half_x, +half_x], y in [-half_y, +half_y], half_x/half_y = 4.0 / 3.0 m
    centre        the world's pad (mission_b.pad, default the origin) unless the world file says otherwise
    altitude      2.00 m in W -- the same altitude rule the circle mission uses (mission_b.rect.altitude,
                  then mission_b.takeoff_alt)
    perimeter     2 * (8 + 6) = 28.0 m, so the default --n 12 gives 28 / 12 = 2.333 m between stops
    direction     COUNTER-CLOCKWISE in the x-y plane (x East, y North: -y edge -> +x edge -> +y edge -> -x
                  edge), starting at the corner nearest the pad; all four corners are (4, -3), (4, 3),
                  (-4, 3), (-4, -3), equidistant (5.0 m) from a pad at the origin, so "nearest" is a tie
                  there and the tie-break is the first corner in (-x,-y), (+x,-y), ... order = (4, -3)

THE FOUR LAYOUT RULES (each one is a decision, not an implementation detail)
  1. default: `--n` stops EVENLY SPACED ALONG THE PERIMETER, i.e. at arc length k * perimeter / n for
     k = 0 .. n-1, starting at the corner nearest the pad.  Equal ARC LENGTH, not equal angle: on a
     rectangle those differ, and equal arc length is what makes the stops evenly spread in time.
  2. `--corners`: the 4 corners only, in the same counter-clockwise order from the same corner, so the
     figure, the goal list and the metrics keep the identical starting point and direction.
  3. after placement every stop is checked against the pillars with the SAME clearance function the circle
     tool used (`planner/ego_adapter.World.clearance`, IMPORTED -- never re-implemented, because a second
     copy could disagree with the gate that actually refuses paths), and a stop closer than `--snap-need`
     SLIDES ALONG THE PERIMETER -- forward, then backward if forward does not clear -- to the nearest
     point that satisfies it.  This is the circle's rule re-parameterised by ARC LENGTH instead of by
     angle.  WHY IT EXISTS: a goal inside a pillar is not an obstacle to avoid, it is an UNREACHABLE
     TARGET (MEASURED 2026-10-14: commanding one made the adapter withhold 81 of 93 paths and left the
     vehicle 0.7338 m short), and a goal 0.02 m from a surface is one the 0.20 m gate refuses.
  4. the header line states how many of the n stops were moved, and the median and max slide in metres.

usage:
    tools/world-rect.py --world worlds/forest_b.yaml                # n lines: index, E, N, U
    tools/world-rect.py --world worlds/forest_b.yaml --json         # one object
    tools/world-rect.py --world worlds/forest_b.yaml --path         # pad + stops + closing point
    tools/world-rect.py --world worlds/forest_b.yaml --corners      # the 4 corners only
    tools/world-rect.py --world worlds/forest_b.yaml --verify       # stops + legs clearance report

--snap-need defaults to the world file's mission_b.rect.snap_need, and to 0.30 m when the file says
nothing (0 = command the rectangle verbatim).  --margin shrinks the rectangle inside the arena
(--margin 0.5 -> x in [-3.5, 3.5], y in [-2.5, 2.5]); the DEFAULT IS 0.0 ON PURPOSE, because the owner
named the bounds x(-4,4) y(-3,3) and a silent shrink would fly a different rectangle than the one asked
for.  Use it to reserve room for the vehicle's footprint or a wall-clearance rule.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

import yaml

# The owner's rectangle, in metres.  Used only when neither the command line nor the world file says
# otherwise -- a world that stores `mission_b.rect` overrides these, so a future world can fly a
# different rectangle without editing this file.
HALF_X = 4.0
HALF_Y = 3.0
ALT = 2.0
N_DEFAULT = 12
SNAP_NEED_DEFAULT = 0.30       # the clearance a commanded stop must have when nobody says otherwise
STEP_M = 0.05                  # arc-length step of the slide search, m
MAX_SLIDE_M = 0.25 * 28.0      # furthest a stop may slide, forward or backward: a quarter of the perimeter
EDGE_STEP_M = 0.05             # arc-length step of the leg sampling in --verify, m
FOOTPRINT_M = 0.20             # PLAN.md 4.1's hard contract: the same number the adapter gates on
SECOND_LEG_TAU = 0.95          # cos threshold for "these two legs are collinear"


def read(world):
    """Return the rectangle as a dict, or None if this world has no mission-B rectangle.

    Every number is looked up in the world file FIRST and falls back to the owner's rectangle, so the
    file stays the source of truth for the geometry (the rule the whole project follows) while a world
    without the new `mission_b.rect` block still gets the rectangle the owner asked for.
    """
    cfg = yaml.safe_load(open(world)) or {}
    mb = cfg.get("mission_b") or {}
    r = mb.get("rect")
    if r is None:
        r = {}
    if not isinstance(r, dict):
        return None
    pad_cfg = cfg.get("pad") or {}
    centre = [float(v) for v in r.get("centre", [pad_cfg.get("x", 0.0), pad_cfg.get("y", 0.0)])]
    alt = float(r.get("altitude", mb.get("takeoff_alt", ALT)))
    n = int(r.get("points", N_DEFAULT))
    snap = r.get("snap_need", r.get("snap"))
    return {"centre": centre,
            "half_x": float(r.get("half_x", HALF_X)), "half_y": float(r.get("half_y", HALF_Y)),
            "altitude": alt, "n": n,
            "margin": float(r.get("margin", 0.0)),
            "step": float(r.get("step", STEP_M)),
            "edge_step": float(r.get("edge_step", EDGE_STEP_M)),
            "snap_need": (None if snap is None else float(snap))}


def resolve(world, args=None):
    """The rectangle configuration the run will use: world file first, command line second, built-ins last.

    `args` is an argparse namespace (or None).  Only an option the USER actually gave on the command line
    may override the world file, which is why `read()` reports snap_need as None rather than 0.0 when the
    file is silent -- "the file says 0" and "the file says nothing" are different answers.
    """
    r = read(world)
    if r is None:
        return None
    if args is not None:
        if getattr(args, "centre", None) is not None:
            r["centre"] = [float(v) for v in args.centre]
        if getattr(args, "half_x", None) is not None:
            r["half_x"] = float(args.half_x)
        if getattr(args, "half_y", None) is not None:
            r["half_y"] = float(args.half_y)
        if getattr(args, "altitude", None) is not None:
            r["altitude"] = float(args.altitude)
        if getattr(args, "n", None) is not None:
            r["n"] = int(args.n)
        if getattr(args, "margin", None) is not None:
            r["margin"] = float(args.margin)
        if getattr(args, "step", None) is not None:
            r["step"] = float(args.step)
        if getattr(args, "edge_step", None) is not None:
            r["edge_step"] = float(args.edge_step)
        if getattr(args, "snap_need", -1.0) is not None and getattr(args, "snap_need", -1.0) >= 0:
            r["snap_need"] = float(args.snap_need)
    if r["snap_need"] is None:
        r["snap_need"] = SNAP_NEED_DEFAULT
        r["snap_need_src"] = "default"
    elif args is not None and getattr(args, "snap_need", -1.0) is not None \
            and getattr(args, "snap_need", -1.0) >= 0:
        r["snap_need_src"] = "command line"
    else:
        r["snap_need_src"] = "world file"
    r["centre"] = list(r["centre"])
    r["bounds"] = [r["centre"][0] - r["half_x"] + r["margin"], r["centre"][0] + r["half_x"] - r["margin"],
                   r["centre"][1] - r["half_y"] + r["margin"], r["centre"][1] + r["half_y"] - r["margin"]]
    return r


def pad(world):
    """The take-off point in W: the origin of the mission, and the centre the rectangle is built about."""
    cfg = yaml.safe_load(open(world)) or {}
    p = cfg.get("pad") or {}
    return [float(p.get("x", 0.0)), float(p.get("y", 0.0)),
            float((cfg.get("mission_b") or {}).get("takeoff_alt", ALT))]


def arena_walls(world):
    cfg = yaml.safe_load(open(world)) or {}
    a = cfg.get("arena") or {}
    w = a.get("walls") or {}
    return {"x": [float(v) for v in w.get("x", [-6.0, 6.0])],
            "y": [float(v) for v in w.get("y", [-6.0, 6.0])],
            "floor_z": float(a.get("floor_z", 0.0)),
            "ceiling_z": (None if a.get("ceiling_z") is None else float(a["ceiling_z"]))}


def corner_points(r):
    """The 4 corners in (bound-order) list form: (-x,-y), (+x,-y), (+x,+y), (-x,+y)."""
    x0, x1, y0, y1 = r["bounds"]
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def start_corner(r):
    """Index of the corner nearest the pad, and the distance.  Ties break on the corner order above.

    "Starting at the corner nearest the pad" is the owner-facing rule: the entry leg from the pad is
    shortest from that corner, and the corner is where the turn is sharpest, so it is the worst case for
    the slide rule -- starting anywhere else would hide it.  On the owner's rectangle with the pad at
    the origin all four corners are exactly 5.000 m away, so the tie-break (the first corner in the list,
    i.e. (-4, -3)) is what actually decides, and it is stated here so nobody has to infer it.
    """
    c = corner_points(r)
    cx, cy = r["centre"]
    d = [math.hypot(q[0] - cx, q[1] - cy) for q in c]
    i = int(min(range(4), key=lambda k: (d[k], k)))
    return i, d


def perimeter_path(r):
    """The rectangle as a closed polyline of (point, arc length) vertices, CCW from the start corner.

    Arc length is the parameter: 0 at the start corner, growing counter-clockwise once around, so
    `perimeter` is the same point as 0.  All four edges have integer length in the owner's rectangle,
    but nothing here assumes that.
    """
    c = corner_points(r)
    i0, _ = start_corner(r)
    seq = [c[(i0 + k) % 4] for k in range(4)] + [c[i0]]        # counter-clockwise, closed
    verts, s = [(seq[0], 0.0)], 0.0
    for a, b in zip(seq, seq[1:]):
        s += math.hypot(b[0] - a[0], b[1] - a[1])
        verts.append((b, s))
    return verts, s


def at_arc(r, s, verts=None, perim=None):
    """The world-frame point at arc length `s` (wrapped) on the rectangle, at the mission altitude."""
    if verts is None:
        verts, perim = perimeter_path(r)
    s = float(s) % perim
    for (a, sa), (b, sb) in zip(verts, verts[1:]):
        if s <= sb + 1e-12:
            t = 0.0 if sb <= sa else (s - sa) / (sb - sa)
            return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), r["altitude"]]
    return [verts[0][0][0], verts[0][0][1], r["altitude"]]


_WORLD_CACHE = {}


def world_clearance(world, pts):
    """Signed distance from each point to the nearest TRUE pillar surface, via the adapter's own function.

    IMPORTED, not re-implemented: PLAN.md 4.1's contract is defined by `planner/ego_adapter.World.clearance`
    (finite cylinders, corner distance above the cap), and a second copy here could disagree with the gate
    that actually refuses paths.  `ego_adapter` imports rclpy only inside its Node class, so this stays
    usable from a plain python3 with no ROS sourced.

    The loaded world is CACHED: MEASURED 2026-10-14 on the circle, the first version re-parsed the yaml and
    re-imported the adapter on EVERY point test, which made the 24-waypoint snap search take longer than a
    600 s timeout.  The rectangle's slide search tests ~350 candidate points per stop, so this matters more
    here, not less.

    Points are passed to `clearance` in ONE batched call wherever the search allows it, which is why the
    slide search below is vectorised over its candidate list.
    """
    import numpy as np
    key = os.path.abspath(world)
    if key not in _WORLD_CACHE:
        import importlib.util
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "planner", "ego_adapter.py")
        spec = importlib.util.spec_from_file_location("ego_adapter_wr", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _WORLD_CACHE[key] = mod.World.load(key)
    w = _WORLD_CACHE[key]
    return np.asarray(w.clearance(np.atleast_2d(np.asarray(pts, dtype=float))), dtype=float)


def _slide_one(world, r, s0, need, verts, perim, step, max_slide):
    """(arc length that satisfies `need`, signed slide, direction) for ONE stop.

    Rule 3, verbatim in arc length: test the stop itself; if it is closer than `need` to a pillar, walk
    FORWARD in `step` increments -- the direction of travel, so the stop keeps moving the way the mission
    flies -- and take the FIRST arc length that satisfies the clearance.  Only if no forward point within
    `max_slide` clears does it try BACKWARD the same way, and only if neither clears does the stop stay
    where it was (with a NaN slide the caller reports).
    """
    import numpy as np
    cand = np.arange(0, int(round(max_slide / step)) + 1) * step
    pts = np.array([at_arc(r, s0 + d, verts, perim) for d in cand])
    ok = world_clearance(world, pts) >= need
    if bool(ok[0]):
        return s0, 0.0, "none"
    for d, good in zip(cand[1:], ok[1:]):
        if bool(good):
            return s0 + float(d), float(d), "forward"
    for d, good in zip(cand[1:], ok[1:]):
        if bool(good):
            return s0 - float(d), -float(d), "backward"
    return s0, float("nan"), "none"


def placed(r, corners=False):
    """The arc lengths the mission will command: evenly spaced along the perimeter, or the 4 corners."""
    verts, perim = perimeter_path(r)
    n = 4 if corners else max(1, int(r["n"]))
    return [perim * k / n for k in range(n)], verts, perim


def commanded_points(world, n=None, corners=False, snap_need=None, margin=None, step=None,
                     with_slide=False, args=None):
    """The stops the mission will actually command, in W, with rule 3 applied.

    API (the one the wiring agent calls): `commanded_points(world, n=12, corners=False, snap_need=-1.0)`.
    `snap_need < 0` means "nobody overrode it": the world file's `mission_b.rect.snap_need` decides, and
    0.30 m when the file is silent.  0 commands the rectangle verbatim.

    Returns the list of [E, N, U] stops, or None when the world has no `mission_b.rect` and no pad.  With
    `with_slide=True` it returns (stops, slide_info) where slide_info["slide"] is the signed arc-length
    slide per stop (NaN = no clearing point was found) and slide_info["moved"] counts the stops that moved.
    """
    a = argparse.Namespace(n=n, corners=corners, snap_need=(-1.0 if snap_need is None else snap_need),
                           margin=margin, step=step)
    r = resolve(world, a if (n is not None or snap_need is not None or margin is not None
                             or step is not None) else args)
    if r is None:
        return (None, None) if with_slide else None
    ss, verts, perim = placed(r, corners=corners)
    need = float(r["snap_need"])
    info = {"need": need, "need_src": r.get("snap_need_src", "default"), "perim": perim,
            "step": r["step"], "max_slide": MAX_SLIDE_M, "corners": bool(corners),
            "start": at_arc(r, 0.0, verts, perim), "arc": list(ss), "slide": [], "how": []}
    if need <= 0:
        pts = [at_arc(r, s, verts, perim) for s in ss]
        info["slide"] = [0.0] * len(ss)
        info["how"] = ["none"] * len(ss)
        info["moved"] = 0
    else:
        pts, slid, how, landed = [], [], [], []
        for s in ss:
            s1, d, w = _slide_one(world, r, s, need, verts, perim, r["step"], MAX_SLIDE_M)
            pts.append(at_arc(r, s1, verts, perim))
            slid.append(d)
            how.append(w)
            landed.append(s1 % perim)
        info["slide"], info["how"], info["landed"] = slid, how, landed
        info["moved"] = sum(1 for d in slid if not math.isnan(d) and abs(d) > 1e-9)
        info["no_exit"] = sum(1 for d in slid if math.isnan(d))
    info["rect"] = {"centre": r["centre"], "half_x": r["half_x"], "half_y": r["half_y"],
                    "bounds": r["bounds"], "altitude": r["altitude"], "margin": r["margin"],
                    "perimeter": perim, "n": len(pts)}
    return (pts, info) if with_slide else pts


def path_points(world, n=None, corners=False, snap_need=None, margin=None):
    """The whole flown path: the pad, the commanded stops, and the closing stop (as world-circle.py)."""
    pts = commanded_points(world, n=n, corners=corners, snap_need=snap_need, margin=margin)
    if not pts:
        return None
    return [pad(world)] + [list(q) for q in pts] + [list(pts[0])]


# --------------------------------------------------------------------------- verification
def _sample_leg(a, b, step):
    """Points along the leg a -> b, ~`step` metres apart, ENDPOINTS INCLUDED.

    Sampled rather than solved: the pillar field is a union of 64 finite cylinders, so the exact minimum
    over a leg would need a case analysis per pillar per leg; `step` metres of sampling bounds the error
    of "the leg's minimum clearance" by the clearance gradient (<= 1) times step / 2, i.e. 2.5 cm at the
    default 0.05 m.  The vehicle flies the LEG, not the stop, so this is the number that matters.
    """
    import numpy as np
    d = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    k = max(1, int(math.ceil(d / step)))
    out = []
    for i in range(k + 1):
        t = i / float(k)
        out.append([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2]])
    return out


def leg_report(world, stops, step=None):
    """Per-leg clearance: the leg from each stop to the next, plus the closing leg back to the first.

    A stop can be perfectly clear while the straight line between two stops passes through a pillar --
    and it is the legs the vehicle flies.  Each leg is sampled every `step` metres and the MINIMUM
    clearance on it is what gets reported; a leg below the footprint is a leg the 0.20 m contract forbids
    even though both its endpoints satisfy it.
    """
    import numpy as np
    if step is None:
        step = EDGE_STEP_M
    legs = []
    m = len(stops)
    for i in range(m):
        a, b = stops[i], stops[(i + 1) % m]
        s = _sample_leg(a, b, step)
        clr = world_clearance(world, s)
        legs.append({"i": i, "j": (i + 1) % m, "a": list(a), "b": list(b),
                     "length": float(np.hypot(b[0] - a[0], b[1] - a[1])),
                     "n_samples": len(s), "min": float(clr.min()), "max": float(clr.max()),
                     "median": float(np.median(clr)),
                     "at": [float(v) for v in s[int(np.argmin(clr))]],
                     "below": int((clr < FOOTPRINT_M).sum())})
    return legs


def second_leg_collinear(leg_a, leg_b):
    """True when two legs are (nearly) the same straight line through three stops -- one physical edge.

    At n = 4 the four stops of the default layout ARE the corners, so the four "legs" are the rectangle's
    four edges; at n = 12 a leg is a piece of an edge and two consecutive legs are just one edge cut in
    two, which is worth saying in the report because "6 legs below 0.20 m" reads very differently when
    three of them are one edge.
    """
    import numpy as np
    da = np.array([leg_a["b"][0] - leg_a["a"][0], leg_a["b"][1] - leg_a["a"][1]], dtype=float)
    db = np.array([leg_b["b"][0] - leg_b["a"][0], leg_b["b"][1] - leg_b["a"][1]], dtype=float)
    na, nb = float(np.linalg.norm(da)), float(np.linalg.norm(db))
    if na < 1e-9 or nb < 1e-9:
        return False
    return float((da / na) @ (db / nb)) > SECOND_LEG_TAU


def inside_report(world, stops, r, margin=0.0):
    """Which stops are outside the arena walls, and which are outside the commanded rectangle."""
    w = arena_walls(world)
    x0, x1, y0, y1 = r["bounds"]
    out = {"arena": [], "rect": []}
    for i, q in enumerate(stops):
        if not (w["x"][0] + margin <= q[0] <= w["x"][1] - margin
                and w["y"][0] + margin <= q[1] <= w["y"][1] - margin
                and w["floor_z"] <= q[2] and (w["ceiling_z"] is None or q[2] <= w["ceiling_z"])):
            out["arena"].append(i)
        # 1e-9: the corners are built by adding half_x to the centre, so the comparison is exact up to
        # representation and a stop ON the rectangle's boundary is inside it, not outside.
        if not (x0 - 1e-9 <= q[0] <= x1 + 1e-9 and y0 - 1e-9 <= q[1] <= y1 + 1e-9):
            out["rect"].append(i)
    out["walls"] = w
    return out


def verify(world, n=None, corners=False, snap_need=None, margin=None, step=None, edge_step=None,
           pad_leg=True):
    """The whole check as data: stops, per-stop clearance, per-leg clearance, arena/rectangle containment.

    Returns a dict so a caller can print it, store it or test it.  `main(--verify)` prints exactly this.
    """
    import numpy as np
    a = argparse.Namespace(n=n, corners=corners, snap_need=(-1.0 if snap_need is None else snap_need),
                           margin=margin, step=step, edge_step=edge_step)
    r = resolve(world, a)
    if r is None:
        return None
    stops, info = commanded_points(world, n=n, corners=corners, snap_need=snap_need, margin=margin,
                                   step=step, with_slide=True)
    stop_clr = world_clearance(world, stops)
    steps = r["edge_step"] if edge_step is None else float(edge_step)
    legs = leg_report(world, stops, step=steps)
    entry = None
    if pad_leg:
        p = pad(world)
        s = _sample_leg(p, stops[0], steps)
        c = world_clearance(world, s)
        entry = {"a": list(p), "b": list(stops[0]), "length": float(np.hypot(stops[0][0] - p[0],
                                                                             stops[0][1] - p[1])),
                 "n_samples": len(s), "min": float(c.min()), "median": float(np.median(c)),
                 "at": [float(v) for v in s[int(np.argmin(c))]],
                 "below": int((c < FOOTPRINT_M).sum())}
    col = [second_leg_collinear(legs[i], legs[(i + 1) % len(legs)]) for i in range(len(legs))]
    inside = inside_report(world, stops, r)
    walls = inside.pop("walls")
    return {"world": os.path.abspath(world), "rect": info["rect"], "pad": pad(world),
            "need": info["need"], "need_src": info["need_src"], "step": info["step"],
            "edge_step": steps, "max_slide": info["max_slide"], "corners": bool(corners),
            "n": len(stops), "stops": [[float(v) for v in q] for q in stops],
            "arc": [float(s) for s in info["arc"]],
            "clearance": [float(v) for v in stop_clr],
            "slide": info["slide"], "how": info["how"], "moved": info["moved"],
            "no_exit": info.get("no_exit", 0),
            "stop_min": float(np.min(stop_clr)),
            "stop_below": int((stop_clr < FOOTPRINT_M).sum()),
            "stop_inside": int((stop_clr < 0.0).sum()),
            "legs": legs,
            "leg_min": min((l["min"] for l in legs), default=float("nan")),
            "leg_below": sum(1 for l in legs if l["min"] < FOOTPRINT_M),
            "leg_inside": sum(1 for l in legs if l["min"] < 0.0),
            "leg_collinear_with_previous": col,
            "entry_leg": entry,
            "inside": inside, "walls": walls,
            "footprint": FOOTPRINT_M}


def _print_verify(v, out=sys.stdout):
    """The clearance report, human-readable.  Every number here comes from `verify()`."""
    f = out.write
    b = v["rect"]["bounds"]
    w = v["walls"]
    f("world-rect verify: %s\n" % os.path.basename(v["world"]))
    f("  rectangle   x [%+.2f, %+.2f]  y [%+.2f, %+.2f]  centre (%+.3f, %+.3f)  z %.2f m  "
      "perimeter %.3f m  margin %.2f\n"
      % (b[0], b[1], b[2], b[3], v["rect"]["centre"][0], v["rect"]["centre"][1], v["rect"]["altitude"],
         v["rect"]["perimeter"], v["rect"]["margin"]))
    f("  arena       x %s  y %s  floor %.2f  ceiling %s\n"
      % (w["x"], w["y"], w["floor_z"], w["ceiling_z"] if w["ceiling_z"] is not None else "none"))
    f("  clearance   planner/ego_adapter.World.clearance -- the adapter's own function, IMPORTED, not "
      "re-implemented;\n              the project's hard footprint contract is %.2f m\n" % v["footprint"])
    f("  layout      %s, %d stops, snap_need %.2f m (%s), slide step %.3f m, max slide %.3f m\n"
      % ("the 4 corners" if v["corners"] else "evenly spaced along the perimeter", v["n"], v["need"],
         v["need_src"], v["step"], v["max_slide"]))
    f("  slide       %d of %d stops moved; %s\n"
      % (v["moved"], v["n"], "every stop found a clearing point"
         if not v["no_exit"] else "%d stop(s) found NO clearing point and were left blocked"
         % v["no_exit"]))
    f("\n  stops:\n")
    f("   idx    arc m        E          N        U     clearance m   slide m  how\n")
    for i, q in enumerate(v["stops"]):
        c = v["clearance"][i]
        note = ""
        if c < 0.0:
            note = "   <-- INSIDE A PILLAR"
        elif c < v["footprint"]:
            note = "   <-- closer than %.2f m" % v["footprint"]
        f("  %3d  %8.3f  %+9.4f  %+9.4f  %6.3f  %+10.4f  %+8.3f  %s%s\n"
          % (i, v["arc"][i], q[0], q[1], q[2], c, v["slide"][i], v["how"][i], note))
    f("  STOP MINIMUM over the %d stops: %+.4f m;  stops closer than %.2f m: %d of %d;  inside a "
      "pillar: %d\n" % (v["n"], v["stop_min"], v["footprint"], v["stop_below"], v["n"], v["stop_inside"]))
    f("\n  legs (each leg sampled every %.3f m, endpoints included; the vehicle flies the LEG):\n"
      % v["edge_step"])
    for l in v["legs"]:
        mark = "   <-- BELOW %.2f m" % v["footprint"] if l["min"] < v["footprint"] else ""
        if l["min"] < 0.0:
            mark = "   <-- CROSSES A PILLAR"
        f("   leg %2d -> %2d  len %6.3f m  %4d samples  min %+8.4f m  median %+7.3f  at E %+7.3f "
          "N %+7.3f%s\n" % (l["i"], l["j"], l["length"], l["n_samples"], l["min"], l["median"],
                            l["at"][0], l["at"][1], mark))
    f("  LEG MINIMUM over the %d legs: %+.4f m;  legs whose minimum is below %.2f m: %d of %d\n"
      % (len(v["legs"]), v["leg_min"], v["footprint"], v["leg_below"], len(v["legs"])))
    if v["entry_leg"]:
        e = v["entry_leg"]
        f("  entry leg pad -> stop 0: %.3f m, min %+.4f m, median %+.3f m, at E %+7.3f N %+7.3f%s\n"
          % (e["length"], e["min"], e["median"], e["at"][0], e["at"][1],
             "   <-- BELOW %.2f m" % v["footprint"] if e["min"] < v["footprint"] else ""))
    f("  sanity      stops outside the arena: %s;  stops outside the rectangle: %s\n"
      % ("none" if not v["inside"]["arena"] else v["inside"]["arena"],
         "none" if not v["inside"]["rect"] else v["inside"]["rect"]))


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "forest_b.yaml"))
    ap.add_argument("--n", type=int, default=None,
                    help="stops evenly spaced along the perimeter (default %d, or the world file's "
                         "mission_b.rect.points)" % N_DEFAULT)
    ap.add_argument("--corners", action="store_true", help="fly only the 4 corners")
    ap.add_argument("--snap-need", type=float, default=-1.0,
                    help="clearance a commanded stop must have; a blocked one SLIDES ALONG THE PERIMETER "
                         "(forward, then backward) to the nearest free spot (metres).  Default: the world "
                         "file's mission_b.rect.snap_need, else %.2f m.  0 = command the rectangle verbatim"
                         % SNAP_NEED_DEFAULT)
    ap.add_argument("--margin", type=float, default=None,
                    help="shrink the rectangle this far inside the arena (0 = the owner's bounds exactly)")
    ap.add_argument("--half-x", type=float, default=None, help="override the rectangle's half width")
    ap.add_argument("--half-y", type=float, default=None, help="override the rectangle's half height")
    ap.add_argument("--centre", type=float, nargs=2, default=None, metavar=("E", "N"),
                    help="override the rectangle's centre (default: the world's pad)")
    ap.add_argument("--altitude", type=float, default=None, help="override the mission altitude, m")
    ap.add_argument("--step", type=float, default=None, help="arc-length step of the slide search, m")
    ap.add_argument("--edge-step", type=float, default=None, help="sampling step of the leg check, m")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--path", action="store_true",
                    help="print the whole path (pad + stops + closing point) instead of the rectangle")
    ap.add_argument("--verify", action="store_true",
                    help="print the clearance report: per stop, per leg, and the arena/rectangle sanity "
                         "check (rule 3's evidence)")
    ap.add_argument("--no-pad-leg", action="store_true", help="omit the pad -> stop 0 entry leg")
    args = ap.parse_args()

    if not os.path.exists(args.world):
        print("no world file at %s" % args.world, file=sys.stderr)
        return 1
    r = resolve(args.world, args)
    if r is None:
        print("%s has no mission_b.rect and no pad" % args.world, file=sys.stderr)
        return 1
    stops, info = commanded_points(args.world, n=args.n, corners=args.corners, snap_need=args.snap_need,
                                   margin=args.margin, step=args.step, with_slide=True)
    if not stops:
        print("%s has no mission_b.rect" % args.world, file=sys.stderr)
        return 1

    moved = [d for d in info["slide"] if not math.isnan(d) and abs(d) > 1e-9]
    if info["need"] > 0:
        print("# slid along the perimeter to clearance >= %.2f m (%s): %d of %d stops moved, "
              "arc median %.3f / max %.3f m"
              % (info["need"], info["need_src"], info["moved"], len(stops),
                 (sorted(moved)[len(moved) // 2] if moved else 0.0), (max(moved) if moved else 0.0)),
              file=sys.stderr)
    else:
        print("# the rectangle is commanded VERBATIM (snap_need 0): no stop was checked against the "
              "pillars", file=sys.stderr)

    if args.verify:
        v = verify(args.world, n=args.n, corners=args.corners, snap_need=args.snap_need,
                   margin=args.margin, step=args.step, edge_step=args.edge_step,
                   pad_leg=not args.no_pad_leg)
        if args.json:
            print(json.dumps(v, indent=1))
            return 0
        _print_verify(v)
        return 0

    if args.json:
        out = {"world": os.path.abspath(args.world), "pad": pad(args.world), "rect": info["rect"],
               "n": len(stops), "corners": bool(args.corners), "need": info["need"],
               "need_src": info["need_src"], "perimeter": info["perim"], "moved": info["moved"],
               "slide": info["slide"], "how": info["how"], "points": stops,
               "path": path_points(args.world, n=args.n, corners=args.corners, snap_need=args.snap_need,
                                   margin=args.margin)}
        print(json.dumps(out, indent=1))
        return 0

    pts = path_points(args.world, n=args.n, corners=args.corners, snap_need=args.snap_need,
                      margin=args.margin) if args.path else stops
    for i, q in enumerate(pts):
        print("%4d  E %+8.4f  N %+8.4f  U %+6.3f" % (i, q[0], q[1], q[2]))
    print("# %d points, rectangle x [%+.2f, %+.2f] y [%+.2f, %+.2f], centre (%+.3f, %+.3f), "
          "perimeter %.3f m, altitude %.3f m, counter-clockwise from the corner nearest the pad"
          % (len(pts), r["bounds"][0], r["bounds"][1], r["bounds"][2], r["bounds"][3],
             r["centre"][0], r["centre"][1], info["perim"], r["altitude"]))
    return 0


# --------------------------------------------------------------------------------------------
# HOW TO WIRE THIS  (for the agent adding `--mission rect`; no other file was touched by this commit)
#
#   * THE STOP LIST.  Replace the world-circle call with this one, same call shape:
#         tools/world-rect.py::commanded_points(world, n=12, corners=False, snap_need=-1.0)
#     It returns [[E, N, U], ...] in world_w.  `snap_need=-1.0` (the default) means "the world file
#     decides, else 0.30 m", so the wiring does not have to know the number.  Pass n=<int> for --n, and
#     corners=True for --corners; pass snap_need=0 to command the raw rectangle.
#   * tools/mission-params.py: swap its `load_world_module()` target from `tools/world-circle.py` to
#     `tools/world-rect.py` and keep calling `commanded_points(args.world)` -- the signature is
#     compatible, so the W->L conversion, the round-trip check and the YAML key form all stay as they are.
#   * planner/ego_adapter.py::_stops(): point the importlib load at world-rect.py and call
#     `mod.commanded_points(self.args.world)`.  Do NOT change the footprint gate; it is the same 0.20 m
#     this tool measures against.
#   * fsm/mission_fsm.py and tools/run-mission.sh: the mission name/option is theirs to add -- the only
#     thing needed here is that "rect" routes to the stop list above and NOT to the circle.
#   * THE FIGURE/metrics: tools/plot-forest.py calls `wc.commanded_points` / `wc.path_points` on the
#     circle module; the same two names exist here with the same meaning (`path_points` = pad + stops +
#     closing stop), plus `verify()` for the clearance table and `--verify` on the command line.
#   * WORLD FILE (nobody has to edit it for the owner's rectangle): if `mission_b.rect` is present it wins
#     -- keys: centre [E, N] (default the pad), half_x (4.0), half_y (3.0), altitude (2.00), points (12),
#     margin (0.0), step (0.05), edge_step (0.05), snap_need (0.30).
# --------------------------------------------------------------------------------------------
if __name__ == "__main__":
    sys.exit(main())
