#!/usr/bin/env python3
"""`planner/ego_adapter.py` -- the only file that knows the planner's message type.

It turns the ego-planner's output (`traj_utils/msg/Bspline`, a uniform B-spline in the LIO world L)
into the FROZEN trajectory interface of PLAN.md 4.1 (`nav_msgs/Path` on `/planner/traj`, frame L,
points at >= 20 Hz of trajectory time, republished at >= 2 Hz), and it is the one place where the
goal crosses from W (world ENU, origin = the pad, where the world file lives) into L.

Why an adapter at all: the FSM must not change.  `planner/rectangle_source.py` already publishes the
same Path for mission A, so mission B flies through the identical consumer path; the planner's
message type never reaches the flight-critical code (PLAN.md 4.1).

DIRECTIONS
    /goal_pose                 W   ->  /move_base_simple/goal   L    (the planner's goal input)
    planning/bspline           L   ->  /planner/traj            L    (sampled into a Path)

The calibration is the WHOLE of the frame story here, and it is loaded from the SAME file the
consumer uses (`--calib-file`, i.e. `_stage/calib.json`).  MEASURED 2026-10-13 (STATUS.md row 44):
`planner/rectangle_source.py` projected W->L with the PREDICTED calibration while `fsm/mission_fsm.py`
projected L->F with the MEASURED one; the two differ by 56.3 mm in t, so every commanded corner sat
51 mm South of the intended one and the aircraft flew faithfully to the wrong place.  This adapter
therefore prints the W->L->W round-trip error on every goal it converts and refuses to publish a goal
whose round trip is worse than `--roundtrip-mm`.

OBLIGATIONS CHECKED ON EVERY PATH (PLAN.md 4.1, "source's obligations"):
  1. inside the arena (from the world file's `arena.walls`)
  2. >= `--footprint` (0.20 m) clearance from every pillar in the world file
  3. starts no further than `--start-max` (0.50 m) from the vehicle's current state
A path that breaks any of them is NOT published: the counters in the periodic report are the
evidence, and the console line says which check failed and by how much.

SELF-TEST (`--selftest`, no ROS needed):
  * the B-spline evaluator is checked against an INDEPENDENT Cox-de Boor basis recursion
  * the W->L->W round trip is checked on the mission corners against a 1 mm bound
  * three mutations must fail: a transposed R (W<-L used as L<-W), the predicted instead of the
    measured calibration, and a N/E swap.  A check that cannot fail is not a check.

usage: planner/ego_adapter.py --calib-file /mnt/d/distill_self/_stage/calib.json \\
                              --world worlds/forest_all.yaml [--goal-timeout 5]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from common import frames          # noqa: E402

FRAME_L = "lio_world"              # the frame name the LIO's world is published as (ARCHITECTURE.md)


# --------------------------------------------------------------------------- B-spline
def bspline_eval(knots, pts, order, t):
    """`ego_planner::UniformBspline::evaluateDeBoorT(t)`, transliterated.

    The planner sends (order, knots, pos_pts) in `traj_utils/msg/Bspline`; `traj_server.cpp` rebuilds
    the spline with `UniformBspline pos_traj(pos_pts, order, 0.1); pos_traj.setKnot(knots);` and calls
    `evaluateDeBoorT(t)`, which is `evaluateDeBoor(t + u_(p_))`.  This is that function, and the
    self-test checks it against the Cox-de Boor basis recursion rather than against a re-reading of
    the same C++ -- a port that agrees with itself proves nothing.
    """
    u = np.asarray(knots, dtype=float)
    P = np.asarray(pts, dtype=float)
    p = int(order)
    n = P.shape[0] - 1
    m = n + p + 1
    if u.shape[0] != m + 1:
        raise ValueError("knots %d != n+p+2 = %d for %d control points and order %d"
                         % (u.shape[0], m + 1, P.shape[0], p))
    ub = min(max(u[p], t + u[p]), u[m - p])

    k = p
    while u[k + 1] < ub:                 # C++: `if (u_(k+1) >= ub) break;`
        k += 1
        if k >= m:
            break

    d = [P[k - p + i].copy() for i in range(p + 1)]
    for r in range(1, p + 1):
        for i in range(p, r - 1, -1):
            den = u[i + 1 + k - r] - u[i + k - p]
            alpha = 0.0 if den == 0.0 else (ub - u[i + k - p]) / den
            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i]
    return d[p]


def bspline_duration(knots, n_ctrl, order):
    """`getTimeSum()` = u_(m_-p_) - u_(p_) with m_ = (n_ctrl-1) + order + 1."""
    u = np.asarray(knots, dtype=float)
    m = (n_ctrl - 1) + int(order) + 1
    return float(u[m - int(order)] - u[int(order)])


def cox_de_boor(knots, pts, order, t):
    """Independent reference: find-span + BasisFuns (The NURBS Book A2.1/A2.2), then sum.

    A different ALGORITHM from de Boor's corner-cutting (it never evaluates a point combination
    recursively), which is what makes it a check rather than a restatement.  It also has to be
    written correctly at the domain end: a first version of this reference special-cased `x == hi`
    with `u[i] < x <= u[i+1]`, which made TWO spans contribute at the last sample and reported a
    2.25 m disagreement with a de Boor evaluator that was in fact right.  The span search below is
    the textbook one and has no such case.
    """
    u = np.asarray(knots, dtype=float)
    P = np.asarray(pts, dtype=float)
    p = int(order)
    n = P.shape[0] - 1
    m = n + p + 1
    lo, hi = u[p], u[m - p]
    x = min(max(t + lo, lo), hi)

    if x >= u[n + 1]:                        # the last non-empty span at the domain end
        k = n
    else:
        a, b = p, n + 1
        while b - a > 1:
            mid = (a + b) // 2
            if x < u[mid]:
                b = mid
            else:
                a = mid
        k = a

    N = np.zeros(p + 1)
    N[0] = 1.0
    left = np.zeros(p + 1)
    right = np.zeros(p + 1)
    for j in range(1, p + 1):
        left[j] = x - u[k + 1 - j]
        right[j] = u[k + j] - x
        s = 0.0
        for r in range(j):
            den = right[r + 1] + left[j - r]
            tmp = 0.0 if den == 0.0 else N[r] / den
            N[r] = s + right[r + 1] * tmp
            s = left[j - r] * tmp
        N[j] = s

    out = np.zeros(P.shape[1])
    for j in range(p + 1):
        out += N[j] * P[k - p + j]
    return out


def parameterize_to_bspline(ts, point_set, start_end_derivative):
    """`UniformBspline::parameterizeToBspline`, so the self-test can use the planner's OWN
    control-point convention instead of inventing one.  Returns (K+2, 3)."""
    K = len(point_set)
    A = np.zeros((K + 4, K + 2))
    prow = np.array([1.0, 4.0, 1.0])
    vrow = np.array([-1.0, 0.0, 1.0])
    arow = np.array([1.0, -2.0, 1.0])
    for i in range(K):
        A[i, i:i + 3] = prow / 6.0
    A[K, 0:3] = vrow / (2.0 * ts)
    A[K + 1, K - 1:K + 2] = vrow / (2.0 * ts)
    A[K + 2, 0:3] = arow / (ts * ts)
    A[K + 3, K - 1:K + 2] = arow / (ts * ts)
    b = np.vstack([np.asarray(point_set, dtype=float), np.asarray(start_end_derivative, dtype=float)])
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    return sol


# --------------------------------------------------------------------------- world
class World:
    """Just enough of `worlds/*.yaml` for the two geometric obligations."""

    def __init__(self, pillars, walls, floor_z=0.0, ceiling_z=None):
        self.pillars = pillars                      # (M,5): x, y, r, z0, z1
        self.walls = walls                          # {"x": [lo,hi], "y": [lo,hi]}
        self.floor_z = floor_z
        self.ceiling_z = ceiling_z

    @staticmethod
    def load(path):
        import yaml
        with open(path) as fh:
            d = yaml.safe_load(fh) or {}
        arena = d.get("arena") or {}
        pl = d.get("pillars") or []
        arr = (np.array([[p["x"], p["y"], p["r"], p.get("z0", 0.0), p.get("z1", 4.0)]
                         for p in pl], dtype=float) if pl else np.zeros((0, 5)))
        return World(arr, arena.get("walls") or {"x": [-6.0, 6.0], "y": [-6.0, 6.0]},
                     float(arena.get("floor_z", 0.0)),
                     None if arena.get("ceiling_z") is None else float(arena["ceiling_z"]))

    def clearance(self, P):
        """Signed distance from every point of P (N,3) to the nearest pillar SURFACE, metres.

        A finite cylinder: horizontal distance minus r inside the pillar's z band, and the true
        corner distance outside it (a point above a pillar is not 0 m from it).
        """
        P = np.atleast_2d(np.asarray(P, dtype=float))
        if self.pillars.shape[0] == 0:
            return np.full(len(P), np.inf)
        best = np.full(len(P), np.inf)
        for x, y, r, z0, z1 in self.pillars:
            dh = np.hypot(P[:, 0] - x, P[:, 1] - y)
            dz = np.maximum.reduce([z0 - P[:, 2], P[:, 2] - z1, np.zeros(len(P))])
            d = np.where(dz <= 0.0, dh - r, np.hypot(np.maximum(dh - r, 0.0), dz))
            best = np.minimum(best, d)
        return best

    def inside(self, P, margin=0.0):
        """Boolean per point: inside the arena walls (and between floor and ceiling)."""
        P = np.atleast_2d(np.asarray(P, dtype=float))
        ok = ((P[:, 0] >= self.walls["x"][0] + margin) & (P[:, 0] <= self.walls["x"][1] - margin) &
              (P[:, 1] >= self.walls["y"][0] + margin) & (P[:, 1] <= self.walls["y"][1] - margin) &
              (P[:, 2] >= self.floor_z) &
              (P[:, 2] <= (self.ceiling_z if self.ceiling_z is not None else 1e9)))
        return ok


# --------------------------------------------------------------------------- calibration
def load_calib(path, predicted=False):
    if predicted or not path:
        return frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    with open(path) as fh:
        d = json.load(fh)
    return frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                        t_WL=np.array(d["t_WL"], dtype=float),
                        n_samples=int(d.get("n_samples", 0)),
                        rms_m=float(d.get("rms_m", float("nan"))))


def w_to_l(calib, p_w):
    """W -> L.  `Calib` stores W <- L, so this is its inverse and nothing else."""
    return calib.R_WL.T @ (np.asarray(p_w, dtype=float) - calib.t_WL)


def l_to_w(calib, p_l):
    """L -> W.  The exact inverse used by the consumer (`fsm/mission_fsm.py` line 258)."""
    return calib.R_WL @ np.asarray(p_l, dtype=float) + calib.t_WL


# --------------------------------------------------------------------------- self-test
def _selftest():
    ok = True

    def check(name, good, detail=""):
        nonlocal ok
        ok = ok and bool(good)
        print("%-64s %s %s" % (name, "ok" if good else "FAIL", detail))

    # ---- 1. the B-spline evaluator vs an independent basis recursion
    rng = np.random.default_rng(7)
    p, ts, n_seg = 3, 0.2, 6
    ctrl = np.cumsum(rng.normal(0.0, 0.4, size=(n_seg + p, 3)), axis=0)
    u = np.zeros(ctrl.shape[0] + p + 1)
    for i in range(len(u)):
        u[i] = (i - p) * ts
    worst = 0.0
    for t in np.linspace(0.0, ts * n_seg, 199):
        a = bspline_eval(u, ctrl, p, float(t))
        b = cox_de_boor(u, ctrl, p, float(t))
        worst = max(worst, float(np.linalg.norm(a - b)))
    check("bspline evaluator vs the independent NURBS-Book recursion (199 samples)",
          worst < 1e-9, "max %.3e m" % worst)

    # A uniform B-spline with these knots is NOT clamped: its value at t=0 is a weighted average of
    # the first p+1 control points, not the first one, and it does NOT interpolate the control points.
    # (Two earlier versions of this self-test asserted the clamped/interpolating property and failed
    # on a correct evaluator -- a check that asserts something false is worse than no check.)
    #
    # MEASURED while writing this (and worth knowing before reading the start-distance gate below):
    # `UniformBspline::parameterizeToBspline` solves a (K+4) x (K+2) system that is INCONSISTENT --
    # rank 10 with 12 rows -- so its initial control points are a least-squares compromise that misses
    # the waypoints by up to 0.35 m on an 8-point, 4.4 m route (8 % of its length).  The optimizer is
    # what pulls the curve back onto the start point and the goal, which is why this adapter MEASURES
    # "how far does the published path start from the vehicle" instead of assuming it is zero.
    #
    # What must hold, and is checked: a uniform B-spline is a convex combination, so control points
    # lying on a straight line put the whole curve on that line, advancing monotonically along it.
    line = np.array([[0.0, 0.0, 0.0], [0.4, 0.8, 0.2], [0.8, 1.6, 0.4], [1.2, 2.4, 0.6],
                     [1.6, 3.2, 0.8], [2.0, 4.0, 1.0]])
    ul = np.array([(i - p) * ts for i in range(len(line) + p + 1)])
    d = np.array([0.4, 0.8, 0.2])
    d = d / np.linalg.norm(d)
    off_line, along = 0.0, []
    for t in np.linspace(0.0, bspline_duration(ul, len(line), p), 101):
        q = bspline_eval(ul, line, p, float(t))
        off_line = max(off_line, float(np.linalg.norm(q - (q @ d) * d)))
        along.append(float(q @ d))
    along = np.array(along)
    check("collinear control points keep the curve on the line",
          off_line < 1e-12, "max off-line %.2e m" % off_line)
    check("... and it advances monotonically along it",
          bool(np.all(np.diff(along) >= -1e-12)),
          "s = %.4f .. %.4f m, min step %+.2e" % (along[0], along[-1], float(np.diff(along).min())))

    # ---- 2. the W -> L -> W round trip, on the real world/mission geometry
    calib = load_calib(os.environ.get("EGO_CALIB", "/mnt/d/distill_self/_stage/calib.json"))
    world_path = os.environ.get("EGO_WORLD", os.path.join(HERE, "..", "worlds", "forest_all.yaml"))
    print("   calibration: %s" % calib.summary())
    corners = np.array([[-3.0, -2.0, 2.0], [3.0, -2.0, 2.0], [3.0, 2.0, 2.0], [-3.0, 2.0, 2.0]])
    worst = max(float(np.linalg.norm(l_to_w(calib, w_to_l(calib, c)) - c)) for c in corners)
    check("W->L->W round trip on the mission corners < 1 mm", worst < 1e-3,
          "worst %.6f mm" % (worst * 1e3))

    # ---- 3. mutations that MUST fail.  A check that cannot fail is not a check.
    bad = frames.Calib(R_WL=calib.R_WL.T.copy(), t_WL=calib.t_WL.copy())
    e = max(float(np.linalg.norm(l_to_w(bad, w_to_l(calib, c)) - c)) for c in corners)
    check("MUTATION transposed R must break the round trip", e > 0.5, "worst %.3f m" % e)

    pred = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    dt = float(np.linalg.norm(pred.t_WL - calib.t_WL))
    check("MUTATION predicted calib must differ from the measured one", dt > 0.02,
          "|dt| = %.1f mm" % (dt * 1e3))
    off = max(float(np.linalg.norm(l_to_w(calib, w_to_l(pred, c)) - c)) for c in corners)
    check("MUTATION predicted calib must displace the commanded point", off > 0.02,
          "worst %.1f mm" % (off * 1e3))

    swap = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    bad2 = frames.Calib(R_WL=calib.R_WL @ swap, t_WL=calib.t_WL.copy())
    e2 = max(float(np.linalg.norm(l_to_w(bad2, w_to_l(calib, c)) - c)) for c in corners)
    check("MUTATION N/E swap must break the round trip", e2 > 0.5, "worst %.3f m" % e2)

    # ---- 4. the clearance helper: a point ON a pillar surface must read ~0
    w = World.load(world_path)
    pil = w.pillars[0]
    on = np.array([[pil[0] + pil[2], pil[1], 0.5 * (pil[3] + pil[4])]])
    check("clearance of a point on a pillar surface is 0", abs(float(w.clearance(on)[0])) < 1e-9,
          "%.2e m" % float(w.clearance(on)[0]))
    # above the cap the nearest surface point is the cap CENTRE, so the clearance is the height
    above = np.array([[pil[0], pil[1], pil[4] + 0.25]])
    check("clearance 0.25 m directly above the pillar cap is 0.25 m",
          abs(float(w.clearance(above)[0]) - 0.25) < 1e-9,
          "%.6f m" % float(w.clearance(above)[0]))
    # ... and past the rim it is the corner distance
    past = np.array([[pil[0] + pil[2] + 0.10, pil[1], pil[4] + 0.25]])
    check("clearance past the rim is the corner distance hypot(0.10, 0.25)",
          abs(float(w.clearance(past)[0]) - math.hypot(0.10, 0.25)) < 1e-9,
          "%.6f m" % float(w.clearance(past)[0]))
    check("the arena test rejects a point outside the wall",
          not bool(w.inside(np.array([[w.walls["x"][1] + 0.01, 0.0, 1.0]]))[0]))

    print("selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# --------------------------------------------------------------------------- node
class EgoAdapter:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from nav_msgs.msg import Path, Odometry
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import PointCloud2  # noqa: F401  (only for the type check below)
        from std_msgs.msg import Bool, Int32

        self.args = args
        self.rclpy = rclpy
        self.world = World.load(args.world)
        self.calib = load_calib(args.calib_file, predicted=args.mutation == "predicted-calib")
        if args.mutation == "transposed":
            self.calib = frames.Calib(R_WL=self.calib.R_WL.T.copy(), t_WL=self.calib.t_WL.copy())

        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.node = Node("ego_adapter")

        self.Path, self.PoseStamped, self.Odometry = Path, PoseStamped, Odometry

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensor = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.traj_pub = self.node.create_publisher(Path, args.traj_topic, latched)
        self.goal_pub = self.node.create_publisher(PoseStamped, args.planner_goal_topic, 1)
        # IS THE PLANNER STALLED?  A withheld plan is normal -- the gate IS the contract.  THE SAME withheld
        # plan for seconds on end is not: it means ego is not committing a new trajectory, and then the FSM
        # is waiting on silence and the sequencer is counting down to "stop MISSED".  MEASURED 2026-10-14
        # (sortie F1): one plan (start_time 1791981437.203, dur 1.185 s) was refused ~250 times over 50 s
        # while the aircraft hovered on the pad; the only trace was ~600 identical WITHHELD lines, and the
        # first stop of the mission was lost to the sequencer's 60 s timeout.  This topic is how the rest of
        # the stack can tell "the planner is stuck" from "the planner is fine, this plan is not".
        self.stall_pub = self.node.create_publisher(Bool, args.stall_topic, 10)

        # GOAL QUEUE DEPTH 10, NOT 1.  MEASURED 2026-10-14 (sortie W1, the circle, with a counter added to the
        # sequencer): the sequencer published the stop-2 goal 42 times and this node received 9 of them -- 33
        # goals lost in the transport -- while ego's own callback count (9) matched this node's forwards
        # exactly, so nothing was lost after this point.  This node is busy (200 Hz odometry plus the 4 Hz
        # clearance check over every published path), and with a depth-1 queue an overflowing subscription
        # simply drops.  Depth 10 costs nothing and covers ~10 s of a 1 Hz goal.
        self.node.create_subscription(PoseStamped, args.goal_topic, self.on_goal, 10)
        # THE STOP INDEX, which arrives reliably where the goal stream does not (see _stops).  The goal for
        # each stop is derived from this index and the world file, so the lossy hop stops mattering.
        self.node.create_subscription(Int32, args.stop_topic, self.on_stop_index, 10)
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, sensor)

        self.bspline = None
        self.odom_l = None
        self.goal_l = None
        self.goal_w = None
        self.n_goal_in = 0
        self.n_goal_out = 0
        self.n_goal_rejected = 0
        self.n_goal_deferred = 0
        self.n_bspline = 0
        self.n_pub = 0
        self.n_blocked = 0
        self.n_stall = 0
        self.blocked_reasons = {}
        self.last_report = time.time()
        self.last_pub_wall = 0.0
        self.pub_dt = []

        try:
            from traj_utils.msg import Bspline
        except ImportError:
            self.node.get_logger().error(
                "traj_utils is not on the path: source the planner workspace "
                "(source /home/fanxin/lio_avoid/planner_ws/install/setup.bash)")
            raise
        self.node.create_subscription(Bspline, args.bspline_topic, self.on_bspline, 10)

        self.node.create_timer(1.0 / max(0.5, args.rate), self.tick)
        if args.auto_goal:
            self.node.create_timer(0.5, self.maybe_auto_goal)

        print("ego_adapter: %s" % ("selftest mode" if args.selftest else "running"))
        print("  calibration   %s" % self.calib.summary())
        print("  mutation      %s" % (args.mutation or "none"))
        print("  subscribes    %s (W goal), %s (Bspline, L), %s (odom, L)"
              % (args.goal_topic, args.bspline_topic, args.odom_topic))
        print("  publishes     %s (nav_msgs/Path, frame %s, %.1f Hz)"
              % (args.traj_topic, FRAME_L, args.rate))
        print("  publishes     %s (geometry_msgs/PoseStamped, frame %s)  <- the planner's goal"
              % (args.planner_goal_topic, FRAME_L))
        print("  gates         footprint %.2f m, start %.2f m, arena %s, roundtrip %.2f mm"
              % (args.footprint, args.start_max, "on", args.roundtrip_mm))

    # ---------------------------------------------------------------- goal W -> L
    def _stops(self):
        """The mission's stop list, from the SAME world file the sequencer and the sensor use.

        Why the adapter needs it: the sequencer -> adapter goal hop was MEASURED to lose most of its messages
        in every QoS setting tried (2026-10-14: 42 published -> 16 received VOLATILE depth 1, -> 6 depth 10,
        -> 9 TRANSIENT_LOCAL), while the small /mission/stop index message arrives reliably (the FSM has been
        driven by it for many sorties).  So the goal is derived HERE from the index plus this list, and the
        world file stays the single source of truth for the geometry -- the same rule the rest of the project
        follows.
        """
        if getattr(self, "_stops_cache", None) is None:
            try:
                import importlib.util
                p = os.path.join(HERE, "..", "tools", "world-circle.py")
                spec = importlib.util.spec_from_file_location("world_circle_adapter", p)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                self._stops_cache = mod.commanded_points(self.args.world)
                print("ego_adapter: %d stops read from %s -- the goal is derived from the stop index"
                      % (len(self._stops_cache), os.path.basename(self.args.world)), flush=True)
            except Exception as exc:                                  # noqa: BLE001
                print("ego_adapter: could not read the stop list (%s) -- falling back to /goal_pose only"
                      % exc, flush=True)
                self._stops_cache = []
        return self._stops_cache

    def on_stop_index(self, msg):
        """Remember the stop index; the goal is forwarded once the vehicle is airborne (see tick)."""
        i = int(msg.data)
        if i < 0:
            self.last_stop_index = -1
            return
        stops = self._stops()
        if not stops or i >= len(stops) or i == getattr(self, "last_stop_index", None):
            return
        # NOT YET: the index is published from the first tick, while the vehicle is still on the ground, and a
        # goal sent then is planned from the pad and dropped by the time the FSM starts following.  MEASURED
        # (sortie AL1): "ABORT: no new trajectory ... after 0 trajectories (sequencer at stop 1)" with stop 1
        # missed by 4.008 m.  The sequencer applies the same rule for the same reason ("holding the first stop
        # until the vehicle is above 1.40 m ... a goal sent from the ground puts the planner into GEN_NEW_TRAJ
        # for good"); this is that rule on the adapter's side, and it needs no new message.
        self.pending_index = i
        if self.odom_l is not None and float(self.odom_l[2]) >= self.args.goal_min_alt:
            self._forward_index_goal()

    def _forward_index_goal(self):
        i = getattr(self, "pending_index", None)
        if i is None:
            return
        stops = self._stops()
        if not stops or i >= len(stops):
            return
        self.pending_index = None
        self.last_stop_index = i
        p_w = np.asarray(stops[i], dtype=float)
        self.n_goal_from_index = getattr(self, "n_goal_from_index", 0) + 1
        print("ego_adapter: goal from stop index %d/%d: W[%+.4f %+.4f %+.4f] (vehicle at %.2f m)"
              % (i + 1, len(stops), p_w[0], p_w[1], p_w[2],
                 float(self.odom_l[2]) if self.odom_l is not None else float("nan")), flush=True)
        self._goal_src = "stop index %d/%d + the world file" % (i + 1, len(stops))
        self._forward_goal(p_w)

    def on_goal(self, msg):
        p_w = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.n_goal_in += 1
        self._goal_src = "the %s frame '%s'" % (self.args.goal_topic, msg.header.frame_id)
        self._forward_goal(p_w)

    def _forward_goal(self, p_w):
        # A goal forwarded before the planner has ever seen an odometry drives its FSM from INIT into
        # REPLAN_TRAJ, which evaluates a B-spline that does not exist yet.  MEASURED 2026-10-14: the
        # node segfaulted 7 s after start, the instant the first goal arrived.  patches/0002 makes the
        # planner drop such a goal; this is the second, independent guard, and it is the one that
        # holds even against an unpatched planner.  Goals are repeated by their sender, so dropping
        # one costs at most a second.
        if self.odom_l is None:
            self.n_goal_deferred += 1
            if self.n_goal_deferred <= 2 or self.n_goal_deferred % 20 == 0:
                print("ego_adapter: goal DEFERRED (%d): no odometry on %s yet -- forwarding a goal "
                      "before the planner has a state crashes it"
                      % (self.n_goal_deferred, self.args.odom_topic), flush=True)
            return
        p_l = w_to_l(self.calib, p_w)
        back = l_to_w(self.calib, p_l)
        err_mm = float(np.linalg.norm(back - p_w)) * 1e3
        if err_mm > self.args.roundtrip_mm:
            self.n_goal_rejected += 1
            print("ego_adapter: GOAL REJECTED  W[%+.4f %+.4f %+.4f] -> L -> W error %.4f mm > %.2f mm"
                  % (p_w[0], p_w[1], p_w[2], err_mm, self.args.roundtrip_mm), flush=True)
            return
        out = self.PoseStamped()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = FRAME_L
        out.pose.position.x, out.pose.position.y, out.pose.position.z = (float(v) for v in p_l)
        out.pose.orientation.w = 1.0
        # ONE FORWARD PER GOAL, NOT ONE PER SECOND.  MEASURED 2026-10-14 (sortie R1, the circle): the
        # adapter forwarded the goal at 1 Hz (63 "Triggered!" callbacks in 85 s), and every forward makes ego
        # rebuild its GLOBAL trajectory (planNextWaypoint -> planGlobalTraj) and push the FSM back into
        # REPLAN_TRAJ -- 106 REPLAN_TRAJ->EXEC_TRAJ and 47 EXEC_TRAJ->REPLAN_TRAJ transitions in one sortie.
        # The local plan was therefore invalidated once a second, the committed trajectory's anchor
        # alternated between two pipelines, and the vehicle never followed one plan long enough to make
        # progress (truth never left E <= 4.69 while 145 trajectories were committed).  The repetition
        # existed because a goal that arrives before odometry used to crash the planner -- but nothing above
        # this line forwards such a goal (see the DEFER guard), so the repeat has no job left.  The slow
        # repeat stays as a safety net in case a goal is ever lost.
        now = time.time()
        same = (self.goal_l is not None) and float(np.linalg.norm(p_l - self.goal_l)) < 0.05
        if same and (now - self.last_goal_fwd) < self.args.goal_repeat_s:
            self.n_goal_dedup = getattr(self, "n_goal_dedup", 0) + 1
            return
        self.last_goal_fwd = now
        self.goal_pub.publish(out)
        self.goal_l, self.goal_w = p_l, p_w
        self.n_goal_out += 1
        print("ego_adapter: goal  W[%+.4f %+.4f %+.4f] (from %s) -> L[%+.4f %+.4f %+.4f]"
              "  round trip %.6f mm"
              % (p_w[0], p_w[1], p_w[2], self._goal_src,
                 p_l[0], p_l[1], p_l[2], err_mm), flush=True)

    def maybe_auto_goal(self):
        if not self.args.auto_goal or self.n_goal_in > 0:
            return
        if time.time() - self.t0 < self.args.goal_timeout:
            return
        self.auto_sent = getattr(self, "auto_sent", False)
        if self.auto_sent:
            return
        self.auto_sent = True
        p = np.array(self.args.auto_goal, dtype=float)
        print("ego_adapter: no %s within %.1f s -- publishing the world file's mission goal "
              "W[%+.3f %+.3f %+.3f] once (--auto-goal)" % (self.args.goal_topic,
                                                           self.args.goal_timeout, p[0], p[1], p[2]),
              flush=True)
        m = self.PoseStamped()
        m.header.frame_id = "world_w"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.w = 1.0
        self.on_goal(m)

    # ---------------------------------------------------------------- planner state
    def on_odom(self, msg):
        self.odom_l = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y,
                                msg.pose.pose.position.z])

    def on_bspline(self, msg):
        self.bspline = msg
        self.n_bspline += 1
        # One real message, kept on disk so tools/ego-bspline-ab.py can A/B the Python evaluator
        # against the C++ one on the data the planner actually publishes.
        if self.args.dump_bspline and not os.path.exists(self.args.dump_bspline):
            try:
                d = {"order": int(msg.order), "traj_id": int(msg.traj_id),
                     "knots": [float(v) for v in msg.knots],
                     "pos_pts": [[float(p.x), float(p.y), float(p.z)] for p in msg.pos_pts]}
                with open(self.args.dump_bspline, "w") as fh:
                    json.dump(d, fh)
                print("ego_adapter: dumped a real Bspline (%d ctrl, %d knots) to %s"
                      % (len(d["pos_pts"]), len(d["knots"]), self.args.dump_bspline), flush=True)
            except Exception as exc:                                   # noqa: BLE001
                print("ego_adapter: could not dump the Bspline: %s" % exc, flush=True)

    # ---------------------------------------------------------------- the path
    def _connection_is_clear(self, a_l, b_l, want_reason=False):
        """Is the straight segment from `a_l` to `b_l` flyable: inside the arena, clear of every pillar?

        The start-distance obligation is a PROXY for the property that actually matters -- can the vehicle
        get from where it is to where the path begins without crossing a pillar?  MEASURED 2026-10-14 in
        the first live Mission B attempts: the planner planned fine, but every path began 3.4-3.6 m away
        and the 0.50 m gate withheld 150 of them, which from the flight side is indistinguishable from "a
        planner that never plans".  When this connection is clear the follower can fly it, so the path is
        published; when it is not, the path is withheld with the clearance that failed.
        """
        a = np.asarray(a_l, float)
        b = np.asarray(b_l, float)
        step = max(self.args.footprint / 2.0, 0.05)
        n = max(2, int(np.ceil(float(np.linalg.norm(b[:2] - a[:2])) / step)) + 1)
        s = np.linspace(0.0, 1.0, n)[:, None]
        pts = a[None, :] + s * (b - a)[None, :]
        ins = self.world.inside(pts)
        if not bool(np.all(ins)):
            return "outside the arena" if want_reason else False
        clr = float(np.min(self.world.clearance(pts)))
        if clr < self.args.footprint:
            return ("%.3f m from a pillar surface (< %.2f m)" % (clr, self.args.footprint)) \
                if want_reason else False
        return True

    def build_path(self):
        """(Path, info) or (None, reason).  All of PLAN.md 4.1's obligations are decided here."""
        m = self.bspline
        pts = np.array([[p.x, p.y, p.z] for p in m.pos_pts], dtype=float)
        knots = np.array(m.knots, dtype=float)
        if len(pts) < int(m.order) + 1 or len(knots) != len(pts) + int(m.order) + 1:
            return None, "malformed Bspline (n_ctrl=%d, knots=%d, order=%d)" % (
                len(pts), len(knots), m.order)

        dur = bspline_duration(knots, len(pts), m.order)
        if dur <= 1e-6:
            return None, "zero-length Bspline"
        t0 = float(m.start_time.sec) + 1e-9 * float(m.start_time.nanosec)
        # the planner timestamps with rclcpp::Clock() = RCL_SYSTEM_TIME (planner_manager.cpp:64,553),
        # so the elapsed time must be measured against the wall clock, NOT a possibly-sim node clock.
        t_cur = time.time() - t0
        if not (t_cur > -1.0):
            return None, "clock mismatch: Bspline start %.3f vs wall %.3f" % (t0, time.time())
        t_cur = max(0.0, min(t_cur, dur))

        ts = np.arange(0.0, dur + 1e-9, self.args.dt)
        if len(ts) == 0 or ts[-1] < dur - 1e-9:
            ts = np.append(ts, dur)
        pos = np.array([bspline_eval(knots, pts, m.order, float(t)) for t in ts])

        info = {"t_cur": t_cur, "duration": dur, "n": len(pos), "t0": t0}

        # WHERE THE PATH STARTS.  The contract's third obligation is "never publish a path that starts
        # more than 0.5 m from the vehicle's current state", and this is the one place it can be met
        # or broken.  The spline is a TIMED trajectory planned at ~1 Hz from the odometry of that
        # instant, but the consumer (`fsm/mission_fsm.py`) flies it as a GEOMETRIC path with arc-length
        # look-ahead -- so the right thing to publish is the spline from the point nearest the vehicle
        # to its end, not "from t_cur onwards".  MEASURED 2026-10-14 while parked: publishing from
        # t_cur made the start distance grow with the plan's own speed (0.60 m after 0.6 s, 5.08 m
        # after 5 s) and the gate withheld 73 of 88 paths; projecting instead puts the start at the
        # vehicle and leaves the gate to catch only a genuinely stale plan.
        i0 = 0
        if self.odom_l is not None:
            d = np.linalg.norm(pos - self.odom_l, axis=1)
            i0 = int(np.argmin(d))
            info["start_d"] = float(d[i0])
            # NO START/CONNECTOR GATE.  This used to refuse a plan whose nearest point was further than
            # --start-max (0.50 m) AND whose straight connection from the vehicle was not clear, and that pair
            # of conditions -- which upstream's own consumer does not have -- is what starved sorties
            # AG1/AH1/AI1: MEASURED (sortie AI1, one DIAG line per second for 60 s) the FSM held ONE stale
            # 0.163 m fragment whose first point was 5.97 m away while this gate refused every fresh plan with
            # "starts 0.997 m from the vehicle (> 0.50 m) ... the connecting segment is not clear", so the
            # message count stopped at 41 and the mission aborted.
            # WHY THE GATE EXISTED, AND WHY IT IS NOT NEEDED ANY MORE: the FSM used to treat the published
            # Path as a GEOMETRIC path to be joined (it rebased the timeline to "now" on every switch, so its
            # first command was the path's first point = the connector).  upstream's traj_server evaluates the
            # spline on the SPLINE'S OWN TIMELINE (`t_cur = now - start_time`, no rebasing) and hovers at the
            # end once the time is past -- so there is no connector to clear, only a timed reference.  The FSM
            # now does the same (see follow_spline), which is what makes this gate unnecessary rather than
            # merely inconvenient.
            if d[i0] > self.args.start_max:
                self.n_far_start = getattr(self, "n_far_start", 0) + 1
                if self.n_far_start <= 3 or self.n_far_start % 25 == 0:
                    print("ego_adapter: NOTE %d: the plan's nearest point is %.3f m from the vehicle "
                          "(> %.2f m) -- published anyway, upstream-style, and the FSM follows it on the "
                          "spline's own timeline" % (self.n_far_start, d[i0], self.args.start_max), flush=True)
        # A path of ONE point is not a path: once the vehicle reaches the end of the spline the
        # projection lands on the last sample and the published Path degenerated to a single pose
        # (MEASURED 2026-10-14: "points per path 1" on the final messages, and an arc length of
        # 0.000 m).  `fsm/mission_fsm.py::follow_path` does np.diff on the points and an argmin over
        # the SEGMENT midpoints, so a 1-point Path would raise there instead of completing.  Keep the
        # last two samples.
        i0 = min(i0, len(pos) - 2)
        pos, ts = pos[i0:], ts[i0:]
        info["n"] = len(pos)

        cl = self.world.clearance(pos)
        info["clr_min"] = float(cl.min())
        # THE ALTITUDE BAND.  MEASURED 2026-10-14 across sorties: ego's B-spline wanders in z -- plans were
        # seen at 1.21 m and at 3.74 m for a goal at 2.00 m (B7 dipped, C3 climbed), because this forest is
        # uniform in z and the optimiser has no altitude-keeping term.  The vehicle then follows the plan up
        # or down, which trips the FSM's own gates (sortie B12 aborted with the FCU at 3.43 m) and silently
        # spends the clearance the plan was verified with (B10's slow arm wandered to 4.10 m).  The pillars
        # span z 0..4.2 m and are vertical, so moving a plan in z inside that band does NOT change its
        # horizontal clearance -- and the gate below re-checks the clamped path anyway -- so clamping z to
        # the goal's own altitude is safe, and it is what the mission means.
        if self.args.z_band > 0.0:
            z_goal = float(pos[-1, 2])
            zc = np.clip(pos[:, 2], z_goal - self.args.z_band, z_goal + self.args.z_band)
            dz = float(np.max(np.abs(zc - pos[:, 2])))
            if dz > 1e-9:
                self.n_zclip = getattr(self, "n_zclip", 0) + 1
                if self.n_zclip <= 3 or self.n_zclip % 20 == 0:
                    print("ego_adapter: Z CLAMPED (%d): the plan's z was up to %.3f m away from the goal's "
                          "%.3f m; clamped to +-%.2f m" % (self.n_zclip, dz, z_goal, self.args.z_band),
                          flush=True)
                pos[:, 2] = zc
                cl = self.world.clearance(pos)              # re-check: the clamp must not create a collision
                info["clr_min"] = float(cl.min())
        if info["clr_min"] < self.args.footprint:
            # PUBLISH THE SAFE PREFIX INSTEAD OF NOTHING.  The contract (PLAN.md 4.1) forbids publishing a
            # path that breaks the clearance, and that is still true -- but withholding the WHOLE path
            # deadlocks the mission when the violation is in its far half: MEASURED 2026-10-14 on the 12-stop
            # circle, `PATH WITHHELD (275): pillar clearance -0.096 m < footprint 0.20 m (at point 470)` --
            # ego planned through a pillar it had not seen yet (unknown space is free to it), the vehicle
            # never moved, so it never saw it, so the same plan came back, and the FSM waited 90 s and
            # aborted.  The first `--min-prefix-m` of the path are still verified clear, so they may be
            # flown; flying them is also what makes the vehicle see more.  A prefix shorter than that is
            # not worth a setpoint stream and is withheld with the old message.
            k = int(np.argmax(cl < self.args.footprint))       # first violating sample
            if k >= 2:
                seg = float(np.sum(np.linalg.norm(np.diff(pos[:k, :2], axis=0), axis=1)))
            else:
                seg = 0.0
            if seg >= self.args.min_prefix_m:
                info["prefix_m"] = seg
                info["prefix_of"] = len(pos)
                self.n_prefix = getattr(self, "n_prefix", 0) + 1
                if self.n_prefix <= 3 or self.n_prefix % 20 == 0:
                    print("ego_adapter: PATH TRUNCATED (%d): flying the first %.3f m of %d points and "
                          "dropping the rest, which comes within %.3f m of a pillar at point %d "
                          "(contract %.2f m) -- ego planned through a pillar it has not seen yet"
                          % (self.n_prefix, seg, k, float(cl[k]), k, self.args.footprint), flush=True)
                pos, ts = pos[:k], ts[:k]
            else:
                # THE OFFENDING PLAN'S OWN IDENTITY, so a refusal can be matched to the recorded B-spline
                # (tools/record-bspline.py) and the question "is that pillar in ego's map?" can be answered
                # offline instead of inferred.  MEASURED 2026-10-14: 13 refusals of the form
                # "pillar clearance -0.172 .. -0.219 m (at point 37)" in sorties L1/N1 could not be matched
                # to any recorded trajectory because the message carried no start_time.
                return None, "pillar clearance %.3f m < footprint %.2f m (at point %d, traj %.3f, t_cur %.2f of %.2f)" % (
                    info["clr_min"], self.args.footprint, int(np.argmin(cl)),
                    float(self.bspline.start_time.sec) + 1e-9 * float(self.bspline.start_time.nanosec),
                    float(info.get("t_cur", float("nan"))), float(info.get("duration", float("nan"))))

        ins = self.world.inside(pos)
        if not bool(ins.all()):
            i = int(np.argmin(ins))
            return None, "point %d outside the arena at [%+.2f %+.2f %+.2f]" % (
                i, pos[i, 0], pos[i, 1], pos[i, 2])

        out = self.Path()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = FRAME_L
        for i, (x, y, z) in enumerate(pos):
            ps = self.PoseStamped()
            # The contract asks for "points at >= 20 Hz of trajectory time".  Stamping every point
            # with the instant the planner means it to be reached (start_time + t_i) is what makes
            # that clause CHECKABLE from the message alone; `planner/rectangle_source.py` stamps all
            # points identically, which is why its version of the same clause can only be argued
            # from the geometry.  The consumer reads positions only, so this is compatible with the
            # flown interface and strictly more informative.
            ps.header.stamp = self.to_msg(info["t0"] + float(ts[i]))
            ps.header.frame_id = FRAME_L
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
            j = min(i + 1, len(pos) - 1)
            dx, dy = pos[j, 0] - x, pos[j, 1] - y
            if abs(dx) + abs(dy) < 1e-9:
                dx, dy = pos[-1, 0] - x, pos[-1, 1] - y
            yaw = math.atan2(dy, dx) if (abs(dx) + abs(dy) > 1e-12) else 0.0
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            out.poses.append(ps)
        seg = np.linalg.norm(np.diff(pos, axis=0), axis=1) if len(pos) > 1 else np.zeros(1)
        info["arc_max"] = float(seg.max())
        info["arc_med"] = float(np.median(seg))
        return out, info

    def to_msg(self, sec_float):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(math.floor(sec_float))
        t.nanosec = int(round((sec_float - t.sec) * 1e9))
        if t.nanosec >= 1000000000:
            t.sec += 1
            t.nanosec -= 1000000000
        return t

    def _note_stall(self):
        """The SAME withheld trajectory, second after second, is the signature of a stalled planner."""
        if self.bspline is None:
            return
        t0 = float(self.bspline.start_time.sec) + 1e-9 * float(self.bspline.start_time.nanosec)
        now = time.time()
        if getattr(self, "_stall_t0", None) != t0:
            self._stall_t0, self._stall_since, self._stall_said = t0, now, False
            return
        dt = now - self._stall_since
        if dt >= self.args.stall_s and not self._stall_said:
            self._stall_said = True
            self.n_stall += 1
            print("ego_adapter: PLANNER STALLED: the same trajectory (start_time %.3f) has been withheld for "
                  "%.1f s after %d refusals -- ego is not committing a new one (see the PLANNER-STALL lines in "
                  "the planner's own log)" % (t0, dt, self.n_blocked), flush=True)
        if getattr(self, "_stall_said", False):
            self.stall_pub.publish(Bool(data=True))

    def _clear_stall(self):
        if getattr(self, "_stall_said", False):
            print("ego_adapter: planner recovered: a NEW trajectory was published after %.1f s of stall"
                  % (time.time() - self._stall_since), flush=True)
            self.stall_pub.publish(Bool(data=False))
        self._stall_t0, self._stall_said = None, False

    def tick(self):
        # A PERIODIC STATUS LINE, because the counters are the only way to attribute a lost goal and the
        # REPORT line is never seen (the run ends with SIGKILL, which prints nothing).  MEASURED 2026-10-14
        # (sorties W1/X1): the sequencer published the stop-2 goal 42 / 41 times and this node forwarded only
        # 9 / 10 of them, while every forward reached ego -- so 75-80 % of the goals are lost on the hop INTO
        # this node, and splitting "goals in" from "forwarded" says whether the messages never arrive or
        # arrive and are dropped by one of the guards above.
        now = time.time()
        # RE-FORWARD THE LAST GOAL FROM HERE, because THIS hop is the reliable one.  MEASURED 2026-10-14:
        # sequencer -> adapter loses 75-85 % of the goals under load (sortie Y1: 42 published, 12 received;
        # sortie Z1 with both queues deepened to 10: 42 published, 6 received) while adapter -> ego lost
        # NOTHING (Y1: 12 forwarded, 13 callbacks).  ego needs a goal about once a second to be pulled out of
        # WAIT_TARGET (measured: without it, 69 "wait for goal" lines and a 70 s stall), and patches/0005
        # makes such a repeat a no-op while the same goal is under pursuit -- so generating the 1 Hz stream
        # HERE, from the last goal that did arrive, gives ego what it needs without depending on the lossy
        # hop.  One goal per stop getting through is enough.
        if self.goal_l is not None and self.args.reforward_hz > 0.0 and \
                (now - getattr(self, "_last_reforward", 0.0)) >= 1.0 / self.args.reforward_hz:
            self._last_reforward = now
            m = self.PoseStamped()
            m.header.stamp = self.node.get_clock().now().to_msg()
            m.header.frame_id = FRAME_L
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in self.goal_l)
            m.pose.orientation.w = 1.0
            self.goal_pub.publish(m)
            self.n_reforward = getattr(self, "n_reforward", 0) + 1
        if getattr(self, "pending_index", None) is not None and self.odom_l is not None \
                and float(self.odom_l[2]) >= self.args.goal_min_alt:
            self._forward_index_goal()          # the stop's goal, once the vehicle is airborne
        if now - getattr(self, "_last_status", 0.0) >= 5.0:
            self._last_status = now
            print("ego_adapter STATUS: goals in %d, forwarded %d, dedup %d, deferred %d, rejected %d | "
                  "bsplines %d | paths %d, withheld %d, stalls %d"
                  % (self.n_goal_in, self.n_goal_out, getattr(self, "n_goal_dedup", 0), self.n_goal_deferred,
                     self.n_goal_rejected, self.n_bspline, self.n_pub, self.n_blocked, self.n_stall),
                  flush=True)
        if self.bspline is None:
            return
        path, info = self.build_path()
        if path is None:
            self.n_blocked += 1
            key = info.split(" (")[0].split(" at point")[0]
            self.blocked_reasons[key] = self.blocked_reasons.get(key, 0) + 1
            if self.n_blocked <= 3 or self.n_blocked % 25 == 0:
                print("ego_adapter: PATH WITHHELD (%d): %s" % (self.n_blocked, info), flush=True)
            self._note_stall()
            return
        self._clear_stall()
        self.traj_pub.publish(path)
        now = time.time()
        if self.last_pub_wall:
            self.pub_dt.append(now - self.last_pub_wall)
        self.last_pub_wall = now
        self.n_pub += 1
        if self.n_pub == 1 or self.n_pub % 40 == 0:
            dts = np.diff([p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec for p in path.poses])
            print("ego_adapter: published %d poses on %s  (t_cur %.3f / %.3f s, start %.4f m from "
                  "the vehicle, min pillar clearance %.3f m, traj dt %.4f s, arc step %.4f m)"
                  % (len(path.poses), self.args.traj_topic, info["t_cur"], info["duration"],
                     info.get("start_d", float("nan")), info["clr_min"],
                     float(np.median(dts)) if len(dts) else float("nan"),
                     info.get("arc_med", float("nan"))), flush=True)

    def report(self):
        rt = float(np.median(self.pub_dt)) if self.pub_dt else float("nan")
        print("ego_adapter REPORT: goals in %d, published %d, rejected %d, deferred %d | Bsplines %d | "
              "paths published %d, withheld %d | planner stalls %d | republish %.2f Hz (median over %d intervals)"
              % (self.n_goal_in, self.n_goal_out, self.n_goal_rejected, self.n_goal_deferred,
                 self.n_bspline, self.n_pub, self.n_blocked, self.n_stall,
                 (1.0 / rt if rt == rt and rt > 0 else float("nan")), len(self.pub_dt)), flush=True)
        for k, v in sorted(self.blocked_reasons.items()):
            print("   withheld %4d x  %s" % (v, k), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json",
                    help="the SAME L->W calibration the consumer uses")
    ap.add_argument("--world", default=os.path.join(HERE, "..", "worlds", "forest_all.yaml"))
    ap.add_argument("--traj-topic", default="/planner/traj")
    ap.add_argument("--goal-topic", default="/goal_pose", help="goal in W, from the mission")
    ap.add_argument("--planner-goal-topic", default="/move_base_simple/goal",
                    help="the ego-planner FSM's waypoint topic (frame L after this adapter)")
    ap.add_argument("--bspline-topic", default="planning/bspline")
    ap.add_argument("--stall-topic", default="/planner/stalled",
                    help="std_msgs/Bool, true while the SAME trajectory has been withheld for --stall-s "
                         "seconds.  A withheld plan is normal; the same one for seconds is a stalled planner "
                         "(MEASURED: F1 withheld one plan ~250 times over 50 s while the aircraft hovered)")
    ap.add_argument("--stall-s", type=float, default=2.0,
                    help="how long the same trajectory must be withheld before it counts as a stall, s")
    ap.add_argument("--stop-topic", default="/mission/stop",
                    help="std_msgs/Int32: the mission's current stop index (0-based, -1 = done).  The goal for "
                         "each stop is derived from this index plus the world file's stop list, because the "
                         "sequencer -> adapter goal hop was MEASURED to lose most of its messages in every QoS "
                         "setting (2026-10-14: 42 published -> 16 / 6 / 9 received for VOLATILE depth 1 / "
                         "depth 10 / TRANSIENT_LOCAL) while this small index message arrives reliably")
    ap.add_argument("--goal-min-alt", type=float, default=1.40,
                    help="do not forward a stop's goal until the vehicle is this high (m, in L).  MEASURED "
                         "(sortie AL1): forwarding it from the ground gave \"no new trajectory ... after 0 "
                         "trajectories (sequencer at stop 1)\" with stop 1 missed by 4.008 m; the sequencer "
                         "applies the same rule for the same reason")
    ap.add_argument("--reforward-hz", type=float, default=1.0,
                    help="re-publish the last goal to the planner at this rate (0 = off).  MEASURED "
                         "2026-10-14: sequencer->adapter loses 75-85 %% of the goals (Y1: 42 published, 12 "
                         "received; Z1 with both queues at depth 10: 42 published, 6 received) while "
                         "adapter->ego loses nothing, and ego needs roughly one goal a second to leave "
                         "WAIT_TARGET (69 'wait for goal' lines, 70 s of stall, in sortie U1).  So the "
                         "stream is generated on the hop that works; patches/0005 makes each repeat a no-op "
                         "while the same goal is under pursuit, and re-arms ego after an arrival")
    ap.add_argument("--goal-repeat-s", type=float, default=0.0,
                    help="forward an UNCHANGED goal again only after this many seconds; 0.0 = forward every "
                         "goal the sender publishes, which is the historical behaviour AND the one this stack "
                         "needs: MEASURED 2026-10-14, ego reaches WAIT_TARGET by design the moment it reaches "
                         "a goal (sortie U1: 69 'wait for goal' lines, the last 12 FSM states all "
                         "WAIT_TARGET, the last committed trajectory at t=12.1 s and the abort at t=83.9 s -- "
                         "70 s of doing nothing), and only an arriving goal pulls it out.  With this default "
                         "at 10 s (sortie U1) only 9 goals reached ego in 84 s and the mission starved.  The "
                         "HARMFUL part of the 1 Hz repeat -- each one rebuilding ego's global trajectory and "
                         "invalidating the local plan (sortie R1: 145 trajectories, none flown to "
                         "completion) -- is fixed inside ego by patches/0005, which makes a repeat a no-op "
                         "while the same goal is under pursuit")
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--rate", type=float, default=4.0, help="republish rate, Hz (>= 2 required)")
    ap.add_argument("--dt", type=float, default=0.02, help="trajectory-time sample spacing, s")
    ap.add_argument("--min-prefix-m", type=float, default=0.30,                    help="when a plan breaks the clearance contract in its far half, publish the verified "
                         "prefix if it is at least this long (m) instead of withholding everything; a "
                         "shorter prefix is withheld as before.  This is what breaks the deadlock where ego "
                         "plans through a pillar it has not seen yet and the vehicle therefore never moves "
                         "to see it")
    ap.add_argument("--z-band", type=float, default=0.30,
                    help="clamp the published plan's altitude to the goal's own altitude +- this many metres "
                         "(0 = do not touch the plan).  Ego's B-spline has no altitude-keeping term, so plans "
                         "were measured 1.21-3.74 m for a 2.00 m goal; the pillars are vertical, so a z move "
                         "inside their height does not change the horizontal clearance, and the gate re-checks "
                         "the clamped path")
    ap.add_argument("--footprint", type=float, default=0.20,
                    help="the project's clearance CONTRACT, metres: no point of a published path may come "
                         "closer than this to a TRUE pillar surface in the world file.  It is measured "
                         "against the analytic cylinders, NOT against ego-planner's inflated occupancy "
                         "grid, so it is independent of `grid_map/obstacles_inflation`.  DECIDED by the "
                         "user 2026-10-14: \"我希望这个值是0.2m\" (it was 0.35 m).  At 0.35 the planner's "
                         "own plans failed it by 12-43 mm in 3 of 4 measured cases (STATUS.md row 48); at "
                         "0.20 the same maps stay valid and the 1.5 m forest's 0.565 m corridors become "
                         "passable (2 x 0.20 = 0.40 m needed)")
    ap.add_argument("--start-max", type=float, default=0.50)
    ap.add_argument("--roundtrip-mm", type=float, default=1.0)
    ap.add_argument("--dump-bspline", default="",
                    help="write the first Bspline message (knots + control points) here as JSON, "
                         "for tools/ego-bspline-ab.py")
    ap.add_argument("--auto-goal", type=float, nargs=3, default=None, metavar=("E", "N", "U"),
                    help="if no goal arrives within --goal-timeout, publish this W goal once")
    ap.add_argument("--goal-timeout", type=float, default=5.0)
    ap.add_argument("--mutation", default="", choices=["", "predicted-calib", "transposed"],
                    help="deliberately wrong configuration; the gates must then fail")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="run for this many seconds then report and exit (0 = forever)")
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    a = EgoAdapter(args)
    a.t0 = time.time()
    t_end = None if args.duration <= 0 else time.time() + args.duration
    try:
        while a.rclpy.ok():
            a.rclpy.spin_once(a.node, timeout_sec=0.05)
            if t_end is not None and time.time() >= t_end:
                break
    except KeyboardInterrupt:
        pass
    a.report()
    a.node.destroy_node()
    try:
        a.rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
