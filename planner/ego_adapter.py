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
  2. >= `--footprint` (0.35 m) clearance from every pillar in the world file
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

        self.node.create_subscription(PoseStamped, args.goal_topic, self.on_goal, 1)
        self.node.create_subscription(Odometry, args.odom_topic, self.on_odom, sensor)

        self.bspline = None
        self.odom_l = None
        self.goal_l = None
        self.goal_w = None
        self.n_goal_in = 0
        self.n_goal_out = 0
        self.n_goal_rejected = 0
        self.n_bspline = 0
        self.n_pub = 0
        self.n_blocked = 0
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
    def on_goal(self, msg):
        p_w = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.n_goal_in += 1
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
        self.goal_pub.publish(out)
        self.goal_l, self.goal_w = p_l, p_w
        self.n_goal_out += 1
        print("ego_adapter: goal  W[%+.4f %+.4f %+.4f] (in frame '%s') -> L[%+.4f %+.4f %+.4f]"
              "  round trip %.6f mm"
              % (p_w[0], p_w[1], p_w[2], msg.header.frame_id,
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

    # ---------------------------------------------------------------- the path
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
        t_cur = max(0.0, min(t_cur, dur - self.args.dt))

        ts = np.arange(t_cur, dur + 1e-9, self.args.dt)
        if len(ts) < 2:
            ts = np.array([t_cur, dur])
        ts = np.minimum(ts, dur)
        pos = np.array([bspline_eval(knots, pts, m.order, float(t)) for t in ts])

        info = {"t_cur": t_cur, "duration": dur, "n": len(pos)}

        if self.odom_l is not None:
            d = float(np.linalg.norm(pos[0] - self.odom_l))
            info["start_d"] = d
            if d > self.args.start_max:
                return None, "starts %.3f m from the vehicle (> %.2f m)" % (d, self.args.start_max)

        cl = self.world.clearance(pos)
        info["clr_min"] = float(cl.min())
        if info["clr_min"] < self.args.footprint:
            return None, "pillar clearance %.3f m < footprint %.2f m (at point %d)" % (
                info["clr_min"], self.args.footprint, int(np.argmin(cl)))

        ins = self.world.inside(pos)
        if not bool(ins.all()):
            i = int(np.argmin(ins))
            return None, "point %d outside the arena at [%+.2f %+.2f %+.2f]" % (
                i, pos[i, 0], pos[i, 1], pos[i, 2])

        out = self.Path()
        stamp = self.node.get_clock().now().to_msg()
        out.header.stamp = stamp
        out.header.frame_id = FRAME_L
        for i, (x, y, z) in enumerate(pos):
            ps = self.PoseStamped()
            ps.header.stamp = stamp
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
        return out, info

    def tick(self):
        if self.bspline is None:
            return
        path, info = self.build_path()
        if path is None:
            self.n_blocked += 1
            key = info.split(" (")[0].split(" at point")[0]
            self.blocked_reasons[key] = self.blocked_reasons.get(key, 0) + 1
            if self.n_blocked <= 3 or self.n_blocked % 25 == 0:
                print("ego_adapter: PATH WITHHELD (%d): %s" % (self.n_blocked, info), flush=True)
            return
        self.traj_pub.publish(path)
        now = time.time()
        if self.last_pub_wall:
            self.pub_dt.append(now - self.last_pub_wall)
        self.last_pub_wall = now
        self.n_pub += 1
        if self.n_pub == 1 or self.n_pub % 40 == 0:
            dts = np.diff([p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec for p in path.poses])
            print("ego_adapter: published %d poses on %s  (t_cur %.3f / %.3f s, start %.4f m from "
                  "the vehicle, min pillar clearance %.3f m, sample dt %.4f s)"
                  % (len(path.poses), self.args.traj_topic, info["t_cur"], info["duration"],
                     info.get("start_d", float("nan")), info["clr_min"],
                     float(np.median(dts)) if len(dts) else float("nan")), flush=True)

    def report(self):
        rt = float(np.median(self.pub_dt)) if self.pub_dt else float("nan")
        print("ego_adapter REPORT: goals in %d, published %d, rejected %d | Bsplines %d | "
              "paths published %d, withheld %d | republish %.2f Hz (median over %d intervals)"
              % (self.n_goal_in, self.n_goal_out, self.n_goal_rejected, self.n_bspline,
                 self.n_pub, self.n_blocked, (1.0 / rt if rt == rt and rt > 0 else float("nan")),
                 len(self.pub_dt)), flush=True)
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
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--rate", type=float, default=4.0, help="republish rate, Hz (>= 2 required)")
    ap.add_argument("--dt", type=float, default=0.02, help="trajectory-time sample spacing, s")
    ap.add_argument("--footprint", type=float, default=0.35)
    ap.add_argument("--start-max", type=float, default=0.50)
    ap.add_argument("--roundtrip-mm", type=float, default=1.0)
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
