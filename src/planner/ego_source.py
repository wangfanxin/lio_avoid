#!/usr/bin/env python3
"""`planner/ego_source.py` -- the PLAN BOX for ego-planner, behind the SAME `/plan/cmd` contract.

WHAT THIS IS.  A planner in this stack is whatever publishes the contract in `docs/PLANNERS.md`:

    /plan/cmd   quadrotor_msgs/PositionCommand   frame lio_world (L)
                position = the point being flown to; velocity/acceleration are ignored on the wire
                trajectory_id   changes when the MISSION moves on (per waypoint, not per replan)
                trajectory_flag 1 while flying, 3 when the mission is over -- and the stream KEEPS TALKING

`planner/rectangle_source.py` produces that from a waypoint tour it flies itself.  ego-planner is a different
animal: it optimises a TIMED CUBIC B-SPLINE through a local occupancy map and replans continuously, and its
own `traj_server.cpp` cannot satisfy the contract on its own -- measured in THIS tree:

  * `traj_server.cpp:211` writes `trajectory_flag = TRAJECTORY_STATUS_READY` and NOTHING else in the whole
    tree ever writes another value, so ego can never say COMPLETED;
  * `traj_server.cpp:191-202` keeps publishing the endpoint past the end of the spline, so it never goes
    silent either -- both discriminators the supervisor has are absent;
  * `planner_manager.cpp:680` increments `traj_id_` on every REPLAN (~1 Hz inside one leg), not per waypoint;
  * `yaw_pts` is never filled (`traj_utils/msg/Bspline.msg:10-11`), so yaw has to be ours.

So this file is the missing half: it is the plan box that OWNS THE MISSION (which waypoint is being flown,
when it is reached, how long to hold, when the tour is over) and asks ego for one goal at a time, while
translating ego's B-spline into the contract.  The bridge and the supervisor do not change -- they still read
one topic.

DIRECTIONS
    /lio/robo/odom (L, 200 Hz)          -> this box's own state (arrival test, dwell, initial hold)
    plan_manage's goal topic (L)        <- ONE waypoint at a time, W->L with the sortie's calibration
    planning/bspline (L)                -> the trajectory to sample
    /plan/cmd (L, 100 Hz)               -> the contract above

THE HONEST LIMITS (docs/PLANNERS.md section 5): this adapter has NOT been flown on this stack since F1/F2.
Its sampling arithmetic is verified (`planner/bspline.py --selftest`), its contract behaviour is asserted by
`--selftest` here, and a full ego sortie would additionally need `ego_planner_node` AND `traj_server` built and
launched, plus ego's own map fed from `/lio/cloud_world`.

usage:
  planner/ego_source.py --selftest                     # no ROS, no flight: the contract, asserted
  planner/ego_source.py --world worlds/pillars.yaml --calib-file _stage/calib.json [--hold-s 5.0]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from planner.bspline import bspline_derivs, bspline_duration                     # noqa: E402
from planner.cmd_stream import (CmdSample, FRAME_L, TRAJECTORY_STATUS_COMPLETED,  # noqa: E402
                                TRAJECTORY_STATUS_READY, KX, KV)

DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"      # ego's own input, `advanced_param.launch.py:152`
DEFAULT_BSPLINE_TOPIC = "planning/bspline"         # published by ego_replan_fsm.cpp:799-824
DEFAULT_CMD_TOPIC = "/plan/cmd"                    # OUR contract


class Spline:
    """One received `Bspline` message, with the clock the caller will sample it on."""

    __slots__ = ("order", "knots", "pts", "traj_id", "start_wall", "duration", "received_wall")

    def __init__(self, order, knots, pts, traj_id, start_wall, received_wall):
        self.order = int(order)
        self.knots = np.asarray(knots, dtype=float)
        self.pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        self.traj_id = int(traj_id)
        self.start_wall = float(start_wall)
        self.received_wall = float(received_wall)
        self.duration = bspline_duration(self.knots, len(self.pts), self.order)

    def at(self, t):
        """(position, velocity, acceleration) at trajectory-relative time `t` (the evaluator clamps)."""
        return bspline_derivs(self.knots, self.pts, self.order, float(t))


class EgoPlanBox:
    """The mission owner: a waypoint tour, plus the translation of ego's output into /plan/cmd.

    NO ROS IN HERE.  Everything the box needs arrives through `on_spline`, `on_odom` and `sample`, so the
    contract behaviour can be asserted with no simulator, no ROS graph and no flight (`--selftest`).
    """

    def __init__(self, targets_l, hold_s=5.0, arrive_m=0.060, settle_mps=0.080, rate_hz=100.0,
                 start_wall=None, log=None, stale_s=2.0, yaw_mode="velocity", max_start_age_s=60.0):
        self.targets = np.asarray(targets_l, dtype=float).reshape(-1, 3)
        self.hold_s = float(hold_s)
        self.arrive_m = float(arrive_m)
        self.settle_mps = float(settle_mps)
        self.rate_hz = float(rate_hz)
        self.stale_s = float(stale_s)
        #: how far a spline's `start_time` may be from this clock before it is REFUSED as a different clock
        #: domain.  Generous, because the point is to catch epoch-vs-monotonic, not to police latency.
        self.max_start_age_s = float(max_start_age_s)
        self.n_refused_tb = 0
        self.yaw_mode = yaw_mode
        self.log = log or (lambda s: None)
        self.t0 = time.monotonic() if start_wall is None else float(start_wall)
        # ---- state
        self.i_goal = 0                    # the waypoint being flown to
        self.traj_id = 1                   # OUR id: per waypoint, never per replan
        self.flag = TRAJECTORY_STATUS_READY
        self.done = False
        self.t_goal_sent = None
        self.t_dwell = None
        self.p = None                      # the newest odometry position, in L
        self.v = None
        self._p_prev, self._t_prev = None, None
        self.v_est = None                  # pose-differentiated speed, used when no twist arrives
        self.spline = None
        self.n_splines = 0
        self.n_samples = 0
        self.goals_sent = []
        self.yaw = 0.0
        self.held = 0                      # samples published while no spline had arrived yet

    # ---- inputs ------------------------------------------------------------------------------------
    def on_odom(self, p_l, v_l=None, now=None):
        now = time.monotonic() if now is None else now
        p = np.asarray(p_l, dtype=float).reshape(3)
        if self._p_prev is not None and now > (self._t_prev or 0.0):
            self.v_est = float(np.linalg.norm(p - self._p_prev)) / (now - self._t_prev)
        self._p_prev, self._t_prev = p, now
        self.p = p
        self.v = None if v_l is None else np.asarray(v_l, dtype=float).reshape(3)
        self._sequence(now)

    def on_spline(self, order, knots, pos_pts, traj_id, start_wall, now=None):
        """One `Bspline` message.  `start_wall` is its `start_time` in the SAME clock as `now`.

        THE TIME BASE IS CHECKED, BECAUSE A WRONG ONE IS INVISIBLE.  `bspline_derivs` CLAMPS outside the
        spline's domain -- correct behaviour, and it is what turned this adapter's first real sortie into a
        400 s hang: ego's `start_time` is on the SYSTEM clock (epoch seconds, `ego_replan_fsm.cpp:845`) while
        the sampler ran on `time.monotonic()`, so `t = now - start_wall` was about -1.79e9, every sample
        clamped to the START of the trajectory, and the vehicle was commanded to where it already was.  The
        command looked like a plausible position, the stream looked healthy, and the plan never arrived.
        A time base that is not within a plausible age of `now` is REFUSED here and SAID OUT LOUD.
        """
        now = time.monotonic() if now is None else now
        age = now - float(start_wall)
        if not (-self.max_start_age_s <= age <= self.max_start_age_s):
            self.n_refused_tb += 1
            self.log("ego_source: REFUSING a spline whose start_time is %.3g s from this clock -- a "
                     "CLOCK-DOMAIN mismatch (epoch vs monotonic?), and sampling it would silently clamp to "
                     "the trajectory's START.  Convert the message's clock before calling on_spline."
                     % age)
            return False
        s = Spline(order, knots, pos_pts, traj_id, start_wall, now)
        if s.duration != s.duration or s.duration <= 0.0:
            self.log("ego_source: REFUSING a spline whose domain is %.3f s" % s.duration)
            return False
        self.spline = s
        self.n_splines += 1
        return True

    # ---- the mission -------------------------------------------------------------------------------
    @property
    def target(self):
        return self.targets[min(self.i_goal, len(self.targets) - 1)]

    def _sequence(self, now):
        """Arrival + dwell.  The goal is NOT advanced by ego's replans -- only by the vehicle arriving."""
        if self.done or self.p is None:
            return
        d = float(np.linalg.norm(self.p - self.target))
        # THE SETTLE NEEDS A SPEED FROM SOMEWHERE.  `v` is the twist when the topic that carries it is
        # wired; otherwise the pose is differentiated.  "Unknown" is NOT "slow": with neither, the arrival
        # waits, which is the safe direction (a missed arrival stalls the leg, an early one moves the box).
        if self.v is not None:
            speed = float(np.linalg.norm(self.v))
        elif self.v_est is not None:
            speed = float(self.v_est)
        else:
            speed = float("inf")
        slow = speed < self.settle_mps
        if self.t_dwell is None:
            if d <= self.arrive_m and slow:
                self.t_dwell = now
                self.log("ego_source: ARRIVED at waypoint %d (%.3f m), dwelling %.1f s"
                         % (self.i_goal + 1, d, self.hold_s))
            return
        if now - self.t_dwell >= self.hold_s:
            if self.i_goal >= len(self.targets) - 1:
                self.done = True
                self.flag = TRAJECTORY_STATUS_COMPLETED
                self.log("ego_source: TOUR COMPLETE -- the stream now carries trajectory_flag = COMPLETED "
                         "and KEEPS TALKING (it does not stop asking for more)")
                return
            self.i_goal += 1
            self.traj_id += 1                      # A NEW WAYPOINT: the id changes here and nowhere else
            self.t_dwell = None
            self.log("ego_source: next waypoint %d of %d" % (self.i_goal + 1, len(self.targets)))

    def goal_to_send(self):
        """The one goal ego should be working on, or None when the tour is over."""
        if self.done:
            return None
        return self.target

    # ---- the output --------------------------------------------------------------------------------
    def sample(self, now=None):
        """One `/plan/cmd` sample: the contract's fields, and nothing about ego's."""
        now = time.monotonic() if now is None else now
        self._sequence(now)
        p = None
        if self.spline is not None:
            age = now - self.spline.received_wall
            if age <= self.stale_s:
                t = now - self.spline.start_wall
                p, v, _a = self.spline.at(t)
                if self.yaw_mode == "velocity" and float(np.linalg.norm(v[:2])) > 0.05:
                    self.yaw = math.atan2(float(v[1]), float(v[0]))
            else:
                # A STALE SPLINE IS NOT A PLAN.  Hold the last commanded point rather than chase an old
                # trajectory; the stream stays alive (silence means failure to the supervisor) and the
                # supervisor's own gates get their chance.
                p = self.target if self.p is None else self.p
                self.held += 1
        elif self.p is not None:
            p = self.p                            # no spline yet: ask for "hold where you are"
            self.held += 1
        if p is None:
            return None
        self.n_samples += 1
        return CmdSample(p=np.asarray(p, dtype=float), v=[0.0, 0.0, 0.0], a=[0.0, 0.0, 0.0],
                         yaw=self.yaw, yaw_dot=0.0, trajectory_id=self.traj_id, trajectory_flag=self.flag,
                         frame_id=FRAME_L, kx=KX, kv=KV)

    def summary(self):
        return ("ego_source: %d waypoint(s), %d spline(s) received, %d samples published (%d held), "
                "trajectory_id %d, flag %d%s"
                % (len(self.targets), self.n_splines, self.n_samples, self.held, self.traj_id, self.flag,
                   ", TOUR COMPLETE" if self.done else ""))


# -------------------------------------------------------------------------------- the self-test
def _selftest() -> int:
    """The CONTRACT, asserted without ROS: what a consumer will see from this plan box.

    A straight-line spline is synthesised the way ego produces one (cubic, control points, knots starting
    negative) and fed in, together with a point-vehicle that walks the same line.  The assertions are the
    four in docs/PLANNERS.md section 1, in the order a consumer meets them.
    """
    print("EGO SOURCE SELFTEST -- the /plan/cmd contract, no ROS, no flight\n")
    t0 = 1000.0
    targets = [[0.6, 0.0, 0.5], [1.8, 0.0, 0.5]]
    box = EgoPlanBox(targets, hold_s=1.0, arrive_m=0.05, settle_mps=0.08, rate_hz=100.0,
                     start_wall=t0, log=lambda s: print("      " + s))
    bad = []

    def spline_for(a, b, start, seconds=2.0, interval=0.05):
        """A cubic B-spline whose domain [0, seconds] runs from a to b (uniform knots, ego's shape)."""
        n = max(4, int(round(seconds / interval)) + 3)
        ts = np.linspace(0.0, 1.0, n)
        pts = np.array([a + (b - a) * t for t in ts])
        order = 3
        knots = np.array([(i - order) * interval for i in range(n + order + 1)])
        return order, knots, pts

    # --- 1. before any spline: the stream is ALIVE and asks for "hold where you are"
    box.on_odom([0.0, 0.0, 0.5], [0.0, 0.0, 0.0], now=t0)
    s = box.sample(now=t0)
    ok = s is not None and s.frame_id == FRAME_L and s.trajectory_id == 1 \
        and s.trajectory_flag == TRAJECTORY_STATUS_READY
    print("   before any spline: sample %s, id %s, flag %s"
          % ("yes" if s else "NONE", s.trajectory_id if s else "-", s.trajectory_flag if s else "-"))
    if not ok:
        bad.append("a plan box that has not seen a spline yet must still publish the contract")
    if s is not None and not np.allclose(s.p, [0.0, 0.0, 0.5]):
        bad.append("the initial hold must be where the vehicle IS, not a guess")

    # --- 2. ego's spline, sampled: the positions must follow it
    order, knots, pts = spline_for(np.array([0.0, 0.0, 0.5]), np.array([0.6, 0.0, 0.5]), t0)
    box.on_spline(order, knots, pts, traj_id=7, start_wall=t0, now=t0)
    dur = bspline_duration(knots, len(pts), order)
    xs = []
    for dt in (0.0, 0.5 * dur, dur):
        box.on_spline(order, knots, pts, traj_id=7, start_wall=t0, now=t0)
        xs.append(float(box.sample(now=t0 + dt).p[0]))
    s2 = box.sample(now=t0 + dur)
    print("   with a spline (domain %.2f s): positions along it %s (ego's own traj_id was 7; ours is %s)"
          % (dur, np.round(xs, 3), s2.trajectory_id))

    # --- 2b. THE CLOCK DOMAIN.  This is the assertion whose absence cost a sortie: a spline stamped in
    #     ANOTHER clock must be REFUSED, not sampled.  Without the refusal the evaluator clamps to the
    #     trajectory's start and the vehicle is commanded to where it already is -- a plausible-looking
    #     position, a healthy-looking stream, and a plan that never arrives.
    print()
    print("   THE CLOCK DOMAIN (the defect that hung the first assembled ego sortie):")
    fresh = EgoPlanBox([[0.6, 0.0, 0.5]], hold_s=1.0, start_wall=t0, log=lambda s: None)
    epoch_like = 1.792e9                                   # what `builtin_interfaces/Time` really looks like
    accepted = fresh.on_spline(order, knots, pts, traj_id=1, start_wall=epoch_like, now=t0)
    print("      a spline stamped %.3g against a clock reading %.3g: accepted=%s, refused=%d"
          % (epoch_like, t0, accepted, fresh.n_refused_tb))
    if accepted:
        bad.append("a spline whose start_time is 1.79e9 s from this clock was ACCEPTED -- sampling it clamps "
                   "to the trajectory's start, which is the defect this assertion exists for")
    if fresh.n_refused_tb != 1:
        bad.append("the refusal was not counted (%d)" % fresh.n_refused_tb)
    # and the same spline, converted, MUST be accepted -- the guard must not refuse everything
    mono_like = t0 + 0.05
    ok_again = fresh.on_spline(order, knots, pts, traj_id=1, start_wall=mono_like, now=t0)
    if not ok_again:
        bad.append("a spline whose start_time IS in this clock was refused -- the guard is too strict")
    print("      the same spline converted into this clock: accepted=%s" % ok_again)
    # and the CONVERSION itself, as arithmetic
    epoch_now, mono_now, start_epoch = 1.792e9, 21288.0, 1.792e9 - 0.25
    start_mono = start_epoch + (mono_now - epoch_now)
    print("      conversion: start_time %.6f (epoch) -> %.6f (monotonic), age at receipt %.3f s"
          % (start_epoch, start_mono, mono_now - start_mono))
    if abs((mono_now - start_mono) - 0.25) > 1e-6:
        bad.append("the clock conversion does not preserve the age (%.6f)" % (mono_now - start_mono))
    if s2.trajectory_id != 1:
        bad.append("the id must be OUR per-waypoint id, not ego's per-replan id (got %s)" % s2.trajectory_id)
    if not all(a <= b + 1e-9 for a, b in zip(xs, xs[1:])):
        bad.append("the samples do not advance monotonically along the spline: %s" % np.round(xs, 3))
    if not (xs[0] - 0.10 <= xs[0] <= xs[-1] + 0.10 and abs(xs[-1] - 0.6) < 0.05):
        bad.append("the samples leave the segment [0.0, 0.6]: %s" % np.round(xs, 3))

    # --- 3. MANY replans must NOT look like a moving mission
    ids = set()
    for k in range(6):
        box.on_spline(order, knots, pts, traj_id=7 + k, start_wall=t0, now=t0 + 1.0 + 0.05 * k)
        ids.add(box.sample(now=t0 + 1.05 + 0.05 * k).trajectory_id)
    print("   after 6 replans: distinct trajectory_id values = %s" % sorted(ids))
    if ids != {1}:
        bad.append("a replan changed trajectory_id -- the consumer would think the mission advanced: %s"
                   % sorted(ids))

    # --- 4. arrival, dwell, then the NEXT waypoint gets a NEW id
    box.on_odom([0.6, 0.0, 0.5], [0.0, 0.0, 0.0], now=t0 + 2.0)          # arrived, slow
    s3 = box.sample(now=t0 + 2.1)
    if s3.trajectory_id != 1:
        bad.append("the dwell must not change the id")
    box.on_odom([0.6, 0.0, 0.5], [0.0, 0.0, 0.0], now=t0 + 3.2)          # dwell over
    s4 = box.sample(now=t0 + 3.3)
    print("   after the 1.0 s dwell: id %s, target %s" % (s4.trajectory_id, np.round(box.target, 3)))
    if s4.trajectory_id != 2:
        bad.append("the id must change when the MISSION moves to the next waypoint (got %s)"
                   % s4.trajectory_id)

    # --- 5. the tour ends with a STATEMENT, and the stream keeps talking
    box.on_odom([1.8, 0.0, 0.5], [0.0, 0.0, 0.0], now=t0 + 4.0)
    box.sample(now=t0 + 4.1)
    box.on_odom([1.8, 0.0, 0.5], [0.0, 0.0, 0.0], now=t0 + 5.2)
    tail = [box.sample(now=t0 + 5.3 + 0.01 * k) for k in range(20)]
    flags = {t.trajectory_flag for t in tail}
    print("   after the last dwell: %d further samples, flags %s, ids %s"
          % (len(tail), sorted(flags), sorted({t.trajectory_id for t in tail})))
    if flags != {TRAJECTORY_STATUS_COMPLETED}:
        bad.append("the end of the tour must be a sustained COMPLETED, got %s" % sorted(flags))
    print("   " + box.summary())

    print()
    if bad:
        print("EGO SOURCE SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("EGO SOURCE SELFTEST PASSED: the contract, from ego's spline, with no ego and no ROS")
    return 0


# -------------------------------------------------------------------------------- the ROS node
def _ros_main(args) -> int:
    try:
        import rclpy
    except ModuleNotFoundError:
        # Running this file bare means "publish /plan/cmd from ego's splines", which needs ROS 2.  Say so
        # instead of dying in a traceback: the offline half of this module is one flag away.
        print("ego_source: this half is the ROS 2 node that publishes %s from ego's splines." % args.cmd_topic)
        print("            rclpy is not importable here (source /opt/ros/humble/setup.bash and your workspace).")
        print("            For the offline contract test with no ROS:  %s --selftest"
              % os.path.relpath(os.path.abspath(__file__)))
        return 2
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from quadrotor_msgs.msg import PositionCommand

    import json
    import yaml
    from common import frames

    with open(args.calib_file) as fh:
        d = json.load(fh)
    calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))
    corners = yaml.safe_load(open(args.world))["mission"]["corners"]
    targets_w = np.array([[c[0], c[1], c[2] if len(c) > 2 else args.z] for c in corners], dtype=float)
    targets_l = (calib.R_WL.T @ (targets_w - calib.t_WL).T).T

    rclpy.init()
    node = Node("ego_source")
    box = EgoPlanBox(targets_l, hold_s=args.hold_s, arrive_m=args.arrive_m, settle_mps=args.settle_mps,
                     rate_hz=args.rate, log=lambda s: print(s, flush=True))
    print("ego_source: %d waypoints in L, dwell %.1f s, publishing %s at %.0f Hz"
          % (len(targets_l), args.hold_s, args.cmd_topic, args.rate))
    print("ego_source: goals go to %s in frame %s; ego's trajectory is read from %s"
          % (args.goal_topic, FRAME_L, args.bspline_topic))
    print("ego_source: THE BRIDGE AND THE SUPERVISOR DO NOT KNOW THIS IS EGO -- they read %s" % args.cmd_topic)

    # THE PUBLISHER'S QoS IS PART OF THE CONTRACT, AND THE FIRST VERSION OF THIS LINE GOT IT WRONG.
    # MEASURED 2026-10-15, in the second parked adapter run: the recorder printed
    #   "New subscription discovered on topic '/plan/cmd', requesting incompatible QoS. No messages will be
    #    sent to it. Last incompatible policy: DURABILITY"
    # and `/plan/cmd` reached nobody.  The contract is RELIABLE + KEEP_LAST(5) + TRANSIENT_LOCAL, and BOTH
    # consumers require it: the BRIDGE subscribes latched (`bridge/bridge_node.py:113-115`) so that a
    # consumer starting AFTER the plan box -- which is the F2 start-up order -- still finds the newest
    # sample, and `tools/record-traj.py:123-128` does the same.  `create_publisher(..., 10)` is the integer
    # overload, i.e. RELIABLE **VOLATILE**, and a VOLATILE publisher cannot match a TRANSIENT_LOCAL
    # subscriber: the bridge would simply never have heard from an ego planner.  The flown plan box says the
    # same thing in the same words at `planner/rectangle_source.py:329-335`; this is the same QoS, and
    # `tools/test-planner-swap.py` now checks that the two producers AGREE on it rather than trusting that
    # they do.
    pub_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PositionCommand, args.cmd_topic, pub_qos)
    # THE B-SPLINE SUBSCRIPTION'S QoS MUST MATCH EGO'S PUBLISHER, and that one is VOLATILE, not latched:
    # `create_publisher<Bspline>("planning/bspline", 10)` (`ego_replan_fsm.cpp:147`).  A TRANSIENT_LOCAL
    # subscriber CANNOT MATCH a VOLATILE publisher, so asking for durability here would have produced a node
    # that ran and never received a single plan, saying nothing.
    qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    #: one-element lists rather than `nonlocal` rebinding: the closure reads AND writes both, and a rebind
    #: without a `nonlocal` declaration silently becomes a LOCAL (pyflakes' "referenced before assignment",
    #: which `tools/test-lint.py` refuses -- it caught this on the first run of the change that introduced it).
    last_goal = [None]

    def on_odom(m):
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        box.on_odom(p, v if float(np.linalg.norm(v)) > 1e-6 else None)

    def on_odom_vel(m):
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        if float(np.linalg.norm(v)) > 1e-6 and box.p is not None:
            box.on_odom(box.p, v)

    def on_bspline(m):
        """Ego's plan, with its TIME BASE TRANSLATED into this process's clock.

        THE ONE PLACE THE TWO CLOCKS CAN BE TIED TOGETHER, and the first version of this adapter did not do
        it.  `start_time` is a `builtin_interfaces/Time` on the SYSTEM clock (`rclcpp::Clock().now()`,
        `ego_replan_fsm.cpp:845`); `EgoPlanBox` samples on `time.monotonic()`.  Reading both clocks at the
        SAME INSTANT gives the offset, and the conversion is exact:

            start_wall(monotonic) = start_time(epoch) + (monotonic_now - epoch_now)

        MEASURED, and this is what the first assembled ego sortie cost: without it, `t = now - start_wall`
        was about -1.79e9 s, `bspline_derivs` CLAMPED every sample to the trajectory's START, the vehicle was
        commanded to where it already was, the tour never left waypoint 1, and the supervisor waited 400 s
        for a COMPLETED that could not come.  `EgoPlanBox.on_spline` now refuses an implausible time base, so
        the same mistake can only be loud.
        """
        epoch_now = node.get_clock().now().nanoseconds * 1e-9      # ego's clock, read now
        mono_now = time.monotonic()                                # this process's clock, read now
        start_epoch = m.start_time.sec + 1e-9 * m.start_time.nanosec
        start_mono = start_epoch + (mono_now - epoch_now)
        box.on_spline(m.order, np.array(m.knots), np.array([[q.x, q.y, q.z] for q in m.pos_pts]),
                      m.traj_id, start_mono, now=mono_now)

    node.create_subscription(Odometry, args.odom_topic, on_odom, 10)
    node.create_subscription(Odometry, args.odom_vel_topic, on_odom_vel, 10)
    try:
        from traj_utils.msg import Bspline
        node.create_subscription(Bspline, args.bspline_topic, on_bspline, qos)
    except ImportError:
        print("ego_source: traj_utils/msg/Bspline is not importable -- this node cannot read ego's output. "
              "Build ego-planner first (its own planner_ws)", file=sys.stderr)
        return 3

    #: THE GOAL IS RE-SENT, NOT ANNOUNCED ONCE.  `ego_replan_fsm` returns to WAIT_TARGET -- "wait for goal or
    #: trigger" -- whenever it decides a plan has stalled, and it then waits for a NEW goal on this topic.  A
    #: producer that sends each waypoint exactly once therefore DEADLOCKS with it: ego waits for a goal, the
    #: plan box waits for an arrival that ego has stopped planning towards.
    #:
    #: MEASURED, EGO2-accept-clock-fixed: the patched FSM's own trace says it in one line --
    #:   PLANNER-STALL: the plan's clock is out (t 1.590 s of 1.600 s) but the vehicle is 1.890 m from the
    #:   goal -- a stalled plan, NOT an arrival; replanning instead of declaring the goal reached
    #: followed by `from EXEC_TRAJ to WAIT_TARGET`, and then 378 `WAIT_TARGET` lines against 22 `EXEC_TRAJ`
    #: while the aircraft held at 1.5 m for 360 s.  The HISTORICAL adapter did this too: the pre-F1
    #: `planner/ego_adapter.py` forwarded its goal at 1 Hz, and dropping that was a regression this branch
    #: introduced.  The `/plan/cmd` contract is unchanged: `trajectory_id` still advances only on ARRIVAL, so
    #: re-sending a goal is invisible to the consumer.
    GOAL_RESEND_S = 1.0
    last_goal_wall = [0.0]

    def tick():
        s = box.sample()
        if s is None:
            return
        m = PositionCommand()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = FRAME_L
        m.position.x, m.position.y, m.position.z = (float(s.p[0]), float(s.p[1]), float(s.p[2]))
        m.velocity.x = m.velocity.y = m.velocity.z = 0.0
        m.acceleration.x = m.acceleration.y = m.acceleration.z = 0.0
        m.yaw, m.yaw_dot = float(s.yaw), 0.0
        m.trajectory_id, m.trajectory_flag = int(s.trajectory_id), int(s.trajectory_flag)
        pub.publish(m)
        g = box.goal_to_send()
        now = time.monotonic()
        if g is not None and (now - last_goal_wall[0]) >= GOAL_RESEND_S:
            gm = PoseStamped()
            gm.header.stamp = m.header.stamp
            gm.header.frame_id = FRAME_L
            gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = (float(g[0]), float(g[1]), float(g[2]))
            gm.pose.orientation.w = 1.0
            goal_pub.publish(gm)
            last_goal_wall[0] = now
            if last_goal[0] is None or np.linalg.norm(g - last_goal[0]) > 1e-9:
                last_goal[0] = g
                print("ego_source: goal %d of %d -> %s (L)   (re-sent every %.1f s until it is reached)"
                      % (box.i_goal + 1, len(targets_l), np.round(g, 3), GOAL_RESEND_S), flush=True)

    node.create_timer(1.0 / args.rate, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(box.summary(), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="assert the contract with no ROS and no flight")
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    "..", "..", "worlds", "pillars.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--z", type=float, default=2.0, help="mission altitude for the world's corners, m")
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--arrive-m", type=float, default=0.060)
    ap.add_argument("--settle-mps", type=float, default=0.080)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC)
    ap.add_argument("--goal-topic", default=DEFAULT_GOAL_TOPIC)
    ap.add_argument("--bspline-topic", default=DEFAULT_BSPLINE_TOPIC)
    ap.add_argument("--odom-topic", default="/lio/robo/odom",
                    help="pose, 200 Hz, twist identically zero (Super-LIO fills only the pose there)")
    ap.add_argument("--odom-vel-topic", default="/lio/odom", help="10 Hz, carries the twist")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()
    return _ros_main(args)


if __name__ == "__main__":
    sys.exit(main())
