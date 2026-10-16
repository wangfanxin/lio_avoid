#!/usr/bin/env python3
"""THE CONTRACT SHELL FOR ROUTE B: ego's OWN traj_server samples the plan; this only says WHO/WHERE/WHEN.

WHY THIS EXISTS AT ALL.  `traj_server` (ego's own, unmodified) turns `planning/bspline` into a 100 Hz
`quadrotor_msgs/PositionCommand` on `/position_cmd`, carrying **position AND velocity AND acceleration** -- the
reference ego actually wrote.  What it does NOT carry is our MISSION semantics, and it never will: it hard-codes
`trajectory_flag = READY` (`traj_server.cpp:211`) and forwards ego's per-replan `traj_id`, which changes about
once a second.  Two consumers need the other behaviour:

  * the SUPERVISOR ends the sortie on `trajectory_flag = COMPLETED` ("the plan box has declared the tour
    complete") and lands;
  * the acceptance report counts `trajectory_id` per WAYPOINT (`docs/PLANNERS.md`), which is what makes "which
    leg is this" answerable in a recording.

SO THIS SHELL DOES EXACTLY TWO THINGS, and nothing else:
  1. it OWNS THE TOUR -- sends one goal at a time on `/move_base_simple/goal` (in L, re-sent every 1 s until the
     aircraft arrives within `--arrive-m`, then a `--hold-s` dwell) and counts the waypoint index;
  2. it REWRITES THE CONTRACT on the way through -- copies `/position_cmd` to `/plan/cmd` verbatim, except
     `trajectory_id = waypoint index + 1` and `trajectory_flag = READY` while flying, `COMPLETED` once the last
     waypoint is reached and settled -- and it KEEPS TALKING after COMPLETED, because that rule is what let the
     supervisor tell "done" from "dead" in the first place.  **From COMPLETED on it also HOLDS**: the forwarded
     point is pinned to the last waypoint with zero velocity, zero acceleration and a latched yaw.  The contract
     asks the stream to keep TALKING, not to keep FLYING, and the sortie that showed the difference spent ~20 s of
     its landing window being thrown around by ego's own sweeping reference (round 81).

IT DOES NOT SAMPLE, FILTER, RATE-LIMIT OR PREDICT ANYTHING -- before COMPLETED.  If the numbers on the wire differ
from ego's own there, that is a bug in this file, not a design choice -- which is why every forwarded message is
also written to a CSV (`--trace-csv`), so the forwarding can be checked against `/position_cmd` rather than
trusted.  The post-COMPLETED hold is the ONE deliberate exception, it is a few lines, and the bench measures it:
after the flag turns COMPLETED the forwarded point is a single coordinate with |v| = 0 (`_stage/s12-rig.sh`).

THE POSE IT USES for arrival/dwell is the same state the planner is fed (`/ego/odom_fcu`, the FCU's own estimate
in L) -- the owner's call: the mission must not be sequenced on a different estimate from the one it plans on.

usage: ego_contract_shim.py --world worlds/diagonal.yaml --calib-file _stage/calib.json \
                            --odom-topic /ego/odom_fcu [--hold-s 5.0] [--arrive-m 0.30] [--trace-csv PATH]
       ego_contract_shim.py --selftest
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import frames                                                       # noqa: E402

FRAME_L = "lio_world"
DEFAULT_CMD_TOPIC = "/plan/cmd"
DEFAULT_IN_TOPIC = "/position_cmd"          # traj_server's own publisher (traj_server.cpp:244)
DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"
TRAJECTORY_STATUS_READY = 1                 # ego's own constants (traj_server.cpp:211)
TRAJECTORY_STATUS_COMPLETED = 3


class TourSequencer:
    """The mission's bookkeeping, with NO ROS IN IT, so `--selftest` can assert it with no graph and no flight.

    The rule for arrival is the adapter's, unchanged, because it is the rule the accepted sorties used: the
    vehicle must be within `arrive_m` AND slow (|v| < `settle_mps`), then it must STAY there for `hold_s` before
    the next waypoint is released.  `traj_server` has no opinion about any of this -- it flies whatever spline
    arrives, and when the spline runs out it hovers at its end (`traj_server.cpp:191-202`).
    """

    def __init__(self, targets_l, arrive_m=0.30, hold_s=5.0, settle_mps=0.08, goal_resend_s=1.0,
                 log=None):
        self.targets = np.asarray(targets_l, dtype=float).reshape(-1, 3)
        self.arrive_m = float(arrive_m)
        self.hold_s = float(hold_s)
        self.settle_mps = float(settle_mps)
        self.goal_resend_s = float(goal_resend_s)
        self.log = log or (lambda s: None)
        self.i_goal = 0
        self.t_dwell = None
        self.done = False
        self.p = None
        self.v = None
        self.t_last_goal = None
        self.goals_sent = 0
        self.n_ready = 0
        self.n_completed = 0
        #: the yaw latched when the tour completed, so the post-COMPLETED hold cannot command a spin
        self._hold_yaw = None
        #: how many forwarded messages were rewritten to the hold (round 81)
        self.n_held_after = 0

    # ---- inputs ---------------------------------------------------------------------------------------
    def on_odom(self, p_l, v_l=None, now=0.0):
        self.p = np.asarray(p_l, dtype=float).reshape(3)
        self.v = None if v_l is None else np.asarray(v_l, dtype=float).reshape(3)
        self._sequence(now)

    def _sequence(self, now: float) -> None:
        if self.done or self.p is None:
            return
        d = float(np.linalg.norm(self.p - self.target))
        speed = float(np.linalg.norm(self.v)) if self.v is not None else float("inf")
        if self.t_dwell is None:
            if d <= self.arrive_m and speed < self.settle_mps:
                self.t_dwell = now
                self.log("shim: ARRIVED at waypoint %d of %d (%.3f m), dwelling %.1f s"
                         % (self.i_goal + 1, len(self.targets), d, self.hold_s))
            return
        if now - self.t_dwell >= self.hold_s:
            if self.i_goal >= len(self.targets) - 1:
                self.done = True
                self.log("shim: TOUR COMPLETE -- trajectory_flag becomes COMPLETED and the stream KEEPS TALKING")
                return
            self.i_goal += 1
            self.t_dwell = None
            self.log("shim: next waypoint %d of %d" % (self.i_goal + 1, len(self.targets)))

    # ---- outputs --------------------------------------------------------------------------------------
    @property
    def target(self):
        return self.targets[min(self.i_goal, len(self.targets) - 1)]

    def goal_to_send(self, now: float):
        """The one goal ego should be working on, or None when the tour is over -- re-sent at `goal_resend_s`
        because ego's FSM falls back to WAIT_TARGET when it decides a plan has stalled, and only a NEW goal
        pulls it out (the same reason `planner/ego_source.py:480-493` re-sends)."""
        if self.done:
            return None
        if self.t_last_goal is not None and (now - self.t_last_goal) < self.goal_resend_s:
            return None
        self.t_last_goal = now
        self.goals_sent += 1
        return self.target

    def rewrite(self, msg, now: float):
        """Apply the CONTRACT to one forwarded message: id per waypoint, READY -> COMPLETED.  In place.

        `trajectory_id` is `i_goal + 1`, so it advances exactly when the MISSION advances and at no other time
        (the adapter's rule, `ego_source.py:195`).  Everything else in the message is ego's own and is not
        touched -- including `header.frame_id`, which stays what `traj_server` wrote (`world`); the bridge
        accepts both names for this frame (`bridge/bridge_node.py:54-57`).

        ONE EXCEPTION, AND IT IS MEASURED (round 81): **after the tour is complete the message is rewritten to a
        HOLD.**  The contract requires the stream to keep TALKING after COMPLETED, not to keep FLYING, and the
        sortie that showed the difference (`DIAG34-routeb-pv`) spent **~20 s of the landing window being thrown
        around**: `setpoints.csv` records the commanded point going to (-2.5,-1.2), then (+0.4,-1.25), then
        (+1.1,-2.0) and back at 1.4-2.1 m/s, with the command's implied speed between messages reaching 4.25 m/s
        at the 95th percentile and 14.5 m/s at the maximum (median 0).  That was ego's own age rule sweeping and
        snapping (`traj_server.cpp:171-172`) -- motion the mission was over.  So from the moment `done` is set,
        the forwarded position is pinned to the LAST WAYPOINT (the point the aircraft has just arrived at, within
        the arrival gate), the velocity and acceleration are zeroed, and the yaw is latched at whatever the first
        post-COMPLETED message carried, so nothing spins during the landing.  Before COMPLETED the forwarding is
        still verbatim.
        """
        msg.trajectory_id = int(self.i_goal) + 1
        if self.done:
            if self._hold_yaw is None:
                self._hold_yaw = float(msg.yaw)          # latch once: a yaw jump mid-landing is a spin command
            p = self.targets[-1]
            msg.position.x, msg.position.y, msg.position.z = float(p[0]), float(p[1]), float(p[2])
            msg.velocity.x = msg.velocity.y = msg.velocity.z = 0.0
            msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
            msg.yaw, msg.yaw_dot = self._hold_yaw, 0.0
            msg.trajectory_flag = TRAJECTORY_STATUS_COMPLETED
            self.n_completed += 1
            self.n_held_after += 1
        else:
            msg.trajectory_flag = TRAJECTORY_STATUS_READY
            self.n_ready += 1
        return msg

    def summary(self):
        return ("shim: %d waypoint(s), %d goal(s) sent, %d READY and %d COMPLETED messages forwarded "
                "(%d of them held at the last waypoint), %s"
                % (len(self.targets), self.goals_sent, self.n_ready, self.n_completed, self.n_held_after,
                   "TOUR COMPLETE" if self.done else "waypoint %d" % (self.i_goal + 1)))


# -----------------------------------------------------------------------------------------------------------
def _targets_from_world(path, calib, z_default=2.0):
    w = yaml.safe_load(open(path))
    corners = w["mission"]["corners"]
    tw = np.array([[c[0], c[1], c[2] if len(c) > 2 else z_default] for c in corners], dtype=float)
    return (calib.R_WL.T @ (tw - calib.t_WL).T).T


def _selftest() -> int:
    bad = []
    t0 = 1000.0
    seq = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.30, hold_s=1.0, log=lambda s: print("   " + s))

    class Vec:                                      # the shape of PositionCommand's x/y/z triples
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class M:                                        # a stand-in for PositionCommand, fields and all: the hold
        def __init__(self):                         # rewrite touches position/velocity/acceleration/yaw, so a
            self.trajectory_id = 0                  # two-field stub cannot exercise it (it raised AttributeError
            self.trajectory_flag = 0                # the first time this test ran, which is the test working)
            self.position, self.velocity, self.acceleration = Vec(), Vec(), Vec()
            self.yaw = 0.0
            self.yaw_dot = 0.0

    m = M()
    # THE FLIGHT'S OWN ORDER, because the first version of this test teleported the vehicle and asserted a
    # completion that a real sequence only reaches after the SECOND arrival:  arrive at WP1 -> dwell -> advance
    # -> arrive at WP2 -> dwell -> the tour is DONE.
    seq.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)          # at WP1's target: arrival + dwell start
    seq.rewrite(m, now=t0)
    print("   at waypoint 1: id %d flag %d (READY=%d)" % (m.trajectory_id, m.trajectory_flag,
                                                          TRAJECTORY_STATUS_READY))
    if (m.trajectory_id, m.trajectory_flag) != (1, TRAJECTORY_STATUS_READY):
        bad.append("the first waypoint must be id 1 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 0.5)   # mid-dwell, vehicle moved on: id must NOT move
    seq.rewrite(m, now=t0 + 0.5)
    if m.trajectory_id != 1:
        bad.append("the dwell changed the id to %d -- the consumer would think the mission advanced"
                   % m.trajectory_id)
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.6)   # dwell over: the MISSION advances
    seq.rewrite(m, now=t0 + 1.6)
    print("   after waypoint 1's dwell: id %d flag %d (still READY)" % (m.trajectory_id, m.trajectory_flag))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_READY):
        bad.append("advancing to waypoint 2 must give id 2 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                       m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.7)   # arrival at WP2 starts its dwell
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 2.8)   # ... and the dwell ends the tour
    seq.rewrite(m, now=t0 + 2.8)
    print("   after the LAST waypoint's dwell: id %d flag %d (COMPLETED=%d)"
          % (m.trajectory_id, m.trajectory_flag, TRAJECTORY_STATUS_COMPLETED))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_COMPLETED):
        bad.append("the last waypoint's dwell must complete the tour: got id %d flag %d"
                   % (m.trajectory_id, m.trajectory_flag))
    if seq.goal_to_send(now=t0 + 3.0) is not None:
        bad.append("a goal was offered after the tour was complete")
    # IT KEEPS TALKING: further messages are still forwarded, still COMPLETED -- AND IT NO LONGER MOVES (round 81)
    msg = m                                          # the same stand-in, reused with its new fields
    msg.position.x, msg.position.y, msg.position.z = 9.0, 9.0, 9.0
    msg.velocity.x, msg.velocity.y, msg.velocity.z = 3.0, 3.0, 3.0
    msg.acceleration.x, msg.acceleration.y = 5.0, 5.0
    msg.yaw = 1.234
    seq.rewrite(msg, now=t0 + 4.0)
    if msg.trajectory_flag != TRAJECTORY_STATUS_COMPLETED:
        bad.append("the stream stopped saying COMPLETED -- the supervisor needs it on every message")
    last_wp = seq.targets[-1]
    held = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
    if max(abs(held[k] - float(last_wp[k])) for k in range(3)) > 1e-9:
        bad.append("after COMPLETED the forwarded point is not the last waypoint: %s vs %s"
                   % (held, tuple(float(x) for x in last_wp)))
    if any(abs(v) > 1e-12 for v in (msg.velocity.x, msg.velocity.y, msg.velocity.z)):
        bad.append("after COMPLETED the forwarded VELOCITY is not zero: %s"
                   % (msg.velocity.x, msg.velocity.y, msg.velocity.z))
    if any(abs(a) > 1e-12 for a in (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)):
        bad.append("after COMPLETED the forwarded ACCELERATION is not zero")
    yaw1 = float(msg.yaw)
    msg.position.x = -7.0
    msg.yaw = -3.0
    seq.rewrite(msg, now=t0 + 4.1)
    print("   after COMPLETED: point pinned to the last waypoint %s, |v| = 0, yaw latched at %.3f (second"
          " message keeps %.3f)"
          % (tuple(round(float(x), 3) for x in last_wp), yaw1, float(msg.yaw)))
    if abs(float(msg.yaw) - yaw1) > 1e-12:
        bad.append("the post-COMPLETED hold let the yaw move (%.3f -> %.3f)" % (yaw1, float(msg.yaw)))
    if seq.n_held_after < 2:
        bad.append("the hold counter did not move (%d)" % seq.n_held_after)
    # and the goal cadence
    seq2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], goal_resend_s=1.0, log=lambda s: None)
    seq2.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)
    n_sent = sum(1 for k in range(30) if seq2.goal_to_send(now=t0 + 0.1 * k) is not None)
    print("   3 s of ticks at 10 Hz, goal_resend_s 1.0 -> %d goal(s) (expect 3-4)" % n_sent)
    if not 3 <= n_sent <= 4:
        bad.append("the goal cadence is %d in 3 s, expected 3-4" % n_sent)
    if bad:
        print("\nCONTRACT SHIM SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("\nCONTRACT SHIM SELFTEST PASSED: id per waypoint, COMPLETED keeps talking WITHOUT moving, yaw latched, "
          "goal cadence")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    "worlds", "diagonal.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--z", type=float, default=2.0)
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--arrive-m", type=float, default=0.30)
    ap.add_argument("--settle-mps", type=float, default=0.08)
    ap.add_argument("--in-topic", default=DEFAULT_IN_TOPIC)
    ap.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC)
    ap.add_argument("--goal-topic", default=DEFAULT_GOAL_TOPIC)
    ap.add_argument("--odom-topic", default="/ego/odom_fcu")
    ap.add_argument("--trace-csv", default="")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    import json                                                                  # noqa: E402
    import rclpy                                                                 # noqa: E402
    from geometry_msgs.msg import PoseStamped                                    # noqa: E402
    from nav_msgs.msg import Odometry                                            # noqa: E402
    from quadrotor_msgs.msg import PositionCommand                               # noqa: E402
    from rclpy.node import Node                                                  # noqa: E402
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

    d = json.load(open(args.calib_file))
    calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))
    targets_l = _targets_from_world(args.world, calib, args.z)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                                            # noqa: BLE001
        rclpy.init(args=[])
    node = Node("ego_contract_shim")
    seq = TourSequencer(targets_l, arrive_m=args.arrive_m, hold_s=args.hold_s,
                        settle_mps=args.settle_mps, log=lambda s: print(s, flush=True))
    print("shim: %d waypoint(s) in L from %s" % (len(targets_l), args.world))
    for i, t in enumerate(targets_l):
        print("shim: waypoint %d -> %s (L)" % (i + 1, np.round(t, 3)))
    print("shim: TRAJ_SERVER's %s is forwarded to %s with trajectory_id per waypoint and a real COMPLETED"
          % (args.in_topic, args.cmd_topic))

    # the CONTRACT's QoS is not negotiable: RELIABLE + KEEP_LAST(5) + TRANSIENT_LOCAL, exactly as
    # `planner/ego_source.py:421-424` publishes it (a latched subscriber must still find the newest sample)
    cmd_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PositionCommand, args.cmd_topic, cmd_qos)
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    # traj_server publishes with a plain depth-50 RELIABLE profile (`traj_server.cpp:244`)
    sub_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    trace = open(args.trace_csv, "w") if args.trace_csv else None
    if trace:
        trace.write("t_wall,wp,flag,px,py,pz,vx,vy,vz,ax,ay,az,yaw,jump_m,frame\n")
    last = [None]
    n = {"fwd": 0}

    def on_odom(m: Odometry):
        # A REPORTED ZERO IS "SLOW", NOT "UNKNOWN".  The first version passed `None` whenever the twist was
        # (0,0,0), and the sequencer reads `None` as "speed unknown" -- which BLOCKS the arrival and therefore the
        # whole tour.  In flight it worked only because the FCU's velocity noise is never exactly zero; a
        # stationary vehicle reporting a clean zero would have stalled the mission.  A message callback always
        # HAS a twist, so it is passed through as it is.
        v = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        seq.on_odom([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
                    v, now=node.get_clock().now().nanoseconds * 1e-9)

    def on_cmd(m: PositionCommand):
        seq.rewrite(m, now=node.get_clock().now().nanoseconds * 1e-9)
        p = np.array([m.position.x, m.position.y, m.position.z])
        jump = 0.0 if last[0] is None else float(np.linalg.norm(p - last[0]))
        last[0] = p
        pub.publish(m)
        n["fwd"] += 1
        if trace:
            trace.write("%.4f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n"
                        % (node.get_clock().now().nanoseconds * 1e-9, m.trajectory_id, m.trajectory_flag,
                           p[0], p[1], p[2], m.velocity.x, m.velocity.y, m.velocity.z,
                           m.acceleration.x, m.acceleration.y, m.acceleration.z, m.yaw, jump,
                           m.header.frame_id))

    node.create_subscription(Odometry, args.odom_topic, on_odom, sub_qos)
    node.create_subscription(PositionCommand, args.in_topic, on_cmd, sub_qos)

    def tick():
        g = seq.goal_to_send(now=node.get_clock().now().nanoseconds * 1e-9)
        if g is None:
            return
        gm = PoseStamped()
        gm.header.stamp = node.get_clock().now().to_msg()
        gm.header.frame_id = FRAME_L
        gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = float(g[0]), float(g[1]), float(g[2])
        gm.pose.orientation.w = 1.0
        goal_pub.publish(gm)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(seq.summary(), flush=True)
    if trace:
        trace.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
