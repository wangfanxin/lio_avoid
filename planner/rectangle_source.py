#!/usr/bin/env python3
"""The PLAN box for mission A: the square's waypoints, flown by its OWN executor, as a command stream.

THE INTERFACE, AND WHAT F1 CHANGED.  Until F1 this node published `nav_msgs/Path` on `/planner/traj` and
`fsm/mission_fsm.py` was the executor: it consumed the Path, walked its corners, judged arrival, settled,
dwelled and streamed the setpoints.  F1 moves execution into the plan box (amendment A1 of
`_stage/refactor-bridge-plan.md`: the PLAN box owns plan *and* its native execution, because that is the box
a whole-replacement like ego swaps out).  So this node now publishes ONE output:

    /plan/cmd   quadrotor_msgs/msg/PositionCommand, in L, at --cmd-rate Hz (default 100)

and `bridge/bridge_node.py` consumes it and puts it on MAVLink 84 at 50 Hz.  **The Path is gone, and it is
gone on purpose**: keeping two interfaces with one producer is how the adapter came back last time
(`planner/ego_adapter.py`, deleted with the ego line, section 8 item 2 of the plan).  What the Path carried
that the stream does not is the *record* of the plan, and that is `tools/record-traj.py`'s, which F1
repointed at `/plan/cmd` in the same change (design risk R5: otherwise every run silently loses its
plan-clearance column).

THE STREAM HAS ITS OWN CLOCK, IN ITS OWN EXECUTOR.  F1's first requirement, and it is a measured one: the
shadow stream of S3 declared 100 Hz and emitted **4 Hz**, because it rode the producer's `--rate 4` republish
loop ("a loop that runs slower than the declared period can only emit once per pass",
`_stage/refactor-S3-frames-and-discrepancies.txt` section 1.3).  Flying a 4 Hz hold into 48 Hz messages is
the failure that requirement exists to prevent, so the stream is published from a `create_wall_timer` at
`--cmd-rate` on a `MultiThreadedExecutor`, and the pacer is stepped from EVERY odometry sample (~200 Hz)
instead of from a 4 Hz loop -- which also makes the arrival decision sharper than the 50 Hz it was taken at
before.

THE CORNER HOLD IS NOW THIS NODE'S DECISION, NOT A CONVENTION.  `planner/cmd_stream.py::TourPacer` owns
which waypoint is in hand, the arrival gate (estimate-only, from `/lio/robo/odom`), the settle and the
dwell; the stream says `position = waypoint, velocity = 0` while it dwells, which is what
`traj_server.cpp:191-202` does at the end of a trajectory.  The tour is the five waypoints
`[c1, c2, c3, c4, c1]` (see `cmd_stream.py`: the entry leg has no dwell, and the five leg phases are what
`tools/metrics.py` windows the route with).

usage: planner/rectangle_source.py [--z 0.8] [--hold-s 5] [--cmd-rate 100]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import frames          # noqa: E402
from planner import cmd_stream     # noqa: E402

try:
    import yaml
except ImportError:                # pragma: no cover
    yaml = None


def load_square(world_path):
    """The mission square, in W, straight from the world file (so there is one source of geometry)."""
    cfg = {}
    if yaml is not None:
        cfg = yaml.safe_load(open(world_path)) or {}
    mission = cfg.get("mission") or {}
    corners = mission.get("corners")
    if corners is not None:
        pts = np.asarray(corners, dtype=float).reshape(-1, 3)
        # the file lists the closing point explicitly; drop it, `densify` closes the loop itself
        if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
            pts = pts[:-1]
        return pts, mission
    sq = mission.get("square")
    if sq is None:
        raise SystemExit("%s has neither mission.corners nor mission.square" % world_path)
    x0, y0, x1, y1 = (float(v) for v in sq)
    z = float(mission.get("takeoff_alt", 2.0))
    return np.array([[x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z]]), mission


def densify(corners, spacing=0.05):
    """Walk the closed polygon, emitting points every `spacing` metres.

    KEPT even though nothing publishes a polyline any more: it is the SHAPE the mission's squares are made
    of, `tools/hold-design-check.py` and the tour tests built their cases from it, and a world whose corner
    list is not closed by the reader is worth one function that says so.
    """
    out = []
    n = len(corners)
    for i in range(n):
        a, b = corners[i], corners[(i + 1) % n]
        d = b - a
        L = float(np.linalg.norm(d))
        steps = max(1, int(round(L / spacing)))
        for k in range(steps):
            out.append(a + d * (k / steps))
    out.append(corners[0])
    return np.array(out)


def tour_targets(corners):
    """The tour's WAYPOINTS in W: the four corners, then the first one again.

    Five targets for what the FSM used to fly as five `leg` phases, and the first one is the ENTRY (the
    aircraft starts on the pad and flies to corner 1).  The dwell belongs to the corners of the tour, so
    `TourPacer(entry_hold_s=0.0)` gives the entry leg none -- the same route, in the same order, with the
    same phase boundaries as RECT7/RECT9/RECT10/RECT11/RECT21 (`cmd_stream.py`, and risk R1).
    """
    corners = np.asarray(corners, dtype=float)
    return np.vstack([corners, corners[0:1]])


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "pillars.yaml"))
    ap.add_argument("--z", type=float, default=None, help="override the mission altitude, m")
    ap.add_argument("--vertical-first-m", type=float, default=None,
                    help="PREPEND a single-point leg straight up from the pad to this altitude (m above the "
                         "pad) and hold it for --vertical-hold-s.  This is the setpoint-driven take-off's "
                         "CLIMB LEG: with the owner's design the take-off command is only ~0.2 m, and this "
                         "leg is what lifts the aircraft to the mission altitude.  Default off: without it "
                         "the first setpoint is corner 1 and the aircraft climbs in transit to it.  When it "
                         "is on, the A4 window is the HOLD at the top of the climb, which is what keeps A4 "
                         "comparable with every row in docs/metrics/")
    ap.add_argument("--vertical-hold-s", type=float, default=3.0,
                    help="how long to HOLD the vertical first target (the A4 window).  Only used with "
                         "--vertical-first-m; 3.0 s is the F1/F2 take-off hold")
    ap.add_argument("--spacing", type=float, default=0.05,
                    help="kept for the tour's own geometry print and the tests; nothing publishes a "
                         "polyline any more")
    ap.add_argument("--hold-s", type=float, default=-1.0,
                    help="THE DWELL: hold this long at every corner of the tour, after the vehicle has "
                         "ARRIVED AND SETTLED at it.  -1 (default) = use the world file's mission.hold_s; "
                         "the harness passes RECT_HOLD_S explicitly so the flown dwell is never a world "
                         "file's default by accident")
    ap.add_argument("--leg-speed", type=float, default=0.30,
                    help="does not pace anything (the legs advance on ARRIVAL); used only to PRINT the "
                         "flight time a leg would take at this speed, next to the measured one")
    ap.add_argument("--odom", default="/lio/robo/odom",
                    help="the vehicle's own pose, L frame, ~200 Hz.  Converted to W with the calibration "
                         "below; this is the ONLY vehicle input the arrival decision has -- the truth tap "
                         "is not subscribed here, because a real planner has no truth tap")
    ap.add_argument("--odom-vel-topic", default="/lio/odom",
                    help="where the SETTLE gate's speed comes from.  `/lio/robo/odom` carries only the "
                         "pose (MEASURED: its twist is identically zero), and `/lio/odom` (10 Hz) has the "
                         "velocity; if neither gives one, the pose is differentiated instead and the log "
                         "says so")
    ap.add_argument("--arrive-m", type=float, default=0.06,
                    help="arrival gate, horizontal, m, measured on THIS node's own odometry.  0.06 m is "
                         "the number every flight in docs/metrics/ was scored against (B3: 40 of 40 "
                         "corners <= 68 mm); the old 0.10 m producer gate existed only so that the "
                         "producer could not lead the CONSUMER into the next leg before the consumer had "
                         "settled, and with one owner there is one number")
    ap.add_argument("--settle-speed", type=float, default=0.08,
                    help="THE SETTLE: the dwell clock starts only when the vehicle is inside --arrive-m "
                         "AND its horizontal speed is below this (m/s).  MEASURED twice (RECT11 c3 4.96 s, "
                         "RECT21 c3 4.80 s): the old rule started the clock on the first radius crossing "
                         "while the aircraft was still ringing from a 1.7 m/s arrival, the ring carried it "
                         "outside the 0.15 m judging radius and back, and one visit split in two")
    ap.add_argument("--settle-s", type=float, default=3.0,
                    help="and the settle's bound: inside the gate but still not slow after this long, the "
                         "arrival is declared anyway so a noisy estimate cannot stall the mission")
    ap.add_argument("--leg-start-m", type=float, default=0.30,
                    help="a leg's clock arms when the vehicle is within this of the leg's start corner "
                         "(the third of the three arm conditions -- see TourPacer._arm)")
    ap.add_argument("--arm-above-m", type=float, default=0.50,
                    help="and the second: the leg clocks do not run until the vehicle is above this W "
                         "altitude.  The plan box starts ~25 s before take-off, so a clock armed at "
                         "start-up would time out on the ground")
    ap.add_argument("--arm-move-m", type=float, default=0.50,
                    help="and the first: the vehicle has moved this far from where the plan box first saw "
                         "it")
    ap.add_argument("--leg-timeout-s", type=float, default=20.0,
                    help="SAFETY NET: if the armed flight clock runs this long without an arrival, say so "
                         "with a NAMED reason and advance anyway (a missed arrival must not hang the "
                         "mission).  The longest leg here is 2.0 m and takes 4-9 s at the measured "
                         "0.22-0.50 m/s")
    ap.add_argument("--calib-file", default="",
                    help="the MEASURED L->W calibration -- the SAME file the bridge converts commands "
                         "with.  Without it this node publishes using the PREDICTED transform, and if the "
                         "bridge then converts with a measured one the commanded points are offset by the "
                         "difference between the two")
    # ---- THE COMMAND STREAM, THE PLAN BOX'S ONLY OUTPUT ----------------------------------------------
    ap.add_argument("--cmd-topic", default="/plan/cmd",
                    help="the PLAN box's command stream, quadrotor_msgs/msg/PositionCommand.  Name and "
                         "type are fixed by the target interface: it is what ego's own traj_server "
                         "publishes (traj_server.cpp:244-250) and what the BRIDGE consumes "
                         "(bridge/bridge_node.py).")
    ap.add_argument("--cmd-rate", type=float, default=100.0,
                    help="the command stream's publishing rate, Hz, on its OWN wall timer in its own "
                         "executor -- ego's traj_server runs at 100 Hz (a 10 ms timer) and the WIRE rate "
                         "stays 50 Hz (the bridge's relay; message 84 at 50 Hz is the flown "
                         "configuration)")
    ap.add_argument("--cmd-id", type=int, default=1,
                    help="the FIRST trajectory_id.  Its number starts from 1 (PositionCommand.msg:20), "
                         "allowing a consumer to compare it with 0 = 'no trajectory yet'; the id "
                         "INCREMENTS PER WAYPOINT, so a consumer that has a stream and no other signal "
                         "can tell 'the same waypoint, republished' from 'the next leg'")
    args = ap.parse_args()

    corners, mission = load_square(args.world)
    if args.z is not None:
        corners[:, 2] = args.z
    hold_s = args.hold_s if args.hold_s >= 0.0 else float(mission.get("hold_s", 0.0) or 0.0)
    targets_w = tour_targets(corners)
    print("rectangle_source: square in W (ENU, origin = pad):")
    for i, c in enumerate(corners):
        print("   corner %d  E %+6.3f  N %+6.3f  U %+6.3f" % (i, c[0], c[1], c[2]))
    print("   the tour is %d WAYPOINTS (entry = corner 1, then the four square legs); dwell %.1f s at "
          "each corner of the tour, none on the entry" % (len(targets_w), hold_s))

    # W -> L.  MEASURED 2026-10-13, and it cost 51 mm: this node used the PREDICTED transform while the
    # FSM converted L->F with the MEASURED one, and the two disagree in t (predicted t_N = +50 mm, from
    # the sensor's 0.05 m lever arm, against a measured t_N = -4 mm).  The commanded corners therefore
    # sat 51 mm SOUTH of the intended ones in every flight, the aircraft flew faithfully to them, and the
    # box looked translated in N -- in the command, not in the aircraft.  The source now takes the same
    # file the bridge takes, and prints the difference either way so a mismatch cannot be silent again.
    predicted = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    calib = predicted
    # THE VERTICAL FIRST TARGET, prepended BEFORE the frame conversion so it goes through exactly the same
    # W->L path as every other target.  It is used with the owner's take-off design (`--takeoff-command-m
    # 0.2` + a trigger on airborne): the small command only leaves the ground, and this leg is what lifts
    # the aircraft to the mission altitude and HOLDS it there for `--vertical-hold-s`, which is the A4
    # window.  It is inserted at index 0 so `TourPacer` gives it `entry_hold_s` and the tour proper keeps
    # its own phase names: the phase log then reads leg1 = the climb, leg2..leg6 = the tour, and
    # `tools/metrics.py` already opens B1/B2/B3 at the SECOND leg entry (the pad is outside the box), so
    # the box metrics are computed over the same four corners as every earlier sortie.
    if args.vertical_first_m is not None:
        vert = np.array([[0.0, 0.0, float(args.vertical_first_m)]])
        targets_w = np.vstack([vert, targets_w])
        print("   VERTICAL FIRST TARGET: straight up from the pad to U %+.3f m, held %.1f s, THEN the tour"
              % (args.vertical_first_m, args.vertical_hold_s))
        print("      (this is the setpoint-driven take-off's climb leg: leg1 = the climb, leg2..leg%d = the "
              "tour; the A4 window is the hold at the top of it)" % len(targets_w))
    if args.calib_file and os.path.exists(args.calib_file):
        with open(args.calib_file) as fh:
            d = json.load(fh)
        calib = frames.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"]))
        print("   using the MEASURED calibration from %s (%s)"
              % (args.calib_file, d.get("yaw_deg", float("nan"))))
    else:
        print("   no --calib-file: using the PREDICTED calibration"
              " (the bridge must use the predicted one too, or the commands will be offset)")
    dt = calib.t_WL - predicted.t_WL
    print("   measured - predicted t = [%+.4f %+.4f %+.4f] m  (|%.1f| mm)"
          % (dt[0], dt[1], dt[2], float(np.linalg.norm(dt)) * 1e3))
    R_LW = calib.R_WL.T
    targets_l = (R_LW @ (targets_w - calib.t_WL).T).T
    back = (calib.R_WL @ targets_l.T).T + calib.t_WL
    err = np.linalg.norm(back - targets_w, axis=1)
    print("   L<-W: yaw %+.3f deg, t = [%+.4f %+.4f %+.4f] m;  worst W->L->W round-trip error %.6f mm"
          % (math.degrees(math.atan2(R_LW[1, 0], R_LW[0, 0])), -calib.t_WL[0], -calib.t_WL[1],
             -calib.t_WL[2], float(err.max()) * 1e3))
    print("   corner hold : %.1f s dwell at each corner, ARRIVAL + SETTLE gated (gate %.3f m, settle "
          "%.3f m/s within %.1f s), leg timeout %.1f s; the vehicle's pose comes from %s, converted L->W "
          "with the SAME calibration printed above"
          % (hold_s, args.arrive_m, args.settle_speed, args.settle_s, args.leg_timeout_s, args.odom))

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Odometry

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("rectangle_source")
    # DEPTH 5, RELIABLE, TRANSIENT_LOCAL.  Latched so a consumer that starts AFTER the plan box (which is
    # the F2 order: bridge_node comes up after the plan box) still finds the newest sample; deep enough
    # that a momentarily busy 50 Hz relay does not leave the publisher discarding every sample it has
    # (MEASURED 2026-10-14: RELIABLE + depth 1 lost 40 of 41 messages to a slow subscriber).
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    try:
        from quadrotor_msgs.msg import PositionCommand as CmdMsg
    except ImportError as exc:
        # NOT a silent downgrade: this message IS the plan box's output now, so a missing package has to
        # stop the node rather than leave a mission with nothing to fly.
        print("CANNOT START: quadrotor_msgs/msg/PositionCommand is not importable (%s).  Build the "
              "vendored package: see ros_ws/src/quadrotor_msgs/README.md" % exc, flush=True)
        return 2
    cmd_pub = node.create_publisher(CmdMsg, args.cmd_topic, qos)

    # the vehicle's own pose in L.  BEST_EFFORT/depth 200 is what every other consumer of this topic uses
    # (tools/record-odom.py, the FSM): the LIO publishes it as a sensor stream.
    best_effort = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
    odom = {"p_l": None, "n": 0, "wall": 0.0, "v_h": None, "v_l": None, "v_source": "diff",
            "v_seen": 0, "prev_p": None, "prev_wall": 0.0}

    def on_odom(m):
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        now = time.monotonic()
        dt = now - odom["prev_wall"]
        if odom["v_source"] == "diff" and odom["prev_p"] is not None and dt > 1e-3:
            # a light low-pass: differencing a ~mm-noisy pose at ~200 Hz is a noisy velocity
            raw = (p - odom["prev_p"]) / dt
            odom["v_l"] = raw if odom["v_l"] is None else 0.8 * odom["v_l"] + 0.2 * raw
        odom["prev_p"] = p
        odom["prev_wall"] = now
        odom["p_l"] = p
        odom["n"] += 1
        odom["wall"] = now
        # THE PACER IS STEPPED FROM EVERY ODOMETRY SAMPLE, not from the publishing loop: the arrival gate
        # is a decision about the vehicle and it belongs on the vehicle's own stream (~200 Hz here, against
        # the FSM's 50 Hz before F1).
        step(now)

    def on_odom_vel(m):
        """The velocity from `/lio/odom`.

        MEASURED 2026-10-13, and it cost a flight: Super-LIO fills ONLY the pose in `/lio/robo/odom`
        (`ROSWrapper.cpp:396-407` sets position and orientation, no twist), so that topic's velocity is
        identically ZERO.  The settle gate needs a speed that is not noise, so it prefers this topic and
        falls back to the differentiated pose, saying which one it used.
        """
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        if float(np.linalg.norm(v)) > 1e-6:
            odom["v_l"] = v
            odom["v_source"] = "lio/odom"
            odom["v_seen"] += 1

    node.create_subscription(Odometry, args.odom, on_odom, best_effort)
    node.create_subscription(Odometry, args.odom_vel_topic, on_odom_vel, best_effort)

    def here_w():
        """The vehicle in W.  The SAME calibration this node publishes with, read the other way.

        Every commanded point travels W -> L as `R_LW @ (p - t_WL)`; this is `R_WL @ p_l + t_WL` with
        `R_WL = R_LW.T`, i.e. the inverse of exactly that expression -- not a second convention.
        """
        if odom["p_l"] is None:
            return None
        return calib.R_WL @ odom["p_l"] + calib.t_WL

    def seg_len_w(i):
        """The length of leg `i`: the entry leg starts wherever the plan box first saw the vehicle."""
        if i == 0:
            if pacer.p_first is None:
                return float("nan")
            return float(np.linalg.norm(targets_w[0][:2] - pacer.p_first[:2]))
        return float(np.linalg.norm(targets_w[i][:2] - targets_w[i - 1][:2]))

    pacer = cmd_stream.TourPacer(targets_w, hold_s, args.arrive_m, args.settle_speed, args.settle_s,
                                 args.leg_timeout_s, time.monotonic(),
                                 log=lambda s: print(s, flush=True), seg_len_w=seg_len_w,
                                 odom_label=args.odom, leg_speed=args.leg_speed,
                                 arm_above_m=args.arm_above_m, arm_move_m=args.arm_move_m,
                                 arm_near_m=args.leg_start_m,
                                 # WITH THE VERTICAL FIRST TARGET the entry leg IS the A4 window, so it
                                 # gets a real dwell (the default entry_hold_s is 0.0 because the tour's
                                 # dwells belong to the tour's corners -- true before this leg existed).
                                 entry_hold_s=(args.vertical_hold_s
                                               if args.vertical_first_m is not None else 0.0))

    def step(now):
        """One pacing tick, from the odometry stream.  Keeps a W sample of the leg's start in L for the
        stream's own yaw (the pacer owns the decision, not the calibration)."""
        p_w = here_w()
        v_h = None if odom["v_l"] is None else float(np.linalg.norm(odom["v_l"][:2]))
        pacer.odom_n = odom["n"]
        pacer.odom_wall = odom["wall"]
        pacer.step(p_w, v_h, now)
        if pacer.p_start_w is not None and getattr(pacer, "p_start_l", None) is None:
            pacer.p_start_l = R_LW @ (pacer.p_start_w - calib.t_WL)

    stream = cmd_stream.CmdStream(pacer, args.cmd_rate, targets_l, trajectory_id=args.cmd_id,
                                  log=lambda s: print(s, flush=True))

    def on_timer():
        """THE COMMAND STREAM'S OWN 100 Hz CLOCK.  Nothing else in this node runs at this rate."""
        now = time.monotonic()
        if stream.next_t is None:
            stream.next_t = now
        if not stream.due(now):
            return
        s = stream.sample()
        m = CmdMsg()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = s.frame_id
        m.position.x, m.position.y, m.position.z = (float(s.p[0]), float(s.p[1]), float(s.p[2]))
        m.velocity.x, m.velocity.y, m.velocity.z = (float(s.v[0]), float(s.v[1]), float(s.v[2]))
        m.acceleration.x, m.acceleration.y, m.acceleration.z = (float(s.a[0]), float(s.a[1]),
                                                                float(s.a[2]))
        m.yaw = s.yaw
        m.yaw_dot = s.yaw_dot
        m.kx = [float(v) for v in cmd_stream.KX]      # ZERO, and documented as ignored (see cmd_stream)
        m.kv = [float(v) for v in cmd_stream.KV]
        m.trajectory_id = s.trajectory_id
        m.trajectory_flag = s.trajectory_flag
        cmd_pub.publish(m)
        stream.advance(now)

    timer = node.create_timer(1.0 / args.cmd_rate, on_timer)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    print("command stream: %s on %s at %.1f Hz (its OWN wall timer), frame %s, %d waypoints, "
          "trajectory_id %d..%d -- the PLAN box's only output.  kx/kv are sent as ZERO (message 84 has no "
          "gain fields); the dwell is CONTENT: position = the waypoint, velocity = 0."
          % (CmdMsg.__name__, args.cmd_topic, args.cmd_rate, cmd_stream.FRAME_L, len(targets_l),
             args.cmd_id, args.cmd_id + len(targets_l) - 1), flush=True)
    print("pacing      : stepped from %s (~200 Hz), arrival gate %.3f m ESTIMATE-ONLY, settle %.3f m/s "
          "within %.1f s, dwell %.1f s, velocity source %s (%d samples so far)"
          % (args.odom, args.arrive_m, args.settle_speed, args.settle_s, hold_s, args.odom_vel_topic,
             odom["v_seen"]), flush=True)

    stop = {"flag": False}

    def on_term(signum, frame):                          # noqa: ARG001
        stop["flag"] = True

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)
    while rclpy.ok() and not stop["flag"]:
        executor.spin_once(timeout_sec=0.1)
    # THE RECORD OF WHAT THE STREAM ACTUALLY DID.  Its rate is the number the shadow stream got wrong
    # (4 Hz declared as 100), so the node's last line states the achieved one.
    elapsed = max(1e-9, time.monotonic() - pacer.tour_t0)
    print("%s -- achieved %.2f Hz over %.1f s; pacing ticks %d odometry samples (%s)"
          % (stream.summary(), stream.n / elapsed, elapsed, pacer.odom_n, args.odom), flush=True)
    executor.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
