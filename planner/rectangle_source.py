#!/usr/bin/env python3
"""The rectangle trajectory source: publishes `/planner/traj` for mission A.

This is the *frozen* interface of PLAN.md decision 11 and 4.1: a `nav_msgs/Path` in L, points dense
enough that the FSM can difference it for velocity.  It is the ONLY trajectory source in the tree: an
ego-planner used to publish the same Path behind an adapter, but the owner dropped that line on
2026-10-15 and did not change one line of the FSM -- which is exactly what building the rectangle
against the interface rather than hard-coding corners in the flight code bought.

The path is written in W (the world file's frame, ENU, origin at the pad) and converted to L with the
PREDICTED calibration: the vehicle spawns facing North (decision 14), so L's x axis is the body's
forward axis at init, i.e. W's North, and L's origin is the housing's position then
(`Calib.predicted(world_axes="body")`).  What the FSM then MEASURES at preflight is compared against
this prediction and logged -- a mismatch is a finding, not something to smooth over.

THE CORNER HOLD IS PACED BY THE VEHICLE, NOT BY A CLOCK (2026-10-15).  With `--hold-s > 0` this node
publishes ONE LEG PER MESSAGE and advances to the next leg only when the vehicle has ARRIVED at the leg's
end -- measured from `/lio/robo/odom`, converted L -> W with the SAME calibration this node publishes
with.  After the arrival it holds the corner for `--hold-s` (keeping the same leg published, so the FSM
stays parked on that corner) and only then publishes the next leg.  The old version advanced on a TIMER
(`leg_len / --leg-speed`), which is what the dwell was measured against on 2026-10-15: three sorties with
the hold on, **1 of 4 corners hovered >= 5 s in each** (`tools/dwell-audit.py`), and the two most recent
ones flew ONE leg and landed, because the FSM reads the published path once and a one-leg message means
"two corners to visit".  A timer can only ever say where the aircraft OUGHT to be; the arrival gate is the
only thing that can say where it IS, and `--leg-timeout-s` bounds the wait so a missed arrival cannot hang
the mission.  The consumer half is `fsm/mission_fsm.py::legs()`, which keeps walking its corner list and
accepts the next leg mid-mission.

usage: planner/rectangle_source.py [--z 0.8] [--rate 4] [--speed 0.25]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys

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
    """Walk the closed polygon, emitting points every `spacing` metres."""
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


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(here, "..", "worlds", "pillars.yaml"))
    ap.add_argument("--topic", default="/planner/traj")
    ap.add_argument("--z", type=float, default=None, help="override the mission altitude, m")
    ap.add_argument("--spacing", type=float, default=0.05, help="path point spacing, m")
    ap.add_argument("--rate", type=float, default=4.0, help="republish rate, Hz")
    ap.add_argument("--hold-s", type=float, default=-1.0,
                    help="HOVER THIS LONG AT EVERY CORNER: the producer then publishes one LEG per message "
                         "and holds each corner this long after ARRIVING at it (measured on --odom), so the "
                         "consumer holds there.  -1 (default) = use the world file's mission_a.hold_s; "
                         "0 = one continuous path (old)")
    ap.add_argument("--leg-speed", type=float, default=0.30,
                    help="NO LONGER PACES ANYTHING (kept because the log line that used it is how the "
                         "timed version was read): the legs advance on ARRIVAL.  It is still used to PRINT "
                         "the flight time a leg would take at this speed, next to the measured one.")
    ap.add_argument("--odom", default="/lio/robo/odom",
                    help="the vehicle's own pose, L frame, ~200 Hz (that topic carries ONLY the pose: its "
                         "twist is identically zero).  Converted to W with the calibration below; this is "
                         "what 'the vehicle has arrived at the corner' is measured from")
    ap.add_argument("--arrive-m", type=float, default=0.10,
                    help="arrival gate, horizontal, m.  From the measurement, not from taste: the corner "
                         "arrival residuals on RECT1-noego were -42.6 / +71.0 / +1.8 / -51.8 mm, so the "
                         "gate has to be ABOVE the worst of them (71 mm) or the arrival that the FSM "
                         "considers arrived is not the one this node does.  0.10 m = the measured worst "
                         "case + 29 mm, and 1.67x the FSM's own 0.06 m gate (so this node cannot lead the "
                         "consumer into the next leg before the consumer has settled)")
    ap.add_argument("--leg-start-m", type=float, default=0.30,
                    help="a leg's clock arms when the vehicle is within this of the leg's START corner.  "
                         "Needed because the FIRST leg starts at corner 1, which is 0.64 m from the pad: "
                         "the aircraft spends ~25 s of preflight, takeoff and hold before it flies there, "
                         "and a timeout armed at publish time would fire on a healthy mission")
    ap.add_argument("--leg-timeout-s", type=float, default=20.0,
                    help="SAFETY NET: if the armed flight clock runs this long without an arrival, say so "
                         "with a NAMED reason and advance anyway (a missed arrival must not hang the "
                         "mission).  20 s < the FSM's own 25 s per-corner gate (mission_fsm.py "
                         "GATES['leg_timeout_s']), so this fires FIRST and names the cause; the longest leg "
                         "here is 2.0 m and takes 4-9 s at the measured 0.22-0.50 m/s")
    ap.add_argument("--empty-mark-s", type=float, default=2.0,
                    help="after the last leg's dwell, publish an EMPTY Path for this long: that is how the "
                         "source says 'there are no more legs', instead of the FSM having to guess it with "
                         "a timeout")
    ap.add_argument("--calib-file", default="",
                    help="the MEASURED L->W calibration -- the SAME file the FSM converts with.  "
                         "Without it this node publishes the path using the PREDICTED transform, and if "
                         "the FSM then converts L->F with a measured one, the commanded points are "
                         "offset by the difference between the two.")
    # ---- THE COMMAND STREAM, THE PLAN BOX'S OUTPUT (refactor-bridge-plan.md section 3.3) -------------
    # A SHADOW IN THIS ROUND: published beside the Path, subscribed by nobody, so the flown chain is
    # unchanged (tools/test-cmd-stream.py proves that, it does not assert it).
    ap.add_argument("--cmd-topic", default="/plan/cmd",
                    help="the PLAN box's command stream, quadrotor_msgs/msg/PositionCommand.  Name and "
                         "type are fixed by the target interface: it is what ego's own traj_server "
                         "publishes (traj_server.cpp:244-250) and what the BRIDGE will consume.  NOTHING "
                         "SUBSCRIBES to it yet -- it runs beside /planner/traj so the flown chain stays "
                         "byte-for-byte what it was.")
    ap.add_argument("--cmd-rate", type=float, default=100.0,
                    help="the command stream's publishing rate, Hz.  ego's traj_server runs at 100 Hz "
                         "(a 10 ms wall timer, traj_server.cpp:248-250); the WIRE rate stays 50 Hz, "
                         "because message 84 at 50 Hz is the flown configuration and ARCHITECTURE.md "
                         "251-258 explains why 50 is the firmware's own ceiling.  Publisher <= 100, "
                         "consumed at 50: this switch is the publisher half only.")
    ap.add_argument("--no-cmd-stream", action="store_true",
                    help="publish ONLY the Path, exactly as before this switch existed.  The A/B the "
                         "shadow claim is proved with: same seed, same calibration, byte-identical "
                         "Path message sequence with and without the stream.")
    ap.add_argument("--cmd-id", type=int, default=1,
                    help="trajectory_id.  Its ID number starts from 1 (PositionCommand.msg:20), "
                         "allowing a consumer to compare it with 0 -- which is how ego's own message "
                         "documents 'no trajectory yet'.")
    args = ap.parse_args()

    corners, mission = load_square(args.world)
    if args.z is not None:
        corners[:, 2] = args.z
    hold_s = args.hold_s if args.hold_s >= 0.0 else float(mission.get("hold_s", 0.0) or 0.0)
    path_w = densify(corners, args.spacing)
    legs_w = None
    if hold_s > 0.0:
        # OPEN polylines, one per edge: `densify()` closes the loop it is given (it appends corners[0]), so
        # reusing it here produced OUT-AND-BACK legs (c1 -> c2 -> c1) and the aircraft never left the first
        # corner: MEASURED RECT2-hold5 -- "LEG 0 -> corner 1: 4.000 m" for a 2.000 m edge, N extent 0.074 m
        # over the whole route, and no "HOLD at corner 3" line at all.
        legs_w = []
        for i in range(len(corners)):
            a = np.asarray(corners[i], dtype=float)
            b = np.asarray(corners[(i + 1) % len(corners)], dtype=float)
            n = max(2, int(float(np.linalg.norm(b - a)) / max(args.spacing, 1e-3)) + 1)
            legs_w.append(np.linspace(a, b, n))
    print("rectangle_source: square in W (ENU, origin = pad):")
    for i, c in enumerate(corners):
        print("   corner %d  E %+6.3f  N %+6.3f  U %+6.3f" % (i, c[0], c[1], c[2]))
    print("   %d path points at %.0f mm spacing, %d corners"
          % (len(path_w), args.spacing * 1000, len(corners)))

    # W -> L.  MEASURED 2026-10-13, and it cost 51 mm: this node used the PREDICTED transform while the
    # FSM converted L->F with the MEASURED one, and the two disagree in t (predicted t_N = +50 mm, from
    # the sensor's 0.05 m lever arm, against a measured t_N = -4 mm).  The commanded corners therefore
    # sat 51 mm SOUTH of the intended ones in every flight, the aircraft flew faithfully to them, and the
    # box looked translated in N -- in the command, not in the aircraft.  The source now takes the same
    # file the FSM takes, and prints the difference either way so a mismatch cannot be silent again.
    predicted = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
    calib = predicted
    if args.calib_file and os.path.exists(args.calib_file):
        with open(args.calib_file) as fh:
            d = json.load(fh)
        calib = frames.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"]))
        print("   using the MEASURED calibration from %s (%s)"
              % (args.calib_file, d.get("yaw_deg", float("nan"))))
    else:
        print("   no --calib-file: using the PREDICTED calibration"
              " (the FSM must use the predicted one too, or the commands will be offset)")
    dt = calib.t_WL - predicted.t_WL
    print("   measured - predicted t = [%+.4f %+.4f %+.4f] m  (|%.1f| mm)"
          % (dt[0], dt[1], dt[2], float(np.linalg.norm(dt)) * 1e3))
    R_LW = calib.R_WL.T
    path_l = (R_LW @ (path_w - calib.t_WL).T).T
    legs_l = None if legs_w is None else [(R_LW @ (leg - calib.t_WL).T).T for leg in legs_w]
    # what the FSM will compute back, printed here so the round trip is visible in the log
    back = (calib.R_WL @ path_l.T).T + calib.t_WL
    err = np.linalg.norm(back - path_w, axis=1)
    print("   L<-W: yaw %+.3f deg, t = [%+.4f %+.4f %+.4f] m;  worst W->L->F round-trip error %.6f mm"
          % (math.degrees(math.atan2(R_LW[1, 0], R_LW[0, 0])), -calib.t_WL[0], -calib.t_WL[1],
             -calib.t_WL[2], float(err.max()) * 1e3))
    if legs_l is not None:
        print("   corner hold : %.1f s dwell at each corner, ONE LEG PER MESSAGE, advanced ON ARRIVAL "
              "(gate %.3f m) -- the vehicle's pose comes from %s, converted L->W with the SAME calibration "
              "printed above.  The leg's flight clock arms within %.2f m of the leg's START corner, and "
              "%.1f s without an arrival prints a NAMED reason and advances anyway."
              % (hold_s, args.arrive_m, args.odom, args.leg_start_m, args.leg_timeout_s))

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import PoseStamped

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("rectangle_source")
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Path, args.topic, qos)
    # THE COMMAND STREAM'S PUBLISHER.  Same QoS shape as the Path's: depth 1, TRANSIENT_LOCAL, so a
    # consumer that joins late still finds the newest sample -- which is what the BRIDGE will do (it
    # starts after the plan box in the F2 layout).  Its own rate, not the Path's: ego's timer is 10 ms.
    cmd_pub = None
    CmdMsg = None
    if not args.no_cmd_stream:
        try:
            from quadrotor_msgs.msg import PositionCommand as CmdMsg
            cmd_pub = node.create_publisher(CmdMsg, args.cmd_topic, qos)
        except ImportError as exc:
            # NOT a silent downgrade: the stream is the interface this refactor is moving to, so a
            # missing message package has to be visible, and the Path keeps the mission alive meanwhile.
            print("cmd stream DISABLED: cannot import quadrotor_msgs/msg/PositionCommand (%s)."
                  "  Build the vendored package: see ros_ws/src/quadrotor_msgs/README.md" % exc,
                  flush=True)
            cmd_pub = None

    def make_path(pts):
        m = Path()
        m.header.frame_id = "lio_world"
        for i, (x, y, z) in enumerate(pts):
            ps = PoseStamped()
            ps.header.frame_id = "lio_world"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            # yaw per point: the tangent of the path, which is what the FSM holds
            nxt = pts[(i + 1) % len(pts)]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            m.poses.append(ps)
        return m

    msgs = [make_path(leg) for leg in legs_l] if legs_l is not None else [make_path(path_l)]
    # THE SOURCE'S OWN "THERE IS NOTHING MORE TO FLY".  An EMPTY Path needs no new message type and no
    # change to the interface's meaning: a path with no poses is a trajectory with no points.  It is
    # published after the last leg's dwell so the consumer can END THE TOUR ON A STATEMENT rather than on a
    # guessed timeout (it honours both: see --next-leg-wait-s).
    empty_msg = Path()
    empty_msg.header.frame_id = "lio_world"

    import time
    print("publishing %d poses on %s at %.1f Hz (frame lio_world)" % (len(msgs[0].poses), args.topic,
                                                                     args.rate), flush=True)
    period = 1.0 / args.rate
    if legs_l is not None:
        print("corner holds ON: %d legs, %.1f s dwell at each corner, PACED BY ARRIVAL on %s "
              "(gate %.3f m); timeout %.1f s" % (len(msgs), hold_s, args.odom, args.arrive_m,
                                                 args.leg_timeout_s), flush=True)
    else:
        print("corner holds OFF: one continuous path, republished (no arrival pacing)", flush=True)

    # the vehicle's own pose in L.  BEST_EFFORT/depth 200 is what every other consumer of this topic uses
    # (tools/record-odom.py, fsm/mission_fsm.py): the LIO publishes it as a sensor stream.
    best_effort = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
    odom = {"p_l": None, "n": 0, "wall": 0.0}

    def on_odom(m):
        odom["p_l"] = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        odom["n"] += 1
        odom["wall"] = time.monotonic()

    node.create_subscription(Odometry, args.odom, on_odom, best_effort)

    def here_w():
        """The vehicle in W.  The SAME calibration this node publishes with, read the other way.

        Every commanded point travels W -> L as `R_LW @ (p - t_WL)`; this is `R_WL @ p_l + t_WL` with
        `R_WL = R_LW.T`, i.e. the inverse of exactly that expression -- not a second convention.  The
        first version of the corner hold compared an L-frame pose with a W-frame world and would have
        measured a 90 deg rotated error, which is the same defect `tools/record-traj.py` had.
        """
        if odom["p_l"] is None:
            return None
        return calib.R_WL @ odom["p_l"] + calib.t_WL

    def publish(msg):
        stamp = node.get_clock().now().to_msg()
        msg.header.stamp = stamp
        for ps in msg.poses:
            ps.header.stamp = stamp
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)

    def leg_len_w(i):
        return float(np.sum(np.linalg.norm(np.diff(np.asarray(legs_w[i])[:, :2], axis=0), axis=1)))

    if legs_l is None:
        # ---- hold OFF: one continuous path, republished forever.  The historical behaviour, unchanged.
        while rclpy.ok():
            publish(msgs[0])
            time.sleep(period)
        return 0

    # ---- hold ON: one LEG per message, and the next leg is published only after an ARRIVAL -------------
    #
    # THE PACING MOVED INTO `planner/cmd_stream.py::TourPacer` AND NOTHING ELSE DID.  The decision -- when
    # a leg's clock arms, when the vehicle has arrived, how long the dwell lasts, when the tour is over --
    # is now ONE object, because the command stream has to read the same decision the Path is published
    # from.  Two pacing rules is how the producer's gate and the consumer's gate came to be two different
    # numbers in the first place (refactor-bridge-plan.md section 2(b) item 2).  The LOG LINES this class
    # emits are unchanged, character for character, because those lines are the record of which leg armed
    # when and a flight's log is evidence.
    n_leg = len(msgs)
    pacer = cmd_stream.TourPacer(corners, n_leg, hold_s, args.arrive_m, args.leg_start_m,
                                 args.leg_timeout_s, time.monotonic(),
                                 log=lambda s: print(s, flush=True), leg_len_w=leg_len_w,
                                 odom_label=args.odom, empty_mark_s=args.empty_mark_s,
                                 leg_speed=args.leg_speed)

    # ---- THE COMMAND STREAM (the PLAN box's output; a SHADOW in this round) ---------------------------
    #
    # `stream.sample()` is a pure function of the PACER'S state, so the two outputs read one decision:
    # the Path is `msgs[pacer.i_leg]`, and the command stream is the leg's end corner and the leg's yaw
    # while flying, or the corner with velocity 0 while it dwells.  The rate is the stream's own
    # (`CmdStream.due` / `.advance`), so the Path's 4 Hz republish and the stream's 100 Hz do not have to
    # be the same number for the stream to be correctly clocked.
    stream = None
    if cmd_pub is not None:
        stream = cmd_stream.CmdStream(pacer, args.cmd_rate, legs_l, trajectory_id=args.cmd_id,
                                      log=lambda s: print(s, flush=True))
        print("command stream: %s on %s at %.1f Hz, frame %s, %d legs -- the PLAN box's own output."
              "  NOTHING SUBSCRIBES to it in this round: it runs BESIDE /planner/traj so the flown chain"
              "  is byte-for-byte unchanged.  kx/kv are sent as ZERO (message 84 has no gain fields) and"
              "  the dwell is content: position = the corner, velocity = 0."
              % (CmdMsg.__name__, args.cmd_topic, args.cmd_rate, cmd_stream.FRAME_L, len(legs_l)),
              flush=True)

    def publish_cmd(now):
        """One command sample, if the stream's own clock says it is due.  Silent when disabled."""
        if stream is None:
            return
        if stream.next_t is None:
            stream.next_t = now          # the FIRST sample is emitted at once; the clock starts there
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

    while rclpy.ok():
        now = time.monotonic()
        if pacer.done:
            # THE EMPTY PATH IS THE END OF THE TOUR, and it is published for --empty-mark-s so a consumer
            # that is still settling at the last corner cannot miss it.  After that the node stops
            # publishing and stays alive: the sample is LATched (TRANSIENT_LOCAL depth 1), so a consumer
            # that joins later still finds it, and the sortie's launcher owns this process.
            #
            # THE COMMAND STREAM'S VERSION OF THE SAME STATEMENT IS A FIELD, not an empty message:
            # trajectory_flag = TRAJECTORY_STATUS_COMPLETED, published for the same --empty-mark-s.
            if now - pacer.t_done < args.empty_mark_s:
                publish(empty_msg)
            else:
                rclpy.spin_once(node, timeout_sec=period)
            publish_cmd(now)
            time.sleep(period)
            continue
        pacer.odom_n = odom["n"]
        pacer.odom_wall = odom["wall"]
        pacer.step(here_w(), now)
        publish_cmd(now)
        publish(msgs[pacer.i_leg])
        time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
