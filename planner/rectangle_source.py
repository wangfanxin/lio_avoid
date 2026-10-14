#!/usr/bin/env python3
"""The rectangle trajectory source: publishes `/planner/traj` for mission A.

This is the *frozen* interface of PLAN.md decision 11 and 4.1: a `nav_msgs/Path` in L, points dense
enough that the FSM can difference it for velocity.  Today the square comes from here; later the
ego-planner arrives as an adapter that publishes the same Path, and nothing in the FSM changes -- which
is the point of building the rectangle this way rather than hard-coding corners in the flight code.

The path is written in W (the world file's frame, ENU, origin at the pad) and converted to L with the
PREDICTED calibration: the vehicle spawns facing North (decision 14), so L's x axis is the body's
forward axis at init, i.e. W's North, and L's origin is the housing's position then
(`Calib.predicted(world_axes="body")`).  What the FSM then MEASURES at preflight is compared against
this prediction and logged -- a mismatch is a finding, not something to smooth over.

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
                    help="HOVER THIS LONG AT EVERY CORNER: the producer then publishes one LEG per message and "
                         "keeps publishing the leg past its flight time, so the consumer holds at the corner. "
                         "-1 (default) = use the world file's mission_a.hold_s; 0 = one continuous path (old)")
    ap.add_argument("--leg-speed", type=float, default=0.30,
                    help="assumed speed used to decide that a leg has been flown (leg_len / this = the wait "
                         "before the corner dwell starts).  The measured route speed on this mission is "
                         "~0.15-0.25 m/s including the corners, so the default is deliberately slow: the "
                         "dwell then begins after the aircraft has almost certainly arrived")
    ap.add_argument("--calib-file", default="",
                    help="the MEASURED L->W calibration -- the SAME file the FSM converts with.  "
                         "Without it this node publishes the path using the PREDICTED transform, and if "
                         "the FSM then converts L->F with a measured one, the commanded points are "
                         "offset by the difference between the two.")
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

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from nav_msgs.msg import Path
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
    msg = msgs[0]

    import time
    print("publishing %d poses on %s at %.1f Hz (frame lio_world)" % (len(msg.poses), args.topic,
                                                                      args.rate), flush=True)
    t0 = time.monotonic()
    period = 1.0 / args.rate
    if len(msgs) > 1:
        print("corner holds ON: %d legs, %.1f s dwell at each corner (assumed leg speed %.2f m/s)"
              % (len(msgs), hold_s, args.leg_speed), flush=True)
    i_leg, leg_len, t_leg, t_hold = 0, None, time.monotonic(), None
    while rclpy.ok():
        if len(msgs) > 1:
            if leg_len is None:
                leg_len = float(np.sum(np.linalg.norm(np.diff(legs_l[i_leg][:, :2], axis=0), axis=1)))
                t_leg, t_hold = time.monotonic(), None
                print("LEG %d -> corner %d: %.3f m, %.1f s of flight then %.1f s of hold"
                      % (i_leg, (i_leg + 1) % len(legs_l), leg_len, leg_len / args.leg_speed, hold_s),
                      flush=True)
            if time.monotonic() - t_leg >= leg_len / args.leg_speed:
                if t_hold is None:
                    t_hold = time.monotonic()
                    print("HOLD at corner %d: dwelling %.1f s" % ((i_leg + 1) % len(legs_l), hold_s), flush=True)
                elif time.monotonic() - t_hold >= hold_s and i_leg < len(msgs) - 1:
                    i_leg += 1
                    leg_len = None
        msg = msgs[i_leg]
        stamp = node.get_clock().now().to_msg()
        msg.header.stamp = stamp
        for ps in msg.poses:
            ps.header.stamp = stamp
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
