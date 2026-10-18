#!/usr/bin/env python3
"""THE FCU'S OWN ESTIMATE, FED TO EGO INSTEAD OF THE RAW LIO.

*** CORRECTED 2026-10-16 (round 79), BECAUSE THE FIRST VERSION OF THIS FILE HAD THE WRONG SOURCE ***

What this file fed ego until round 79 was `/sim/truth/odom`, and its docstring called that "the FCU's own pose".
**It is not.**  `/sim/truth/odom` is republished by `sim/truth_rx.py` from the SITL **truth tap**, which
`ardupilot/libraries/SITL/SIM_Aircraft.cpp:708-716` describes in its own words:

    "Truth tap: one ASCII line per model step with **the simulator's own state**, for external estimators (the
     LIO) that need a ground truth far finer than SIMSTATE."

So that channel is the SIMULATOR'S MODEL STATE -- ground truth.  It never passes through the EKF, it cannot be
fed to a planner on a real aircraft (the channel does not exist there), and the rationale written here ("the
EKF's innovation gates stand between the sensor and the planner") was therefore FALSE.

WHAT IT CONVERTS NOW, and this is the owner's actual request -- "ego should read what the FLIGHT CONTROLLER
believes": the FCU's **MAVLink `LOCAL_POSITION_NED` (#32)**, which our bridge already receives (it requests it at
`bridge/mavlink_impl.py:93` and parses it at `:115-119`) and now republishes as a ROS topic
(`bridge_node.py --fcu-odom-topic`, W frame = ENU, z up, pose AND velocity).  Both of ego's odometry inputs point
at this node's output -- the map reads `pose.pose.position` only, the FSM reads pose AND `twist.twist.linear` --
so the planner and the controller close their loops on ONE state: the estimate the flight controller is actually
controlling in.

WHY IT MATTERS EVEN THOUGH THE TWO AGREE TODAY: measured on this stack the LIO and the FCU's estimate differ by
**6.58 mm** (metric A1), so this is not an accuracy change.  It is a FAILURE change: an LIO that flies away moves
a planner fed directly from it immediately, while an estimate moves only as far as the EKF's innovation gates
allow -- and the supervisor's geofence and climb gates watch the FCU, not the LIO.

THE CONVERSION, unchanged and shared with the rest of the repo (`common/frames.Calib`):

    p_L = R_WL^T (p_W - t_WL)      v_L = R_WL^T v_W      q_L = R_WL^T q_W

usage: fcu_pose_feed.py --calib-file _stage/calib.json [--in /fcu/odom] [--out /ego/odom_fcu]
       fcu_pose_feed.py --in /sim/truth/odom      # the SITL truth tap: a TEST channel, not the FCU estimate
       fcu_pose_feed.py --selftest
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import frames                                                       # noqa: E402


def pose_w_to_l(p_w, v_w, q_w, calib):
    """W -> L for a pose, its velocity and its orientation, in ONE place.

    The rotation is `R_WL^T` because `R_WL` maps L->W (`common/frames.py:l_to_wire` uses it the other way), and a
    VELOCITY is a difference of two positions: `t_WL` cancels, so applying it to the velocity would add 4 mm and
    a frame bug for nothing (`frames.py:191-196` says the same thing for the wire direction).
    """
    p_l = calib.R_WL.T @ (np.asarray(p_w, dtype=float) - calib.t_WL)
    v_l = calib.R_WL.T @ np.asarray(v_w, dtype=float)
    R_w = frames.R_from_q(np.asarray(q_w, dtype=float))
    q_l = frames.q_from_R(calib.R_WL.T @ R_w)
    return p_l, v_l, q_l


def _selftest() -> int:
    import json
    bad = []
    cal = json.load(open("/mnt/d/distill_self/_stage/calib.json"))
    calib = frames.Calib(R_WL=np.array(cal["R_WL"], dtype=float), t_WL=np.array(cal["t_WL"], dtype=float))
    # a point in L, pushed to W and back, must return
    p_l = np.array([1.5, -2.25, 1.75])
    p_w = (calib.R_WL @ p_l) + calib.t_WL
    back, _v, _q = pose_w_to_l(p_w, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], calib)
    print("   L -> W -> L: %s -> %s" % (np.round(p_l, 6), np.round(back, 6)))
    if not np.allclose(p_l, back, atol=1e-9):
        bad.append("the round trip does not close: %s" % np.round(back - p_l, 9))
    # a velocity must NOT get the translation (the defect this function exists to prevent)
    v_w = np.array([0.4, -0.3, 0.1])
    _p, v_l, _q = pose_w_to_l([0.0, 0.0, 0.0], v_w, [1.0, 0.0, 0.0, 0.0], calib)
    print("   a velocity rotated only: |v| %.6f -> %.6f (must be unchanged)" % (np.linalg.norm(v_w),
                                                                               np.linalg.norm(v_l)))
    if abs(np.linalg.norm(v_w) - np.linalg.norm(v_l)) > 1e-12:
        bad.append("the velocity's magnitude changed: %.9f" % (np.linalg.norm(v_l) - np.linalg.norm(v_w)))
    # yaw of the identity orientation must equal the frame offset's own yaw
    _p, _v, q_l = pose_w_to_l([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], calib)
    print("   identity orientation in W -> L gives yaw %.2f deg (the frame offset, not zero)"
          % np.degrees(frames.yaw_from_R(frames.R_from_q(q_l))))
    if bad:
        print("\nFCU POSE FEED SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("\nFCU POSE FEED SELFTEST PASSED: the W->L conversion, one place, both directions")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--in", dest="in_topic", default="/fcu/odom",
                    help="the FCU's OWN estimate (bridge --fcu-odom-topic).  `/sim/truth/odom` is the SITL "
                         "simulator's model state, a TEST channel that does not exist on hardware")
    ap.add_argument("--out", dest="out_topic", default="/ego/odom_fcu")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    import json                                                                  # noqa: E402
    import rclpy                                                                 # noqa: E402
    from nav_msgs.msg import Odometry                                            # noqa: E402
    from rclpy.node import Node                                                  # noqa: E402
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy           # noqa: E402

    d = json.load(open(args.calib_file))
    calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))
    print("fcu_pose_feed: %s -> %s, W->L with R_WL from %s" % (args.in_topic, args.out_topic, args.calib_file))
    print("fcu_pose_feed: %s" % calib.summary())

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                                            # noqa: BLE001
        rclpy.init(args=[])
    node = Node("fcu_pose_feed")
    # the tap publishes BEST_EFFORT (sim/truth_rx.py:291-295): a RELIABLE subscriber cannot match it
    sub_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    # ego's own subscribers are plain depth-10 RELIABLE ("create_subscription<Odometry>(..., 10)")
    pub_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(Odometry, args.out_topic, pub_qos)
    n = {"in": 0, "out": 0}
    last_log = [0.0]

    def on_odom(m: Odometry):
        n["in"] += 1
        p_w = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
        v_w = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        q_w = [m.pose.pose.orientation.w, m.pose.pose.orientation.x, m.pose.pose.orientation.y,
               m.pose.pose.orientation.z]
        p_l, v_l, q_l = pose_w_to_l(p_w, v_w, q_w, calib)
        o = Odometry()
        o.header = m.header
        o.header.frame_id = "lio_world"        # ego reads no frames, but OUR recorders do
        o.child_frame_id = "fcu_base_link"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(p_l[0]), float(p_l[1]),
                                                                                 float(p_l[2]))
        o.pose.pose.orientation.w, o.pose.pose.orientation.x = float(q_l[0]), float(q_l[1])
        o.pose.pose.orientation.y, o.pose.pose.orientation.z = float(q_l[2]), float(q_l[3])
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = (float(v_l[0]), float(v_l[1]),
                                                                                  float(v_l[2]))
        o.twist.twist.angular = m.twist.twist.angular
        pub.publish(o)
        n["out"] += 1
        now = node.get_clock().now().nanoseconds * 1e-9
        if now - last_log[0] >= 1.0:
            last_log[0] = now
            print("fcu_pose_feed: in %d out %d | p_L (%7.3f,%7.3f,%7.3f) v_L (%6.3f,%6.3f,%6.3f)"
                  % (n["in"], n["out"], p_l[0], p_l[1], p_l[2], v_l[0], v_l[1], v_l[2]), flush=True)

    node.create_subscription(Odometry, args.in_topic, on_odom, sub_qos)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print("fcu_pose_feed: final in %d out %d" % (n["in"], n["out"]), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
