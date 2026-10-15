#!/usr/bin/env python3
"""Record any `nav_msgs/Odometry` (or `geometry_msgs/PoseStamped`) topic to CSV.

Deliberately dumb and complete: every message, with its own stamp, so the analysis afterwards is
offline and repeatable and nothing depends on the live run's timing.

usage: tools/record-odom.py --topic /lio/robo/odom --csv out.csv [--seconds 60]
"""
from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", required=True)
    ap.add_argument("--csv", required=True)
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until Ctrl-C")
    args = ap.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("record_odom")
    qos = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.BEST_EFFORT)
    fh = open(args.csv, "w")
    fh.write("t_us,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,frame_id,child_frame_id\n")
    state = {"n": 0}

    def on_odom(m: Odometry):
        t = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        p, q, v, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear, \
            m.twist.twist.angular
        fh.write("%d,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%s\n" % (
            t, p.x, p.y, p.z, q.w, q.x, q.y, q.z, v.x, v.y, v.z, w.x, w.y, w.z,
            m.header.frame_id, m.child_frame_id))
        state["n"] += 1

    node.create_subscription(Odometry, args.topic, on_odom, qos)
    print("recording %s -> %s (ctrl-c or --seconds to stop)" % (args.topic, args.csv), flush=True)
    t0 = time.monotonic()
    try:
        while args.seconds == 0.0 or (time.monotonic() - t0) < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    fh.close()
    print("recorded %d messages" % state["n"])
    node.destroy_node()
    rclpy.shutdown()
    return 0 if state["n"] > 10 else 1


if __name__ == "__main__":
    sys.exit(main())
