#!/usr/bin/env python3
"""Register the raw lidar scan into the LIO's world frame, so the planner gets obstacle SURFACES.

WHY (MEASURED 2026-10-14): the only cloud this LIO publishes is `/lio/cloud_world`, which is its VOXEL map
(`OctVoxMap init. Resolution: 0.25` in its own start-up log).  A voxel centre on a pillar's surface sits up
to half a voxel inside it, and the measurement says exactly that: the closest published point to a pillar
surface is **-0.244 m**, i.e. inside the pillar.  The planner therefore plans against obstacles ~0.25 m
thinner than the real ones and its plans land 0.306 m from the true surface -- the number that has withheld
every Mission B path.  The cloud is NOT misplaced (a rigid-translation search puts the map within
(+0.02, +0.04) m of the world file), so the defect is resolution, not frame.

This node rebuilds the registered scan the planner wants, from the two things the stack already publishes:
the RAW scan (`/livox/lidar`, in the sensor housing frame, per-point times, 10 Hz) and the LIO's own body
pose (`/lio/robo/odom`, L, 200 Hz).  p_L = p_body + R_LB (R_BS p_S + lever).  No new estimate is introduced:
the pose is the LIO's, the points are the sensor's.

usage: tools/scan_relay.py [--in /livox/lidar] [--odom /lio/robo/odom] [--out /cloud_registered]
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", default="/livox/lidar")
    ap.add_argument("--odom", default="/lio/robo/odom")
    ap.add_argument("--out", dest="dst", default="/cloud_registered")
    ap.add_argument("--frame", default="mid360_link", help="the scan's own frame_id")
    args = ap.parse_args()

    sys.path.insert(0, "/mnt/d/distill_self/lio_avoid")
    from common import frames

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry

    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])
    node = Node("scan_relay")
    qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = node.create_publisher(PointCloud2, args.dst, qos)
    # RELIABLE_FOR_PLANNER: the planner subscribes with RELIABLE and a BEST_EFFORT publisher
    # cannot deliver to it (MEASURED 2026-10-14: "offering incompatible QoS. No messages will be
    # sent to it" -- the planner had NO map for every Mission B attempt until this line).
    qos_pub = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, args.dst, qos_pub)
    state = {"n": 0, "t0": time.monotonic(), "p": None, "R": None, "odom": 0}

    def on_odom(m: Odometry):
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        state["p"] = np.array([p.x, p.y, p.z])
        state["R"] = frames.R_from_q(np.array([q.w, q.x, q.y, q.z]))
        state["odom"] += 1

    def on_cloud(m: PointCloud2):
        if state["p"] is None:
            return
        raw = bytes(m.data)
        step = m.point_step
        n = m.width * m.height
        pts = np.empty((n, 3), dtype=np.float32)
        for i in range(n):
            pts[i] = struct.unpack_from("<fff", raw, i * step)
        good = np.isfinite(pts).all(axis=1)
        p_s = pts[good].astype(float)
        # sensor housing -> body -> L, one conversion each, through common/frames.py
        R_bs = frames.R_body_sensor(mount_pitch_deg=45.0)
        lever = frames.SENSOR_LEVER_ARM_B
        p_b = (R_bs @ p_s.T).T + lever
        p_l = (state["R"] @ p_b.T).T + state["p"]
        out = PointCloud2()
        out.header = m.header
        out.header.frame_id = "lio_world"
        out.height = 1
        out.width = len(p_l)
        out.fields = m.fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(p_l)
        out.is_dense = True
        buf = bytearray()
        for x, y, z in p_l[:, :3]:
            buf += struct.pack("<fff", x, y, z) + b"\x00\x00\x00\x00"
        out.data = bytes(buf)
        pub.publish(out)
        state["n"] += 1

    node.create_subscription(Odometry, args.odom, on_odom, qos)
    node.create_subscription(PointCloud2, args.src, on_cloud, qos)
    print("scan_relay: %s + %s -> %s (frame lio_world, raw points, no voxelisation)"
          % (args.src, args.odom, args.dst), flush=True)
    last = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - last > 5.0:                 # never fail silently
            last = time.monotonic()
            print("  %d scan(s) registered, %d odom message(s) in %.0f s%s"
                  % (state["n"], state["odom"], time.monotonic() - state["t0"],
                     "" if state["odom"] else "  (no odometry on %s)" % args.odom), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
