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

ROLLING ACCUMULATION (`--keep-s`, default 15 s).  WHY IT EXISTS (MEASURED 2026-10-14, sorties L1/K1, the
circle): publishing only the CURRENT scan makes the planner blind to pillars that have left the 45 deg field
of view -- ego's occupancy grid forgets them (p_miss) and it then plans STRAIGHT THROUGH them.  The adapter,
which knows the world file, refuses those plans, and the evidence is unambiguous: in sortie L1 the 13
withheld plans were refused with "pillar clearance -0.189 ... -0.230 m" -- 19-23 cm INSIDE a 0.40 m pillar --
while all 51 PUBLISHED plans held the contract (min 0.278 m, median 0.350 m).  The aircraft then cannot move,
so the pillar is never re-observed, and the mission starves: "no new trajectory on /planner/traj within 60 s"
with the vehicle hovering.  A real system does not forget what it saw a second ago; this node now publishes
the union of the last `--keep-s` seconds of registered scans, voxel-downsampled (`--voxel`), which is still
purely sensor data -- the world file is NOT used as a prior.
`--keep-s 0` restores the historical behaviour, so the two can be A/B'd.

usage: tools/scan_relay.py [--in /livox/lidar] [--odom /lio/robo/odom] [--out /cloud_registered]
                           [--keep-s 15] [--voxel 0.12] [--max-points 80000]
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="src", default="/livox/lidar")
    ap.add_argument("--odom", default="/lio/robo/odom")
    ap.add_argument("--out", dest="dst", default="/cloud_registered")
    ap.add_argument("--frame", default="mid360_link", help="the scan's own frame_id")
    ap.add_argument("--keep-s", type=float, default=0.0,
                    help="publish the union of every registered scan from the last this many seconds; "
                         "0 = publish only the current scan.  DEFAULT IS 0 BECAUSE 15 s WAS MEASURED WORSE: "
                         "see the ROLLING ACCUMULATION note below -- the idea is right (the planner must not "
                         "forget a pillar it saw a second ago) but this implementation lost 3x the point "
                         "density to the 0.12 m voxel and smeared 15 s of pose error into the map, and sortie "
                         "M1 (the circle) ended with C2 74.87 % of airborne samples inside the 0.20 m "
                         "footprint, the aircraft stuck against a pillar for 70 s (truth speed median "
                         "0.01 m/s), the LIO drifting 0.77 m in that degenerate pose (A4 datum offset "
                         "N +0.7696 m, against +0.0004 m while parked) and 1 of 12 stops, where the same "
                         "config with keep-s 0 had 0.00 % and 1-5 stops with A1 3-6 mm.  Keep the knob for a "
                         "careful retry (shorter window, finer voxel, or no voxelisation at all).")
    ap.add_argument("--voxel", type=float, default=0.12,
                    help="voxel size for the accumulated cloud, m (0 = keep every point)")
    ap.add_argument("--max-points", type=int, default=80000,
                    help="hard cap on the published cloud; beyond it the oldest voxels are dropped")
    args = ap.parse_args()
    if args.keep_s > 0.0 and args.voxel <= 0.0:
        args.voxel = 0.05           # deduplicate something, or 15 s of 10 Hz scans is millions of points

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
    # the BEST_EFFORT publisher is NOT created at all: a stray
    # advertisement is what kept the planner complaining
    # (MEASURED 2026-10-14, missionB-16)
    # RELIABLE_FOR_PLANNER: the planner subscribes with RELIABLE and a BEST_EFFORT publisher
    # cannot deliver to it (MEASURED 2026-10-14: "offering incompatible QoS. No messages will be
    # sent to it" -- the planner had NO map for every Mission B attempt until this line).
    qos_pub = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(PointCloud2, args.dst, qos_pub)
    state = {"n": 0, "t0": time.monotonic(), "p": None, "R": None, "odom": 0,
             "vox": {}, "last_prune": 0.0, "pts": 0}

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
        if step < 12 or n == 0:
            return
        a = np.frombuffer(raw, dtype=np.uint8)[: n * step].reshape(n, step)
        pts = a[:, :12].copy().view("<f4").reshape(n, 3)
        good = np.isfinite(pts).all(axis=1)
        p_s = pts[good].astype(float)
        # sensor housing -> body -> L, one conversion each, through common/frames.py
        R_bs = frames.R_body_sensor(mount_pitch_deg=45.0)
        lever = frames.SENSOR_LEVER_ARM_B
        p_b = (R_bs @ p_s.T).T + lever
        p_l = (state["R"] @ p_b.T).T + state["p"]
        now = time.monotonic()
        if args.keep_s > 0.0:
            keys = np.floor(p_l / args.voxel).astype(np.int64)
            vox = state["vox"]
            for k, q in zip(map(tuple, keys.tolist()), p_l.tolist()):
                vox[k] = (now, q)
            if now - state["last_prune"] > 2.0:                  # prune and cap every 2 s, not every frame
                state["last_prune"] = now
                vox = state["vox"]
                for k in [k for k, (t, _) in vox.items() if now - t > args.keep_s]:
                    del vox[k]
                if len(vox) > args.max_points:
                    keep = sorted(vox.items(), key=lambda kv: kv[1][0])[-args.max_points:]
                    state["vox"] = dict(keep)
            xyz = np.array([v[1] for v in state["vox"].values()], dtype=np.float32)
        else:
            xyz = p_l.astype(np.float32)
        state["pts"] = len(xyz)
        out = PointCloud2()
        out.header = m.header
        out.header.frame_id = "lio_world"
        out.height = 1
        out.width = len(xyz)
        out.fields = m.fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(xyz)
        out.is_dense = True
        # vectorised packing: x,y,z as float32 + one zero float (the same 16-byte layout as before)
        out.data = np.hstack([xyz.astype("<f4"), np.zeros((len(xyz), 1), "<f4")]).tobytes()
        pub.publish(out)
        state["n"] += 1

    node.create_subscription(Odometry, args.odom, on_odom, qos)
    node.create_subscription(PointCloud2, args.src, on_cloud, qos)
    print("scan_relay: %s + %s -> %s (frame lio_world, raw points, keep_s %.1f s, voxel %.2f m)"
          % (args.src, args.odom, args.dst, args.keep_s, args.voxel), flush=True)
    last = time.monotonic()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() - last > 5.0:                 # never fail silently
            last = time.monotonic()
            print("  %d scan(s) registered, %d odom message(s) in %.0f s, publishing %d points (%d voxels)%s"
                  % (state["n"], state["odom"], time.monotonic() - state["t0"], state["pts"], len(state["vox"]),
                     "" if state["odom"] else "  (no odometry on %s)" % args.odom), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
