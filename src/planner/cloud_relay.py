#!/usr/bin/env python3
"""`/lio/cloud_world` -> `/ego/cloud`: the SAME points, at the resolution the map can represent.

WHY THIS EXISTS, with the numbers rather than an argument.  ego's map (`plan_env/grid_map.cpp`) does its
ENTIRE update INLINE in the cloud callback — for every accepted point it writes 27 voxels of
`occupancy_buffer_inflate_` (`:860-885`) — in a callback group it shares with the odometry it needs, and it
accepts only what lies within +-5.5 x +-5.5 x +-4.5 m of the vehicle (`:845-887`, `:832-833`).  MEASURED on
the RECT48 recording: our registered scan arrives at **5.0 Hz** with about **19480 points per scan**
(`cloud.csv` was recorded with `--decimate 40`, 487 points per scan, three scans 0.200 s apart), spanning
x -4.97..5.02, y -5.03..5.02, z -0.11..4.42 m.  An arena of +-6 m fits almost entirely inside ego's accepted
box, so CROPPING barely helps; DECIMATION is what matters, and a 0.10 m voxel grid is exactly the
resolution the map stores anyway (`grid_map/resolution: 0.1`), so the points this drops are points the map
could not have distinguished.

THE RELAY DOES NOT INFLATE.  ego inflates by one voxel itself (`grid_map/obstacles_inflation: 0.099`), and
inflating twice would quietly move every clearance claim.

THE QOS, and why the subscription is BEST_EFFORT.  Super-LIO publishes `/lio/cloud_world` through
`create_publisher<PointCloud2>("/lio/cloud_world", 10)` (`ROSWrapper.cpp:338-340`) — the integer-depth
overload, i.e. RELIABLE.  A RELIABLE subscriber connects to that, but it would SILENTLY NEVER CONNECT to a
best-effort publisher if that ever changed, and ego's own subscription is RELIABLE
(`grid_map.cpp:168-172`).  So: subscribe BEST_EFFORT (compatible with either publisher) and publish
RELIABLE (which is what ego requires).  The frame is passed through UNCHANGED: the source labels it "world",
ego never reads `header.frame_id`, and L IS that frame numerically — renaming it would be a claim that
something was converted.

usage: tools/../planner/cloud_relay.py [--voxel 0.10] [--rate 5.0] [--selftest]
       [--in /lio/cloud_world] [--out /ego/cloud] [--odom /lio/robo/odom]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

FRAME_L = "lio_world"
DEFAULT_IN = "/lio/cloud_world"
DEFAULT_OUT = "/ego/cloud"
DEFAULT_ODOM = "/lio/robo/odom"
#: ego's own acceptance box, from `grid_map.cpp:832-833` (x, y, z half-extents)
ACCEPT_M = (5.5, 5.5, 4.5)


# ------------------------------------------------------------------------------ the arithmetic (no ROS)
def voxel_decimate(points, size):
    """One point per `size` cell: the LAST point seen in each cell wins.

    Deterministic and order-preserving on the input, so the same scan always gives the same cloud -- a
    decimator that picks "any" point makes a flight unreproducible.
    """
    if size <= 0.0:
        return list(points)
    keep = {}
    for p in points:
        key = (int(math.floor(p[0] / size)), int(math.floor(p[1] / size)), int(math.floor(p[2] / size)))
        keep[key] = p
    return [keep[k] for k in sorted(keep)]


def crop_box(points, centre, half):
    """The points ego's map will accept, and how many were dropped for being outside."""
    if centre is None:
        return list(points), 0
    cx, cy, cz = centre
    hx, hy, hz = half
    out, dropped = [], 0
    for p in points:
        if abs(p[0] - cx) <= hx and abs(p[1] - cy) <= hy and abs(p[2] - cz) <= hz:
            out.append(p)
        else:
            dropped += 1
    return out, dropped


def cost_estimate(n_points):
    """Voxel writes ego's update will do, for a cloud of this size: 27 per accepted point."""
    return n_points * 27


def _selftest() -> int:
    bad = []
    print("   the decimator, on a grid whose answer is known by hand")

    # a 0.10 m lattice point survives exactly once, whatever it is repeated
    pts = [(0.0, 0.0, 0.0)] * 50
    out = voxel_decimate(pts, 0.10)
    print("      50 copies of one point        -> %d point(s)" % len(out))
    if len(out) != 1:
        bad.append("50 copies of one point became %d points" % len(out))

    # two points in the SAME 0.10 cell collapse; two in different cells do not
    same = voxel_decimate([(0.001, 0.0, 0.0), (0.099, 0.0, 0.0)], 0.10)
    diff = voxel_decimate([(0.099, 0.0, 0.0), (0.101, 0.0, 0.0)], 0.10)
    print("      0.001 and 0.099 (one cell)    -> %d point(s)" % len(same))
    print("      0.099 and 0.101 (two cells)   -> %d point(s)" % len(diff))
    if len(same) != 1:
        bad.append("two points inside one 0.10 m cell did not collapse")
    if len(diff) != 2:
        bad.append("two points in different 0.10 m cells were collapsed")
    if len(voxel_decimate(pts, 0.0)) != 50:
        bad.append("size 0 must be a pass-through")

    # IDEMPOTENCE, because a relay runs this on every scan and a decimator that keeps shrinking is a leak
    many = [(i * 0.037, (i % 17) * 0.041, (i % 7) * 0.023) for i in range(5000)]
    once = voxel_decimate(many, 0.10)
    twice = voxel_decimate(once, 0.10)
    print("      5000 scattered points         -> %d point(s), and again -> %d" % (len(once), len(twice)))
    if len(once) != len(twice):
        bad.append("the decimator is not idempotent: %d then %d" % (len(once), len(twice)))

    # a real surface must survive: a 6 m x 6 m x 4.5 m wall at 0.02 m spacing
    wall = [(x * 0.02 - 3.0, 2.0, z * 0.02) for x in range(301) for z in range(226)]
    wall_out = voxel_decimate(wall, 0.10)
    want = 61 * 46                       # 6.0/0.1 + 1 by 4.5/0.1 + 1
    print("      a 6.0 x 4.5 m wall, 0.02 m spacing: %d -> %d point(s) (0.1 m lattice: %d)"
          % (len(wall), len(wall_out), want))
    if len(wall_out) != want:
        bad.append("the wall decimated to %d points, not %d" % (len(wall_out), want))

    # the crop is ego's own box, and it is a BOX not a radius -- so all THREE axes are probed.  (The first
    # version of this check probed y at 4.51 m against a 5.5 m half-extent and expected it OUTSIDE; it is
    # inside, the expectation was wrong, and the check failed on its own arithmetic rather than the code's.)
    pts2 = [(0.0, 0.0, 0.0),         # in
            (5.49, 0.0, 0.0),        # in, just inside x
            (5.51, 0.0, 0.0),        # out, just outside x
            (0.0, 5.51, 0.0),        # out, just outside y (the y half-extent is also 5.5)
            (0.0, 0.0, 4.51)]        # out, just outside z (the z half-extent is 4.5)
    inside, dropped = crop_box(pts2, (0.0, 0.0, 0.0), ACCEPT_M)
    print("      crop at +-5.5/+-5.5/+-4.5 m     -> %d kept, %d dropped" % (len(inside), dropped))
    if len(inside) != 2 or dropped != 3:
        bad.append("the crop kept %d and dropped %d; expected 2 and 3" % (len(inside), dropped))
    if crop_box(pts2, None, ACCEPT_M)[1] != 0:
        bad.append("with no odometry the crop must be a pass-through and must not claim to drop anything")

    # WHAT THE RELAY BUYS, on a surface whose answer is known, and ONLY that.  The number that matters for
    # the real scan is the input, which is MEASURED -- 19480 points per scan at 5.0 Hz (RECT48's own
    # recording, see the header) -- and the ratio the relay achieves on it is reported at RUNTIME by the
    # node itself, because a synthetic point distribution cannot stand in for that measurement.
    print("      MEASURED input: 19480 points per scan at 5.0 Hz, i.e. %d voxel writes per scan at ego's"
          % cost_estimate(19480))
    print("                      27 writes per accepted point (grid_map.cpp:860-885)")
    print("      on the known wall the 0.10 m lattice keeps %.1f%% of the points, and the relay prints its"
          % (100.0 * len(wall_out) / len(wall)))
    print("      own achieved ratio on the real scan every 5 s")

    print()
    if bad:
        print("CLOUD RELAY SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("CLOUD RELAY SELFTEST PASSED: one point per 0.10 m cell, idempotent, ego's box respected")
    return 0


# ------------------------------------------------------------------------------ the node
def _ros_main(args) -> int:
    try:
        import rclpy
    except ModuleNotFoundError:
        print("cloud_relay: this half is the ROS 2 relay.  rclpy is not importable here.")
        print("             For the offline arithmetic:  %s --selftest"
              % os.path.relpath(os.path.abspath(__file__)))
        return 2
    import numpy as np
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    from sensor_msgs_py import point_cloud2
    from nav_msgs.msg import Odometry

    class Relay(Node):
        def __init__(self):
            super().__init__("cloud_relay")
            sub_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            pub_qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.pub = self.create_publisher(PointCloud2, args.out, pub_qos)
            self.create_subscription(PointCloud2, args.topic_in, self.on_cloud, sub_qos)
            self.create_subscription(Odometry, args.odom, self.on_odom, 10)
            self.pos = None
            self.n_in = self.n_out = self.n_drop = self.n_cloud = 0
            self.last_pub = 0.0
            self.last_frame = None
            self.create_timer(5.0, self.report)
            self.get_logger().info("cloud relay %s -> %s: %.3f m voxels, <= %.1f Hz, RELIABLE out, "
                                   "frame passed through unchanged"
                                   % (args.topic_in, args.out, args.voxel, args.rate))

        def on_odom(self, m):
            self.pos = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z)

        def on_cloud(self, msg):
            now = time.monotonic()
            if self.last_pub and (now - self.last_pub) < 1.0 / max(1e-9, args.rate):
                return                                     # the rate cap: ego needs 5-10 Hz, not more
            pts = [(float(x), float(y), float(z)) for x, y, z, *_ in
                   point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
            self.n_cloud += 1
            self.n_in += len(pts)
            kept, dropped = crop_box(pts, self.pos if args.crop else None, ACCEPT_M)
            out = voxel_decimate(kept, args.voxel)
            self.n_drop += dropped
            self.n_out += len(out)
            self.last_frame = msg.header.frame_id
            if not out:
                return
            arr = np.asarray(out, dtype=np.float32)
            m = PointCloud2()
            m.header = msg.header                           # the frame is passed through, not renamed
            m.height, m.width = 1, len(out)
            m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate(("x", "y", "z"))]
            m.is_bigendian, m.point_step, m.row_step = False, 12, 12 * len(out)
            m.is_dense = True
            m.data = arr.tobytes()
            self.pub.publish(m)
            self.last_pub = now

        def report(self):
            if not self.n_cloud:
                self.get_logger().warn("no cloud on %s yet (is the LIO up?)" % args.topic_in)
                return
            self.get_logger().info("%d scan(s): %d -> %d points (%.1f%% kept), %d cropped away, frame %r, "
                                   "odometry %s"
                                   % (self.n_cloud, self.n_in, self.n_out,
                                      100.0 * self.n_out / max(1, self.n_in), self.n_drop, self.last_frame,
                                      "yes" if self.pos else "NO -- crop disabled, forwarding whole scans"))

    rclpy.init()
    node = Relay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="the arithmetic, with no ROS and no LIO")
    ap.add_argument("--voxel", type=float, default=0.10, help="decimation cell, m (= ego's map resolution)")
    ap.add_argument("--rate", type=float, default=5.0, help="max output rate, Hz (the source is 5 Hz)")
    ap.add_argument("--crop", type=int, default=1, help="crop to ego's acceptance box (needs odometry)")
    ap.add_argument("--in", dest="topic_in", default=DEFAULT_IN)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--odom", default=DEFAULT_ODOM)
    args = ap.parse_args()
    if args.selftest:
        return _selftest()
    return _ros_main(args)


if __name__ == "__main__":
    sys.exit(main())
