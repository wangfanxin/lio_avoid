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

WHAT IT DOES ADD, since round 8, IS MEMORY (`--window`, and 0 keeps the old behaviour exactly).  ego's map
does not accumulate -- `cloudCallback` resets its local box and then writes `occupancy_buffer_inflate_[idx]=1`
per accepted point (`grid_map.cpp:835,910`), with no ESDF, no ray-casting and no probability update anywhere in
that variant -- so the map is a SINGLE-SCAN SHELL one voxel thick.  MEASURED with PATCH DIAG/2 on the diagonal
crossing (`DIAG8`): it held **0 % of the pillar the vehicle was inside** while holding 31-39 cells of the
pillars 1.1-1.6 m away, and every observation with a pillar's centre within 0.50 m found zero cells; the
planner's own hard collision check agreed with the map, which is why five crossings were caught by the C2 row
while the planner logged no collision at all.  Unioning the last few seconds of scans restores the memory the
planner assumes it has, using only our code, and it is valid because this cloud is in the WORLD frame: in a
static world the union of consecutive scans is the swept view, not a smear.  ~3 s is the value the geometry
asks for (measured closing speed on the diagonal ~0.4 m/s, so 3 s spans the whole approach the map forgets).

THE QOS, and why the subscription is BEST_EFFORT.  Super-LIO publishes `/lio/cloud_world` through
`create_publisher<PointCloud2>("/lio/cloud_world", 10)` (`ROSWrapper.cpp:338-340`) — the integer-depth
overload, i.e. RELIABLE.  A RELIABLE subscriber connects to that, but it would SILENTLY NEVER CONNECT to a
best-effort publisher if that ever changed, and ego's own subscription is RELIABLE
(`grid_map.cpp:168-172`).  So: subscribe BEST_EFFORT (compatible with either publisher) and publish
RELIABLE (which is what ego requires).  The frame is passed through UNCHANGED: the source labels it "world",
ego never reads `header.frame_id`, and L IS that frame numerically — renaming it would be a claim that
something was converted.

usage: tools/../planner/cloud_relay.py [--voxel 0.10] [--rate 5.0] [--window 0.0|3.0] [--selftest]
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


def self_blind(points, centre, radius):
    """Drop every point within `radius` of the vehicle.

    WHY: ego's hard collision check treats ITS OWN POSITION inside the inflated map as fatal --
    `check_collision_and_rebound()` scans backwards for a free control point, and when it does not find one it
    prints `ERROR! the drone is in obstacle. This should not happen.` and plans from an occupied start
    (`bspline_optimizer.cpp:1341-1345`).  MEASURED with two cells of inflation (`DIAG13`): **112 729** of those,
    an FSM that stops, a vehicle that sinks, lands, and then sits in the ground's own marked layer for ever --
    the run hung.  Two cells is otherwise the only way to give the C2 row its 0.20 m footprint, because a free
    cell one voxel out from a marked cell is only about 0.15 m from a pillar surface (measured: the plan's 1st
    percentile plateaus at +0.12 m).

    A clear bubble around the machine is what a real sensor has anyway (minimum range, self-returns), and it is
    the map-side statement of `the robot is never its own obstacle`.  The cost is honest and bounded: nothing
    closer than `radius` can be marked, so the radius must stay under the distance at which the window's older
    scans -- which are NOT dropped, since they were taken from a different position and are already far -- still
    hold the obstacle.  Default 0.0 = off, i.e. every earlier round's behaviour exactly.
    """
    if not radius or centre is None:
        return list(points), 0
    cx, cy, cz = centre
    out, dropped = [], 0
    r2 = radius * radius
    for p in points:
        dx, dy, dz = p[0] - cx, p[1] - cy, p[2] - cz
        if dx * dx + dy * dy + dz * dz < r2:
            dropped += 1
        else:
            out.append(p)
    return out, dropped


class VoxelWindow:
    """THE LAST `seconds` OF DECIMATED SCANS, unioned by voxel key.

    WHY THIS EXISTS, and it is the whole reason the relay may touch the cloud at all.  ego's map does not
    accumulate: `GridMap::cloudCallback` calls `resetBuffer(camera_pos +- local_update_range)` and then writes
    `occupancy_buffer_inflate_[idx] = 1` per accepted point (`grid_map.cpp:835,910`) -- a DIRECT WRITE, with no
    ESDF, no ray-casting and no probability update anywhere in that variant.  The map is therefore a
    SINGLE-SCAN SHELL, one voxel thick, and MEASURED on the diagonal crossing (`DIAG8`, PATCH DIAG/2, which
    dumps the same array the planner reads) it held **0 % of the pillar the vehicle was inside** while holding
    31-39 cells of the pillars 1.1-1.6 m away, and **every** observation with a pillar's centre within 0.50 m
    found zero cells.

    The planner nonetheless assumes memory: it plans through space its map calls free, and its own hard
    collision check agrees with the map, which is why five crossings were caught by the C2 row while the
    planner logged no collision at all.  A cloud that is a UNION OF RECENT SCANS gives that memory back
    without touching ego: a pillar that is unmarked at 0.4 m was marked at 1.0-1.5 m a second or two earlier,
    and at this mission's closing speed (~0.4 m/s measured from the tap) a 3 s window spans the whole approach.

    It is only valid because this cloud is in the WORLD frame, where a static world's points do not move: the
    union of consecutive scans is the swept view, not a smear.  Deterministic: entries are kept in time order
    and each voxel is overwritten by the newest point in it.
    """

    def __init__(self, seconds: float, voxel: float) -> None:
        self.seconds = float(seconds)
        self.voxel = float(voxel)
        self.scans = []                       # [(t, {key: point}), ...], oldest first

    def add(self, t: float, points) -> dict:
        """Add one decimated scan and return the merged voxel map (newest point wins per voxel)."""
        self.scans.append((t, {self._key(p): p for p in points}))
        if self.seconds > 0.0:
            cut = t - self.seconds
            self.scans = [s for s in self.scans if s[0] >= cut]
        else:
            self.scans = self.scans[-1:]
        merged = {}
        for _, d in self.scans:
            merged.update(d)
        return merged

    def _key(self, p):
        return (int(math.floor(p[0] / self.voxel)), int(math.floor(p[1] / self.voxel)),
                int(math.floor(p[2] / self.voxel)))

    @property
    def factor(self) -> float:
        """How much bigger the union is than its newest scan -- the relay reports it, so the cost is visible."""
        if not self.scans or not self.scans[-1][1]:
            return 1.0
        return len(set().union(*[set(d) for _, d in self.scans])) / float(len(self.scans[-1][1]))


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

    # THE WINDOW, and its two failure modes: growing without bound, and forgetting too early.  A static world
    # must NOT grow (that would inflate the map's write cost every message), a voxel inside the span must
    # survive, and a voxel outside it must not.
    print("      the window, on a world that does not move")
    scan = [(0.00, 0.00, 0.0), (1.00, 0.00, 0.0), (2.00, 0.00, 0.0)]
    w = VoxelWindow(seconds=3.0, voxel=0.10)
    w.add(0.0, scan)
    w.add(1.0, scan)
    merged = w.add(2.0, scan)
    print("      a STATIC scene, seen 3 times     -> %d voxels (one scan: %d)" % (len(merged), len(scan)))
    if len(merged) != len(scan):
        bad.append("a static scene grew to %d voxels from %d" % (len(merged), len(scan)))

    w2 = VoxelWindow(seconds=3.0, voxel=0.10)
    w2.add(0.0, [(5.00, 5.00, 0.0)])
    w2.add(4.1, [(0.00, 0.00, 0.0)])
    m2 = w2.add(4.2, [(0.00, 0.00, 0.0)])
    print("      a voxel last seen 4.2 s ago      -> %d voxel(s) (must be 1: the window is 3 s)" % len(m2))
    if len(m2) != 1:
        bad.append("a 3 s window kept %d voxels; the one from t=0 must be gone" % len(m2))

    w3 = VoxelWindow(seconds=3.0, voxel=0.10)
    w3.add(0.0, [(5.00, 5.00, 0.0)])
    m3 = w3.add(2.0, [(0.00, 0.00, 0.0)])
    print("      a voxel last seen 2.0 s ago      -> %d voxel(s) (must be 2: still inside)" % len(m3))
    if len(m3) != 2:
        bad.append("the window dropped a voxel that is inside its span")

    w0 = VoxelWindow(seconds=0.0, voxel=0.10)
    w0.add(0.0, [(5.00, 5.00, 0.0)])
    m0 = w0.add(1.0, [(0.00, 0.00, 0.0)])
    print("      window 0 s (one scan, as before) -> %d voxel(s) (must be 1)" % len(m0))
    if len(m0) != 1:
        bad.append("window 0 kept %d voxels; it must be exactly one scan" % len(m0))

    w4 = VoxelWindow(seconds=3.0, voxel=0.10)
    w4.add(0.0, [(0.00, 0.00, 0.0), (1.00, 0.00, 0.0)])
    m4 = w4.add(0.5, [(1.05, 0.00, 0.0), (2.00, 0.00, 0.0)])
    print("      two views, one voxel shared      -> %d voxels (must be 3)" % len(m4))
    if len(m4) != 3:
        bad.append("the union across views gave %d voxels, expected 3" % len(m4))

    # THE SELF-BLIND BUBBLE, whose two failure modes are dropping too much and dropping nothing.
    print("      the self-blind bubble")
    near = [(0.10, 0.0, 0.0), (0.40, 0.0, 0.0), (0.60, 0.0, 0.0)]
    keptb, dropb = self_blind(near, (0.0, 0.0, 0.0), 0.45)
    print("      points at 0.10 / 0.40 / 0.60 m       -> %d kept, %d dropped (bubble 0.45 m)"
          % (len(keptb), dropb))
    if len(keptb) != 1 or dropb != 2:
        bad.append("the bubble kept %d and dropped %d; expected 1 and 2" % (len(keptb), dropb))
    if self_blind(near, (0.0, 0.0, 0.0), 0.0)[1] != 0:
        bad.append("a bubble of 0.0 must drop nothing")
    if self_blind(near, None, 0.45)[1] != 0:
        bad.append("with no odometry the bubble must be a pass-through")

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
            self.win = VoxelWindow(args.window, args.voxel)
            self.n_in = self.n_out = self.n_drop = self.n_cloud = 0
            self.n_blind = 0
            self.last_pub = 0.0
            self.last_frame = None
            self.create_timer(5.0, self.report)
            self.get_logger().info("cloud relay %s -> %s: %.3f m voxels, <= %.1f Hz, RELIABLE out, "
                                   "frame passed through unchanged, %s"
                                   % (args.topic_in, args.out, args.voxel, args.rate,
                                      "SINGLE SCAN (no persistence)" if args.window <= 0.0 else
                                      "UNION of the last %.1f s of scans (persistence for a map that has none)"
                                      % args.window))

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
            kept, blinded = self_blind(kept, self.pos if args.self_blind else None, args.self_blind)
            merged = self.win.add(now, kept)      # the union of the last `--window` seconds, by voxel
            out = [merged[k] for k in sorted(merged)]
            self.n_drop += dropped
            self.n_blind += blinded
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
            self.get_logger().info("%d scan(s): %d -> %d points (%.1f%% kept), %d cropped away, %d inside the "
                                   "self-blind bubble (%.2f m), frame %r, odometry %s, window %.1f s "
                                   "(union is %.2fx the newest scan)"
                                   % (self.n_cloud, self.n_in, self.n_out,
                                      100.0 * self.n_out / max(1, self.n_in), self.n_drop, self.n_blind,
                                      args.self_blind, self.last_frame,
                                      "yes" if self.pos else "NO -- crop disabled, forwarding whole scans",
                                      self.win.seconds, self.win.factor))

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
    ap.add_argument("--self-blind", type=float, default=0.0, dest="self_blind",
                    help="drop cloud points within this many metres of the vehicle, so the map cannot mark the "
                         "machine's own position as an obstacle.  0.0 = off (every round up to 13).  The measured "
                         "reason to use it: with two cells of inflation the planner printed 112 729 'the drone "
                         "is in obstacle' and hung (DIAG13), and two cells is the only way to give the C2 row its "
                         "0.20 m footprint.")
    ap.add_argument("--window", type=float, default=0.0,
                    help="seconds of scans to UNION into each message; 0 = one scan, the behaviour every "
                         "earlier round was flown with.  This is the persistence ego's map does not have "
                         "(`grid_map.cpp:835,910`), and the value to fly is ~3 s: measured closing speed on "
                         "the diagonal is ~0.4 m/s, so 3 s spans the whole approach that the map forgets.")
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
