#!/usr/bin/env python3
"""End-to-end check of the ego-planner chain WITHOUT the simulator.

It feeds the planner exactly what the flying stack feeds it -- a registered point cloud in L, a body
odometry in L, and a goal in W -- and then measures the `/planner/traj` that comes back out:

    sim/raycast.py  ->  /cloud_registered (L)  ->  grid_map  ->  ego_planner  ->  planning/bspline
                                                                                    |
                                                        planner/ego_adapter.py  <---+
                                                                 |
                                                        /planner/traj (nav_msgs/Path, frame L)

Every criterion prints the number it measured, and the run is only PASS if all of them hold.  The
deliberately wrong configurations (`--mutation`) exist so the criteria can be shown to have power:
a check that passes no matter what is not a check.

  goal-frame       the goal is published in L where the adapter expects W  -> must MISS the goal
  predicted-calib  the adapter uses the PREDICTED W<-L instead of the measured one (STATUS.md row
                   44's 56.3 mm trap)                                     -> must MISS by ~59 mm
  no-cloud         no map at all: the planner sees no pillars                -> must CLIP a pillar

usage:
  tools/ego-offline-test.py --out docs/metrics/ego-offline.txt
  tools/ego-offline-test.py --mutation no-cloud --out /tmp/neg.txt
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "sim"))
sys.path.insert(0, os.path.join(ROOT, "planner"))

from common import frames                      # noqa: E402
import raycast                                 # noqa: E402
from ego_adapter import World, load_calib, w_to_l, l_to_w   # noqa: E402

FRAME_L = "lio_world"


# --------------------------------------------------------------------------- scenario geometry
def scenario_poses(world_path, start_w, goal_w):
    """Start/goal in W, plus the sensor pose that produces the cloud (the 45 deg nose-down mount)."""
    return np.asarray(start_w, float), np.asarray(goal_w, float)


def raycast_cloud(world, cfg, p_body_w, heading_deg, rays, *, pattern_n=20000):
    """One MID-360 scan from a body at `p_body_w` facing `heading_deg`, returned in W.

    The sensor sits at the body's lever arm, pitched 45 deg nose-down; `raycast.cast` is in the world
    file's frame, so the cast happens in W and the caller converts the result to L.  Casting straight
    into L would aim W-frame cylinders at L-frame rays -- the world file is in W, full stop.
    """
    mount = (cfg.get("sensor", {}) or {}).get("mount", {}) or {}
    R_WB = frames.R_WB_from_heading(heading_deg)
    R_WS = R_WB @ raycast.make_mount_rotation(mount)
    lever = np.array([mount.get("x", 0.0), mount.get("y", 0.0), mount.get("z", 0.0)])
    origin = p_body_w + R_WB @ lever
    pts, ranges = raycast.cast(origin, R_WS, rays, world)
    return pts, ranges


# --------------------------------------------------------------------------- the ROS side
class Harness:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from nav_msgs.msg import Path, Odometry
        from geometry_msgs.msg import PoseStamped
        from sensor_msgs.msg import PointCloud2, PointField
        import std_msgs.msg

        self.args = args
        self.rclpy = rclpy
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.node = Node("ego_offline_test")
        self.PointCloud2, self.PointField = PointCloud2, PointField
        self.Odometry, self.Path, self.PoseStamped = Odometry, Path, PoseStamped

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_odom = self.node.create_publisher(Odometry, args.odom_topic, 10)
        self.pub_cloud = self.node.create_publisher(PointCloud2, args.cloud_topic, 10)
        self.pub_goal = self.node.create_publisher(PoseStamped, args.goal_topic, 10)
        self.paths = []
        self.node.create_subscription(Path, args.traj_topic, self.on_path, latched)

    def on_path(self, msg):
        self.paths.append((time.time(),
                           np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                     for p in msg.poses], float),
                           np.array([p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec
                                     for p in msg.poses], float),
                           msg.header.frame_id))

    # ---------------------------------------------------------------- publishers
    def send_odom(self, p_l, q_l):
        m = self.Odometry()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = FRAME_L
        m.child_frame_id = "base_link"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(v) for v in p_l)
        m.pose.pose.orientation.w, m.pose.pose.orientation.x = float(q_l[0]), float(q_l[1])
        m.pose.pose.orientation.y, m.pose.pose.orientation.z = float(q_l[2]), float(q_l[3])
        self.pub_odom.publish(m)

    def send_cloud(self, pts_l):
        m = self.PointCloud2()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = FRAME_L
        m.height, m.width = 1, len(pts_l)
        m.is_bigendian = False
        m.point_step = 16
        m.row_step = 16 * len(pts_l)
        m.is_dense = True
        m.fields = [self.PointField(name=n, offset=4 * i, datatype=self.PointField.FLOAT32, count=1)
                    for i, n in enumerate(("x", "y", "z", "intensity"))]
        buf = np.zeros((len(pts_l), 4), dtype=np.float32)
        buf[:, :3] = np.asarray(pts_l, dtype=np.float32)
        m.data = buf.tobytes()
        self.pub_cloud.publish(m)

    def send_goal(self, p, frame_id):
        m = self.PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = frame_id
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in p)
        m.pose.orientation.w = 1.0
        self.pub_goal.publish(m)


# --------------------------------------------------------------------------- metric
def metrics(paths, world, calib, odom_l, goal_w, args):
    """Every number the run is judged on, each with the window it was measured over."""
    m = {}
    if not paths:
        return {"n_paths": 0}, ["no /planner/traj message arrived"]
    fails = []
    m["n_paths"] = len(paths)
    m["frames"] = sorted({p[3] for p in paths})
    m["n_points_median"] = int(np.median([len(p[1]) for p in paths]))
    m["n_points_min"] = int(min(len(p[1]) for p in paths))

    # republish rate: median interval between messages
    w = np.diff([p[0] for p in paths])
    m["republish_hz_median"] = float(1.0 / np.median(w)) if len(w) else float("nan")
    m["republish_hz_worst"] = float(1.0 / np.max(w)) if len(w) else float("nan")

    # trajectory-time spacing, from the per-point stamps of the LAST message
    st = paths[-1][2]
    dts = np.diff(st)
    m["traj_dt_min"], m["traj_dt_med"], m["traj_dt_max"] = (
        float(dts.min()), float(np.median(dts)), float(dts.max()))
    m["traj_hz_min"] = float(1.0 / dts.max()) if len(dts) else float("nan")

    # geometry over EVERY point of EVERY published path
    allp = np.vstack([p[1] for p in paths])
    clr = world.clearance(allp)
    m["clearance_min"] = float(clr.min())
    m["clearance_p05"] = float(np.percentile(clr, 5))
    m["outside_arena"] = int((~world.inside(allp)).sum())

    # start distance, per message, against the (static) vehicle state
    m["start_d_max"] = float(max(np.linalg.norm(p[1][0] - odom_l) for p in paths))

    # does the LAST path reach the goal?  L -> W, then compare in W (the frame the goal is given in)
    last_w = np.array([l_to_w(calib, q) for q in paths[-1][1]])
    m["path_last_w"] = last_w[-1]
    m["goal_w"] = goal_w
    m["goal_err"] = float(np.linalg.norm(last_w[-1] - goal_w))
    m["goal_err_h"] = float(np.linalg.norm((last_w[-1] - goal_w)[:2]))
    m["path_len"] = float(np.linalg.norm(np.diff(last_w, axis=0), axis=1).sum())
    m["straight_len"] = float(np.linalg.norm(last_w[-1] - last_w[0]))
    m["path_box"] = (last_w[:, 0].min(), last_w[:, 0].max(), last_w[:, 1].min(), last_w[:, 1].max())

    # ---- the criteria
    if m["frames"] != [FRAME_L]:
        fails.append("frame is %s, not ['%s']" % (m["frames"], FRAME_L))
    if m["traj_dt_max"] > 1.0 / args.min_traj_hz + 1e-6:
        fails.append("trajectory-time spacing %.4f s is below the %.0f Hz the contract requires"
                     % (m["traj_dt_max"], args.min_traj_hz))
    if m["republish_hz_worst"] < args.min_republish_hz - 1e-6:
        fails.append("republish rate %.2f Hz is below the %.1f Hz the contract requires"
                     % (m["republish_hz_worst"], args.min_republish_hz))
    if m["clearance_min"] < args.footprint - 1e-9:
        fails.append("minimum pillar clearance %.4f m < the %.2f m footprint"
                     % (m["clearance_min"], args.footprint))
    if m["outside_arena"]:
        fails.append("%d published points are outside the arena" % m["outside_arena"])
    if m["start_d_max"] > args.start_max + 1e-9:
        fails.append("a published path starts %.4f m from the vehicle (> %.2f m)"
                     % (m["start_d_max"], args.start_max))
    if m["goal_err"] > args.goal_tol:
        fails.append("the last path ends %.4f m from the goal (> %.3f m)" % (m["goal_err"], args.goal_tol))
    return m, fails


# --------------------------------------------------------------------------- driver
def planner_cmd(args):
    return ["ros2", "run", "ego_planner", "ego_planner_node", "--ros-args",
            "--params-file", args.params,
            "-r", "odom_world:=%s" % args.odom_topic,
            "-r", "grid_map/odom:=%s" % args.odom_topic,
            "-r", "grid_map/cloud:=%s" % args.cloud_topic]


def adapter_cmd(args):
    c = [sys.executable, os.path.join(ROOT, "planner", "ego_adapter.py"),
         "--calib-file", args.calib_file,
         "--world", args.world,
         "--traj-topic", args.traj_topic,
         "--goal-topic", args.goal_topic,
         "--planner-goal-topic", args.planner_goal_topic,
         "--footprint", str(args.footprint),
         "--start-max", str(args.start_max),
         "--rate", str(args.adapter_rate)]
    if args.mutation == "predicted-calib":
        c += ["--mutation", "predicted-calib"]
    return c


def spawn(cmd, log_path):
    fh = open(log_path, "w")
    p = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
                         cwd=ROOT)
    return p, fh


def kill(p):
    if p is None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
    except Exception:
        try:
            p.terminate()
        except Exception:
            pass
    try:
        p.wait(timeout=6)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=os.path.join(ROOT, "worlds", "forest_all.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--params", default=os.path.join(ROOT, "params", "ego_planner_humble.yaml"))
    ap.add_argument("--start", default="0,0,2.0", help="start pose in W (E,N,U)")
    ap.add_argument("--goal", default=None, help="goal in W (E,N,U); default: mission_b.goal")
    ap.add_argument("--heading-deg", type=float, default=0.0)
    ap.add_argument("--odom-topic", default="/lio/robo/odom")
    ap.add_argument("--cloud-topic", default="/cloud_registered")
    ap.add_argument("--traj-topic", default="/planner/traj")
    ap.add_argument("--goal-topic", default="/goal_pose")
    ap.add_argument("--planner-goal-topic", default="/move_base_simple/goal")
    ap.add_argument("--adapter-rate", type=float, default=4.0)
    ap.add_argument("--duration", type=float, default=25.0, help="seconds to publish and collect")
    ap.add_argument("--cloud-hz", type=float, default=5.0)
    ap.add_argument("--odom-hz", type=float, default=50.0)
    ap.add_argument("--footprint", type=float, default=0.35)
    ap.add_argument("--start-max", type=float, default=0.50)
    ap.add_argument("--goal-tol", type=float, default=0.050)
    ap.add_argument("--min-traj-hz", type=float, default=20.0)
    ap.add_argument("--min-republish-hz", type=float, default=2.0)
    ap.add_argument("--mutation", default="", choices=["", "goal-frame", "predicted-calib", "no-cloud"])
    ap.add_argument("--out", default="")
    ap.add_argument("--no-spawn", action="store_true",
                    help="use nodes that are already running instead of starting them")
    args = ap.parse_args()

    import yaml
    with open(args.world) as fh:
        cfg = yaml.safe_load(fh) or {}
    world = World.load(args.world)
    raw = raycast.World.load(args.world)
    calib = load_calib(args.calib_file)

    start_w = np.array([float(v) for v in args.start.split(",")])
    if args.goal:
        goal_w = np.array([float(v) for v in args.goal.split(",")])
    else:
        g = (cfg.get("mission_b") or {}).get("goal", [3.0, 4.0])
        goal_w = np.array([float(g[0]), float(g[1]),
                           float((cfg.get("mission_b") or {}).get("takeoff_alt", 2.0))])

    print("=== ego-planner offline chain test ===")
    print("world     %s  (%d pillars, arena %s)" % (args.world, len(world.pillars), world.walls))
    print("calib     %s" % calib.summary())
    print("mutation  %s" % (args.mutation or "none"))
    print("start W   [%+.3f %+.3f %+.3f]  heading %.1f deg" % (*start_w, args.heading_deg))
    print("goal  W   [%+.3f %+.3f %+.3f]" % tuple(goal_w))

    # --- the cloud, cast in W and converted to L ------------------------------------------------
    pattern = raycast.default_pattern_path()
    rays, times, src = raycast.load_pattern(pattern, n=20000)
    print("pattern   %s" % src)
    import yaml as _y
    pts_w, ranges = raycast_cloud(raw, cfg, start_w, args.heading_deg, rays)
    R_LW = calib.R_WL.T
    pts_l = (R_LW @ (pts_w - calib.t_WL).T).T
    print("cloud     %d points, range %.2f..%.2f m, in frame %s"
          % (len(pts_l), float(ranges.min()), float(ranges.max()), FRAME_L))

    # --- the vehicle state in L -----------------------------------------------------------------
    R_WB = frames.R_WB_from_heading(args.heading_deg)
    q_w = frames.q_from_R(R_WB)
    R_LB = R_LW @ R_WB
    q_l = frames.q_from_R(R_LB)
    odom_l = w_to_l(calib, start_w)
    print("odom  L   [%+.4f %+.4f %+.4f]" % tuple(odom_l))

    goal_l = w_to_l(calib, goal_w)
    back = l_to_w(calib, goal_l)
    rt_mm = float(np.linalg.norm(back - goal_w)) * 1e3
    print("goal  L   [%+.4f %+.4f %+.4f]   W->L->W round trip %.6f mm" % (*goal_l, rt_mm))

    logdir = os.path.join("/tmp", "ego-offline-%d" % int(time.time()))
    os.makedirs(logdir, exist_ok=True)
    procs = []
    if not args.no_spawn:
        p, _ = spawn(planner_cmd(args), os.path.join(logdir, "planner.log"))
        procs.append(p)
        print("started   planner  pid %d  log %s" % (p.pid, os.path.join(logdir, "planner.log")))
        time.sleep(4.0)
        p, _ = spawn(adapter_cmd(args), os.path.join(logdir, "adapter.log"))
        procs.append(p)
        print("started   adapter  pid %d  log %s" % (p.pid, os.path.join(logdir, "adapter.log")))
        time.sleep(3.0)

    h = Harness(args)
    # --- publish loop ---------------------------------------------------------------------------
    t_end = time.time() + args.duration
    t_cloud = t_odom = t_goal = 0.0
    n_cloud = n_odom = n_goal = 0
    while time.time() < t_end and h.rclpy.ok():
        now = time.time()
        if now >= t_odom:
            t_odom = now + 1.0 / args.odom_hz
            h.send_odom(odom_l, q_l)
            n_odom += 1
        if args.mutation != "no-cloud" and now >= t_cloud:
            t_cloud = now + 1.0 / args.cloud_hz
            h.send_cloud(pts_l)
            n_cloud += 1
        if now >= t_goal:
            t_goal = now + 1.0
            if args.mutation == "goal-frame":
                h.send_goal(goal_l, FRAME_L)          # deliberately the WRONG frame
            else:
                h.send_goal(goal_w, "world_w")
            n_goal += 1
        h.rclpy.spin_once(h.node, timeout_sec=0.005)
    for p in procs:
        kill(p)
    time.sleep(0.5)

    print("\npublished %d odom, %d clouds, %d goals in %.0f s" % (n_odom, n_cloud, n_goal, args.duration))
    m, fails = metrics(h.paths, world, calib, odom_l, goal_w, args)

    lines = []
    lines.append("ego-planner offline chain test")
    lines.append("when            %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    lines.append("world           %s (%d pillars)" % (args.world, len(world.pillars)))
    lines.append("calib           %s" % calib.summary())
    lines.append("mutation        %s" % (args.mutation or "none"))
    lines.append("start W         [%+.3f %+.3f %+.3f] heading %.1f deg" % (*start_w, args.heading_deg))
    lines.append("goal W          [%+.3f %+.3f %+.3f]" % tuple(goal_w))
    lines.append("goal L          [%+.4f %+.4f %+.4f]" % tuple(goal_l))
    lines.append("W->L->W goal round trip   %.6f mm" % rt_mm)
    lines.append("cloud           %d points from the start pose, frame %s, %s"
                 % (len(pts_l), FRAME_L, src))
    lines.append("published       %d odom / %d clouds / %d goals at %.1f / %.1f / 1 Hz"
                 % (n_odom, n_cloud, n_goal, args.odom_hz, args.cloud_hz))
    lines.append("logs            %s" % logdir)
    lines.append("")
    if "n_paths" not in m or m.get("n_paths", 0) == 0:
        lines.append("MEASURED  no /planner/traj message arrived  -> FAIL")
        print("\n".join(lines))
        return 1
    lines.append("MEASURED")
    lines.append("  paths published            %d" % m["n_paths"])
    lines.append("  frame_id                   %s" % m["frames"])
    lines.append("  points per path            %d (min %d)"
                 % (m["n_points_median"], m["n_points_min"]))
    lines.append("  republish rate             %.2f Hz median, %.2f Hz worst"
                 % (m["republish_hz_median"], m["republish_hz_worst"]))
    lines.append("  trajectory-time spacing    min %.4f / median %.4f / max %.4f s  (= %.1f Hz worst)"
                 % (m["traj_dt_min"], m["traj_dt_med"], m["traj_dt_max"], m["traj_hz_min"]))
    lines.append("  min pillar clearance       %.4f m (p05 %.4f) over %d points"
                 % (m["clearance_min"], m["clearance_p05"], len(np.vstack([p[1] for p in h.paths]))))
    lines.append("  points outside the arena   %d" % m["outside_arena"])
    lines.append("  worst start distance       %.4f m from the vehicle state" % m["start_d_max"])
    lines.append("  last path ends at W        [%+.4f %+.4f %+.4f]" % tuple(m["path_last_w"]))
    lines.append("  distance to the goal       %.4f m (horizontal %.4f)"
                 % (m["goal_err"], m["goal_err_h"]))
    lines.append("  path length                %.3f m (straight line %.3f m)"
                 % (m["path_len"], m["straight_len"]))
    lines.append("  path box in W              E[%+.2f %+.2f] N[%+.2f %+.2f]" % m["path_box"])
    lines.append("")
    if fails:
        lines.append("VERDICT  FAIL")
        for f in fails:
            lines.append("   %s" % f)
    else:
        lines.append("VERDICT  PASS   (frame, >=%.0f Hz points, >=%.1f Hz republish, clearance >= %.2f m, "
                     "inside the arena, start <= %.2f m, goal within %.3f m)"
                     % (args.min_traj_hz, args.min_republish_hz, args.footprint, args.start_max,
                        args.goal_tol))
    text = "\n".join(lines)
    print("\n" + text)
    if args.out:
        with open(args.out, "w") as fh:
            fh.write(text + "\n")
        print("\nwritten to %s" % args.out)
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
