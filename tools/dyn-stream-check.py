#!/usr/bin/env python3
"""The two published streams must be the SAME plan: an offline check of planner/ego_adapter.py's build_path.

WHY.  The Path and the dyn message are built in one function and published in one tick, and the whole point
of the second stream is that the consumer may take its velocity and acceleration from it WITHOUT having to
wonder whether it describes the plan being flown.  That is a property of the code, and it can be checked
without a flight: this script feeds build_path() synthetic splines (including the two cases that make the
streams diverge if the gate is applied to only one of them -- a plan whose clock has run out, and a plan that
must be truncated to its verified prefix) and compares the two messages point by point.

Requires ROS's message packages on PYTHONPATH (nav_msgs, geometry_msgs, trajectory_msgs) but NOT a running
ROS graph: `rclpy.init` is never called and no node is created, because the geometry under test is pure
arithmetic over the message fields.  Source /opt/ros/humble/setup.bash before running.

usage: bash -c 'source /opt/ros/humble/setup.bash && python3 tools/dyn-stream-check.py'
"""
from __future__ import annotations

import importlib.util
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)


def load_adapter():
    path = os.path.join(ROOT, "planner", "ego_adapter.py")
    spec = importlib.util.spec_from_file_location("ego_adapter_dsc", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _Clock:
    class _Now:
        @staticmethod
        def to_msg():
            from builtin_interfaces.msg import Time
            return Time(sec=1, nanosec=0)
    def now(self):
        return self._Now()


class _Node:
    def get_clock(self):
        return _Clock()


class _Pt:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = float(x), float(y), float(z)


class _Stamp:
    def __init__(self, t):
        self.sec = int(t)
        self.nanosec = int(round((t - int(t)) * 1e9))


class _Msg:
    """Just the fields build_path reads from traj_utils/Bspline."""
    def __init__(self, pos_pts, knots, order, start_time):
        self.pos_pts = [_Pt(*p) for p in pos_pts]
        self.knots = [float(v) for v in knots]
        self.order = int(order)
        self.start_time = _Stamp(start_time)
        self.traj_id = 1
        self.drone_id = 0


def make_adapter(ea, world, dt, z_band, min_prefix):
    import argparse
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, Transform, Twist
    from builtin_interfaces.msg import Duration
    from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

    a = object.__new__(ea.EgoAdapter)
    a.args = argparse.Namespace(dt=dt, z_band=z_band, min_prefix_m=min_prefix, footprint=0.20,
                               start_max=0.50, traj_topic="/planner/traj",
                               traj_dyn_topic="/planner/traj_dyn")
    a.world = ea.World.load(world)
    a.bspline = None
    a.odom_l = None
    a.Path, a.PoseStamped = Path, PoseStamped
    a.Transform, a.Twist, a.Duration = Transform, Twist, Duration
    a.MDJTraj, a.MDJPoint = MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
    a.node = _Node()
    a.n_zclip = 0
    a.n_prefix = 0
    a.n_far_start = 0
    a.n_no_plan = 0
    return a


def spline_from_points(pts, order=3, ts=0.2):
    """A uniform B-spline through `pts` using the planner's own parameterisation
    (`UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivative)`)."""
    ea = load_adapter()
    P = np.asarray(pts, dtype=float)
    sed = np.zeros((4, 3))
    sed[0] = (P[1] - P[0]) / ts                 # v(t=0)
    sed[1] = (P[-1] - P[-2]) / ts               # v(t=end)
    ctrl = ea.parameterize_to_bspline(ts, P, sed)
    n = ctrl.shape[0]
    u = np.array([(i - order) * ts for i in range(n + order + 1)], dtype=float)
    return ctrl, u


def main() -> int:
    ea = load_adapter()
    world = os.path.join(ROOT, "worlds", "forest_b.yaml")
    ok = True

    def check(name, good, detail=""):
        nonlocal ok
        ok = ok and bool(good)
        print("%-72s %s %s" % (name, "ok" if good else "FAIL", detail))

    # a spline through points the ADAPTER ITSELF PUBLISHED in a real sortie: they are in L and they are
    # verified clear of every pillar (the 0.20 m gate let them through), so the test exercises a legal plan
    real = os.environ.get("DYN_CHECK_POINTS", "")
    pts = None
    if real and os.path.exists(real):
        import csv as _csv
        rows = list(_csv.DictReader(open(real)))
        first = rows[0]["t_us"]
        grp = [[float(r["e"]), float(r["n"]), float(r["u"])] for r in rows if r["t_us"] == first]
        if len(grp) >= 12:
            pts = grp[::max(1, len(grp) // 8)]
    if pts is None:                       # fall back to a straight, low, short hop
        pts = [[0.0, 0.0, 1.884], [0.3, 0.3, 1.884], [0.6, 0.6, 1.884], [0.9, 0.9, 1.884]]
    a = make_adapter(ea, world, dt=0.02, z_band=0.30, min_prefix=0.15)
    a.odom_l = np.array(pts[0], dtype=float)
    ctrl, u = spline_from_points(pts)
    t0 = time.time()          # the planner stamps with RCL_SYSTEM_TIME, and the adapter compares against
    #                           the wall clock: a start_time in the past is an EXPIRED plan and is withheld
    a.bspline = _Msg(ctrl, u, 3, t0)
    path, dyn, info = a.build_path()
    check("a legal plan is published on both streams", path is not None and dyn is not None,
          "%d poses / %d points over %s" % (0 if path is None else len(path.poses),
                                            0 if dyn is None else len(dyn.points),
                                            "a real published path" if real else "a synthetic hop"))
    if path is None:
        print("      withheld: %s" % info[:110])
        return 1

    check("the two streams carry the same number of points", len(path.poses) == len(dyn.points),
          "%d vs %d" % (len(path.poses), len(dyn.points)))
    ps = np.array([[p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec for p in path.poses]]).ravel()
    hs = float(dyn.header.stamp.sec) + 1e-9 * float(dyn.header.stamp.nanosec)
    ds = np.array([hs + p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in dyn.points])
    check("every dyn point carries the SAME absolute stamp as the Path's pose of the same index",
          float(np.max(np.abs(ds - ps))) <= 1e-9, "max |dyn - Path| %.3e s" % float(np.max(np.abs(ds - ps))))

    pos_dyn = np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y,
                         p.transforms[0].translation.z] for p in dyn.points])
    pos_path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in path.poses])
    check("the two streams carry the same positions", float(np.max(np.abs(pos_dyn - pos_path))) == 0.0,
          "max %.3e m" % float(np.max(np.abs(pos_dyn - pos_path))))

    # the STREAMED velocity must BE the analytic derivative of the recorded spline at the streamed time.
    # The one legitimate difference is the adapter's ALTITUDE CLAMP: where it made the published z constant
    # it zeroes vz/az (so the two streams describe the same points), and this check has to encode that rule
    # rather than pretend it does not exist.
    vels = np.array([[p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z]
                     for p in dyn.points])
    accs = np.array([[p.accelerations[0].linear.x, p.accelerations[0].linear.y,
                      p.accelerations[0].linear.z] for p in dyn.points])
    t_rel = ds - t0
    ref = np.array([ea.bspline_derivs(u, ctrl, 3, float(t)) for t in t_rel])
    # WHICH POINTS THE ALTITUDE CLAMP ACTUALLY MOVED.  The criterion is "the published z differs from the
    # RAW spline z", with a threshold that separates the two things it can be: 0 +- the stamp quantisation
    # (|vz| * 0.24 us ~ 1e-7 m, because the stamps are epoch seconds in float64) or an actual clamp step
    # (MEASURED 3.4e-4 .. 7e-3 m here).  1e-5 m sits between them.  Note that "published vs CLIP(raw)" does
    # NOT work: where the clamp applies, clip(raw) IS the published value by construction.
    zg, band = float(pos_path[-1, 2]), 0.30
    zc = np.abs(pos_path[:, 2] - ref[:, 0, 2]) > 1e-5
    # The same quantisation bounds how exactly the streamed numbers can be reproduced from the stamps:
    # |dv| <= |a| * 0.24 us ~ 5e-7 m/s and |da| <= |jerk| * 0.24 us ~ 1e-5 m/s^2.  MEASURED below.
    dxy_v = float(np.max(np.abs(vels[:, :2] - ref[:, 1, :2])))
    dxy_a = float(np.max(np.abs(accs[:, :2] - ref[:, 2, :2])))
    check("dyn velocity (x,y) == bspline_derivs at the same stamps", dxy_v < 1e-5,
          "max %.3e m/s (bounded by the ~240 ns epoch-stamp quantisation, not by the derivative)" % dxy_v)
    check("dyn acceleration (x,y) == bspline_derivs at the same stamps", dxy_a < 1e-4,
          "max %.3e m/s^2" % dxy_a)
    # THE Z RULE, stated as the two halves it actually is: where the altitude clamp did not bite, the exact
    # derivative goes out unchanged; where it did, the published z is constant and vz/az are zero.
    dz_v = float(np.max(np.abs(vels[~zc, 2] - ref[~zc, 1, 2]))) if (~zc).any() else 0.0
    dz_a = float(np.max(np.abs(accs[~zc, 2] - ref[~zc, 2, 2]))) if (~zc).any() else 0.0
    zz_v = float(np.max(np.abs(vels[zc, 2]))) if zc.any() else 0.0
    zz_a = float(np.max(np.abs(accs[zc, 2]))) if zc.any() else 0.0
    check("dyn velocity (z): exact where the z clamp did not bite, 0 where it did",
          dz_v < 1e-5 and zz_v < 1e-12,
          "%d of %d points clamped; max %.3e m/s unclamped, %.3e m/s clamped"
          % (int(zc.sum()), len(zc), dz_v, zz_v))
    check("dyn acceleration (z): exact where the z clamp did not bite, 0 where it did",
          dz_a < 1e-4 and zz_a < 1e-12,
          "max %.3e m/s^2 unclamped, %.3e m/s^2 clamped" % (dz_a, zz_a))
    # ... and the finite difference the consumer used to be forced into is measurably different
    v_fd = np.diff(pos_path, axis=0) / np.diff(ps)[:, None]
    check("... and the finite difference it replaces is NOT the same number",
          float(np.max(np.abs(v_fd - vels[:-1]))) > 1e-3,
          "max |v_fd - v_exact| %.4f m/s" % float(np.max(np.abs(v_fd - vels[:-1]))))

    # WITHHOLD: the same spline, its clock run out -> NEITHER stream may be published
    a2 = make_adapter(ea, world, dt=0.02, z_band=0.30, min_prefix=0.15)
    a2.odom_l = np.array(pts[0], dtype=float)
    a2.bspline = _Msg(ctrl, u, 3, t0 - ea.bspline_duration(u, ctrl.shape[0], 3) - 1.0)
    p2, d2, why = a2.build_path()
    check("a plan whose clock ran out withholds BOTH streams", p2 is None and d2 is None, why[:60])

    # TRUNCATION: a path that breaks the clearance contract in its far half must be cut to the SAME prefix
    # on both streams -- this is the case that would silently diverge if the dyn message were built from the
    # untruncated samples.
    a3 = make_adapter(ea, world, dt=0.02, z_band=0.30, min_prefix=0.10)
    a3.odom_l = np.array([0.0, 0.0, 1.884])
    bad = [[0.0, 0.0, 1.884], [0.5, -0.5, 1.884], [1.0, -1.0, 1.884], [1.5, -1.5, 1.884]]
    c3, u3 = spline_from_points(bad)
    a3.bspline = _Msg(c3, u3, 3, t0)
    p3, d3, i3 = a3.build_path()
    if p3 is None:
        check("a plan that breaks the contract in its far half is truncated on BOTH streams (or withheld)",
              d3 is None, "withheld: %s" % i3[:60])
    else:
        check("a truncated plan is truncated IDENTICALLY on both streams",
              len(p3.poses) == len(d3.points) and i3.get("prefix_of", 0) > len(p3.poses),
              "%d of %d points on both streams (prefix %.3f m)"
              % (len(p3.poses), i3.get("prefix_of", -1), i3.get("prefix_m", float("nan"))))

    ok_all = ok
    print("dyn-stream-check:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


if __name__ == "__main__":
    raise SystemExit(main())
