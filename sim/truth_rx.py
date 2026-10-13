#!/usr/bin/env python3
"""Receive the SITL truth tap and turn it into (a) a gate verdict and (b) ROS 2 topics.

The tap (patches/0001-sitl-truth-tap.patch, enabled by `LIO_TRUTH_TAP=<port>` in the SITL
environment) sends one ASCII line per model step, in the simulator's own frames:

    TAP <instance> <t_us> <pn> <pe> <pd> <vn> <ve> <vd> <qw> <qx> <qy> <qz> <wx> <wy> <wz> <ax> <ay> <az>

  * position and velocity: NED, metres and m/s, origin = the simulator origin (the EKF origin)
  * quaternion: body FRD -> NED, scalar first
  * gyro: body rates, FRD, rad/s
  * specific force: body, FRD, m/s^2   (so a vehicle at rest reads a_z = -9.7946, |a| = 9.7946)

That is PLAN.md decision 13: the wire keeps the simulator's frames and `common/frames.py` is the one
place they are crossed.  In ROS mode this node publishes **W** (ENU, body FLU) because that is the
frame RViz, the metrics and `worlds/pillars.yaml` all use.

Two modes:

  --print   no ROS at all: 1 Hz progress lines plus a final verdict against the S3 gate numbers.
            Exit code 0 = gate passed.  This is the S3 gate and it must run before anything else.
  --ros     additionally publishes /clock, /sim/rtf, /sim/truth/odom and world_w -> truth_base_link.

The parked gate (S3): rate >= 380 Hz, |a| within 0.05 of 9.7946, quaternion norm within 1e-6 of 1,
position span < 1 mm, no gap above 10 ms.
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import frames  # noqa: E402  (path set above)

GRAVITY = 9.80665     # AP_Math GRAVITY_MSS: what SITL's simulated accelerometer reads at rest.
                      # MEASURED 2026-10-13: |a| = 9.8067 on the wire.  The LIO's own config carries
                      # 9.7946 (the real aircraft's local gravity), which is 0.12 % away from this --
                      # a difference to settle in S6, not to hide here.
GATE = {
    "rate_min_hz": 380.0,
    "gravity_tol": 0.01,
    "quat_norm_tol": 1e-6,
    "parked_span_m": 1e-3,
    "gap_max_s": 0.010,
}


class Sample:
    __slots__ = ("t_us", "instance", "p_ned", "v_ned", "q_ned", "gyro", "accel")

    def __init__(self, parts):
        self.instance = int(parts[1])
        self.t_us = int(parts[2])
        self.p_ned = np.array([float(x) for x in parts[3:6]])
        self.v_ned = np.array([float(x) for x in parts[6:9]])
        self.q_ned = np.array([float(x) for x in parts[9:13]])   # w, x, y, z
        self.gyro = np.array([float(x) for x in parts[13:16]])
        self.accel = np.array([float(x) for x in parts[16:19]])


def _install_stop_handler():
    """SIGTERM/SIGINT set a flag instead of tearing the ROS context down mid-publish."""
    import signal

    stop = {"flag": False}

    def handler(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)
    return stop


def udp_socket_drops(port: int) -> int:
    """How many datagrams the KERNEL dropped for our socket (Linux /proc/net/udp, field 13).

    Without this, a hole in the stream is a mystery: "the tap stalled", "the model jumped" and "the
    receiver was too slow to read" all look identical from the samples alone.  MEASURED 2026-10-13:
    a 20.8 ms hole appeared exactly once, during a run that also had `ros2 topic hz` competing for
    the CPU, and this counter is what tells those cases apart.
    """
    needle = ":%04X" % port
    try:
        with open("/proc/net/udp") as fh:
            next(fh)
            for line in fh:
                f = line.split()
                if len(f) > 12 and f[1].upper().endswith(needle):
                    return int(f[12])
    except (OSError, StopIteration, ValueError):
        pass
    return -1


def parse(line: str):
    parts = line.split()
    if len(parts) != 19 or parts[0] != "TAP":
        return None
    try:
        return Sample(parts)
    except ValueError:
        return None


class Stats:
    """Everything the gate needs, accumulated over the run."""

    def __init__(self, gate=GATE):
        self.gate = gate
        self.n = 0
        self.bad_lines = 0
        self.t_first_us = None
        self.t_last_us = None
        self.wall_first = None
        self.wall_last = None
        self.p_min = None
        self.p_max = None
        self.grav_min = math.inf
        self.grav_max = -math.inf
        self.grav_sum = 0.0
        self.az_sum = 0.0
        self.qnorm_dev_max = 0.0
        self.gap_max_s = 0.0
        self.gap_count = 0
        self.backwards = 0
        self.recent_window_start_us = None
        self.recent_window_start_wall = None
        self.window_count = 0
        self.worst_window_rate = math.inf
        self.instances = set()
        self.ros_published = 0

    def add(self, s: Sample):
        self.n += 1
        self.instances.add(s.instance)
        self.wall_last = time.monotonic()
        if self.wall_first is None:
            self.wall_first = self.wall_last
        if self.t_first_us is None:
            self.t_first_us = s.t_us
            self.recent_window_start_us = s.t_us
            self.recent_window_start_wall = self.wall_last
            self.p_min = s.p_ned.copy()
            self.p_max = s.p_ned.copy()
        else:
            dt = (s.t_us - self.t_last_us) * 1e-6
            if dt < 0.0:
                self.backwards += 1
            if dt > self.gap_max_s:
                self.gap_max_s = dt
            if dt > self.gate["gap_max_s"]:
                self.gap_count += 1
        self.t_last_us = s.t_us

        # worst 1 s window: a rate is SAMPLES over the elapsed simulated time of that window
        self.window_count += 1
        span_s = (s.t_us - self.recent_window_start_us) * 1e-6
        if span_s >= 1.0:
            self.worst_window_rate = min(self.worst_window_rate, self.window_count / span_s)
            self.recent_window_start_us = s.t_us
            self.window_count = 0

        self.p_min = np.minimum(self.p_min, s.p_ned)
        self.p_max = np.maximum(self.p_max, s.p_ned)
        g = float(np.linalg.norm(s.accel))
        self.grav_min = min(self.grav_min, g)
        self.grav_max = max(self.grav_max, g)
        self.grav_sum += g
        self.az_sum += float(s.accel[2])
        self.qnorm_dev_max = max(self.qnorm_dev_max, abs(float(np.linalg.norm(s.q_ned)) - 1.0))

    def report(self) -> dict:
        sim_s = (self.t_last_us - self.t_first_us) * 1e-6 if self.n > 1 else 0.0
        wall_s = (self.wall_last - self.wall_first) if self.n > 1 else 0.0
        span = (self.p_max - self.p_min) if self.n else np.zeros(3)
        return {
            "samples": self.n,
            "bad_lines": self.bad_lines,
            "instances": sorted(self.instances),
            "wall_s": wall_s,
            "sim_s": sim_s,
            "rtf": (sim_s / wall_s) if wall_s > 0 else float("nan"),
            "rate_hz": (self.n / sim_s) if sim_s > 0 else float("nan"),
            "rate_worst_1s_hz": self.worst_window_rate,
            "gap_max_s": self.gap_max_s,
            "gap_count": self.gap_count,
            "gravity_mean": (self.grav_sum / self.n) if self.n else float("nan"),
            "gravity_min": self.grav_min,
            "gravity_max": self.grav_max,
            "accel_z_mean": (self.az_sum / self.n) if self.n else float("nan"),
            "quat_norm_dev_max": self.qnorm_dev_max,
            "p_span_m": span,
            "p_span_max_m": float(np.max(span)) if self.n else float("nan"),
            "p_first_ned": (self.p_min + 0.0) if self.n else None,
        }

    def verdict(self) -> list:
        """(name, ok, text) rows against the S3 gate."""
        r = self.report()
        g = self.gate
        rows = [
            ("rate", r["rate_hz"] >= g["rate_min_hz"],
             "mean %.1f Hz (>= %.0f)" % (r["rate_hz"], g["rate_min_hz"])),
            ("rate worst 1 s", (r["rate_worst_1s_hz"] >= g["rate_min_hz"]),
             "%.1f Hz (>= %.0f)" % (r["rate_worst_1s_hz"], g["rate_min_hz"])),
            ("gravity", abs(r["gravity_mean"] - GRAVITY) <= g["gravity_tol"],
             "|a| mean %.4f, want %.4f +- %.2f (min %.4f max %.4f)" % (
                 r["gravity_mean"], GRAVITY, g["gravity_tol"], r["gravity_min"], r["gravity_max"])),
            ("accel z sign", r["accel_z_mean"] < 0.0,
             "a_z mean %+.4f m/s^2 (FRD: negative at rest)" % r["accel_z_mean"]),
            ("quaternion norm", r["quat_norm_dev_max"] <= g["quat_norm_tol"],
             "max |norm-1| = %.2e (<= %.0e)" % (r["quat_norm_dev_max"], g["quat_norm_tol"])),
            ("no gaps", r["gap_max_s"] <= g["gap_max_s"],
             "max gap %.4f s, %d gaps above %.0f ms" % (r["gap_max_s"], r["gap_count"],
                                                        g["gap_max_s"] * 1e3)),
            ("monotonic time", self.backwards == 0,
             "%d samples went backwards in simulated time" % self.backwards),
            ("position span (parked)", r["p_span_max_m"] <= g["parked_span_m"],
             "max span %.6f m (<= %.4f)" % (r["p_span_max_m"], g["parked_span_m"])),
            ("lines parsed", r["bad_lines"] == 0, "%d unparsable lines" % r["bad_lines"]),
        ]
        return rows

    def text(self) -> str:
        r = self.report()
        out = []
        out.append("truth tap report")
        out.append("  samples            %d over %d instance(s)" % (r["samples"], len(r["instances"])))
        out.append("  sim time           %.3f s, wall %.3f s, rtf %.4f" % (
            r["sim_s"], r["wall_s"], r["rtf"]))
        out.append("  rate               %.1f Hz mean, %.1f Hz worst 1 s" % (
            r["rate_hz"], r["rate_worst_1s_hz"]))
        out.append("  gaps               max %.4f s, %d above %.0f ms" % (
            r["gap_max_s"], r["gap_count"], self.gate["gap_max_s"] * 1e3))
        out.append("  |a|                mean %.4f, min %.4f, max %.4f" % (
            r["gravity_mean"], r["gravity_min"], r["gravity_max"]))
        out.append("  a_z mean           %+.4f m/s^2" % r["accel_z_mean"])
        out.append("  quat norm dev      %.2e" % r["quat_norm_dev_max"])
        out.append("  position span      E %.6f N %.6f U %.6f m (first %.4f %.4f %.4f NED)" % (
            r["p_span_m"][0], r["p_span_m"][1], r["p_span_m"][2],
            r["p_first_ned"][0], r["p_first_ned"][1], r["p_first_ned"][2]))
        out.append("")
        ok_all = True
        for name, ok, detail in self.verdict():
            ok_all = ok_all and ok
            out.append("  [%s] %-24s %s" % ("PASS" if ok else "FAIL", name, detail))
        out.append("")
        out.append("GATE: %s" % ("PASS" if ok_all else "FAIL"))
        return "\n".join(out)

    def ok(self) -> bool:
        return all(ok for _, ok, _ in self.verdict())


class RosSink:
    """Publishes the tap in W.  Imported lazily so --print works without ROS.

    Rates, MEASURED 2026-10-13 on this machine: asking for 200 Hz odometry produced 172 Hz, because
    the node also parses 1200 lines/s of UDP and publishes /clock.  The *lossless* truth channel is
    the CSV (every sample, at the model rate); these topics are for RViz and for liveness, so they are
    set to rates the node actually achieves and the achieved rate is checked, not assumed.
    """

    def __init__(self, clock_hz=50.0, odom_hz=100.0, rtf_hz=1.0):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float32
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import TransformBroadcaster

        # rclpy's default signal handlers shut the context down on SIGTERM, and the next publish then
        # raises "publisher's context is invalid" from inside the receive loop: an exception that hides
        # the real event.  Our own handler only sets a flag (see sim/lidar_imu.py for the measurement).
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.rclpy = rclpy
        self.node = Node("truth_rx")
        qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_clock = self.node.create_publisher(Clock, "/clock", qos)
        self.pub_rtf = self.node.create_publisher(Float32, "/sim/rtf", 10)
        self.pub_odom = self.node.create_publisher(Odometry, "/sim/truth/odom", qos)
        self.tf = TransformBroadcaster(self.node)
        self.clock_period_us = int(1e6 / clock_hz)
        self.odom_period_us = int(1e6 / odom_hz)
        self.rtf_period_us = int(1e6 / rtf_hz)
        self.odom_hz_requested = odom_hz
        self.next_clock_us = 0
        self.next_odom_us = 0
        self.next_rtf_us = 0
        self._last_rtf_wall = None
        self._last_rtf_sim = None
        self._Clock = Clock
        self._Float32 = Float32
        self._Odometry = Odometry
        self._TransformStamped = TransformStamped

    def publish(self, s: Sample, stats: Stats, forced_rtf=None):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(s.t_us // 1_000_000)
        t.nanosec = int((s.t_us % 1_000_000) * 1000)

        if s.t_us >= self.next_clock_us:
            self.next_clock_us = s.t_us + self.clock_period_us
            msg = self._Clock()
            msg.clock = t
            self.pub_clock.publish(msg)

        if s.t_us >= self.next_rtf_us:
            self.next_rtf_us = s.t_us + self.rtf_period_us
            wall = time.monotonic()
            if self._last_rtf_wall is not None and wall > self._last_rtf_wall:
                rtf = (s.t_us - self._last_rtf_sim) * 1e-6 / (wall - self._last_rtf_wall)
                m = self._Float32()
                m.data = float(rtf)
                self.pub_rtf.publish(m)
            self._last_rtf_wall = wall
            self._last_rtf_sim = s.t_us

        if s.t_us >= self.next_odom_us:
            self.next_odom_us = s.t_us + self.odom_period_us
            p_w, v_w, q_w = frames.wire_to_world(s.p_ned, s.v_ned, s.q_ned)
            odom = self._Odometry()
            odom.header.stamp = t
            odom.header.frame_id = "world_w"
            odom.child_frame_id = "truth_base_link"
            odom.pose.pose.position.x = float(p_w[0])
            odom.pose.pose.position.y = float(p_w[1])
            odom.pose.pose.position.z = float(p_w[2])
            odom.pose.pose.orientation.w = float(q_w[0])
            odom.pose.pose.orientation.x = float(q_w[1])
            odom.pose.pose.orientation.y = float(q_w[2])
            odom.pose.pose.orientation.z = float(q_w[3])
            odom.twist.twist.linear.x = float(v_w[0])
            odom.twist.twist.linear.y = float(v_w[1])
            odom.twist.twist.linear.z = float(v_w[2])
            gyro_w = frames.flu_to_frd(s.gyro)   # body rates: FRD on the wire, FLU in ROS
            odom.twist.twist.angular.x = float(gyro_w[0])
            odom.twist.twist.angular.y = float(gyro_w[1])
            odom.twist.twist.angular.z = float(gyro_w[2])
            self.pub_odom.publish(odom)

            tfm = self._TransformStamped()
            tfm.header.stamp = t
            tfm.header.frame_id = "world_w"
            tfm.child_frame_id = "truth_base_link"
            tfm.transform.translation.x = float(p_w[0])
            tfm.transform.translation.y = float(p_w[1])
            tfm.transform.translation.z = float(p_w[2])
            tfm.transform.rotation.w = float(q_w[0])
            tfm.transform.rotation.x = float(q_w[1])
            tfm.transform.rotation.y = float(q_w[2])
            tfm.transform.rotation.z = float(q_w[3])
            self.tf.sendTransform(tfm)
            stats.ros_published += 1

    def spin_once(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def _line(t_us, p=(0.0, 0.0, -0.205), v=(0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0),
          w=(0.0, 0.0, 0.0), a=(0.0, 0.0, -GRAVITY), instance=0):
    return ("TAP %d %d %.6f %.6f %.6f %.6f %.6f %.6f %.8f %.8f %.8f %.8f "
            "%.6f %.6f %.6f %.5f %.5f %.5f" % (instance, t_us, p[0], p[1], p[2], v[0], v[1], v[2],
                                               q[0], q[1], q[2], q[3], w[0], w[1], w[2],
                                               a[0], a[1], a[2]))


def _selftest() -> int:
    """Feed the receiver synthetic taps: the good case must PASS, every broken case must FAIL.

    A gate that cannot fail is not a gate (PLAN.md rule 5), so each mutation below is checked to
    actually turn the verdict over.
    """
    ok = True

    def run(lines, gate_overrides=None):
        st = Stats(dict(GATE, **(gate_overrides or {})))
        for ln in lines:
            s = parse(ln)
            if s is None:
                st.bad_lines += 1
                continue
            st.add(s)
        return st

    def check(name, got, want):
        nonlocal ok
        good = got == want
        ok = ok and good
        print("%-42s %s" % (name, "ok" if good else "FAIL got=%s want=%s" % (got, want)))

    def check_gate(name, stats, want_ok):
        """Gate verdicts: on a mismatch, print which row disagreed -- otherwise this is unhelpful."""
        nonlocal ok
        got = stats.ok()
        good = got == want_ok
        ok = ok and good
        if good:
            print("%-42s ok" % name)
        else:
            print("%-42s FAIL got=%s want=%s" % (name, got, want_ok))
            for row, row_ok, detail in stats.verdict():
                if row_ok != want_ok:
                    print("      row %-24s %s   %s" % (row, "pass" if row_ok else "fail", detail))

    parked = [_line(1_000_000 + i * 2500) for i in range(4000)]   # 400 Hz for 10 s
    check("parse() accepts a good line", parse(parked[0]) is not None, True)
    check("parse() rejects a short line", parse("TAP 0 100 1 2 3") is None, True)
    check("parse() rejects a wrong tag", parse(parked[0].replace("TAP", "XXX", 1)) is None, True)

    check_gate("gate PASSES parked 400 Hz", run(parked), True)
    check_gate("gate FAILS at 100 Hz",
               run([_line(1_000_000 + i * 10000) for i in range(1000)]), False)
    check_gate("gate FAILS on |a| = 5",
               run([_line(1_000_000 + i * 2500, a=(0.0, 0.0, -5.0)) for i in range(400)]), False)
    check_gate("gate FAILS on positive a_z (FRD sign)",
               run([_line(1_000_000 + i * 2500, a=(0.0, 0.0, GRAVITY)) for i in range(400)]), False)
    check_gate("gate FAILS on a 0.1 m span",
               run([_line(1_000_000 + i * 2500, p=(0.0, 0.1 if i > 200 else 0.0, -0.205))
                    for i in range(400)]), False)
    # a real 50 ms hole: the second run of samples starts 50 ms after the first ended
    check_gate("gate FAILS on a 50 ms gap",
               run([_line(1_000_000 + i * 2500) for i in range(200)] +
                   [_line(1_550_000 + i * 2500) for i in range(200)]), False)
    check_gate("gate FAILS on time going backwards",
               run([_line(1_000_000 + i * 2500) for i in range(200)] +
                   [_line(1_000_000 + i * 2500) for i in range(200)]), False)
    check_gate("gate FAILS on a non-unit quaternion",
               run([_line(1_000_000 + i * 2500, q=(1.01, 0.0, 0.0, 0.0)) for i in range(400)]), False)
    check_gate("gate FAILS on an unparsable line", run(parked + ["garbage"]), False)
    check_gate("moving mode ignores the span", run(
        [_line(1_000_000 + i * 2500, p=(0.0, 0.1 * i, -0.205)) for i in range(400)],
        {"parked_span_m": math.inf}), True)

    # The wire -> W conversion, on a line the tap could really produce.  A level vehicle with
    # identity NED attitude has body FLU = North/West/Up, i.e. W turned +90 deg about z.
    s = parse(_line(1_000_000, p=(0.0, 0.0, -2.0), v=(1.0, 0.0, 0.0)))
    p_w, v_w, q_w = frames.wire_to_world(s.p_ned, s.v_ned, s.q_ned)
    check("wire NED z=-2 -> W up +2", float(p_w[2]), 2.0)
    check("wire north velocity -> W north", float(v_w[1]), 1.0)
    check("wire body FRD -> body FLU (yaw +90 in W)",
          [round(float(x), 6) for x in q_w], [round(math.cos(math.pi / 4), 6), 0.0, 0.0,
                                              round(math.sin(math.pi / 4), 6)])

    print("truth_rx selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true", help="run the receiver's own checks and exit")
    ap.add_argument("--port", type=int, default=int(os.environ.get("LIO_TRUTH_TAP", "14560")))
    ap.add_argument("--duration", type=float, default=30.0,
                    help="seconds of SIM time to record (0 = forever)")
    ap.add_argument("--wall-timeout", type=float, default=0.0,
                    help="give up after this many seconds of wall time (0 = 3x duration + 10 s).  "
                         "Without it a simulator that never starts would hang the receiver forever, "
                         "which is exactly what a blocked SITL did on the first S3 attempt.")
    ap.add_argument("--print", dest="print_only", action="store_true", help="no ROS")
    ap.add_argument("--ros", action="store_true", help="publish /clock, /sim/rtf, /sim/truth/odom")
    ap.add_argument("--parked", action="store_true", default=True,
                    help="apply the parked gate (position span); default on")
    ap.add_argument("--moving", dest="parked", action="store_false",
                    help="skip the parked position-span check")
    ap.add_argument("--save", default=None, help="write the report to this file")
    ap.add_argument("--csv", default=None,
                    help="write every sample (t_us, p_ned, v_ned, q, gyro, accel) for offline checks")
    ap.add_argument("--fanout", type=int, default=14561,
                    help="forward every raw tap line to this local UDP port so other sim nodes "
                         "(sim/lidar_imu.py) can consume the same stream; 0 = off.  Only one process "
                         "can bind the tap's port, so the fan-out is what lets the sensor node exist "
                         "as a separate process without doubling the SITL-side traffic")
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # 4 MB receive buffer: about 20 000 of our 170-byte lines, so a busy moment inside the receiver
    # cannot spill samples out of the kernel queue
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)
    if udp_socket_drops(args.port) < 0:
        print("note: cannot read /proc/net/udp -- drop accounting unavailable", flush=True)

    gate = dict(GATE)
    if not args.parked:
        gate["parked_span_m"] = math.inf

    stats = Stats(gate)
    sink = None
    if args.ros and not args.print_only:
        sink = RosSink()

    csv = None
    if args.csv:
        csv = open(args.csv, "w")
        csv.write("t_us,inst,pn,pe,pd,vn,ve,vd,qw,qx,qy,qz,wx,wy,wz,ax,ay,az\n")

    fan = None
    if args.fanout:
        fan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        fan.setblocking(False)
        fan_addr = ("127.0.0.1", args.fanout)
        fanned = 0
        print("fanning every tap line out to udp/127.0.0.1:%d" % args.fanout, flush=True)

    print("listening on udp/127.0.0.1:%d  (SITL needs LIO_TRUTH_TAP=%d)" % (args.port, args.port),
          flush=True)
    stop = _install_stop_handler()
    wall_timeout = args.wall_timeout or (3.0 * args.duration + 10.0 if args.duration > 0 else 0.0)
    t_start_wall = time.monotonic()
    last_progress = time.monotonic()
    last_note = 0.0
    try:
        while True:
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                now = time.monotonic()
                if stats.n == 0 and now - last_note >= 2.0:
                    last_note = now
                    print("no samples yet (%.1f s wall, giving up at %.0f s)" % (
                        now - t_start_wall, wall_timeout), flush=True)
                if wall_timeout > 0 and (now - t_start_wall) > wall_timeout:
                    print("WALL TIMEOUT after %.1f s with %d samples" % (
                        now - t_start_wall, stats.n), flush=True)
                    break
                continue
            s = parse(data.decode("ascii", "replace").strip())
            if s is None:
                stats.bad_lines += 1
                continue
            stats.add(s)
            if fan is not None:
                try:
                    fan.sendto(data, fan_addr)
                    fanned += 1
                except OSError:
                    pass
            if csv is not None:
                csv.write("%d,%d,%s,%s,%s\n" % (
                    s.t_us, s.instance,
                    ",".join("%.6f" % x for x in s.p_ned),
                    ",".join("%.6f" % x for x in s.v_ned),
                    ",".join("%.8f" % x for x in list(s.q_ned) + list(s.gyro) + list(s.accel))))
            if sink is not None:
                sink.publish(s, stats)
                sink.spin_once()
            now = time.monotonic()
            if now - last_progress >= 1.0:
                last_progress = now
                r = stats.report()
                print("t=%8.3f s  rate %6.1f Hz  |a| %.4f  span %.6f m  qdev %.1e" % (
                    r["sim_s"], r["rate_hz"], r["gravity_mean"], r["p_span_max_m"],
                    r["quat_norm_dev_max"]), flush=True)
            if stop["flag"]:
                print("\nstop requested", flush=True)
                break
            if args.duration > 0 and stats.n > 1 and \
                    (stats.t_last_us - stats.t_first_us) * 1e-6 >= args.duration:
                break
            if wall_timeout > 0 and (now - t_start_wall) > wall_timeout:
                print("WALL TIMEOUT after %.1f s with %.3f s of simulated time" % (
                    now - t_start_wall, (stats.t_last_us - stats.t_first_us) * 1e-6), flush=True)
                break
    except KeyboardInterrupt:
        print("\ninterrupted", flush=True)
    finally:
        # read the kernel's drop counter while the socket is still open: once it is closed the entry
        # vanishes from /proc/net/udp and the count silently becomes "unavailable"
        drops = udp_socket_drops(args.port)
        sock.close()
        if fan is not None:
            fan.close()

    if csv is not None:
        csv.close()
        print("wrote %s" % args.csv)

    wall_s = stats.report()["wall_s"]
    ros_hz = stats.ros_published / max(1e-9, wall_s)
    extra = []
    if sink is not None:
        extra.append("ros publishes    %d (%.1f Hz over the run, %.0f Hz requested)" % (
            stats.ros_published, ros_hz, sink.odom_hz_requested))
    if drops >= 0:
        extra.append("kernel UDP drops %d" % drops)
    if fan is not None:
        extra.append("fanned out       %d lines to port %d" % (fanned, args.fanout))

    ros_ok = True
    if sink is not None:
        ros_ok = ros_hz >= 0.9 * sink.odom_hz_requested
        extra.append("[%s] ros odom rate  %.1f Hz (>= 90%% of the %.0f Hz requested)" % (
            "PASS" if ros_ok else "FAIL", ros_hz, sink.odom_hz_requested))

    text = "\n".join(extra + ([""] if extra else []) + [stats.text()])
    print()
    print(text)
    if args.save:
        with open(args.save, "w") as fh:
            fh.write(text + "\n")
        print("\nwrote %s" % args.save)
    if sink is not None:
        sink.shutdown()
    # A dropped datagram is a hole in the truth every later metric leans on, so it fails the gate.
    return 0 if (stats.ok() and drops <= 0 and ros_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
