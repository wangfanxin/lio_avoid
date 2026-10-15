#!/usr/bin/env python3
"""The simulated MID-360: a `PointCloud2` and an `Imu`, both in the housing frame S.

This is the sensor half of the "no Gazebo" decision: the world is cylinders and planes, the ray
casting is analytic (`sim/raycast.py`, checked against an independent scalar reference), and the scan
pattern is the real MID-360 one (`scan_mode/mid360.csv`, 200 kHz, non-repetitive).  What the node adds
is the part that makes the estimator problem real rather than convenient:

  * **per-point poses.**  The pattern's own time column says when each point was emitted, and every
    point is cast from the pose the vehicle really had at that instant, using the truth tap at 1200 Hz.
    A 100 ms scan at 1 m/s is 10 cm of motion, so this is the distortion the LIO's undistortion has to
    remove -- casting a whole scan from one pose would hand it a cleaner problem than the real one.
  * **both channels in S, untransformed.**  Exactly as `livox_ros_driver2` does on the aircraft: the
    cloud and the IMU come out of one housing, so the LIO's `lidar_imu` stays identity and the 45 deg
    mount lives in `odom_robo` (see `FRAMES.md` 0.45 and the carried config).

Output, in the layout the clean upstream actually parses (`ROSWrapper.cpp` `stdMsgHandler`):
  `/livox/lidar`  `sensor_msgs/PointCloud2`, frame `mid360_link`, stamp = SCAN START, fields
                  x, y, z, intensity, time -- `time` in SECONDS from the scan start, which is what
                  `LID_TYPE::VELO16` (lidar_type 3, `common/ds.h`) reads.
  `/livox/imu`    `sensor_msgs/Imu`, frame `mid360_link`, rad/s and m/s^2 in S.

Input is the truth tap's UDP stream (see `patches/0001-sitl-truth-tap.patch`); run `sim/truth_rx.py`
first, which fans the stream out to a second port with `--fanout`.

usage:
  sim/lidar_imu.py --port 14561 --scan-hz 10 --imu-hz 200 --burst 3     # 3 scans, then report
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from common import frames                     # noqa: E402
from sim import raycast                       # noqa: E402

try:
    import yaml
except ImportError:                            # pragma: no cover
    yaml = None


# --------------------------------------------------------------------------- truth buffer
class TruthBuffer:
    """The tap's samples, with pose interpolation at any instant inside the window.

    The tap runs at the model rate (1200 Hz measured), so consecutive samples are ~0.83 ms apart; over
    a 5 us point spacing the pose is essentially linear between them, which is what `pose_at` assumes
    (linear position, normalised-linear quaternion, sign-corrected).
    """

    def __init__(self, maxlen=6000):
        self.t = np.zeros(0, dtype=np.int64)
        self.p = np.zeros((0, 3))
        self.q = np.zeros((0, 4))
        self.v = np.zeros((0, 3))
        self.gyro = np.zeros((0, 3))
        self.accel = np.zeros((0, 3))
        self.maxlen = maxlen
        self.total = 0

    def add(self, t_us, p_w, v_w, q_w, gyro_frd, accel_frd):
        p_w = np.asarray(p_w, dtype=float)
        v_w = np.asarray(v_w, dtype=float)
        q_w = np.asarray(q_w, dtype=float)
        gyro_frd = np.asarray(gyro_frd, dtype=float)
        accel_frd = np.asarray(accel_frd, dtype=float)
        self.t = np.append(self.t, t_us)
        self.p = np.vstack([self.p, p_w[None, :]])
        self.v = np.vstack([self.v, v_w[None, :]])
        self.q = np.vstack([self.q, q_w[None, :]])
        self.gyro = np.vstack([self.gyro, gyro_frd[None, :]])
        self.accel = np.vstack([self.accel, accel_frd[None, :]])
        self.total += 1
        if len(self.t) > self.maxlen:
            k = len(self.t) - self.maxlen
            self.t, self.p, self.v = self.t[k:], self.p[k:], self.v[k:]
            self.q, self.gyro, self.accel = self.q[k:], self.gyro[k:], self.accel[k:]

    def covers(self, t_us) -> bool:
        return len(self.t) >= 2 and self.t[0] <= t_us <= self.t[-1]

    def pose_at(self, t_query: np.ndarray):
        """(N,) times -> (p (N,3) in W, R (N,3,3) body FLU -> W).

        Times outside the buffered window are CLAMPED to the nearest sample, not extrapolated: with
        the raw searchsorted index a query 50 ms before the window returned a pose extrapolated 50 ms
        (the self-test measured y = -0.05 m for a vehicle that had not moved yet), and a scan cast from
        an extrapolated pose is worse than a scan not cast at all.  Callers that need certainty use
        `covers()`; this function only promises not to invent motion.
        """
        t_query = np.atleast_1d(np.asarray(t_query, dtype=np.int64))
        idx = np.clip(np.searchsorted(self.t, t_query, side="right") - 1, 0, len(self.t) - 2)
        t0, t1 = self.t[idx], self.t[idx + 1]
        span = np.maximum(1, t1 - t0)
        w = np.clip((t_query - t0) / span, 0.0, 1.0).astype(float)[:, None]
        p = self.p[idx] * (1.0 - w) + self.p[idx + 1] * w
        q = _q_nlerp(self.q[idx], self.q[idx + 1], w)
        return p, _R_from_q_batch(q)

    def sample_nearest(self, t_us):
        """The nearest raw sample (for the IMU, which must not be interpolated across real noise)."""
        i = int(np.clip(np.searchsorted(self.t, t_us), 0, len(self.t) - 1))
        if i > 0 and abs(self.t[i - 1] - t_us) < abs(self.t[i] - t_us):
            i -= 1
        return i


def _q_nlerp(q0, q1, w):
    """Normalised linear quaternion interpolation with the shortest-arc sign fix."""
    d = np.sum(q0 * q1, axis=1, keepdims=True)
    q1 = np.where(d < 0.0, -q1, q1)
    q = q0 * (1.0 - w) + q1 * w
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _R_from_q_batch(q):
    """(N,4) w,x,y,z -> (N,3,3) rotation matrices."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


# --------------------------------------------------------------------------- the sensor
def imu_decimation(tap_hz: float, want_hz: float) -> int:
    """How many arriving tap samples make one published IMU sample (never less than 1).

    This exists because the rate was once assumed instead of measured: `round(1200/want)` published the
    IMU at 33.3 Hz whenever the tap arrived decimated.  A tap slower than the wanted IMU rate can only
    publish every sample it gets -- it cannot invent the rest -- so the answer is 1, not 0.
    """
    if tap_hz <= 0.0 or want_hz <= 0.0:
        return 1
    return max(1, int(round(tap_hz / want_hz)))


class Sensor:
    """Turns one truth window into one cloud in S, plus the IMU samples inside it.

    NOISE: on by default, with the values the carried LIO configuration already assumes
    (`imu_ng: 0.005` rad/s/sqrt(Hz), `imu_na: 0.1` m/s^2/sqrt(Hz)), which the old stack had MEASURED on
    its own simulated IMU ("the gyro's real disturbance density is ~0.0047 rad/s/sqrt(Hz)",
    src/lio-fork-local/config/livox_360.yaml).  MEASURED 2026-10-13 here: with a NOISELESS IMU the
    clean upstream LIO tracked the truth to 0.01 mm parked -- i.e. the estimator was being tested on a
    problem with no noise in it, and no accuracy number from such a run means anything.  A constant
    bias is included because the estimator carries bias states; without any bias those states are
    never exercised.  `--no-noise` reproduces the ideal case for the A/B.
    """

    def __init__(self, world, rays, times, mount_pitch=45.0, lever_arm=None,
                 intensity=100.0, rmin=0.10, rmax=60.0,
                 range_noise=0.010, gyro_noise_density=0.005, accel_noise_density=0.002,
                 gyro_bias=0.001, accel_bias=0.01, seed=20261013, rate_hz=200.0):
        self.world = world
        self.rays = rays
        self.times = times                      # seconds from the scan start, per point
        self.R_BS = frames.R_body_sensor(mount_pitch_deg=mount_pitch)
        self.lever = frames.SENSOR_LEVER_ARM_B if lever_arm is None else np.asarray(lever_arm, float)
        self.intensity = intensity
        self.rmin = rmin
        self.rmax = rmax
        self.last_cast_ms = float("nan")
        self.range_noise = range_noise
        # per-sample standard deviation from a density: sigma = density * sqrt(rate).  The rate is the
        # IMU rate the samples are actually published at -- it used to be hardcoded 200 Hz while the
        # IMU went out at 33.3 Hz (see the decimation note in main), which silently made the delivered
        # noise DENSITY sqrt(6) too low.  It is now the same number the publishing path uses.
        self.rate_hz = rate_hz
        self.gyro_sigma = gyro_noise_density * math.sqrt(rate_hz)
        self.accel_sigma = accel_noise_density * math.sqrt(rate_hz)
        self.gyro_bias = np.asarray(gyro_bias, dtype=float) * np.ones(3)
        self.accel_bias = np.asarray(accel_bias, dtype=float) * np.ones(3)
        self.rng = np.random.default_rng(seed)

    def scan(self, buf: TruthBuffer, t_start_us: int):
        """Cast one scan whose points span [t_start, t_start + times[-1]] with per-point poses."""
        t_pts = t_start_us + np.rint(self.times * 1e6).astype(np.int64)
        p_w, R_w_b = buf.pose_at(t_pts)                       # (N,3), (N,3,3)
        origins = p_w + np.einsum("nij,j->ni", R_w_b, self.lever)
        R_w_s = np.einsum("nij,jk->nik", R_w_b, self.R_BS)
        t0 = time.perf_counter()
        pts_w, rng, idx = raycast.cast(origins, R_w_s, self.rays, self.world,
                                       rmin=self.rmin, rmax=self.rmax, return_index=True)
        self.last_cast_ms = (time.perf_counter() - t0) * 1e3
        # back into the housing frame: p_S = R_WS^T (p_W - origin_W), with range noise applied along
        # the measured direction (a lidar measures a RANGE, so its noise is radial, not per-axis)
        d = pts_w - origins[idx]
        if self.range_noise > 0.0:
            # a lidar measures a RANGE: its noise is radial, not per-axis
            r_noisy = rng + self.rng.normal(0.0, self.range_noise, size=len(rng))
            d = d * (r_noisy / np.maximum(rng, 1e-9))[:, None]
        pts_s = np.einsum("nij,nj->ni", R_w_s[idx].transpose(0, 2, 1), d)
        return pts_s, idx, rng

    def imu_in(self, buf: TruthBuffer, t_from_us: int, t_to_us: int):
        """IMU samples in (t_from, t_to], rotated FRD -> S and returned as (t_us, gyro_S, accel_S)."""
        lo = int(np.searchsorted(buf.t, t_from_us, side="right"))
        hi = int(np.searchsorted(buf.t, t_to_us, side="right"))
        out = []
        F = frames.F_FRD_FLU
        R_SB = self.R_BS.T
        for i in range(lo, hi):
            w_b = F @ buf.gyro[i]
            a_b = F @ buf.accel[i]
            out.append((int(buf.t[i]), R_SB @ w_b, R_SB @ a_b))
        return out

    def imu_sample(self, buf: TruthBuffer, i: int):
        """One IMU sample in S, with bias and noise if enabled (bias first, then noise)."""
        F = frames.F_FRD_FLU
        R_SB = self.R_BS.T
        w = R_SB @ (F @ buf.gyro[i]) + self.gyro_bias
        a = R_SB @ (F @ buf.accel[i]) + self.accel_bias
        if self.gyro_sigma > 0.0:
            w = w + self.rng.normal(0.0, self.gyro_sigma, size=3)
            a = a + self.rng.normal(0.0, self.accel_sigma, size=3)
        return int(buf.t[i]), w, a


# --------------------------------------------------------------------------- ROS publishing
def install_stop_handler():
    """SIGTERM/SIGINT set a flag; they do not tear the ROS context down under a running publish.

    MEASURED 2026-10-13: with rclpy's own handlers (the default), a SIGTERM shut the context down and
    the next publish raised "publisher's context is invalid" from inside the loop -- an exception that
    looks like a code bug and hides the real event (someone asked us to stop).
    """
    import signal

    stop = {"flag": False}

    def handler(signum, frame):
        stop["flag"] = True

    signal.signal(signal.SIGTERM, handler)
    signal.signal(signal.SIGINT, handler)
    return stop


def ros_init_without_signal_handlers(rclpy):
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:
        rclpy.init(args=[])


class RosOut:
    def __init__(self, cloud_topic, imu_topic, frame_id):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2, PointField, Imu

        ros_init_without_signal_handlers(rclpy)
        self.rclpy = rclpy
        self.node = Node("lidar_imu")
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_cloud = self.node.create_publisher(PointCloud2, cloud_topic, qos)
        self.pub_imu = self.node.create_publisher(Imu, imu_topic, qos)
        self.PointCloud2 = PointCloud2
        self.PointField = PointField
        self.Imu = Imu
        self.frame_id = frame_id
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        self.point_step = 20

    @staticmethod
    def _stamp(t_us):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(t_us // 1_000_000)
        t.nanosec = int((t_us % 1_000_000) * 1000)
        return t

    def cloud(self, t_start_us, pts_s, t_offsets, intensity):
        n = len(pts_s)
        arr = np.empty((n, 5), dtype=np.float32)
        arr[:, 0:3] = pts_s.astype(np.float32)
        arr[:, 3] = intensity
        arr[:, 4] = t_offsets.astype(np.float32)
        msg = self.PointCloud2()
        msg.header.stamp = self._stamp(t_start_us)
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.fields = self.fields
        msg.is_bigendian = False
        msg.point_step = self.point_step
        msg.row_step = self.point_step * n
        msg.is_dense = True
        msg.data = arr.tobytes()
        self.pub_cloud.publish(msg)

    def imu(self, t_us, gyro, accel):
        m = self.Imu()
        m.header.stamp = self._stamp(t_us)
        m.header.frame_id = self.frame_id
        m.angular_velocity.x = float(gyro[0])
        m.angular_velocity.y = float(gyro[1])
        m.angular_velocity.z = float(gyro[2])
        m.linear_acceleration.x = float(accel[0])
        m.linear_acceleration.y = float(accel[1])
        m.linear_acceleration.z = float(accel[2])
        self.pub_imu.publish(m)

    def spin_once(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def shutdown(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


# --------------------------------------------------------------------------- main loop
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14561, help="the tap fan-out port")
    ap.add_argument("--world", default=os.path.join(HERE, "..", "worlds", "pillars.yaml"))
    ap.add_argument("--pattern", default=None,
                    help="the real MID-360 pattern csv; by default it is looked up (see "
                         "raycast.default_pattern_path) and the node REFUSES to run without it")
    ap.add_argument("--allow-synth", action="store_true",
                    help="allow the synthesised rosette fallback -- only for a smoke test, never for "
                         "a measurement, because it changes what the sensor can see")
    ap.add_argument("--mirror-azimuth", action="store_true",
                    help="deliberately negate the pattern's azimuth column.  This is the wrong-answer "
                         "case for the frame chain: a mirrored fan is perfectly self-consistent, so "
                         "scan matching and any S<->W round trip cannot see it.  The pillar direction "
                         "test in tools/pillar-fit.py is what can, and this flag proves it")
    ap.add_argument("--points", type=int, default=20000, help="points per scan")
    ap.add_argument("--scan-hz", type=float, default=10.0)
    ap.add_argument("--imu-hz", type=float, default=200.0)
    ap.add_argument("--cloud-topic", default="/livox/lidar")
    ap.add_argument("--imu-topic", default="/livox/imu")
    ap.add_argument("--frame", default="mid360_link")
    ap.add_argument("--mount-pitch", type=float, default=None,
                    help="override the world file's housing pitch, in degrees nose-down.  The override "
                         "exists so a deliberately WRONG mount can be flown through the same gate: a "
                         "check that cannot fail is not a check (and the S4 gate did pass with a wrong "
                         "mount before the surface split was measured)")
    ap.add_argument("--burst", type=int, default=0, help="stop after N scans (0 = run until Ctrl-C)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after this much SIM time")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-noise", action="store_true",
                    help="publish exact ranges and a noiseless, bias-free IMU.  The A/B case: with it, "
                         "the clean upstream LIO tracked the truth to 0.01 mm parked, which is how we "
                         "know the noise model is what makes the accuracy numbers mean something")
    ap.add_argument("--range-noise", type=float, default=0.010, help="lidar range sigma, m")
    ap.add_argument("--gyro-noise", type=float, default=0.005, help="gyro density, rad/s/sqrt(Hz)")
    ap.add_argument("--accel-noise", type=float, default=0.002,
                    help="accel density, m/s^2/sqrt(Hz).  MEASURED 2026-10-13, and this one mattered: "
                         "the first value used here was 0.1 (the LIO config's ASSUMED imu_na, a "
                         "FAST-LIO-style default, not a sensor measurement).  At 200 Hz that is 1.41 "
                         "m/s^2 of noise PER SAMPLE, about 50x a real MEMS accelerometer (100-300 "
                         "ug/sqrt(Hz) = 1-3e-3), and with velocity estimated from it the FCU's position "
                         "loop went unstable the moment it was asked for a 1 m step: the aircraft "
                         "accelerated away past 9 m/s and the EKF failsafe landed it.  The simulated "
                         "SENSOR now carries the sensor's noise; the estimator keeps its own assumed "
                         "noise in its config, which is the correct place for a tuning choice")
    ap.add_argument("--gyro-bias", type=float, default=0.001, help="constant gyro bias, rad/s")
    ap.add_argument("--accel-bias", type=float, default=0.01, help="constant accel bias, m/s^2")
    args = ap.parse_args()

    if args.selftest:
        return _selftest(args)

    world = raycast.World.load(args.world)
    cfg = {}
    if yaml is not None:
        cfg = yaml.safe_load(open(args.world)) or {}
    mount = (cfg.get("sensor", {}) or {}).get("mount", {}) or {}
    pitch = float(mount.get("pitch", 45.0)) if args.mount_pitch is None else float(args.mount_pitch)
    # the two implementations of the mount must agree, since one draws the world and the other feeds
    # the LIO's config; a silent disagreement here would look like an estimator problem later
    mount_eff = dict(mount)
    mount_eff["pitch"] = pitch
    mount_eff.setdefault("roll", 0.0)
    mount_eff.setdefault("yaw", 0.0)
    if not np.allclose(frames.R_body_sensor(mount_pitch_deg=pitch), raycast.make_mount_rotation(mount_eff),
                       atol=1e-12):
        print("FAIL: frames.R_body_sensor and raycast.make_mount_rotation disagree about the mount")
        return 1

    pattern = args.pattern or raycast.default_pattern_path()
    if pattern is None and not args.allow_synth:
        print("REFUSING to run: the real MID-360 pattern was not found.\n"
              "  looked in: $LIO_PATTERN, <sim>/mid360_pattern.csv, <sim>/scan_mode/mid360.csv,\n"
              "             /home/fanxin/lio_avoid/sim/mid360_pattern.csv\n"
              "  it is 17 MB of vendor data and is not tracked in the repo (.gitignore).\n"
              "  pass --pattern, or --allow-synth for a smoke test only (a synthesised pattern has a\n"
              "  different blind zone and repeats, so no measurement may be taken with it).")
        return 1
    rays, times, src = raycast.load_pattern(pattern, n=args.points)
    if "synth" in src.lower() and not args.allow_synth:
        print("REFUSING to run on a synthesised pattern (%s)" % src)
        return 1
    if times is None:
        times = np.linspace(0.0, 1.0 / args.scan_hz, len(rays), endpoint=False)
    if args.mirror_azimuth:
        rays = rays.copy()
        rays[:, 0] = -rays[:, 0]
        print("MIRRORED AZIMUTH (deliberately wrong configuration)")
    print("pattern: %s (%s), %d points spanning %.6f s" % (
        pattern or "synthesised", src, len(rays), float(times[-1]) if len(times) else -1.0))

    sensor = Sensor(world, rays, times, mount_pitch=pitch,
                    range_noise=0.0 if args.no_noise else args.range_noise,
                    gyro_noise_density=0.0 if args.no_noise else args.gyro_noise,
                    accel_noise_density=0.0 if args.no_noise else args.accel_noise,
                    gyro_bias=0.0 if args.no_noise else args.gyro_bias,
                    accel_bias=0.0 if args.no_noise else args.accel_bias,
                    rate_hz=args.imu_hz)
    print("noise: range sigma %.4f m, gyro %.4f rad/s, accel %.4f m/s^2 per sample (density "
          "%.4f / %.3f at %.1f Hz), bias gyro %.4f accel %.4f%s"
          % (sensor.range_noise, sensor.gyro_sigma, sensor.accel_sigma, args.gyro_noise,
             args.accel_noise, args.imu_hz, args.gyro_bias, args.accel_bias,
             "  [DISABLED]" if args.no_noise else ""), flush=True)
    out = RosOut(args.cloud_topic, args.imu_topic, args.frame)
    stop = install_stop_handler()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 << 20)
    except OSError:
        pass
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(0.5)

    # READER THREAD.  MEASURED 2026-10-13: casting 20 000 points takes ~15 ms, and while the main loop
    # was inside numpy the kernel buffer filled and the node silently lost 73 % of the tap -- which
    # starved the LIO, which made the FCU report "VisOdom: not healthy", which aborted every flight.
    # A thread that only reads the socket and appends parsed samples cannot be starved by the caster,
    # and the parsing is light enough to interleave with numpy's GIL-free stretches.
    import threading
    incoming = []
    stop_reader = threading.Event()

    def reader():
        while not stop_reader.is_set():
            try:
                data, _ = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            incoming.append(data)

    reader_thread = threading.Thread(target=reader, daemon=True)
    reader_thread.start()

    buf = TruthBuffer()
    scan_period_us = int(1e6 / args.scan_hz)
    # The IMU decimation depends on the rate the tap ACTUALLY arrives at -- never on a hardcoded 1200 Hz.
    # MEASURED 2026-10-13: this line used to be `round(1200.0 / args.imu_hz)`, but tools/run-mission.sh
    # starts truth_rx with `--fanout-decimate 6`, so the fanout arrives at 200 Hz and the IMU went out
    # every 6th sample = 33.3 Hz.  The run's own log said `imu 2538 (33.3 Hz)` while the LIO, the FSM and
    # every metric believed 200 Hz, so the LIO dead-reckoned on a 33 Hz IMU for the whole campaign.
    # The rate is now measured from the stream and printed before the first IMU message.
    imu_every = None
    imu_rate_hz = None
    rate_probe = []
    next_scan_us = None
    imu_counter = 0
    scans = 0
    skipped_scans = 0
    imu_published = 0
    points_total = 0
    t_start_wall = time.monotonic()
    last_report = t_start_wall
    print("sim/lidar_imu: tap udp/127.0.0.1:%d, %d points, mount pitch %.1f deg, "
          "scan %.1f Hz, imu %.1f Hz" % (args.port, len(rays), pitch, args.scan_hz, args.imu_hz),
          flush=True)
    print("  cloud -> %s (frame %s), imu -> %s" % (args.cloud_topic, args.frame, args.imu_topic),
          flush=True)

    try:
        while True:
            if not incoming:
                if time.monotonic() - t_start_wall > 20.0 and scans == 0:
                    print("FAIL: no tap samples in 20 s -- is truth_rx running with --fanout %d?"
                          % args.port, flush=True)
                    return 1
                time.sleep(0.001)
                continue
            data = incoming.pop(0)
            parts = data.decode("ascii", "replace").split()
            if len(parts) != 19 or parts[0] != "TAP":
                continue
            t_us = int(parts[2])
            p_ned = np.array([float(x) for x in parts[3:6]])
            v_ned = np.array([float(x) for x in parts[6:9]])
            q_ned = np.array([float(x) for x in parts[9:13]])
            gyro = np.array([float(x) for x in parts[13:16]])
            accel = np.array([float(x) for x in parts[16:19]])
            p_w, v_w, q_w = frames.wire_to_world(p_ned, v_ned, q_ned)
            buf.add(t_us, p_w, v_w, q_w, gyro, accel)

            if imu_every is None:
                rate_probe.append(t_us)
                if len(rate_probe) >= 40 and rate_probe[-1] > rate_probe[0]:
                    tap_hz = (len(rate_probe) - 1) * 1e6 / (rate_probe[-1] - rate_probe[0])
                    imu_every = imu_decimation(tap_hz, args.imu_hz)
                    imu_rate_hz = tap_hz / imu_every
                    print("  tap arrives at %.1f Hz -> one IMU sample every %d (%.1f Hz)"
                          % (tap_hz, imu_every, imu_rate_hz), flush=True)
                    if imu_rate_hz < 0.95 * args.imu_hz:
                        print("  WARNING: %.1f Hz IMU asked for, %.1f Hz delivered -- the tap cannot feed"
                              " it (raise the fan-out rate: truth_rx --fanout-decimate 1)"
                              % (args.imu_hz, imu_rate_hz), flush=True)

            # IMU: decimate the tap (never interpolate across real samples)
            imu_counter += 1
            if imu_every is not None and imu_counter % imu_every == 0:
                i = len(buf.t) - 1
                t_i, w_i, a_i = sensor.imu_sample(buf, i)
                out.imu(t_i, w_i, a_i)
                imu_published += 1
                out.spin_once()

            # first complete scan window
            if next_scan_us is None:
                next_scan_us = int(math.ceil(t_us / scan_period_us) * scan_period_us)
            if t_us >= next_scan_us + int(round(times[-1] * 1e6)) + 1000:
                if buf.covers(next_scan_us) and buf.covers(next_scan_us + int(times[-1] * 1e6)):
                    pts_s, idx, rng = sensor.scan(buf, next_scan_us)
                    out.cloud(next_scan_us, pts_s, times[idx], sensor.intensity)
                    scans += 1
                    points_total += len(pts_s)
                else:
                    # never cast a window the truth does not cover: pose_at would clamp and the scan
                    # would silently describe a vehicle that was not there
                    skipped_scans += 1
                next_scan_us += scan_period_us
                out.spin_once()

            now = time.monotonic()
            if now - last_report >= 2.0:
                last_report = now
                print("  scans %d, points/scan %d, cloud %.2f Hz, imu %d (%.1f Hz), cast %.1f ms"
                      % (scans, points_total // max(1, scans), scans / max(1e-9, now - t_start_wall),
                         imu_published, imu_published / max(1e-9, now - t_start_wall),
                         sensor.last_cast_ms), flush=True)

            if args.burst and scans >= args.burst:
                break
            if stop["flag"]:
                print("\nstop requested", flush=True)
                break
            if args.duration > 0 and buf.total > 1 and \
                    (buf.t[-1] - buf.t[0]) * 1e-6 >= args.duration and scans >= 1:
                break
    except KeyboardInterrupt:
        print("\ninterrupted", flush=True)
    finally:
        stop_reader.set()
        sock.close()

    wall = time.monotonic() - t_start_wall
    print("\nsim/lidar_imu report")
    print("  scans          %d in %.2f s wall (%.2f Hz)" % (scans, wall, scans / max(1e-9, wall)))
    print("  points         %d total, %d per scan (expected %d)"
          % (points_total, points_total // max(1, scans), len(rays)))
    print("  imu messages   %d (%.1f Hz)" % (imu_published, imu_published / max(1e-9, wall)))
    print("  imu rate       startup probe saw the tap at %.1f Hz, decimation %s, asked %.1f Hz ->"
          " %.1f Hz"
          % ((len(rate_probe) - 1) * 1e6 / max(rate_probe[-1] - rate_probe[0], 1) if len(rate_probe) > 1 else 0.0,
             imu_every if imu_every is not None else "-", args.imu_hz,
             imu_rate_hz if imu_rate_hz is not None else 0.0))
    print("  raycast        %.2f ms/scan (%.0f Hz capable)" % (sensor.last_cast_ms,
                                                               1000.0 / max(1e-9, sensor.last_cast_ms)))
    print("  truth samples  %d" % buf.total)
    print("  skipped scans  %d (windows the truth did not cover)" % skipped_scans)
    ok = scans >= 1 and abs(points_total / max(1, scans) - len(rays)) < 1 and skipped_scans == 0
    print("GATE: %s" % ("PASS" if ok else "FAIL"))
    out.shutdown()
    return 0 if ok else 1


# --------------------------------------------------------------------------- self-test
def _selftest(args) -> int:
    """No simulator, no ROS: check the interpolation and the cloud's frame arithmetic."""
    ok = True

    def check(name, got, want, tol=1e-9):
        nonlocal ok
        good = bool(np.allclose(got, want, atol=tol))
        ok = ok and good
        print("%-52s %s" % (name, "ok" if good else "FAIL got=%s want=%s" % (got, want)))

    # A body flying straight north at 1 m/s, level, sampled at 1200 Hz for 0.2 s.
    buf = TruthBuffer()
    for i in range(241):
        t = int(i * (1e6 / 1200))
        buf.add(t, [0.0, i / 1200.0, 0.1], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0], [0.0, 0.0, -9.80665])
    p, R = buf.pose_at(np.array([0, 100_000, 200_000]))
    check("interpolated position at t=0", p[0], [0, 0, 0.1])
    check("interpolated position at t=0.1 s", p[1], [0, 0.1, 0.1], 1e-9)
    check("interpolated position at t=0.2 s", p[2], [0, 0.2, 0.1], 1e-9)
    check("attitude is identity", R[1].reshape(-1), np.eye(3).reshape(-1), 1e-12)

    # Pose extrapolation outside the buffer must clamp rather than invent motion.
    p_out, _ = buf.pose_at(np.array([-50_000, 900_000]))
    check("clamped before the window", p_out[0], [0, 0, 0.1])
    check("clamped after the window", p_out[1], [0, 0.2, 0.1])

    # Mount arithmetic: with the housing pitched 45 deg nose-down, a point straight ahead of the
    # sensor at 1 m must appear 0.7071 m forward and 0.7071 m BELOW the housing origin.
    R_BS = frames.R_body_sensor(mount_pitch_deg=45.0)
    check("sensor x in body is 45 deg down", R_BS[:, 0], [math.cos(math.radians(45)), 0,
                                                          -math.sin(math.radians(45))], 1e-12)
    check("sensor z in body tilts forward", R_BS[:, 2], [math.sin(math.radians(45)), 0,
                                                         math.cos(math.radians(45))], 1e-12)

    # The full scan path: one scan from a level, stationary vehicle 2.5 m up in the middle of the
    # world, and the returned points must be in S -- i.e. rotating them back to W with the same pose
    # and adding the origin must reproduce the world-hitting rays (checked by re-casting one point).
    world = raycast.World.load(args.world)
    rays, times, _ = raycast.load_pattern(raycast.default_pattern_path(), n=2000)
    if times is None:
        times = np.linspace(0.0, 0.1, len(rays), endpoint=False)
    # frame arithmetic is checked with NO noise: `scan` returns the NOISELESS range in `rng` and applies
    # the radial noise to the point, so comparing norm(point) with rng only means something at sigma=0.
    # MEASURED 2026-10-13: with the default 10 mm range noise this check failed by 5.6 mm -- the returned
    # range is the one the raycast found, and the point carries the noisy one.  Both facts are now
    # asserted separately, so neither can change silently.
    sens = Sensor(world, rays, times, mount_pitch=45.0, range_noise=0.0,
                  gyro_noise_density=0.0, accel_noise_density=0.0, gyro_bias=0.0, accel_bias=0.0)
    sens_noisy = Sensor(world, rays, times, mount_pitch=45.0, range_noise=0.010,
                        gyro_noise_density=0.0, accel_noise_density=0.0, gyro_bias=0.0, accel_bias=0.0)
    buf2 = TruthBuffer()
    for i in range(241):
        t = int(i * (1e6 / 1200))
        buf2.add(t, [0.0, 0.0, 2.5], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, -9.80665])
    pts_s, idx, rng = sens.scan(buf2, 0)
    check("a stationary scan returns points", len(pts_s) > 100, True)
    check("ranges are positive", bool(np.all(rng > 0)), True)
    # independent reconstruction: take the returned S-frame point, push it through the same pose, and
    # it must lie exactly on the same range from the same origin
    R_w_s = frames.R_body_sensor(mount_pitch_deg=45.0)
    o_w = np.array([0.0, 0.0, 2.5]) + R_w_s @ frames.SENSOR_LEVER_ARM_B
    back = o_w + R_w_s @ pts_s[0]
    check("S -> W reconstruction keeps the range",
          float(np.linalg.norm(back - o_w)), float(rng[0]), 1e-9)
    # ... and with noise ON the point carries a radial offset of about the range sigma, in the same
    # direction, while `rng` still reports the raycast's own range
    pts_n, _, rng_n = sens_noisy.scan(buf2, 0)
    off = np.linalg.norm(pts_n - pts_s, axis=1)
    check("radial range noise is applied to the point, not to `rng`",
          bool(abs(float(np.median(off)) - 0.010) < 0.006 and float(np.max(off)) < 0.06), True)
    # and a motion-compensated scan must NOT equal a single-pose scan
    buf3 = TruthBuffer()
    for i in range(241):
        t = int(i * (1e6 / 1200))
        buf3.add(t, [0.0, 0.0, 2.5 + i * 0.0005], [0.0, 0.0, 0.6], [1.0, 0.0, 0.0, 0.0],
                 [0.0, 0.0, 0.0], [0.0, 0.0, -9.80665])
    pts_moving, _, rng_moving = sens.scan(buf3, 0)
    moved = float(np.max(np.abs(np.sort(rng_moving) - np.sort(rng))))
    check("a climbing vehicle's scan differs from a parked one (motion is simulated)",
          moved > 0.005, True)

    # the decimation must follow the rate that actually arrives (the 2026-10-13 defect: a 200 Hz fan-out
    # with a hardcoded 1200 Hz gave every 6th sample = 33.3 Hz, and the LIO integrated that)
    check("decimation from a full-rate tap", imu_decimation(1200.0, 200.0), 6)
    check("decimation from a decimated tap", imu_decimation(200.0, 200.0), 1)
    check("a tap slower than the wanted IMU rate publishes every sample",
          imu_decimation(100.0, 200.0), 1)
    check("never decimate to zero", imu_decimation(0.0, 200.0), 1)

    print("lidar_imu selftest:", "PASS" if ok else "FAIL")
    return 0 if ok else 1

if __name__ == "__main__":
    raise SystemExit(main())
