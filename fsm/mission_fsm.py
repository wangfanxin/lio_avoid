#!/usr/bin/env python3
"""Mission A: fly the rectangle with the LIO as the EKF's only source.

The phase machine, and the place where every finding of S3-S7 is enforced rather than remembered:

  preflight   wait for the LIO to be live (>= 5 Hz, no jumps, no divergence); MEASURE the L->F
              alignment from the parked data and compare it with the construction prediction; set the
              EKF origin (retrying, because at boot the FCU is on DCM); declare the frame; arm.
  takeoff     command the takeoff and stream ONLY external nav until the vehicle is airborne --
              sending position setpoints here cancels the takeoff (they replace GUIDED's SubMode).
  hold        a few seconds at the takeoff point, which is the A4 window.
  leg1..leg4  stream the path at 50 Hz, leg by leg, advancing on arrival.
  land        LAND, then wait for the disarm.
  abort       any gate firing: report, LAND, stop.

Every phase change is published on `/fsm/phase` AND appended to a CSV, because the metrics window
everything (A1-A4) is defined over is a phase interval -- a number without its window is not a number.

usage: fsm/mission_fsm.py --tap-port 14561 --phase-log out.csv [--bridge mavlink|mavros]
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
from common import frames                    # noqa: E402
from bridge.base import Mode                 # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0
GATES = {
    "lio_rate_min_hz": 5.0,
    "lio_jump_max_m": 0.50,
    "lio_speed_max_mps": 3.0,
    "lio_timeout_s": 1.0,
    "geofence_m": 4.60,
    "arrive_m": 0.06,
    "arrive_z_m": 0.35,
    "slow_radius_m": 0.30,
    "speed_mps": 0.25,
    "settle_s": 3.0,
    "settle_speed_mps": 0.08,
    "leg_timeout_s": 25.0,
    "rate_hz": 50.0,
}


class TapTruth:
    """The truth, as the FSM needs it: gates, the airborne test, and the phase log.

    Taken from `/sim/truth/odom` (about 90 Hz) rather than from the tap's fan-out port, and that is a
    MEASURED decision: only ONE process may bind a UDP port, and MEASURED 2026-10-13 the FSM and the
    sensor node were both bound to 14561, so the kernel split the fan-out stream ~50/50 between them --
    the sensor saw 2012 of 7808 lines, its IMU rate fell to 33 Hz, the LIO starved and every flight
    aborted with "VisOdom: not healthy".  A ~90 Hz truth is plenty for gates and for windowing; the
    sensor keeps the full-rate stream because per-point poses need it.
    """

    def __init__(self):
        self.t_us = 0
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.n = 0

    def on_odom(self, m):
        self.t_us = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        self.p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        # the VELOCITY matters as much as the position here: MEASURED 2026-10-13, streaming position
        # and attitude but no velocity left the EKF to dead-reckon velocity from the simulated
        # accelerometer (0.1 m/s^2/sqrt(Hz) of noise), its estimate lagged the truth by 0.5 m, and the
        # position loop chased the error until the geofence fired -- while the identical test that DID
        # pass the velocity held to +-0.03 m.
        self.v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        self.q = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                           m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        self.n += 1

    def poll(self):
        return None


class Mission:
    def __init__(self, args):
        self.args = args
        self.t_wall0 = time.monotonic()      # set FIRST: log() is called from the bridge's constructor
        self.tap = TapTruth()
        self.bridge = self._make_bridge()
        self.phase = "init"
        self.phase_t0 = time.monotonic()
        self.yaw_last = float(args.yaw)          # the heading actually streamed (velocity mode slews it)
        self.yaw_track_err = 0.0
        self.yaw_hold = False                    # set by the follower near a station: freeze the heading
        self.last_end_dist = float("nan")        # how far the vehicle was from the plan's end at completion
        self.stop_index = None                   # the sequencer's stop index (-1 = sequence complete)
        self.log_fh = open(args.phase_log, "w")
        self.log_fh.write("t_wall,t_sim_us,phase,cmd_x,cmd_y,cmd_z,lio_x,lio_y,lio_z,truth_x,truth_y,"
                          "truth_z\n")
        # the per-tick setpoint trace (see the comment where it is written): commanded position, velocity
        # and acceleration next to the FCU's own position/velocity and the LIO's, so a tracking question is
        # answered by the record instead of by inference from the SITL truth afterwards.
        self.sp_log = open(args.setpoint_log, "w") if args.setpoint_log else None
        if self.sp_log is not None:
            self.sp_log.write("t_wall,t_sim_us,cmd_x,cmd_y,cmd_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,"
                              "fcu_x,fcu_y,fcu_z,fcu_vx,fcu_vy,fcu_vz,lio_x,lio_y,lio_z,yaw_cmd\n")
        self.lio = {"t_us": 0, "p": np.zeros(3), "q": np.array([1.0, 0.0, 0.0, 0.0]),
                    "v": np.zeros(3), "n": 0, "rate": 0.0, "last_wall": 0.0,
                    "prev_p": None, "v_source": "diff", "v_seen": 0}
        self.path_f = None              # corners in F, after the L->F conversion
        # BOOTSTRAP with the PREDICTED alignment, then replace it with the measured one in preflight.
        # The prediction is not a fudge: it is the construction (spawn facing North, so L's axes are the
        # body's at init and L's origin is the housing then), and the measurement that follows is a
        # CHECK of it -- logged either way.  It is used first because the EKF wants external nav from
        # the moment the LIO is live, well before the parked window has been collected.
        self.calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
        self.calib_measured = None
        # A measured calibration, if one was fitted from a calibration flight.  Loaded HERE, in the
        # constructor: an earlier version loaded it inside preflight and returned early, which skipped
        # the EKF origin, the mode, the arming and the whole preflight -- the flight then aborted with
        # "takeoff refused" and looked like a takeoff problem.
        if args.calib_file and os.path.exists(args.calib_file):
            import json
            with open(args.calib_file) as fh:
                d = json.load(fh)
            self.calib = frames.Calib(R_WL=np.array(d["R_WL"]), t_WL=np.array(d["t_WL"]),
                                      n_samples=d.get("n_samples", 0),
                                      rms_m=d.get("rms_m", float("nan")))
            self.calib_measured = self.calib
        self.cmd = np.zeros(3)
        self.abort_reason = None
        self.node = None
        self.phase_pub = None

    # ------------------------------------------------------------------ plumbing
    def _make_bridge(self):
        if self.args.bridge == "mavlink":
            from bridge.mavlink_impl import MavlinkBridge
            return MavlinkBridge(url=self.args.mav, log=self.log)
        from bridge.mavros_impl import MavrosBridge
        return MavrosBridge(log=self.log)

    def log(self, msg):
        print("[%7.2f] %s" % (time.monotonic() - self.t_wall0, msg), flush=True)

    def start_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from nav_msgs.msg import Path
        from std_msgs.msg import String
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.node = Node("mission_fsm")
        best_effort = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        self.phase_pub = self.node.create_publisher(String, "/fsm/phase", 10)
        # THE PLAN SUBSCRIPTION'S DEPTH IS THE MISSION'S THROUGHPUT.  It was depth 1, and with RELIABLE +
        # depth 1 a publisher DISCARDS an unacked sample as soon as the next one is published.  MEASURED
        # 2026-10-14 (sortie AC1, the circle): tools/record-traj.py -- which subscribes with depth 100 --
        # recorded 41 /planner/traj messages while this FSM flew exactly ONE trajectory in the whole sortie
        # and then waited 60 s and aborted ("no new trajectory on /planner/traj within 60 s after 1
        # trajectories (sequencer at stop 2)").  The same pathology was measured on the goal hop the same day
        # (sequencer -> adapter: 42 published, 6-12 received; deepening both queues changed nothing there
        # because the ADAPTER is the slow one -- here the FSM is).  Depth 20 keeps TRANSIENT_LOCAL (so a
        # late-starting FSM still gets the latched plan) and costs nothing.
        latched = QoSProfile(depth=20, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Odometry, self.args.lio_topic, self._on_lio, best_effort)
        self.node.create_subscription(Odometry, self.args.lio_vel_topic, self._on_lio_vel, best_effort)
        self.node.create_subscription(Odometry, self.args.truth_topic,
                                      lambda m: self.tap.on_odom(m), best_effort)
        self.node.create_subscription(Path, self.args.traj_topic, self._on_path, latched)
        # THE SEQUENCER'S STATE.  The FSM does not count stops any more (see follow_spline_stations): it
        # reads the mission's own stop index, so the two cannot drift apart.  Volatile + reliable: the
        # sequencer publishes at 1 Hz and the value is a state, not an event.
        from std_msgs.msg import Int32
        self.node.create_subscription(Int32, self.args.stop_topic, self._on_stop, 10)
        self.rclpy = rclpy

    def _on_stop(self, m):
        """The sequencer's current stop index (0-based; -1 = the sequence is complete)."""
        prev = self.stop_index
        self.stop_index = int(m.data)
        if prev is None or prev != self.stop_index:
            self.log("sequencer stop index %d" % self.stop_index)

    def _on_lio(self, m):
        now = time.monotonic()
        p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        q = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                      m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        dt = max(1e-3, now - self.lio["last_wall"])
        self.lio.update(t_us=int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000,
                        p=p, q=q, n=self.lio["n"] + 1, last_wall=now,
                        rate=1.0 / dt if self.lio["n"] else 0.0)
        self.lio["rate_slow"] = 0.0 if self.lio["n"] == 0 else \
            (0.9 * self.lio.get("rate_slow", 0.0) + 0.1 * (1.0 / dt))
        # velocity: differentiate if no topic provides one (see _on_lio_vel)
        if self.lio["v_source"] == "diff":
            if self.lio["prev_p"] is not None and dt > 1e-3:
                raw = (p - self.lio["prev_p"]) / dt
                # a light low-pass: differencing a ~mm-noisy position at ~90 Hz is a noisy velocity,
                # and the EKF fuses it
                self.lio["v"] = 0.8 * self.lio["v"] + 0.2 * raw
            self.lio["prev_p"] = p.copy()

    def _on_lio_vel(self, m):
        """The velocity from `/lio/odom`.

        MEASURED 2026-10-13, and it cost a flight: Super-LIO fills ONLY the pose in
        `/lio/robo/odom` (`ROSWrapper.cpp:396-407` sets position and orientation, no twist), so that
        topic's velocity is identically ZERO.  The bridge was therefore telling the EKF "you are
        stationary" while also giving it a moving position, and with EK3_SRC1_VELXY = 6 the EKF believed
        the velocity: the position loop then ran away and the geofence stopped it at 4.6 m.  The
        truth-fed flights controlled to 1-2 mm because `/sim/truth/odom` DOES carry a velocity -- which
        is exactly why the difference did not show up until the LIO was the source.
        """
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        if float(np.linalg.norm(v)) > 1e-6:
            self.lio["v"] = v
            self.lio["v_source"] = "lio/odom"
            self.lio["v_seen"] += 1

    def _on_path(self, m):
        pts, ts = [], []
        for ps in m.poses:
            pts.append([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
            # THE PER-POINT STAMP IS THE TIME PARAMETERISATION.  `planner/ego_adapter.py` stamps every point
            # with the spline's start_time + that point's trajectory time, and the planner uses the SYSTEM
            # clock (`RCL_SYSTEM_TIME`, noted in the adapter), i.e. the same domain this node's wall clock is
            # in.  So a follower can recover the planned position, velocity AND acceleration at any t from
            # the Path alone -- no interface change, and no dependency on the planner's message type
            # (PLAN 4.1 keeps those out of the flight-critical code).
            ts.append(float(ps.header.stamp.sec) + 1e-9 * float(ps.header.stamp.nanosec))
        if pts:
            self.path_l = np.array(pts)
            self.path_t = np.array(ts)
            # HOW MANY PLANS ACTUALLY ARRIVE.  MEASURED 2026-10-14 (sortie AH1): the adapter published 38
            # plans, 20 with distinct stamps and 16 usable (arc >= 0.30 m, clearance >= 0.281 m), and this FSM
            # flew ONE -- so the acceptance decision is the last unexplained link and it needs a denominator.
            self.paths_seen = getattr(self, "paths_seen", 0) + 1

    def yaw_command(self, velocity, period):
        """The yaw to stream with this setpoint.

        `--yaw-mode fixed` (the historical behaviour, and what every mission-A number was flown with)
        streams `--yaw` unchanged -- MEASURED 2026-10-14 from the SITL tap's own attitude: the heading held
        -1.0 deg (B1) and -0.8 deg (B5), i.e. North, while the velocity azimuth swept the whole circle, so
        the aircraft flew the entry leg 90.4 deg sideways and spent 83 % of the circle more than 30 deg off
        its velocity vector (docs/metrics/phaseB-yaw-vs-velocity.txt).

        `--yaw-mode velocity` (the user's instruction: "yaw方向最好朝着飞行速度方向") points the nose along the
        horizontal velocity it is being asked to fly.  Three details matter:
          * the source is the PLAN's feed-forward velocity, not the measured one, so the nose leads the
            motion instead of chasing it;
          * below `--yaw-v-min` (0.2 m/s) the last heading is HELD, because the azimuth of a hovering
            vehicle is noise and a spinning yaw would feed back into the EKF's yaw observation;
          * the turn is SLEW LIMITED (`--yaw-rate-max`, 120 deg/s) so a stop-to-stop plan that reverses
            direction does not command an instantaneous 180 deg flip.

        Frame: the bridge writes this number straight into LOCAL_NED's yaw field, where 0 IS North and +90
        deg IS East -- the same convention as `--yaw` and as `atan2(v_E, v_N)`.

        ARRIVAL FREEZE (the user's rule, 2026-10-14: *"命令速度 <0.4 m/s 且 距当前站 <0.5 m 时，保持当时航向
        不转；离开站再开始跟速度"*).  MEASURED before it: with the nose following the velocity, each arrival
        command reversed the direction of travel, so the heading whipped round by up to 49 deg at up to
        68 deg/s while the vehicle was station-keeping -- the "spinning at the waypoint" the user saw.  The
        freeze is exactly the rule above: `yaw_hold` is set by the follower when the vehicle is within
        `--yaw-hold-m` of the trajectory's END (which IS the station) and the commanded speed is below
        `--yaw-v-min`; while it is set the last heading is streamed unchanged.
        """
        if self.args.yaw_mode != "velocity" or velocity is None:
            return self.args.yaw
        v_e, v_n = float(velocity[0]), float(velocity[1])
        speed = math.hypot(v_e, v_n)
        if self.yaw_hold or speed < self.args.yaw_v_min:
            return self.yaw_last
        want = math.atan2(v_e, v_n)
        d = (want - self.yaw_last + math.pi) % (2.0 * math.pi) - math.pi
        mx = math.radians(self.args.yaw_rate_max) * max(period, 1e-3)
        self.yaw_last = (self.yaw_last + max(-mx, min(mx, d)) + math.pi) % (2.0 * math.pi) - math.pi
        self.yaw_track_err = d
        return self.yaw_last

    def pump(self, seconds=None, setpoint=None, until=None, velocity=None, accel=None):
        """The main loop body: poll everything, stream at 50 Hz, run the gates."""
        period = 1.0 / GATES["rate_hz"]
        t_end = None if seconds is None else time.monotonic() + seconds
        n = 0
        while True:
            if t_end is not None and time.monotonic() >= t_end:
                break
            if until is not None and until():
                break
            self.tap.poll()
            self.bridge.poll()
            if self.node is not None:
                self.rclpy.spin_once(self.node, timeout_sec=0.0)
            now = time.monotonic()
            if now >= self.next_stream:
                self.next_stream = now + period
                self.stream_external_nav()
                yaw = self.yaw_command(velocity, period)
                # WHICH FIELDS OF THE SETPOINT GO OUT.  The three arms exist because the tracking error is
                # the open question: MEASURED 2026-10-14, the estimate is good (A4 18-20 mm) and the plans
                # are legal (100 % of the contract-breaking samples followed a plan that HELD it), while the
                # AIRCRAFT sits 0.13-0.19 m from the command it is being streamed at 0.64-0.69 m/s.  If
                # ArduPilot ignores or limits our velocity feed-forward it re-derives its own speed profile,
                # which would look exactly like this: a smooth, slowly varying error with no fixed lag.
                # ArduPilot honours position + velocity; the acceleration field is the one whose effect is
                # unproven, so all three are selectable and the A/B is one flight each.
                mode = self.args.setpoint_mode
                vel = None if mode == "pos" else velocity
                acc = None if mode != "posvelacc" else accel
                if vel is not None and setpoint is not None and acc is not None:
                    # THE TIME-PARAMETERISED FEED: the plan says where it wants to be, how fast it wants to
                    # be going, and how hard it intends to accelerate.  Sending all three is what makes the
                    # FCU follow the planner's profile instead of re-deriving one from WP_SPD/WP_ACC.
                    self.cmd = np.asarray(setpoint, dtype=float)
                    self.bridge.send_position_velocity(self.cmd, vel, yaw, acc)
                elif vel is not None and setpoint is not None:
                    self.cmd = np.asarray(setpoint, dtype=float)
                    self.bridge.send_position_velocity(self.cmd, vel, yaw)
                elif vel is not None:
                    self.bridge.send_velocity(vel, yaw)
                elif setpoint is not None:
                    self.cmd = np.asarray(setpoint, dtype=float)
                    self.bridge.send_position(self.cmd, yaw)
                n += 1
                # THE SETPOINT TRACE.  The phase log records only the command in force when a PHASE changes,
                # which is enough for the metrics and useless for a tracking question: MEASURED 2026-10-14,
                # the FCU flew at 1.28-1.31 m on B5 and 3.28 m on B2b while the mission commanded 2.00 m,
                # and there was no record of what was actually streamed at those instants -- only of what
                # the SITL truth and the FCU's own log said afterwards.  This writes the commanded setpoint
                # next to the FCU's and the LIO's answers, every stream tick.
                if self.sp_log is not None:
                    st_sp = self.bridge.state()
                    self.sp_log.write(
                        "%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n" % (
                            time.monotonic(), self.tap.t_us,
                            self.cmd[0], self.cmd[1], self.cmd[2],
                            (velocity[0] if velocity is not None else 0.0),
                            (velocity[1] if velocity is not None else 0.0),
                            (velocity[2] if velocity is not None else 0.0),
                            (accel[0] if accel is not None else 0.0),
                            (accel[1] if accel is not None else 0.0),
                            (accel[2] if accel is not None else 0.0),
                            st_sp.pos[0], st_sp.pos[1], st_sp.pos[2],
                            st_sp.vel[0], st_sp.vel[1], st_sp.vel[2],
                            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
                            yaw))
                self.check_gates()
                if self.abort_reason:
                    return n
            time.sleep(0.001)
        return n

    def stream_external_nav(self):
        """Feed the FCU.  Two sources, and the choice is explicit because it decides what is proven.

        `--nav-source lio`  the mission: the LIO's odometry, L -> F with the measured calibration.
                            This is the configuration mission A is defined by.
        `--nav-source truth` the CALIBRATION flight: the truth tap converted W -> NED, exactly as
                            tools/test-fcu-link.py does.  It flies correctly, and its RECORDING is what
                            identifies L -> F -- a parked window cannot identify a rotation at all
                            (MEASURED: fitting one gave a 90 deg error and the aircraft ran away East
                            until the geofence caught it).  Nothing else uses this mode: the mission
                            itself must never fly on truth.
        """
        if self.args.nav_source == "truth":
            if self.tap.n == 0:
                return
            self.bridge.send_external_nav(self.tap.t_us, self.tap.p, self.tap.q, self.tap.v)
            return
        if self.lio["n"] == 0:
            return
        R, t = self.calib.R_WL, self.calib.t_WL
        p_f = R @ self.lio["p"] + t
        q_f = frames.q_from_R(R @ frames.R_from_q(self.lio["q"]))
        v_f = R @ self.lio["v"]
        self.bridge.send_external_nav(self.lio["t_us"], p_f, q_f, v_f)

    def calibration_flight(self):
        """A short shaped path on TRUTH-fed nav, purely so the recordings identify L -> F.

        The path is two perpendicular legs: a single straight line leaves the yaw ambiguous by 180 deg,
        so the shape is the point of the exercise (tools/lio-vs-truth.py reports the same identifiability
        numbers this relies on).
        """
        self.set_phase("calib")
        base = self.tap.p.copy()
        legs = [(base + np.array([1.0, 0.0, 0.0]), 4.0),
                (base + np.array([1.0, 1.0, 0.0]), 4.0),
                (base + np.array([0.0, 1.0, 0.0]), 4.0)]
        self.log("calibration flight: %.2f m East, then North, then West, at %.1f m"
                 % (1.0, base[2]))
        for i, (target, secs) in enumerate(legs, start=1):
            self.set_phase("calib_leg%d" % i)
            self.pump(secs, setpoint=target)
            if self.abort_reason:
                return self.abort_reason
            self.log("   leg %d: truth at [%+.3f %+.3f %+.3f], FCU at [%+.3f %+.3f %+.3f]"
                     % (i, *self.tap.p, *self.bridge.state().pos))
        return None

    def check_gates(self):
        st = self.bridge.state()
        reasons = []
        if self.lio["n"] > 5 and self.lio.get("rate_slow", 99.0) < GATES["lio_rate_min_hz"]:
            reasons.append("lio rate %.1f Hz < %.1f" % (self.lio["rate_slow"],
                                                        GATES["lio_rate_min_hz"]))
        if self.tap.n > 0 and (abs(self.tap.p[0]) > GATES["geofence_m"] or
                               abs(self.tap.p[1]) > GATES["geofence_m"]):
            reasons.append("geofence: truth at [%+.2f %+.2f]" % (self.tap.p[0], self.tap.p[1]))
        if st.armed and st.mode == "LAND" and self.phase not in ("land", "done"):
            reasons.append("the FCU left the mission mode (%s)" % st.mode)
        if reasons:
            self.abort_reason = "; ".join(reasons)

    def phase_mark(self, name):
        """A one-shot marker in the phase log, for events that are not phases.

        `--stations N` uses it after every stop, so the metrics can count how many stops the flight actually
        completed and where each one ended -- the "多少分之多少" the user asked for.
        """
        self.log("mark -> %s" % name)
        self.log_fh.write("%.3f,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
            time.monotonic(), self.tap.t_us, name, self.cmd[0], self.cmd[1], self.cmd[2],
            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
            self.tap.p[0], self.tap.p[1], self.tap.p[2]))
        self.log_fh.flush()

    # ------------------------------------------------------------------ phases
    def set_phase(self, phase):
        if phase == self.phase:
            return
        self.phase = phase
        self.phase_t0 = time.monotonic()
        self.log("phase -> %s" % phase)
        self.log_fh.write("%.3f,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
            time.monotonic(), self.tap.t_us, phase, self.cmd[0], self.cmd[1], self.cmd[2],
            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
            self.tap.p[0], self.tap.p[1], self.tap.p[2]))
        # NOTE the second column: the SIM time (from the truth odometry's own stamp).  The recordings
        # (tap.csv, lio.csv) are indexed by sim time, so a phase log with only wall-clock times cannot
        # window them -- which is why B2 printed nothing at all in the first complete flight.
        self.log_fh.flush()
        if self.phase_pub is not None:
            from std_msgs.msg import String
            m = String()
            m.data = phase
            self.phase_pub.publish(m)

    def preflight(self):
        self.set_phase("preflight")
        self.log("navigation source: %s%s" % (self.args.nav_source,
                                              "" if self.args.nav_source == "lio"
                                              else "  (CALIBRATION flight: this is not mission A)"))
        self.log("waiting for the LIO on %s" % self.args.lio_topic)
        # NOTE: `until` ends the pump early, `seconds` is the timeout -- passing a small number here
        # ended the wait after 0.1 s and aborted every flight with "no LIO odometry" (measured).
        self.pump(25.0, until=lambda: self.lio["n"] > 20)
        if self.lio["n"] <= 20:
            return "no LIO odometry"
        self.log("LIO live: %d messages, p_L = [%+.3f %+.3f %+.3f], rate %.1f Hz"
                 % (self.lio["n"], *self.lio["p"], self.lio.get("rate_slow", 0.0)))
        self.pump(1.0)
        self.log("velocity source: %s (%d samples with a non-zero velocity); if 'diff' the FSM is "
                 "differentiating the pose, which is noisier" % (self.lio["v_source"],
                                                                 self.lio["v_seen"]))

        if self.args.calib_file and os.path.exists(self.args.calib_file):
            self.log("L->F from %s: %s" % (self.args.calib_file, self.calib.summary()))
        if self.args.nav_source == "truth":
            self.log("calibration flight: L->F will be fitted from THIS run's recordings, so the "
                     "parked window is used only for the translation")
        # MEASURE what a parked window CAN identify: the translation.  NOT the rotation.
        #
        # MEASURED 2026-10-13, and it cost a flight: fitting yaw+translation to a stationary window
        # returned a yaw 90 deg away from the construction -- because with no motion the yaw is
        # completely unidentifiable and the fit lands anywhere (the same identifiability trap that
        # tools/lio-vs-truth.py already guards against, unapplied here).  The FSM then streamed odometry
        # rotated by that bogus yaw, the EKF saw impossible position innovations, said "EKF variance"
        # and failsafed to LAND.  So: the rotation comes from the construction (spawn facing North:
        # L's axes are the body's at init), the translation is measured here, and the ROTATION is
        # verified separately from the flight data, where the trajectory's shape identifies it.
        samples = []
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            self.pump(0.02)
            if self.tap.n > 0 and self.lio["n"] > 0:
                samples.append((self.lio["p"].copy(), self.tap.p.copy()))
        if len(samples) < 20:
            return "not enough parked samples to measure the alignment"
        L = np.array([s[0] for s in samples])
        W = np.array([s[1] for s in samples])
        predicted = self.calib
        R = predicted.R_WL
        t = W.mean(axis=0) - R @ L.mean(axis=0)
        resid = (R @ L.T).T + t - W
        measured = frames.Calib(R_WL=R, t_WL=t, n_samples=len(samples),
                                rms_m=float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))))
        self.calib = measured
        self.calib_measured = measured
        self.log("L->F: rotation from the construction (%s)" % predicted.summary())
        self.log("      translation measured from %d parked samples: t = [%+.4f %+.4f %+.4f] m, "
                 "residual rms %.4f m" % (len(samples), t[0], t[1], t[2], measured.rms_m))
        self.log("      (the parked mismatch to the prediction is [%+.3f %+.3f %+.3f] m in F; the "
                 "rotation gets checked on the flight data, where the path shape identifies it)"
                 % tuple(t - predicted.t_WL))

        self.log("setting the EKF origin")
        if not self.bridge.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=self.pump):
            return "EKF origin never accepted"
        self.pump(0.5)
        st = self.bridge.state()
        self.log("FCU: armed=%s mode=%s pos=[%+.3f %+.3f %+.3f] alt AMSL %.1f m gps_fix=%d"
                 % (st.armed, st.mode, *st.pos, st.alt_amsl_m, st.gps_fix))
        if st.gps_fix > 1:
            return "the FCU has a GPS fix (%d) -- mission A is defined as no-GPS" % st.gps_fix
        if not self.bridge.set_mode(Mode.GUIDED):
            return "could not enter GUIDED"
        if not self.bridge.calibrate_accel_simple():
            self.log("note: simple accel calibration not ACKed (may already be calibrated)")
        self.pump(0.5)
        if not self.bridge.arm(timeout_s=25.0):
            return "arming refused: %s" % _texts(st)
        self.pump(2.0)
        st = self.bridge.state()
        if not st.armed:
            return "the FCU does not report itself armed"
        self.log("armed; landed_state=%d" % st.landed_state)
        return None

    def takeoff(self):
        self.set_phase("takeoff")
        if not self.bridge.takeoff(self.args.alt):
            return "takeoff refused: %s" % _texts(self.bridge.state())
        z0 = float(self.tap.p[2])
        self.log("takeoff commanded to %.2f m; streaming external nav ONLY until airborne" % self.args.alt)
        self.pump(12.0, until=lambda: self.tap.p[2] > z0 + 0.25)
        if self.tap.p[2] <= z0 + 0.25:
            return "never left the ground"
        self.log("airborne: truth altitude %.3f m" % self.tap.p[2])
        return None

    def hold(self, seconds):
        self.set_phase("hold")
        # the body origin sits ~0.10 m above the pad and the EKF origin is the ground plane, so at rest
        # the FCU's local z reads ~0.08-0.10 while W reads the same: the commanded altitude is an
        # absolute height in W, not an increment (measured in the S7 link test: commanding 0.80 m
        # settled the truth at 0.86-0.91 m, and the FCU's own z at 0.77 because STRICTLY the altitude
        # channel is baro-fed in step (a) and carries its own bias -- which is what A2's "mean removed"
        # is for).
        target = np.array([self.tap.p[0], self.tap.p[1], self.args.alt])
        self.log("holding at [%+.3f %+.3f %+.3f] for %.1f s" % (*target, seconds))
        self.pump(seconds, setpoint=target)
        return None

    def follow_spline(self):
        """Follow the trajectory the way the PLANNER parameterised it: position + velocity feed-forward at t.

        This is what `traj_server` does upstream, and it is the honest consumer of a time-parameterised
        B-spline.  Our `--follow path` instead throws the timing away and re-derives speed from a look-ahead,
        which measured a 12x too-slow first version (`0.45 + 0.6 v`) and then corner cutting (`1.2 + 0.8 v`:
        C2 0.574 m where the plan's own margin was 0.62 m).  Feeding the planned velocity as well removes the
        reason for either compromise: the FCU is told how fast the plan wants to be going, not just where it
        should be.

        Implementation: the Path's per-point stamps ARE the trajectory timeline (`t0 + t_i`, system clock,
        same domain as this node), so the reference at wall time `now` is obtained by interpolating the
        path IN TIME -- position, and by differencing, velocity and acceleration.  ArduPilot honours
        position + velocity in `SET_POSITION_TARGET_LOCAL_NED`; the acceleration field is sent too, but for
        a copter it is generally ignored (`PSC` uses the velocity target and its own accel limits), so the
        A/B below measures what the FCU actually does rather than what the message could carry.

        `--stations N > 1` flies a SEQUENCE: one trajectory per commanded stop, each one waited for
        separately (see `follow_spline_stations`).  That is mission B's shape: the goal sequencer feeds ego
        one stop at a time and the FSM consumes the trajectory each stop produces.
        """
        if self.args.stations > 1:
            return self.follow_spline_stations(self.args.stations)
        if not self._wait_for_spline(None):
            return self.abort_reason or ("no trajectory on %s after %.0f s (a planner that never plans "
                                         "looks exactly like this)"
                                         % (self.args.traj_topic, self.args.traj_wait_s))
        return self._spline_once()

    def _traj_usable(self):
        """Is the trajectory in hand one the FSM should fly?  Length is the test, and it is measured.

        MEASURED 2026-10-14 (the first 12-stop attempt): the adapter republishes the CURRENT spline at 4 Hz
        from `t_cur` onward, so when a plan is nearly exhausted what it publishes is a STUB -- 23 points,
        0.43 s, 0.17 m.  The FSM's freshness test (a different `path_t[0]`) is satisfied by any new Path, so
        it took that stub as "station 2", "flew" it in 2.1 s and burned a station of the mission on 17 cm.
        A trajectory shorter than `--min-traj-m` is therefore not a station's plan and is skipped.
        """
        if self.path_l is None or self.path_t is None:
            return False
        if len(self.path_l) < 2:
            return False
        seg = float(np.sum(np.linalg.norm(np.diff(self.path_l[:, :2], axis=0), axis=1)))
        return seg >= self.args.min_traj_m

    def _traj_starts_here(self):
        """Does the plan in hand start where the vehicle is?

        THE FRESHNESS TEST ALONE STARVES THE MISSION.  `_wait_for_spline` accepts a plan only when its
        first-point stamp differs from the last one flown, on the assumption that every plan the planner
        publishes is new.  MEASURED 2026-10-14 (sorties AA1-AD1, the circle): once the vehicle comes to rest,
        ego's commits collapse to ~0.2 Hz (it enters WAIT_TARGET by design on arrival and each goal re-arms it
        only once), so the adapter re-publishes THE SAME plan 4 times a second -- re-projected onto the
        vehicle's CURRENT position every time, because that is what planner/ego_adapter.py's build_path does.
        The FSM therefore threw away 19 of the 20 distinct plans the recorder saw and waited out its 60 s:
        "ABORT: no new trajectory on /planner/traj within 60 s after 1 trajectories (sequencer at stop 2)".
        A re-projected plan IS a usable plan: its first point is at the vehicle.  This is the test for that.
        """
        if self.path_l is None or len(self.path_l) < 2:
            return False
        st = self.bridge.state()
        here = np.array(st.pos)
        return float(np.linalg.norm(here[:2] - self.path_l[0, :2])) < self.args.reaccept_m

    def _wait_for_spline(self, last_t0):
        """Wait for a trajectory, optionally one NEWER than `last_t0` (its first point stamp).

        Returns True when a usable trajectory is in hand.  A planner's trajectory is ASYNCHRONOUS, so the
        wait is what makes the same FSM work for a source that answers instantly (the rectangle) and for one
        that does not; MEASURED 2026-10-14, without it the spline arm aborted at 16 s with "no trajectory"
        while the planner was still initialising.
        """
        def ok():
            if not self._traj_usable():
                return False
            if last_t0 is None:
                return True
            if self.path_t is not None and float(self.path_t[0]) != float(last_t0):
                return True
            # ... or a re-published copy that the adapter has re-projected onto the vehicle (see
            # _traj_starts_here): MEASURED, without this the FSM flies 1 of 20 published plans.
            return self.args.reaccept_m > 0.0 and self._traj_starts_here()

        if self.path_l is not None and ok():
            return True
        # HOLD AT A VERIFIED POINT WHILE WAITING.  MEASURED 2026-10-14 (sortie AE1, the circle): this loop
        # streamed NOTHING while it waited (`pump` with no setpoint sends no setpoint), so the aircraft stayed
        # wherever the last fragment had left it -- and the truth spent 80.4 % of its airborne samples in a
        # 0.1 x 0.16 m box at (E 4.00..4.10, N 0.31..0.47), 0.35 m north of the stop and only 0.12-0.19 m from
        # pillar #52's surface, i.e. BELOW the 0.20 m contract for most of the flight, while every plan the
        # adapter published was >= 0.210 m clear.  The endpoint of the last usable plan IS a verified point
        # (the adapter's gate guarantees it), so the wait holds the vehicle there with zero velocity instead
        # of letting it sit wherever a time-expired fragment ended.
        hold_pt = None
        if self.path_l is not None and len(self.path_l) >= 2:
            hold_pt = np.array(self.path_l[-1], dtype=float)
        t_wait = time.monotonic()
        t_diag = 0.0
        self.log("waiting up to %.0f s on %s for %s"
                 % (self.args.traj_wait_s, self.args.traj_topic,
                    "a trajectory" if last_t0 is None else "the NEXT stop's trajectory"))
        while time.monotonic() - t_wait < self.args.traj_wait_s:
            if self.abort_reason:
                return False
            if self.path_l is not None and ok():
                self.log("trajectory arrived after %.1f s: %d points, t0 %.3f%s"
                         % (time.monotonic() - t_wait, len(self.path_l),
                            float(self.path_t[0]) if self.path_t is not None else float("nan"),
                            " (a re-projected copy, accepted because it starts at the vehicle)"
                            if last_t0 is not None and self.path_t is not None
                            and float(self.path_t[0]) == float(last_t0) else ""))
                return True
            if hold_pt is not None and self._hold_is_legal(hold_pt):
                self.pump(0.1, setpoint=hold_pt, velocity=np.zeros(3))
            else:
                self.pump(0.1)
            # WHY IS THE PLAN IN HAND NOT ACCEPTED?  Once a second, say which test refuses it -- the same
            # discipline that resolved every other link in this chain (see
            # docs/metrics/phaseB-plan-supply-loop-20261014.txt: 16 usable plans published, 1 flown).
            if time.monotonic() - t_diag >= 1.0:
                t_diag = time.monotonic()
                seen = getattr(self, "paths_seen", 0)
                if self.path_l is None or len(self.path_l) < 2:
                    self.log("DIAG: no plan in hand (%d message(s) received)" % seen)
                else:
                    arc = float(np.sum(np.linalg.norm(np.diff(self.path_l[:, :2], axis=0), axis=1)))
                    here = np.array(self.bridge.state().pos)
                    d0 = float(np.linalg.norm(here[:2] - self.path_l[0, :2]))
                    same = (last_t0 is not None and self.path_t is not None
                            and float(self.path_t[0]) == float(last_t0))
                    why = ("arc %.3f < %.2f m" % (arc, self.args.min_traj_m) if arc < self.args.min_traj_m
                           else ("stamp unchanged and the first point is %.3f m away (need < %.2f)"
                                 % (d0, self.args.reaccept_m) if same and d0 >= self.args.reaccept_m
                                 else "accepted on the next evaluation"))
                    self.log("DIAG: %d message(s), arc %.3f m, %d points, first point %.3f m from the vehicle,"
                             " stamp %s -> refused because: %s"
                             % (seen, arc, len(self.path_l), d0,
                                "same as the last flown" if same else "new", why))
        return False

    def _load_world(self):
        """The adapter's own world model, imported rather than re-implemented (same rule as tools/record-traj).

        Loaded LAZILY and by PATH: planner/ego_adapter.py is not a ROS package, and importing it by path is
        what tools/record-traj.py does for the same reason.  A failure here must never stop a flight -- the
        hold simply falls back to "stream nothing", which is the safe behaviour.
        """
        try:
            import importlib.util
            import os as _os
            p = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "planner", "ego_adapter.py")
            spec = importlib.util.spec_from_file_location("ego_adapter_fsm", p)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            if not _os.path.exists(self.args.world_file):
                self.log("no world file at %s -- the wait-loop hold is disabled" % self.args.world_file)
                return None
            return mod.World.load(self.args.world_file)
        except Exception as exc:                              # noqa: BLE001
            self.log("could not load the world model (%s) -- the wait-loop hold is disabled" % exc)
            return None

    def _hold_is_legal(self, target):
        """Is pulling straight at `target` legal?  The straight line is the whole question.

        MEASURED 2026-10-14, three sorties that differ only in this decision (the circle):
          AF1  hold always at the last plan's endpoint: 20 trajectories flown, 8 of 12 stops REACHED,
               sequence complete, landed -- and C2 min -0.376 m with 14.03 % of the samples INSIDE a pillar,
               because the endpoint belonged to another stop and the aircraft flew STRAIGHT at it;
          AG1  hold only within 0.50 m:                 1 trajectory flown, 1 of 12 stops, C2 min +0.340 m,
               0 of 87032 samples below the contract -- legal and starved;
          AE1  no hold at all:                          3 trajectories, 1 of 12 stops, same starvation.
        The mission advances only while the VEHICLE MOVES (ego's replanning follows the vehicle's state, and
        the adapter refuses plans whose nearest point is far from it), and the only motion the FSM can create
        on its own is this pull.  So the pull must be gated on what actually makes it illegal: the straight
        segment from the vehicle to `target`.  Sample it and check it against the same world model the
        adapter's gate uses; if the segment is clear, the pull is legal motion and keeps the mission fed.
        """
        if getattr(self, "_world", None) is None and not getattr(self, "_world_tried", False):
            self._world_tried = True
            self._world = self._load_world()
        if self.path_l is None or getattr(self, "_world", None) is None:
            return False
        here = np.array(self.bridge.state().pos)
        seg = np.linspace(here, np.asarray(target, dtype=float), 24)[:, :3]
        try:
            return float(self._world.clearance(seg).min()) >= self.args.footprint_contract
        except Exception:                                     # noqa: BLE001  (never block the flight on this)
            return False

    def follow_spline_stations(self, n):
        """Fly the mission's stops in sequence, and land when the SEQUENCER says the sequence is over.

        WHY THIS IS DRIVEN BY THE SEQUENCER AND NOT BY COUNTING TRAJECTORIES.  The first version counted the
        trajectories it flew (`--stations N`) and matched freshness by the plan's start stamp.  MEASURED
        2026-10-14 on four consecutive sorties: that counter drifts out of step with the stops the mission
        actually commanded -- a stale plan still passes the freshness test after the FSM has moved on, so the
        FSM ends up waiting for a "next" trajectory that the sequencer has already stopped sending, and the
        flight aborts on its wait (B2b at station 8, B3 at 8, B4 and B5 at 4, B7 at 4) even though the
        vehicle and the planner were both healthy.

        The sequencer already knows exactly where the mission is: it publishes its current stop index on
        `--stop-topic` (`std_msgs/Int32`, -1 = sequence complete).  This loop therefore does the simple,
        checkable thing: follow each fresh trajectory to its end, print which stop it was, and stop when the
        sequencer says there are no more.  Stop bookkeeping (reached / missed, and with what error) stays in
        ONE place -- the sequencer's own log.
        """
        self.log("following trajectories until the sequencer says the sequence is done "
                 "(stop topic %s); %d stops expected" % (self.args.stop_topic, n))
        last_t0 = None
        flown = 0
        while True:
            if self.abort_reason:
                return self.abort_reason
            if self.stop_index is not None and self.stop_index < 0:
                self.log("sequence complete: %d trajectories flown, the sequencer reports every stop done"
                         % flown)
                return None
            if not self._wait_for_spline(last_t0):
                if self.stop_index is not None and self.stop_index < 0:
                    return None
                return ("no new trajectory on %s within %.0f s after %d trajectories (sequencer at stop %s)"
                        % (self.args.traj_topic, self.args.traj_wait_s, flown, self.stop_index))
            last_t0 = float(self.path_t[0]) if self.path_t is not None else None
            t_stop = time.monotonic()
            stop_at_start = self.stop_index
            rc = self._spline_once()
            flown += 1
            if rc:
                return "trajectory %d: %s" % (flown, rc)
            self.log("trajectory %d done in %.1f s (sequencer now at stop %s)"
                     % (flown, time.monotonic() - t_stop, self.stop_index))
            self.phase_mark("traj%d" % flown)
            # HOLD THE STATION WHILE THE SEQUENCER DWELLS.  MEASURED 2026-10-14 (sortie C3, the user's
            # "hover 10 s" mission): with nothing streamed between trajectories the FCU kept flying to the
            # LAST TARGET, ego kept replanning to the same stop, this loop kept following each new plan, and
            # the aircraft DRIFTED out of the arena -- 12.2 % of the truth samples were outside the walls
            # (N down to -6.89 against a wall at -6.00) while 0 of 3480 published plan points were.  A dwell
            # means standing still, so hold the position the vehicle is at NOW, at 50 Hz, until the
            # sequencer moves on.
            #
            # AND ONLY ON A REAL ARRIVAL.  MEASURED 2026-10-14 (sortie C4): gating the hold on "the stop
            # index did not change" alone froze the aircraft 4.808 m short of the stop, because ego had
            # published a short hop (5.8 s) and the index had simply not advanced yet.  The hold therefore
            # also requires that the trajectory ended where the VEHICLE is -- `last_end_dist`, the distance
            # from the vehicle to the plan's last point at the moment the trajectory completed.
            if self.stop_index is not None and self.stop_index == stop_at_start \
                    and self.last_end_dist < self.args.hold_at_m:
                t_hold = time.monotonic()
                st_h = self.bridge.state()
                hold_pt = np.array(st_h.pos)                 # hold where it ARRIVED, not where it drifts to
                self.log("holding the station (%.3f m from the plan's end) until the sequencer moves on "
                         "(up to %.0f s)" % (self.last_end_dist, self.args.hold_max_s))
                while self.stop_index == stop_at_start:
                    if self.abort_reason:
                        return self.abort_reason
                    self.pump(0.05, setpoint=hold_pt, velocity=np.zeros(3))
                    if time.monotonic() - t_hold > self.args.hold_max_s:
                        self.log("the sequencer did not move on within %.0f s; continuing"
                                 % self.args.hold_max_s)
                        break
                self.log("station hold ended after %.1f s (sequencer at stop %s)"
                         % (time.monotonic() - t_hold, self.stop_index))

    def _spline_once(self):
        """Follow the trajectory currently in `self.path_l` / `self.path_t` in TIME, to its end."""
        if self.path_t is None or len(self.path_t) != len(self.path_l):
            return ("the Path carries no usable per-point stamps (%s), so the time parameterisation cannot "
                    "be recovered -- use --follow path for a geometric source like the rectangle"
                    % ("none" if self.path_t is None else "%d stamps for %d points"
                       % (len(self.path_t), len(self.path_l))))
        R, t = self.calib.R_WL, self.calib.t_WL

        def snapshot():
            """The plan in hand, in F, with its time base: (points, times, duration)."""
            q = (R @ self.path_l.T).T + t
            ts = self.path_t - self.path_t[0]             # seconds from this trajectory's start
            return q, ts, float(ts[-1])

        p, tt, dur = snapshot()
        if dur <= 1e-6:
            return "the Path's stamps span %.6f s -- not a trajectory" % dur
        self.log("follow-spline: %d points, %.2f s of trajectory (%.2f m), stamps in the system clock"
                 % (len(p), dur, float(np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)))))

        def ref_at(pos_p, times, t_rel):
            """(position, velocity, acceleration) of the plan at t_rel seconds from its start."""
            # WHEN THE PLAN IS EXHAUSTED THE FEEDFORWARD MUST GO TO ZERO.  MEASURED 2026-10-14 (sortie F2,
            # the diagonal): the position was clamped to the plan's last point -- but `vel` was the LAST
            # SEGMENT's velocity, so the FSM kept streaming "sit at (-1.79,-1.81) AND fly at 2.43 m/s
            # north-west".  ArduPilot's GUIDED position+velocity setpoint follows the velocity feedforward,
            # so the aircraft flew on at 2.43 m/s, passed the stop, left the arena and tripped the geofence
            # gate: "ABORT: trajectory 1: geofence: truth at [-6.54 -6.64]" -- 8 m of flight on a command
            # that had stopped 4.18 m behind it.  Sortie F3 did the same at its first circle stop
            # ("geofence: truth at [+6.60 -0.25]").  The bug was always there; it only became destructive
            # once the plans got fast, which is what unpinning manager/max_vel did (plans went from a
            # 1.0 m/s time allocation to the params file's 3.0).
            if t_rel >= times[-1] - 1e-9:
                return np.array(pos_p[-1], dtype=float), np.zeros(3), np.zeros(3)
            k = int(np.clip(np.searchsorted(times, t_rel, side="right") - 1, 0, len(times) - 2))
            h = max(times[k + 1] - times[k], 1e-6)
            w = float(np.clip((t_rel - times[k]) / h, 0.0, 1.0))
            pos = pos_p[k] + w * (pos_p[k + 1] - pos_p[k])
            vel = (pos_p[k + 1] - pos_p[k]) / h
            if 0 < k < len(times) - 2:
                acc = ((pos_p[k + 2] - pos_p[k + 1]) / max(times[k + 2] - times[k + 1], 1e-6) - vel) / h
            else:
                acc = np.zeros(3)
            return pos, vel, acc

        self.set_phase("traj")
        t_start = time.monotonic()
        t_wall0 = time.time()                              # the stamps are wall-clock seconds
        t0_cur = float(self.path_t[0])
        n_switch = 0
        while True:
            if self.abort_reason:
                return self.abort_reason
            # FOLLOW THE LATEST PLAN, not the first one seen.  MEASURED 2026-10-14 (B9, the completed 12-stop
            # sortie): the deviation from the plan grows with the plan's AGE (0.200 m at <= 0.05 s old, 0.368 m
            # at 0.30-0.50 s) and the truth then breaks the 0.20 m contract in 20 % of the samples even though
            # every published plan HOLDS it -- because ego replans continuously (4 Hz, 2.6 Hz of genuinely new
            # plans) and bends the path around pillars it has just seen, while this loop kept flying the copy it
            # started with.  Upstream's traj_server does the other thing: it evaluates the NEWEST B-spline at
            # the current time.  Switching rebases the time origin, which is exactly what the new plan's own
            # stamps mean (its first point is the vehicle's current state).
            if self.args.follow_latest and self.path_t is not None \
                    and float(self.path_t[0]) != t0_cur and self._traj_usable():
                p, tt, dur = snapshot()
                t0_cur = float(self.path_t[0])
                t_wall0 = time.time()
                n_switch += 1
                if n_switch <= 3 or n_switch % 20 == 0:
                    self.log("switched to a newer plan (%d so far): %d points, %.2f s, %.2f m"
                             % (n_switch, len(p), dur,
                                float(np.sum(np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)))))
            t_rel = (time.time() - t_wall0) + self.args.spline_lead
            pos, vel, acc = ref_at(p, tt, t_rel)
            self.cmd = pos
            st = self.bridge.state()                       # the FCU's own state, read BEFORE the freeze test
            here = np.array(st.pos)
            # THE ARRIVAL FREEZE (see yaw_command): near the END of this trajectory -- which is the station --
            # and slower than --yaw-v-min, stop steering the nose.  Without it each arrival reversed the
            # commanded direction and the heading whipped round (measured: up to 49 deg at 68 deg/s).
            self.yaw_hold = (float(np.linalg.norm(here[:2] - p[-1, :2])) < self.args.yaw_hold_m
                             and float(np.linalg.norm(np.asarray(vel)[:2])) < self.args.yaw_v_min)
            self.pump(0.05, setpoint=pos, velocity=vel, accel=acc)
            left = dur - t_rel
            if time.monotonic() - t_start > self.args.traj_timeout_s:
                return ("trajectory timed out after %.0f s: t %.2f of %.2f s, %.2f m from the end, "
                        "%d plan switches" % (self.args.traj_timeout_s, t_rel, dur,
                                              float(np.linalg.norm(here[:2] - p[-1, :2])), n_switch))
            if left < self.args.arrive_m / max(self.args.follow_speed, 0.1) and \
                    float(np.linalg.norm(np.array(st.vel)[:2])) < self.args.settle_speed:
                self.last_end_dist = float(np.linalg.norm(here[:2] - p[-1, :2]))
                self.log("trajectory complete (spline): %.2f s of plan, final speed %.3f m/s, "
                         "%.3f m from the last point, %d plan switches"
                         % (dur, float(np.linalg.norm(np.array(st.vel)[:2])),
                            self.last_end_dist, n_switch))
                return None

    def follow_path(self):
        """Follow the published Path itself: look-ahead along the arc, one setpoint per 50 Hz tick.

        This is the consumer half of the interface frozen in PLAN.md 4.1.  The look-ahead distance grows
        with speed (a fixed one either cuts corners at speed or crawls at a hover), the reference point is
        the projection of the CURRENT estimate onto the path rather than the nearest sample (otherwise the
        reference jumps by a whole sample interval, which at 2 m/s and 50 mm spacing is a 25 ms step), and
        the phase log gets one `traj` phase so the metrics can window the route.

        `--follow-mode pos` (the first version, kept for the record) streams the look-ahead POINT as a
        position setpoint.  MEASURED 2026-10-14 on the rectangle: it works but is slow -- 77-134 s for a
        20.00 m path against WP_SPD 2.0 -- because the FCU plans an S-curve to a point 0.45-1.7 m ahead and
        therefore decelerates as hard as it accelerates, and it cuts the inside of a turn (C2 clearance
        0.339 m against the path's own 0.428 m).

        `--follow-mode vel` streams a VELOCITY along the path tangent plus a cross-track correction, which
        is what makes both numbers better: the speed comes from a brake-to-stop profile at the end of the
        path, and the correction is proportional to the actual cross-track error, so the aircraft tracks
        the curve instead of aiming at a point on it.
        """
        if self.path_l is None:
            # A planner's trajectory is ASYNCHRONOUS: it appears some time after the goal is published, once
            # the map is worth planning on.  Waiting for it here (rather than aborting) is what makes the
            # same FSM work for a source that answers instantly (the rectangle) and one that does not.
            t_wait = time.monotonic()
            self.log("no trajectory yet on %s -- waiting up to %.0f s for the source to publish one"
                     % (self.args.traj_topic, self.args.traj_wait_s))
            while self.path_l is None and time.monotonic() - t_wait < self.args.traj_wait_s:
                if self.abort_reason:
                    return self.abort_reason
                self.pump(0.1)
            if self.path_l is None:
                return ("no trajectory on %s after %.0f s (a planner that never plans looks exactly like "
                        "this)" % (self.args.traj_topic, self.args.traj_wait_s))
            self.log("trajectory arrived after %.1f s: %d points"
                     % (time.monotonic() - t_wait, len(self.path_l)))
        R, t = self.calib.R_WL, self.calib.t_WL
        p = (R @ self.path_l.T).T + t                     # the path in F
        seg = np.linalg.norm(np.diff(p[:, :2], axis=0), axis=1)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(arc[-1])
        self.log("follow: %d points, %.2f m of trajectory, mode %s, F box E[%+.2f %+.2f] N[%+.2f %+.2f]"
                 % (len(p), total, self.args.follow_mode, p[:, 0].min(), p[:, 0].max(),
                    p[:, 1].min(), p[:, 1].max()))
        self.set_phase("traj")
        t_start = time.monotonic()
        while True:
            if self.abort_reason:
                return self.abort_reason
            st = self.bridge.state()
            here = np.array(st.pos)
            v_now = float(np.linalg.norm(np.array(st.vel)[:2]))
            # project onto the path: nearest SEGMENT, not nearest sample
            mid = 0.5 * (p[:-1, :2] + p[1:, :2])
            k = int(np.argmin(np.linalg.norm(mid - here[:2], axis=1)))
            a, b = p[k, :2], p[k + 1, :2]
            ab = b - a
            L2 = float(ab @ ab)
            u = 0.0 if L2 < 1e-12 else float(np.clip((here[:2] - a) @ ab / L2, 0.0, 1.0))
            s_now = float(arc[k] + u * seg[k])
            tangent = (ab / np.sqrt(L2)) if L2 > 1e-12 else np.array([1.0, 0.0])
            normal = np.array([-tangent[1], tangent[0]])
            e_ct = float((here[:2] - (a + u * ab)) @ normal)      # signed cross-track error, m
            s_left = max(total - s_now, 0.0)
            look = float(np.clip(self.args.lookahead_base + self.args.lookahead_gain * v_now,
                                 0.20, 2.50))
            s_tgt = min(s_now + look, total)
            j = int(np.searchsorted(arc, s_tgt, side="right") - 1)
            j = max(0, min(j, len(p) - 2))
            w = 0.0 if seg[j] < 1e-12 else (s_tgt - arc[j]) / seg[j]
            tgt = p[j] + w * (p[j + 1] - p[j])
            if self.args.follow_mode == "vel":
                # brake-to-stop profile: v = sqrt(2 a s) capped by the commanded speed
                v_ref = min(self.args.follow_speed,
                            math.sqrt(max(2.0 * self.args.brake_acc * s_left, 0.0)))
                # keep the lateral correction small enough that it cannot dominate the tangent term
                v_lat = float(np.clip(self.args.cross_gain * e_ct, -0.6 * v_ref - 0.05,
                                      0.6 * v_ref + 0.05))
                v_cmd = np.array([tangent[0] * v_ref + normal[0] * v_lat,
                                  tangent[1] * v_ref + normal[1] * v_lat, 0.0])
                self.cmd = tgt
                self.pump(0.05, velocity=v_cmd)
            else:
                self.cmd = tgt
                self.pump(0.05, setpoint=tgt)
            if time.monotonic() - t_start > self.args.traj_timeout_s:
                return ("trajectory timed out after %.0f s: %.2f m of %.2f m followed, %.2f m from the "
                        "end (horizontal)" % (self.args.traj_timeout_s, s_now, total, s_left))
            if s_left < self.args.arrive_m and v_now < self.args.settle_speed:
                self.log("trajectory complete: %.2f m followed in %.1f s (mean %.2f m/s), final speed "
                         "%.3f m/s, %.3f m from the last point"
                         % (total, time.monotonic() - t_start, total / max(time.monotonic() - t_start, 1e-9),
                            v_now, float(np.linalg.norm(here[:2] - p[-1, :2]))))
                return None

    def legs(self):
        if self.path_l is None:
            return "no trajectory on %s" % self.args.traj_topic
        R, t = self.calib.R_WL, self.calib.t_WL
        path_f = (R @ self.path_l.T).T + t
        self.log("path: %d points, F box E[%+.2f %+.2f] N[%+.2f %+.2f] U[%+.2f %+.2f]"
                 % (len(path_f), path_f[:, 0].min(), path_f[:, 0].max(), path_f[:, 1].min(),
                    path_f[:, 1].max(), path_f[:, 2].min(), path_f[:, 2].max()))
        corners = _corners_of(path_f)
        self.log("%d corners to visit" % len(corners))
        for i, corner in enumerate(corners, start=1):
            self.set_phase("leg%d" % i)
            t_leg = time.monotonic()
            while True:
                # APPROACH, THEN HOLD.  MEASURED 2026-10-13: a pure position setpoint lets the
                # controller arrive at its own speed, and it then carries the overshoot PAST the corner
                # while turning -- B2's turning legs came out at 66 mm against a 50 mm target and B1's
                # box 1-2 mm over its limit in some flights.  So the leg is flown as a velocity command
                # that stops at a slow radius, and only inside that radius does the FSM command the
                # position itself.  Both are GUIDED setpoints; only the last few centimetres are flown
                # as a position, which is also what makes the arrival repeatable.
                st = self.bridge.state()
                to_corner = corner - np.array(st.pos)
                d_h = float(np.linalg.norm(to_corner[:2]))
                # APPROACH MODE, and the default is the MEASURED BETTER one.
                #
                # `position` (default): a position setpoint for the whole leg.  Measured over 10 flights:
                #   B1 box N 1.238-1.261 (limit 1.26) and B2 26 of 50 legs passing.
                # `velocity`: a velocity command that stops at a slow radius and then hands over to a
                #   position setpoint, which was meant to kill the corner overshoot.  MEASURED on 3
                #   flights: it made BOTH worse -- B1's north extent 1.306-1.335 m and B2 3 of 15 legs --
                #   because the direction "toward the corner" is recomputed as the vehicle moves, so the
                #   path curves into the corner instead of being a straight leg.  Kept as a switch rather
                #   than deleted, so the next person can see the experiment and its result.
                if self.args.approach == "velocity" and d_h > GATES["slow_radius_m"]:
                    direction = to_corner.copy()
                    direction[2] = 0.0
                    n = np.linalg.norm(direction)
                    v = (direction / n) * GATES["speed_mps"] if n > 1e-6 else np.zeros(3)
                    self.cmd = corner
                    self.pump(0.05, velocity=v)
                else:
                    self.pump(0.05, setpoint=corner)
                if self.abort_reason:
                    return self.abort_reason
                st = self.bridge.state()
                err_cmd = float(np.linalg.norm(np.array(st.pos) - corner))
                err_truth = float(np.linalg.norm(self.tap.p - corner))
                h_cmd = float(np.linalg.norm(np.array(st.pos)[:2] - corner[:2]))
                h_truth = float(np.linalg.norm(self.tap.p[:2] - corner[:2]))
                dz_truth = float(self.tap.p[2] - corner[2])
                # ARRIVAL IS HORIZONTAL, and the vertical gets a band.  MEASURED 2026-10-13, and it is
                # this configuration's own documented property rather than a defect: with step (a) the
                # altitude loop runs on the BAROMETER (EK3_SRC1_POSZ = 1, PLAN.md decision 15), the EKF's
                # z sits at 1.97-2.01 while the truth is 2.09-2.19, so a 3-D arrival test could never
                # pass -- it read as a 0.18 m "estimator bias" when the horizontal error was 3-5 cm.
                # Every route metric (B1 box, B2 straightness, B3 corners) is horizontal; A2 removes the
                # vertical mean for exactly this reason.
                if (h_cmd < GATES["arrive_m"] and h_truth < GATES["arrive_m"] and
                        abs(dz_truth) < GATES["arrive_z_m"]):
                    self.log("corner %d reached: horizontal FCU %.3f m / truth %.3f m, vertical %+.3f m "
                             "(band %.2f), %.1f s" % (i, h_cmd, h_truth, dz_truth,
                                                      GATES["arrive_z_m"],
                                                      time.monotonic() - t_leg))
                    # SETTLE BEFORE TURNING IN.  MEASURED 2026-10-13: the FSM advanced to the next leg
                    # the instant the corner was inside tolerance, while the vehicle was still moving,
                    # so it carried the overshoot PAST the corner -- which is what B2 sees on the turning
                    # legs (53-72 mm against a 50 mm target) and what puts B1's box 1-2 mm over its limit
                    # in some flights.  The corner is therefore held until the FCU's own horizontal speed
                    # is small, with a bounded wait so a noisy estimate cannot stall the mission.
                    t_settle = time.monotonic()
                    while time.monotonic() - t_settle < GATES["settle_s"]:
                        self.pump(0.05, setpoint=corner)
                        v = np.array(self.bridge.state().vel)[:2]
                        if float(np.linalg.norm(v)) < GATES["settle_speed_mps"]:
                            break
                    vs = float(np.linalg.norm(np.array(self.bridge.state().vel)[:2]))
                    self.log("   corner %d settled: FCU speed %.3f m/s after %.1f s"
                             % (i, vs, time.monotonic() - t_settle))
                    break
                if time.monotonic() - t_leg > GATES["leg_timeout_s"]:
                    off = np.array(st.pos) - self.tap.p
                    return ("corner %d timed out after %.0f s: horizontal FCU %.3f m / truth %.3f m, "
                            "vertical %+.3f m, estimator-vs-truth offset [%+.3f %+.3f %+.3f] m"
                            % (i, GATES["leg_timeout_s"], h_cmd, h_truth, dz_truth, *off))
        return None

    def land(self):
        self.set_phase("land")
        self.bridge.land()
        # 40 s, not 20: MEASURED 2026-10-14 a Mission B flight was failed with "still armed after LAND"
        # because the FCU had not disarmed yet when this wait expired.
        self.pump(40.0, until=lambda: not self.bridge.state().armed)
        st = self.bridge.state()
        self.log("landed: armed=%s mode=%s" % (st.armed, st.mode))
        self.set_phase("done")
        return None if not st.armed else "still armed after LAND"

    def run(self):
        self.next_stream = time.monotonic()
        self.path_l = None
        self.path_t = None
        self.start_ros()
        try:
            steps = [self.preflight, self.takeoff, lambda: self.hold(self.args.hold)]
            if self.args.nav_source == "truth":
                steps.append(self.calibration_flight)
            else:
                if not self.args.calib_file:
                    return self._early_abort(
                        "no --calib-file: with --nav-source lio the L->F rotation must come from a "
                        "measured calibration flight (a parked window cannot identify it)")
                # WHICH CONSUMER flies the published Path.  `corner` (the flown default, and what every
                # table in docs/metrics/ was measured with) visits the path's corners one at a time with
                # position setpoints -- correct for the rectangle, whose corners ARE its geometry.
                # `path` follows the trajectory itself with a look-ahead point, which is what PLAN.md 4.1
                # obliges the FSM to do and what a B-spline from ego-planner needs: a planner path has no
                # corners to visit, and corner-to-corner flying would cut straight through the pillars it
                # was planned to avoid.
                if self.args.follow == "spline":
                    steps.append(self.follow_spline)
                elif self.args.follow == "path":
                    steps.append(self.follow_path)
                else:
                    steps.append(self.legs)
            steps.append(self.land)
            for step in steps:
                err = step()
                if err:
                    self.abort_reason = err
                    break
        finally:
            self.finish()
        return 0 if self.abort_reason is None else 1

    def _early_abort(self, reason):
        self.abort_reason = reason
        self.finish()
        return 1

    def finish(self):
        if self.abort_reason:
            self.set_phase("abort")
            self.log("ABORT: %s" % self.abort_reason)
            try:
                self.bridge.land()
                self.pump(6.0)
            except Exception as e:                      # pragma: no cover
                self.log("could not command LAND after the abort: %s" % e)
        st = self.bridge.state()
        self.log("final: armed=%s mode=%s alt AMSL %.1f m; %d setpoints, %d external-nav messages"
                 % (st.armed, st.mode, st.alt_amsl_m, self.bridge._sent_positions,
                    self.bridge._sent_nav))
        self.log("the FCU's last words: %s" % _texts(st))
        self.log_fh.close()
        if self.sp_log is not None:
            self.sp_log.close()
        try:
            self.bridge.close()
        except Exception:
            pass


def _corners_of(path_f, turn_min_deg=20.0):
    """The path's corner points: the samples where the direction changes by more than `turn_min_deg`.

    MEASURED 2026-10-13: with a closed 5-point source path, `densify` produced 129 points and this
    detector found **3** corners where there are 4 -- the sample nearest the closing point sees a
    direction change split across the wrap, so its turn is under the threshold.  Handling the wrap by
    looking one point further on each side fixes it (and the log now prints how many corners were found,
    so a wrong count is visible rather than a silent shortcut).
    """
    out = []
    n = len(path_f)
    if n < 5:
        return out
    for i in range(n):
        a, b, c = path_f[(i - 1) % n], path_f[i], path_f[(i + 1) % n]
        v1, v2 = b - a, c - b
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            # skip the duplicated point at the wrap and look across it
            a, c = path_f[(i - 2) % n], path_f[(i + 2) % n]
            v1, v2 = b - a, c - b
            n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
            if n1 < 1e-6 or n2 < 1e-6:
                continue
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(v1 @ v2 / (n1 * n2))))))
        if ang > turn_min_deg:
            out.append(b)
    return out


def _fit_yaw_translation(a, b):
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    num = np.sum(aa[:, 0] * bb[:, 1] - aa[:, 1] * bb[:, 0])
    den = np.sum(aa[:, 0] * bb[:, 0] + aa[:, 1] * bb[:, 1])
    yaw = math.atan2(num, den)
    R = frames.R_z(yaw)
    return R, cb - R @ ca, yaw


def _texts(st):
    return " | ".join(t for _, t in st.messages[-4:]) or "(nothing)"


def main() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge", default="mavlink", choices=["mavlink", "mavros"])
    ap.add_argument("--mav", default="tcp:127.0.0.1:5760")
    ap.add_argument("--tap-port", type=int, default=0,
                    help="unused: the FSM takes truth from ROS so that only ONE process binds the "
                         "tap's fan-out port (see TapTruth)")
    ap.add_argument("--truth-topic", default="/sim/truth/odom")
    ap.add_argument("--lio-topic", default="/lio/robo/odom")
    ap.add_argument("--lio-vel-topic", default="/lio/odom",
                    help="where the LIO's VELOCITY comes from.  `/lio/robo/odom` carries only the pose "
                         "(measured: its twist is identically zero), and feeding a zero velocity to an "
                         "EKF configured with VELXY=6 drove a runaway; `/lio/odom` has the velocity.  "
                         "If this topic also has none, the FSM differentiates the pose instead and says "
                         "so in the log")
    ap.add_argument("--traj-topic", default="/planner/traj")
    ap.add_argument("--nav-source", default="lio", choices=["lio", "truth"],
                    help="'lio' = mission A as defined; 'truth' = the calibration flight whose "
                         "recordings identify L->F (and which must never be used for the mission)")
    ap.add_argument("--calib-file", default="",
                    help="json with R_WL/t_WL measured by tools/lio-vs-truth.py --write-calib")
    ap.add_argument("--approach", default="position", choices=["position", "velocity"],
                    help="how a leg is flown: 'position' (the measured better one, B1 9 of 10 / B2 26 "
                         "of 50) or 'velocity' (tried, made both worse: B1 0 of 3 / B2 3 of 15)")
    ap.add_argument("--alt", type=float, default=2.0, help="mission altitude above the pad, m")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--geofence-m", type=float, default=GATES["geofence_m"],
                    help="abort if the truth leaves this half-extent in E or N; comes from the world's "
                         "arena through tools/world-info.py, so a bigger mission cannot silently trip it")
    # The corner gates, as arguments rather than only constants, because they are the lever B2 answers
    # to and an A/B must be able to move them without editing the flown defaults.  MEASURED 2026-10-13:
    # the leg starts with the residual of the corner turn -- up to 60 mm of position error (arrive_m) and
    # up to 74 mm/s of speed (the settle gate exiting at its first check) -- and B2's worst perpendicular
    # error peaks at 16-27% of the leg, which is that residual decaying, not a path error.
    ap.add_argument("--arrive-m", type=float, default=GATES["arrive_m"],
                    help="corner arrival tolerance, horizontal, for BOTH the FCU's estimate and the truth")
    ap.add_argument("--settle-speed", type=float, default=GATES["settle_speed_mps"],
                    help="hold the corner until the FCU's horizontal speed is below this (m/s)")
    ap.add_argument("--settle-s", type=float, default=GATES["settle_s"],
                    help="and give up waiting after this long")
    ap.add_argument("--yaw", type=float, default=0.0, help="commanded yaw, rad (0 = North)")
    ap.add_argument("--setpoint-mode", default="posvel", choices=["pos", "posvel", "posvelacc"],
                    help="which fields of SET_POSITION_TARGET_LOCAL_NED carry the plan: position only, "
                         "position+velocity, or position+velocity+acceleration.  posvelacc WAS the default and "
                         "it is the one thing measured to break HOLDING: MEASURED 2026-10-14, with posvelacc "
                         "the FSM streamed pos=(3.98,0) vel=(0,0) acc=(0,0) at the plan's end and the aircraft "
                         "kept flying outbound at 2.6 m/s for 2.6 m until the geofence abort (sortie G2), while "
                         "the same mission with posvel stopped 0.023 m from the commanded point (C1 PASS, "
                         "sortie H-posvel) and with pos 0.020 m (H-pos).  All three modes track the plan the "
                         "same way mid-leg (A1 3.16-3.51 mm), so the acceleration field buys nothing and costs "
                         "the arrival; the historical 'dropping the accel field made it worse' was measured "
                         "before the airframe and speed fixes, on a different failure mode")
    ap.add_argument("--yaw-mode", default="fixed", choices=["fixed", "velocity"],
                    help="fixed: stream --yaw unchanged (the historical behaviour, and what every mission-A "
                         "number was flown with).  velocity: point the nose along the plan's horizontal "
                         "velocity (the user's instruction, 2026-10-14) -- the sensor's 45 deg fan then "
                         "looks along the direction of travel instead of across it")
    ap.add_argument("--yaw-rate-max", type=float, default=120.0,
                    help="velocity mode: maximum yaw slew, deg/s")
    ap.add_argument("--yaw-v-min", type=float, default=0.40,
                    help="velocity mode: below this speed the last heading is held (the azimuth of a hover "
                         "is noise, and a spinning yaw feeds back into the EKF).  Raised from 0.20 to 0.40 m/s "
                         "by the user's rule, 2026-10-14")
    ap.add_argument("--yaw-hold-m", type=float, default=0.50,
                    help="velocity mode: within this horizontal distance of the trajectory's END (the "
                         "station) the heading is FROZEN, so arriving does not spin the aircraft")
    ap.add_argument("--follow", default="corner", choices=["corner", "path", "spline"],
                    help="corner: visit the path's corners (the rectangle; every table in docs/metrics/ "
                         "was flown this way).  path: follow the trajectory itself with a look-ahead "
                         "point, which is what a planner's B-spline needs (PLAN.md 4.1)")
    ap.add_argument("--stations", type=int, default=1,
                    help="spline mode: the number of stops the mission EXPECTS (only printed; the loop is "
                         "driven by --stop-topic so the FSM and the sequencer cannot drift apart).  1 = the "
                         "single-trajectory behaviour")
    ap.add_argument("--stop-topic", default="/mission/stop",
                    help="spline mode with >1 stop: the sequencer's current stop index (std_msgs/Int32, "
                         "-1 = sequence complete).  The FSM follows trajectories until it reads -1")
    ap.add_argument("--hold-max-s", type=float, default=20.0,
                    help="spline mode: how long to HOLD the current position while the sequencer dwells on "
                         "a stop before giving up and waiting for the next trajectory")
    ap.add_argument("--reaccept-m", type=float, default=0.50,
                    help="spline mode: also accept a plan whose first-point STAMP is unchanged, provided its "
                         "first point is within this many metres of the vehicle -- i.e. a re-published copy "
                         "that the adapter has re-projected onto the current position.  MEASURED 2026-10-14 "
                         "(sorties AA1-AD1): ego's commit rate collapses to ~0.2 Hz once the vehicle rests, so "
                         "without this the FSM flew 1 of the 20 distinct plans the recorder saw and aborted on "
                         "its 60 s wait.  0 disables the fallback (the historical behaviour)")
    ap.add_argument("--hold-radius-m", type=float, default=0.50,
                    help="spline mode: DEPRECATED (kept for the A/B record) -- the wait-loop hold is now gated "
                         "on the straight segment's measured clearance (--footprint-contract) instead of on "
                         "distance.  MEASURED 2026-10-14: holding always at a far endpoint advanced the mission "
                         "(8 of 12 stops) but cut 0.376 m inside a pillar for 14.03 %% of the samples; holding "
                         "only within 0.50 m was legal (C2 0 of 87032) and starved (1 of 12)")
    ap.add_argument("--footprint-contract", type=float, default=0.20,
                    help="spline mode: the clearance the FSM itself requires of the straight segment it would "
                         "fly while waiting for the next plan (the same 0.20 m the adapter's gate enforces on "
                         "published plans).  This is what lets the wait pull the vehicle toward a legal point -- "
                         "the pull is the only motion the FSM can create on its own, and the mission only "
                         "advances while the vehicle moves")
    ap.add_argument("--world-file", default="/mnt/d/distill_self/lio_avoid/worlds/forest_b.yaml",
                    help="spline mode: the world the wait-loop hold checks its straight segment against")
    ap.add_argument("--hold-at-m", type=float, default=0.50,
                    help="spline mode: only hold the station if the trajectory ended within this distance of "
                         "the vehicle -- otherwise the plan was a short hop and the mission is still moving")
    ap.add_argument("--min-traj-m", type=float, default=0.50,                    help="spline mode: ignore a published trajectory whose horizontal arc length is below "
                         "this (m).  The adapter republishes the current spline at 4 Hz from t_cur, so an "
                         "almost-exhausted plan arrives as a 0.17 m stub, and a station must not be burned "
                         "on one")
    ap.add_argument("--follow-latest", dest="follow_latest", action="store_true", default=True,
                    help="spline mode: switch to a NEWER published plan the moment one arrives, rebasing the "
                         "trajectory clock (this is what upstream's traj_server does).  Default ON: MEASURED "
                         "2026-10-14, following the plan the FSM happened to start with let ego's 4 Hz "
                         "replans accumulate as deviation -- 0.200 m at <= 0.05 s of plan age against 0.368 m "
                         "at 0.30-0.50 s, which is what pushed the truth below the 0.20 m contract")
    ap.add_argument("--no-follow-latest", dest="follow_latest", action="store_false",
                    help="keep the historical behaviour: follow the plan that was in hand when the "
                         "trajectory started, to its end")
    ap.add_argument("--lookahead-base", type=float, default=0.45,
                    help="path mode: look-ahead distance at zero speed, m")
    ap.add_argument("--lookahead-gain", type=float, default=0.60,
                    help="path mode: extra look-ahead per m/s of speed")
    ap.add_argument("--traj-timeout-s", type=float, default=180.0,
                    help="path mode: give up if the trajectory is not finished in this long")
    ap.add_argument("--spline-lead", type=float, default=0.0,
                    help="spline mode: start this many seconds into the plan (a small lead hides the"
                         " first setpoint's staleness; 0 = follow the plan from its own t=0)")
    ap.add_argument("--traj-wait-s", type=float, default=45.0,
                    help="path mode: wait this long for the source to publish a first trajectory "
                         "(a planner needs the map before it can plan)")
    ap.add_argument("--follow-mode", default="pos", choices=["pos", "vel"],
                    help="pos: stream the look-ahead point as a position setpoint (the verified first "
                         "version).  vel: stream a velocity along the path tangent plus a cross-track "
                         "correction -- faster and it does not cut the inside of a turn")
    ap.add_argument("--follow-speed", type=float, default=1.5,
                    help="vel mode: commanded speed along the path, m/s (the brake-to-stop profile caps it)")
    ap.add_argument("--brake-acc", type=float, default=1.5,
                    help="vel mode: deceleration used to plan the stop at the end of the path, m/s^2")
    ap.add_argument("--cross-gain", type=float, default=1.0,
                    help="vel mode: lateral velocity per metre of cross-track error, 1/s")
    ap.add_argument("--phase-log", default=os.path.join(here, "..", "logs", "phase.csv"))
    ap.add_argument("--setpoint-log", default="",
                    help="write every streamed setpoint (position, velocity, acceleration) next to the "
                         "FCU's and the LIO's answers; empty = off.  This is what makes a tracking error "
                         "measurable at the instant it happens instead of inferred from the truth later")
    args = ap.parse_args()
    # the effective gates go into the run's own log, so a flight's record says what it actually flew
    GATES["arrive_m"] = args.arrive_m
    GATES["settle_speed_mps"] = args.settle_speed
    GATES["settle_s"] = args.settle_s
    GATES["geofence_m"] = args.geofence_m
    print("gates in force: arrive %.3f m, settle speed %.3f m/s within %.1f s, speed %.2f m/s, "
          "geofence %.2f m"
          % (GATES["arrive_m"], GATES["settle_speed_mps"], GATES["settle_s"], GATES["speed_mps"],
             GATES["geofence_m"]), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.phase_log)), exist_ok=True)
    m = Mission(args)
    return m.run()


if __name__ == "__main__":
    sys.exit(main())
