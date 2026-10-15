#!/usr/bin/env python3
"""Mission A: fly the rectangle with the LIO as the EKF's only source.

THE PHASE MACHINE, and the place where every finding of S3-S7 is enforced rather than remembered.  Since
F1 (2026-10-16) this node is NOT the executor: the PLAN box (`planner/rectangle_source.py` +
`planner/cmd_stream.py`) owns the waypoints, the arrival gate, the settle and the dwell, and it says so on
`/plan/cmd`; `bridge/bridge_node.py::CmdRelay` puts that stream on MAVLink 84 at 50 Hz.  What is left here
is what must NOT live inside the box that gets replaced (design section 8 item 3):

  preflight   wait for the LIO to be live (>= 5 Hz, no jumps, no divergence); MEASURE the L->F
              alignment from the parked data and compare it with the construction prediction; set the
              EKF origin (retrying, because at boot the FCU is on DCM); declare the frame; arm.
  takeoff     command the takeoff and stream ONLY external nav until the vehicle is airborne --
              sending position setpoints here cancels the takeoff (they replace GUIDED's SubMode).
  hold        a few seconds at the takeoff point, which is the A4 window.  Since F1 the hold's setpoint is
              built in L from the LIO's own position and sent through `bridge.send_cmd` -- the mission no
              longer converts a frame anywhere, and the truth tap is no longer a command source.
  leg1..legN  THE ROUTE, and the phases come from the PLAN BOX now: each new `trajectory_id` in the
              command stream is one leg, so the phase names, their count and their boundaries are exactly
              what the corner walk produced before (`tools/metrics.py` reads them: `leg*` is the route
              window, leg2 anchors B1 -- risk R1).  Nothing in this file decides when a leg ends.
  land        LAND, then wait for the disarm.
  abort       any gate firing: report, LAND, stop.

Every phase change is published on `/fsm/phase` AND appended to a CSV, because the metrics window
everything (A1-A4) is defined over is a phase interval -- a number without its window is not a number.

WHAT WAS DELETED IN F1, and where it went: `_on_path`, `_path_in_hand`, `_legs_left`, `_await_next_leg`,
`_fly_to_corner`, `legs` and `_corners_of` (~300 lines, the corner walk) moved into the plan box as a
decision about waypoints; `follow_path` and `--follow path` are DELETED with the `nav_msgs/Path` interface
they consumed (design section 7 row 4: port it or delete it deliberately -- a command stream has no arc to
look ahead along, and nothing in the tree used it).  The STALL GATE stays here: it is the only NAMED cause
for a dead trajectory source, and design section 8 item 8 forbids deleting it before its replacement
(the command stream's own liveness, which F1 added beside it in `bridge/bridge_node.py`) exists.

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
from bridge.bridge_node import CmdRelay      # noqa: E402
from planner.cmd_stream import CmdSample, FRAME_L, TRAJECTORY_STATUS_READY   # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0
GATES = {
    "lio_rate_min_hz": 5.0,
    "lio_jump_max_m": 0.50,
    "lio_speed_max_mps": 3.0,
    "lio_timeout_s": 1.0,
    "geofence_m": 4.60,
    "rate_hz": 50.0,
}
# THE CORNER GATES ARE NOT HERE ANY MORE.  `arrive_m`, `arrive_z_m`, `settle_s`, `settle_speed_mps`,
# `slow_radius_m`, `speed_mps` and `leg_timeout_s` were this node's, and F1 moved them to the plan box
# (`planner/rectangle_source.py`, whose defaults carry the measured numbers and their reasons).  Keeping
# a second copy here -- even unused -- is how the two gates came to be two different numbers in the first
# place (design section 2(b) item 2: 0.10 m here and 0.06 m there, on purpose, with a docstring
# explaining the ratio).


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
        # F3 (source-death visibility): the trajectory source's own stall signal, and when the last plan
        # arrived.  KEPT, NEEDS A DECISION: the dropped planner chain was its only publisher.
        self.planner_stalled = False
        self.planner_stall_since = None
        #: the wall time of the last thing the TRAJECTORY SOURCE published, on either of its interfaces
        #: (the stall signal's companion condition).  Since F1 the source's interface is `/plan/cmd`, so
        #: this is refreshed from the relay -- the name is kept because `tools/test-stall-gate.py` builds
        #: its stub around it and the QUANTITY is unchanged: "how long has the source been quiet".
        self.last_path_wall = time.monotonic()
        #: THE FSM'S OWN POSITIONING COMMAND, in L, or None.  Set only where this node must command
        #: something the PLAN box does not own -- the take-off hold and the calibration flight legs -- and
        #: cleared immediately after, so there is never a second stream competing with the plan's.
        self.manual_cmd = None
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
        # THE BRIDGE'S ROS HALF: it consumes the PLAN box's command stream and writes message 84.
        # ONE setpoint writer in this process, and it is the transport's (`bridge/bridge_node.py`) -- the
        # FSM's own calls go through `send_cmd` as well, so this node no longer contains a frame
        # conversion anywhere (S3's finding 1.2 asked for exactly this, and F1 is where it lands).
        self.relay = CmdRelay(self.bridge, self.calib, rate_hz=args.wire_hz,
                              fields=args.setpoint_mode,
                              yaw_rad=(float(args.yaw) if args.yaw_mode == "fixed" else None),
                              log=self.log, silence_abort_s=args.cmd_silence_abort_s,
                              topic=args.cmd_topic)

    # ------------------------------------------------------------------ plumbing
    def _make_bridge(self):
        if self.args.bridge == "mavlink":
            from bridge.mavlink_impl import MavlinkBridge
            return MavlinkBridge(url=self.args.mav, log=self.log)
        # FINDING (2026-10-16, verified at HEAD eba1033): `bridge/mavros_impl.py` DOES NOT EXIST, so this
        # branch raises ModuleNotFoundError at the first call while `bridge/base.py:2`, `docs/PLAN.md:34`
        # (decision 4) and `docs/ARCHITECTURE.md:225` ("mavros_impl exists and is unflown") all say it
        # does.  NOT REPAIRED HERE: this file is being edited by another unit in this round, and writing
        # the missing implementation is explicitly out of scope for the boundary refactor
        # (refactor-bridge-plan.md section 8 item 11).  The doc half is recorded in
        # `_stage/refactor-S2-findings.txt`; the code half is this comment, which is the honest form of
        # "the switch has one setting" until the retraction round.
        from bridge.mavros_impl import MavrosBridge
        return MavrosBridge(log=self.log)

    def log(self, msg):
        print("[%7.2f] %s" % (time.monotonic() - self.t_wall0, msg), flush=True)

    def start_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from nav_msgs.msg import Odometry
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
        self.node.create_subscription(Odometry, self.args.lio_topic, self._on_lio, best_effort)
        self.node.create_subscription(Odometry, self.args.lio_vel_topic, self._on_lio_vel, best_effort)
        self.node.create_subscription(Odometry, self.args.truth_topic,
                                      lambda m: self.tap.on_odom(m), best_effort)
        # THE PLAN BOX'S COMMAND STREAM, and the QoS reasoning that used to sit on the Path subscription
        # still applies to it: the subscription must be deep enough that a momentarily busy subscriber
        # does not lose the publisher's history, and TRANSIENT_LOCAL so this node -- which starts ~25 s
        # after the plan box -- finds the sample that is already in force instead of waiting for the next
        # leg.  MEASURED 2026-10-14: RELIABLE + depth 1 lost 40 of 41 published messages to a slow
        # subscriber (sortie AC1).
        self.relay.attach(self.node)
        # THE TRAJECTORY SOURCE'S OWN LIFELINE (audit F3).  A source that has stopped producing plans can
        # SAY so on `--stall-topic` (`std_msgs/Bool`) -- MEASURED (AP5) nothing in the repository subscribed
        # to it: the FSM waited out its 60 s and aborted naming "the trajectory source", i.e. the reporter
        # blamed the messenger.  KEPT IN F1 ON PURPOSE (design section 8 item 8): it is the only NAMED
        # cause for "the source is dead", and it stays until its replacement -- the command stream's own
        # liveness, `CmdRelay.stalled_reason`, added beside it in the same step -- has been demonstrated.
        from std_msgs.msg import Bool as _Bool
        try:
            self.node.create_subscription(_Bool, self.args.stall_topic, self._on_stall, 10)
        except Exception as exc:                                       # noqa: BLE001
            self.log("cannot subscribe %s (%s): a planner stall will only be visible as a timeout"
                     % (self.args.stall_topic, exc))
        self.rclpy = rclpy

    def _on_stall(self, m):
        """The PLANNER says it is stalled: the same trajectory withheld for seconds on end.

        Two conditions have to hold before this aborts, because each alone has a benign explanation:
          * the signal has been up for `--stall-abort-s`, and
          * NO Path message has arrived for the same time (`last_path_wall`).
        A planner that is merely between stops keeps REPUBLISHING its current spline at 4 Hz, so the second
        condition separates "the planner has nothing to say" from "the planner is waiting for the aircraft"
        -- and it is exactly the state AP5 spent its last 60 s in (66 identical-stamp messages, then silence).
        """
        v = bool(m.data)
        if v and not getattr(self, "planner_stalled", False):
            self.planner_stall_since = time.monotonic()
            self.log("PLANNER STALL SIGNALLED on %s: the trajectory source reports it is not committing new "
                     "plans" % self.args.stall_topic)
        elif not v and getattr(self, "planner_stalled", False):
            self.log("the planner's stall signal cleared after %.1f s"
                     % (time.monotonic() - (self.planner_stall_since or time.monotonic())))
            self.planner_stall_since = None
        self.planner_stalled = v

    def _stall_reason(self):
        """A NAMED abort reason when the planner is dead or wedged, else None (audit F3).

        "quiet" is the longer of the two silences the source can be judged by: the stall signal's own
        companion condition (`last_path_wall`, refreshed from whichever interface the source publishes on)
        and the command stream's own age.  Since F1 the plan box IS the trajectory source, so the second
        term is the live one -- and it is a separate gate from `CmdRelay.stalled_reason`, which names the
        stream rather than a planner.  Both are named causes; neither replaces the other.
        """
        if not getattr(self, "planner_stalled", False) or self.planner_stall_since is None:
            return None
        up = time.monotonic() - self.planner_stall_since
        quiet = time.monotonic() - getattr(self, "last_path_wall", 0.0)
        relay = getattr(self, "relay", None)
        if relay is not None and relay.n_rx:
            quiet = min(quiet, relay.age_s())
        if up < self.args.stall_abort_s or quiet < self.args.stall_abort_s:
            return None
        return ("THE PLANNER IS NOT PLANNING: %s has been up for %.1f s and no plan has arrived on %s for "
                "%.1f s (the planner process is dead or wedged -- check planner.log for 'terminate called', "
                "'std::bad_alloc' or an aborted node; this is NOT a trajectory-source timeout)"
                % (self.args.stall_topic, up, self.args.traj_topic, quiet))

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

    def yaw_command(self, cmd_yaw, period=None):                      # noqa: ARG002
        """The heading to stream.  TWO MODES, and the flown one is `fixed`.

        `--yaw-mode fixed` (the historical behaviour, and what every mission-A number was flown with)
        streams `--yaw` unchanged -- MEASURED 2026-10-14 from the SITL tap's own attitude: the heading held
        -1.0 deg (B1) and -0.8 deg (B5), i.e. North, while the velocity azimuth swept the whole circle.

        `--yaw-mode stream` takes the heading from the command stream: since F1 the plan box publishes the
        leg's own direction in the message's `yaw` field, which is the honest source for "where is this leg
        going".

        REMOVED IN F1: `--yaw-mode velocity` and its `--yaw-rate-max` / `--yaw-v-min` machinery.  Its INPUT
        was the plan's feed-forward VELOCITY (`fsm/mission_fsm.py::pump`'s `velocity=` argument), and a
        geometric waypoint stream does not have one -- every sample it publishes carries `velocity = 0`, by
        construction (`planner/cmd_stream.py`).  The capability is not lost, it MOVED: the plan box's own
        `yaw` field is the leg's direction, which is what the velocity mode was approximating from a
        velocity vector, and it is read under `--yaw-mode stream`.  The measured story of the mode
        (`docs/metrics/phaseB-yaw-vs-velocity.txt`: the nose following the velocity whipped round by up to
        49 deg at up to 68 deg/s at each arrival, and the v-min hold existed to stop a hovering vehicle's
        nose chasing noise) is why `fixed` stays the default either way.
        """
        if self.args.yaw_mode != "stream" or cmd_yaw is None:
            return float(self.args.yaw)
        return float(cmd_yaw)

    # ---- DELETED WITH THE `nav_msgs/Path` INTERFACE (F1): `_on_path` --------------------------------
    #
    # The trajectory source no longer publishes a Path; it publishes the command stream, and the consumer
    # of that stream is `bridge/bridge_node.py::CmdRelay` (this node's `self.relay`), not the FSM.  The two
    # facts `_on_path` used to compute are the two F1 had to replace, and neither is a convention now:
    #
    #   * the leg's identity (a tuple built from the path's LAST POINT, because a republished leg is
    #     indistinguishable from a new one by counting messages) -- now `trajectory_id`, a FIELD of the
    #     message, which is what the type is for (`PositionCommand.msg:20`);
    #   * "there are no more legs" (an EMPTY Path) -- now `trajectory_flag = COMPLETED`, and the
    #     difference between THAT and silence is `CmdRelay.stalled_reason`.  Before F1 a dead producer and
    #     a finished tour were the same signal: the mission landed and `tools/metrics.py` scored it as
    #     COMPLETE with no `abort` phase (design section 2(e), risk R3).

    def pump(self, seconds=None, until=None):
        """The main loop body: poll everything, stream at 50 Hz, run the gates.

        THE SETPOINT THAT GOES OUT is either this node's own `manual_cmd` (the take-off hold and the
        calibration legs -- positioning the supervisor owns) or the PLAN box's newest command, relayed by
        `self.relay`.  There is no third source and no frame arithmetic here: `bridge.send_cmd` is the one
        conversion, and `--setpoint-mode` (now `fields=`) is the one wire-shape switch.
        """
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
                if self.relay.n_rx:
                    self.last_path_wall = max(self.last_path_wall, self.relay.newest_wall or 0.0)
            now = time.monotonic()
            if now >= self.next_stream:
                self.next_stream = now + period
                self.stream_external_nav()
                sent = None
                if self.manual_cmd is not None:
                    c = self.manual_cmd
                    p_f = self.bridge.send_cmd(c, self.calib,
                                               yaw_rad=self.yaw_command(None, period),
                                               fields=self.args.setpoint_mode)
                    sent = (p_f, c.yaw, c)
                else:
                    sent = self.relay.tick(now)
                n += 1
                # THE SETPOINT TRACE.  The phase log records only the command in force when a PHASE changes,
                # which is enough for the metrics and useless for a tracking question: MEASURED 2026-10-14,
                # the FCU flew at 1.28-1.31 m on B5 and 3.28 m on B2b while the mission commanded 2.00 m,
                # and there was no record of what was actually streamed at those instants.  This writes the
                # commanded setpoint next to the FCU's and the LIO's answers, every WIRE tick -- and since
                # F1 only when a setpoint actually went out, so an empty row can no longer be mistaken for
                # a command.
                if sent is not None and self.sp_log is not None:
                    p_f, yaw_out, cmd = sent
                    st_sp = self.bridge.state()
                    self.cmd = np.asarray(p_f, dtype=float)
                    self.sp_log.write(
                        "%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                        "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n" % (
                            time.monotonic(), self.tap.t_us,
                            self.cmd[0], self.cmd[1], self.cmd[2],
                            cmd.v[0], cmd.v[1], cmd.v[2],
                            cmd.a[0], cmd.a[1], cmd.a[2],
                            st_sp.pos[0], st_sp.pos[1], st_sp.pos[2],
                            st_sp.vel[0], st_sp.vel[1], st_sp.vel[2],
                            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
                            yaw_out))
                self.check_gates()
                if self.abort_reason:
                    return n
            time.sleep(0.001)
        return n

    def stream_external_nav(self):
        """Feed the FCU.  Two sources, and the choice is explicit because it decides what is proven.

        `--nav-source lio`  the mission: the LIO's odometry, fed in L and converted ONCE inside the bridge
                            (`send_external_nav_l`).  This is the configuration mission A is defined by.
        `--nav-source truth` the CALIBRATION flight: the truth tap converted W -> NED, exactly as
                            tools/test-fcu-link.py does.  It flies correctly, and its RECORDING is what
                            identifies L -> F -- a parked window cannot identify a rotation at all
                            (MEASURED: fitting one gave a 90 deg error and the aircraft ran away East
                            until the geofence caught it).  Nothing else uses this mode: the mission
                            itself must never fly on truth.

        F1 CLOSED THE ASYMMETRY S4 LEFT HERE.  The mission used to do its own L -> "F" step for the
        estimator input (`p_f = R @ p_l + t`, the same arithmetic as `frames.l_to_wire`) while the command
        path already went through the bridge -- two crossings, one of them above the boundary the
        transport's docstring claims is crossed exactly once (`_stage/refactor-S3-frames-and-discrepancies.txt`
        section 1.2).  It is now one call, and **this file contains no frame arithmetic at all**.
        """
        if self.args.nav_source == "truth":
            if self.tap.n == 0:
                return
            self.bridge.send_external_nav(self.tap.t_us, self.tap.p, self.tap.q, self.tap.v)
            return
        if self.lio["n"] == 0:
            return
        self.bridge.send_external_nav_l(self.lio["t_us"], self.lio["p"], self.lio["q"],
                                        self.lio["v"], self.calib)

    def _manual_from_w(self, target_w, trajectory_id=0):
        """A `CmdSample` in L for a target given in W.

        THE ONE PLACE THIS NODE STILL CONVERTS A FRAME, and the reason is the calibration flight itself:
        its targets are defined in the TRUTH's own frame because its purpose is to MEASURE the L -> W
        transform, so stating them in L would beg the question.  Everything else this node commands is
        built in L (the hold) or arrives in L (the plan box's stream).
        """
        p_l = self.calib.R_WL.T @ (np.asarray(target_w, dtype=float) - self.calib.t_WL)
        return CmdSample(p_l, np.zeros(3), np.zeros(3), float(self.args.yaw), 0.0, trajectory_id,
                         TRAJECTORY_STATUS_READY, FRAME_L)

    def calibration_flight(self):
        """A short shaped path on TRUTH-fed nav, purely so the recordings identify L -> F.

        The path is two perpendicular legs: a single straight line leaves the yaw ambiguous by 180 deg,
        so the shape is the point of the exercise (tools/lio-vs-truth.py reports the same identifiability
        numbers this relies on).
        """
        self.set_phase("calib")
        base = self.tap.p.copy()
        legs_w = [(base + np.array([1.0, 0.0, 0.0]), 4.0),
                  (base + np.array([1.0, 1.0, 0.0]), 4.0),
                  (base + np.array([0.0, 1.0, 0.0]), 4.0)]
        self.log("calibration flight: %.2f m East, then North, then West, at %.1f m"
                 % (1.0, base[2]))
        for i, (target, secs) in enumerate(legs_w, start=1):
            self.set_phase("calib_leg%d" % i)
            self.manual_cmd = self._manual_from_w(target, trajectory_id=i)
            self.pump(secs)
            self.manual_cmd = None
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
        stall = self._stall_reason()
        if stall:
            reasons.append(stall)
        # THE COMMAND STREAM'S OWN LIVENESS (new in F1, and the half of the stall gate that has a
        # publisher today).  It only ever fires while the relay is ENABLED and has seen at least one
        # sample, so it cannot fire during preflight or take-off.
        relay_stall = self.relay.stalled_reason()
        if relay_stall:
            reasons.append(relay_stall)
        if reasons:
            self.abort_reason = "; ".join(reasons)

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
        # THE COMMAND PATH MUST USE THE SAME FRAME AS THE ESTIMATOR FEED.  `send_external_nav_l` reads
        # `self.calib` at every tick, so a relay still holding the constructor's calibration would convert
        # the PLAN box's L points with the file's t while the EKF is fed with the parked-measured one --
        # a systematic offset of exactly their difference (measured on the last flights: +3, +2, -12 mm).
        self.relay.set_calib(measured)
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
        """Hold the take-off point -- THE GO'S HOLD, and since F1 it is built in L and sent in L.

        WHAT CHANGED AND WHY IT MATTERS.  Until F1 this streamed `[truth.x, truth.y, --alt]`: the A4
        window's setpoint came from the TRUTH TAP, which is the sim-only measurement
        `docs/ARCHITECTURE.md:206` says is "never used for control".  It now commands the vehicle to stay
        where its OWN estimate says it is (`/lio/robo/odom`, converted to W with the same calibration the
        bridge uses), at the mission's altitude expressed in L -- so the hold is estimate-only like every
        other command in the stack, and truth survives only as the geofence gate.

        The altitude: the body origin sits ~0.10 m above the pad and the EKF origin is the ground plane,
        so the commanded altitude is an ABSOLUTE height in W, not an increment (measured in the S7 link
        test: commanding 0.80 m settled the truth at 0.86-0.91 m).  `t_WL[2]` is exactly the offset
        between the two frames' vertical datums (`Calib` is a rotation about z plus a translation), so
        `--alt - t_WL[2]` is that same altitude in L and not a second convention.
        """
        self.set_phase("hold")
        p_l = self.lio["p"].copy()
        p_l[2] = float(self.args.alt) - float(self.calib.t_WL[2])
        target_w = self.calib.R_WL @ p_l + self.calib.t_WL
        self.manual_cmd = CmdSample(p_l, np.zeros(3), np.zeros(3), float(self.args.yaw), 0.0, 0,
                                    TRAJECTORY_STATUS_READY, FRAME_L)
        self.log("holding at L [%+.3f %+.3f %+.3f] (= W [%+.3f %+.3f %+.3f], the estimate's own "
                 "position at the mission altitude) for %.1f s"
                 % (*p_l, *target_w, seconds))
        self.pump(seconds)
        self.manual_cmd = None
        return None

    # ------------------------------------------------------------------ THE ROUTE (F1)
    #
    # DELETED HERE, IN ONE CHANGE, WITH THEIR SWITCHES (design section 6 step F1; section 7 row 4 says
    # "port or delete, never leave half-alive"): `follow_path` and `--follow path` + `--follow-mode` +
    # `--lookahead-base/-gain` + `--brake-acc` + `--cross-gain` + `--traj-timeout-s` + `--traj-wait-s`
    # (the look-ahead consumer of the Path -- a command stream has no arc to look ahead along, and
    # nothing in the tree flew it); `_path_in_hand`, `_legs_left`, `_await_next_leg`, `_fly_to_corner`,
    # `legs` and `_corners_of` (the corner walk: ~300 lines whose decision is now the plan box's); and
    # `--closed-m`, `--leg-join-m`, `--approach`, `--arrive-m`, `--settle-speed`, `--settle-s`,
    # `--next-leg-wait-s`.
    #
    # WHAT REPLACED IT.  Not a second corner walk -- a reader of the plan box's stream, which answers
    # three questions and nothing else:
    #
    #   * is there a stream at all?      (`--cmd-wait-s`, then a NAMED abort: design section 2(e), a plan
    #                                     box that never speaks must not look like a finished tour)
    #   * which leg is in hand?          (`trajectory_id` -> the phase name, so `leg1..legN` keep their
    #                                     names, their count and their boundaries -- risk R1)
    #   * is the tour finished?          (`trajectory_flag = COMPLETED`, a FIELD of the message)
    #
    # The arrival gate, the settle, the dwell and the leg timeout are NOT here any more: they are the plan
    # box's (`planner/cmd_stream.py::TourPacer`), measured on its own odometry.  That is change 4 of the F1
    # brief, and it is why this node no longer has a truth term in a control decision at all.

    def route(self):
        """Fly the plan box's tour: relay its stream and follow the phases it declares."""
        self.relay.enable("airborne and holding: the plan box's stream may reach the FCU")
        t0 = time.monotonic()
        while self.relay.n_rx == 0:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            why = self.relay.never_reason(self.args.cmd_wait_s)
            if why:
                return why
            if not self.relay.enabled or time.monotonic() - t0 > self.args.cmd_wait_s + 5.0:
                return ("no command arrived on %s within %.1f s of the relay being enabled"
                        % (self.args.cmd_topic, time.monotonic() - t0))
        self.log("the plan box is talking: %s" % self.relay.status())
        self.set_phase("leg%d" % self.relay.newest.trajectory_id)
        n_sent0 = self.relay.n_sent
        while True:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            tid = self.relay.newest.trajectory_id
            if ("leg%d" % tid) != self.phase:
                # A NEW LEG.  The phase name, its order and its count are the PLAN BOX's now, and they are
                # the same five the corner walk produced: `tools/metrics.py:171-183` windows the route by
                # every phase whose name starts with `leg`, and `:426-435` anchors B1's window at the leg2
                # entry.  The id increments once per waypoint, so "the same leg republished" and "the next
                # leg" are different values of a FIELD (risk R1).
                self.set_phase("leg%d" % tid)
            if self.relay.completed:
                self.log("the plan box has declared the tour COMPLETE (trajectory_flag = COMPLETED after "
                         "waypoint %d); %.1f s of route, %d setpoints on the wire, %d commands received "
                         "at %.1f Hz" % (tid, time.monotonic() - t0, self.relay.n_sent - n_sent0,
                                         self.relay.n_rx, self.relay.rate_hz_rx))
                return None

    def land(self):
        # THE PLAN IS SUPPRESSED BEFORE THE MODE CHANGES.  Measured behaviour of the stack until F1: while
        # landing, NOTHING but external nav was streamed (`pump(40.0)` with no setpoint).  F1 must not
        # change that by accident -- the relay would otherwise keep putting 50 Hz position setpoints into a
        # FCU that has been told to LAND.  It is also the shape the F2 supervisor needs: the override goes
        # ONE WAY and takes the plan off the wire, never the reverse.
        self.relay.disable("LAND")
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
                # THE ROUTE IS THE PLAN BOX'S NOW.  `route()` relays its command stream and follows the
                # phases it declares (see the block above `route`): there is no second consumer to choose
                # between, because `--follow path` was deleted with the Path interface it consumed.
                steps.append(self.route)
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
            self.relay.disable("ABORT: the plan comes off the wire before the mode change")
            self.set_phase("abort")
            self.log("ABORT: %s" % self.abort_reason)
            try:
                self.bridge.land()
                self.pump(6.0)
            except Exception as e:                      # pragma: no cover
                self.log("could not command LAND after the abort: %s" % e)
        st = self.bridge.state()
        # THE TRANSPORT'S OWN TALLY, through the declared interface: until 2026-10-16 this line read
        # `self.bridge._sent_positions` / `._sent_nav`, i.e. the mission depended on a private attribute
        # of the ONLY implementation, which a second implementation would silently break.  Same numbers:
        # `counters()` returns exactly those two fields (`mavlink_impl.py:416-417`).  No `hasattr` guard
        # on purpose -- a bridge that does not declare it must fail `tools/test-bridge-contract.py`
        # BEFORE the sortie, not degrade to a zero in a log line.
        _ctr = self.bridge.counters()
        self.log("final: armed=%s mode=%s alt AMSL %.1f m; %d setpoints, %d external-nav messages"
                 % (st.armed, st.mode, st.alt_amsl_m, _ctr["positions"], _ctr["external_nav"]))
        self.log("the FCU's last words: %s" % _texts(st))
        self.log_fh.close()
        if self.sp_log is not None:
            self.sp_log.close()
        try:
            self.bridge.close()
        except Exception:
            pass


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
    ap.add_argument("--traj-topic", default="/planner/traj",
                    help="the trajectory source's topic, named in the gate messages.  Since F1 the "
                         "interface is the command stream (`--cmd-topic`) and this name survives only in "
                         "the strings `tools/test-stall-gate.py` checks")
    ap.add_argument("--nav-source", default="lio", choices=["lio", "truth"],
                    help="'lio' = mission A as defined; 'truth' = the calibration flight whose "
                         "recordings identify L->F (and which must never be used for the mission)")
    ap.add_argument("--calib-file", default="",
                    help="json with R_WL/t_WL measured by tools/lio-vs-truth.py --write-calib")
    ap.add_argument("--alt", type=float, default=2.0, help="mission altitude above the pad, m")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--geofence-m", type=float, default=GATES["geofence_m"],
                    help="abort if the truth leaves this half-extent in E or N; comes from the world's "
                         "arena through tools/world-info.py, so a bigger mission cannot silently trip it.  "
                         "THE TRUTH'S ONLY REMAINING ROLE IN THE CONTROL PATH is this gate (design section "
                         "5 arrow 4): the arrival gate is the plan box's and is estimate-only")
    # ---- THE PLAN BOX'S COMMAND STREAM (F1) --------------------------------------------------------
    ap.add_argument("--cmd-topic", default="/plan/cmd",
                    help="the PLAN box's command stream, quadrotor_msgs/msg/PositionCommand in L.  This "
                         "node relays it to MAVLink 84 at --wire-hz through bridge/bridge_node.py, and it "
                         "reads its `trajectory_id` (one per leg -> the `leg` phases) and its "
                         "`trajectory_flag` (COMPLETED -> land) and nothing else")
    ap.add_argument("--wire-hz", type=float, default=50.0,
                    help="THE WIRE RATE, and it is not the publisher's: the plan box declares up to 100 Hz "
                         "and 50 Hz is what every mission-A sortie flew.  ARCHITECTURE.md 251-258 records "
                         "why 50 is the firmware's own ceiling.  The relay sends at most one setpoint per "
                         "due tick, taken from the newest sample")
    ap.add_argument("--cmd-wait-s", type=float, default=20.0,
                    help="how long to wait for the plan box's FIRST command after the relay is enabled; "
                         "then a NAMED abort.  It is short on purpose: the plan box starts before this "
                         "node, so its stream is already latched (TRANSIENT_LOCAL) when the relay "
                         "subscribes")
    ap.add_argument("--cmd-silence-abort-s", type=float, default=3.0,
                    help="THE PLAN BOX'S LIFELINE: once the relay has seen a command and is enabled, this "
                         "much silence on --cmd-topic is a NAMED abort.  This is the difference F1 buys "
                         "(design section 2(e), risk R3): before it, a DEAD plan box and a COMPLETED tour "
                         "were the same signal to the consumer -- the mission landed and was scored as "
                         "complete with no abort phase.  The tour's end is a field now, so silence is an "
                         "error.  0 disables the gate")
    ap.add_argument("--yaw", type=float, default=0.0, help="commanded yaw, rad (0 = North)")
    ap.add_argument("--setpoint-mode", default="pos", choices=["pos", "posvel", "posvelacc"],
                    help="which fields of SET_POSITION_TARGET_LOCAL_NED reach the wire -- the old "
                         "--setpoint-mode, now passed to bridge.send_cmd(fields=...).  **DEFAULT CHANGED "
                         "FROM posvel TO pos**: measured at HEAD 940cb50, the corner tour's `posvel` "
                         "default fell through to the position-only branch (`pump(setpoint=...)` passes no "
                         "velocity), so `pos` is what every mission-A sortie actually FLEW; and the "
                         "acceleration arm is the one thing measured to break HOLDING (MEASURED 2026-10-14, "
                         "sortie G2: pos=(3.98,0) vel=(0,0) acc=(0,0) at the plan's end and the aircraft "
                         "flew on at 2.6 m/s for 2.6 m until the geofence abort, against 0.020-0.023 m of "
                         "stopping error on the other two arms)")
    ap.add_argument("--yaw-mode", default="fixed", choices=["fixed", "stream"],
                    help="fixed: stream --yaw unchanged (the historical behaviour, and what every "
                         "mission-A number was flown with).  stream: use the command stream's own yaw, "
                         "which the plan box fills with the leg's direction.  The old `velocity` mode and "
                         "its --yaw-rate-max/--yaw-v-min went with the Path: its input was the plan's "
                         "feed-forward velocity, and every sample of a geometric waypoint stream carries "
                         "velocity = 0 by construction")
    ap.add_argument("--stall-topic", default="/planner/stalled",
                    help="std_msgs/Bool: the trajectory source says it has stopped producing plans.  With "
                         "--stall-abort-s of silence on --traj-topic the FSM aborts NAMING THE SOURCE "
                         "instead of waiting out --traj-wait-s and blaming the trajectory (MEASURED AP5: a "
                         "dead source for 60 s and an abort that said only 'no new trajectory').  KEPT IN "
                         "F1 ON PURPOSE (design section 8 item 8): the dropped planner chain was its only "
                         "publisher, and its replacement -- the command stream's own liveness, "
                         "--cmd-silence-abort-s -- is still young")
    ap.add_argument("--stall-abort-s", type=float, default=3.0,
                    help="abort when the source's stall signal has been up this long AND no plan has arrived "
                         "for the same time")
    ap.add_argument("--phase-log", default=os.path.join(here, "..", "logs", "phase.csv"))
    ap.add_argument("--setpoint-log", default="",
                    help="write every setpoint that went ON THE WIRE (position, velocity, acceleration) "
                         "next to the FCU's and the LIO's answers; empty = off.  This is what makes a "
                         "tracking error measurable at the instant it happens instead of inferred from the "
                         "truth later")
    args = ap.parse_args()
    # the effective gates go into the run's own log, so a flight's record says what it actually flew
    GATES["geofence_m"] = args.geofence_m
    print("gates in force: geofence %.2f m (truth), lio rate >= %.1f Hz; the corner gates (arrival, "
          "settle, dwell, leg timeout) are the PLAN BOX's -- see its own log"
          % (GATES["geofence_m"], GATES["lio_rate_min_hz"]), flush=True)
    print("the wire: %s at %.1f Hz, fields '%s', plan-box silence aborts after %.1f s, yaw mode %s"
          % (args.cmd_topic, args.wire_hz, args.setpoint_mode, args.cmd_silence_abort_s, args.yaw_mode),
          flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.phase_log)), exist_ok=True)
    m = Mission(args)
    return m.run()


if __name__ == "__main__":
    sys.exit(main())
