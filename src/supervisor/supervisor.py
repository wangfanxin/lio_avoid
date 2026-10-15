#!/usr/bin/env python3
"""THE SUPERVISOR: mission A's authority, as an INDEPENDENT PROCESS with its OWN MAVLink endpoint.

WHAT IT IS, AND WHY IT IS NOT PART OF THE PLAN BOX.  `_stage/refactor-bridge-plan.md` section 4: the PLAN
box is the box a whole-replacement swaps out (amendment A1), so anything left inside it disappears with the
swap.  Preflight, arming, the take-off, the modes, LAND and the ABORT are none of them functions of a plan
-- checked against the code, none of them reads a plan quantity -- so they live here, in the one process
whose inputs are the TRUTH and the LINK and never the LIO in flight.

ITS INPUTS, exactly:
  * `/sim/truth/odom`  (W) -- the truth-fed geofence and the airborne test.  SIM-ONLY, and that is the
    point: it is the one box allowed to read it, and it reads it as a GATE, never as a control input;
  * `/plan/cmd`        -- the plan box's own statements: is it still talking, and has it declared COMPLETED.
    This is not the LIO; it is the plan, and the design names this topic as one of the supervisor's inputs;
  * `/bridge/status`   -- the BRIDGE's own report of the streams it is carrying (its LIO input rate, the
    estimator feed it has sent, the relay's state);
  * its OWN endpoint   -- heartbeat/position age, armed, mode, landed_state, statustext.
  It does NOT subscribe to `/lio/robo/odom` or `/lio/odom`: in flight, the LIO is the plan's and the
  bridge's business.  (Before the aircraft is armed it does read the LIO, in the preflight liveness gate --
  the design's own qualification of the rule, section 4.3.)

ITS AUTHORITY, by MESSAGE CLASS and by nothing else (section 4.2): class 76 -- arm, take-off, mode, LAND,
RTL, the EKF origin and the accelerometer calibration.  It NEVER sends a setpoint: message 84 belongs to
`bridge/bridge_node.py`, and a supervisor that streamed setpoints would be a second, silent plan authority.
The split needs no runtime token because the two writers cannot name the same message; that rule is also
free to a dropped-in plan box, because ego's own source never writes to a flight controller at all.

WHAT IT FIXES.  MEASURED, and it is the reason the split is worth its risk: until F2 a DEAD PLAN BOX and a
FINISHED TOUR were the same fact to the consumer -- an empty path meant "no more legs", a timeout meant the
same thing, the mission landed and `tools/metrics.py` scored it COMPLETE with no `abort` phase (design
section 2(e), risk R3).  The tour's end is a FIELD now (`trajectory_flag = COMPLETED`) and the plan box
keeps publishing; so SILENCE is an error, and this process turns it into a NAMED abort.

usage: supervisor/supervisor.py --mav tcp:127.0.0.1:5761 --calib-file _stage/calib.json
       --phase-log logs/<run>/phase.csv [--alt 2.5] [--hold 3.0] [--geofence-m 6.60]
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import frames                    # noqa: E402
from bridge.base import Mode                 # noqa: E402
from planner.cmd_stream import TRAJECTORY_STATUS_COMPLETED   # noqa: E402

HOME_LAT, HOME_LON, HOME_ALT = -35.363261, 149.165230, 584.0
GATES = {
    "lio_rate_min_hz": 5.0,
    "rate_hz": 50.0,
}


class TapTruth:
    """The truth, as the supervisor needs it: the geofence and the airborne test.  Nothing else."""

    def __init__(self):
        self.t_us = 0
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.n = 0

    def on_odom(self, m):
        self.t_us = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        self.p = np.array([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z])
        self.v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        self.q = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                           m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        self.n += 1


class Supervisor:
    def __init__(self, args):
        self.args = args
        self.t_wall0 = time.monotonic()
        self.tap = TapTruth()
        self.bridge = None
        self.node = None
        self.phase_pub = None
        self.phase = "init"
        self.abort_reason = None
        self.next_stream = time.monotonic()
        # the LIO, read ONLY by the preflight liveness gate (design section 4.3)
        self.lio = {"t_us": 0, "p": np.zeros(3), "q": np.array([1.0, 0.0, 0.0, 0.0]),
                    "n": 0, "rate_slow": 0.0, "last_wall": 0.0}
        # the plan box's statements, from /plan/cmd -- NOT the LIO
        self.plan = {"n": 0, "last_wall": None, "ids": [], "flag": None, "last_tid": None}
        # the bridge's own report
        self.bridge_status = {"nav_hz": 0.0, "nav_age": float("inf"), "relay": 0, "sent": 0,
                              "last_wall": None, "mode": "", "armed": 0}
        self.calib = None
        self.cmd = np.zeros(3)
        self.log_fh = None
        self.trigger_sent = False
        #: THE FOUR GATES JUDGE A MISSION IN PROGRESS.  `finish` clears this before it lands, because the
        #: conditions that ended the sortie are still true while the aircraft comes down, and a gate that
        #: re-fires during the landing makes the landing wait return immediately (`check_gates`).  It is set
        #: HERE rather than in `run` because `preflight` reaches `check_gates` through `pump`, and a gate
        #: that is read before it exists is an AttributeError on a code path nothing tests (MEASURED: the
        #: first version of this fix put it in `run` and `tools/test-stall-gate.py` failed).
        self.gates_armed = True
        #: AND THE SAME ARGUMENT APPLIES TO `route_started`, which `check_gates` also reads (`:281`) and which
        #: used to be born in `run()` (`:592`).  `check_gates` is reachable from `preflight` through `pump`,
        #: so an object built WITHOUT `run()` -- which is how `tools/test-stall-gate.py` and
        #: `tools/test-planner-swap.py` drive these gates, with no ROS and no MAVLink -- hit an
        #: AttributeError on the very gate it was about to check.  MEASURED: writing the supervisor half of
        #: the planner-swap test crashed on exactly this line, the second time this file has been bitten by
        #: state born in the wrong place.  `run()` still sets it False before preflight; this only makes the
        #: object usable without it.
        self.route_started = False

    def log(self, msg):
        print("[%7.2f] %s" % (time.monotonic() - self.t_wall0, msg), flush=True)

    # ------------------------------------------------------------------ plumbing
    def start_ros(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        except Exception:
            rclpy.init(args=[])
        self.node = Node("supervisor")
        best_effort = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Odometry, self.args.truth_topic,
                                      lambda m: self.tap.on_odom(m), best_effort)
        self.node.create_subscription(Odometry, self.args.lio_topic, self.on_lio, best_effort)
        self.node.create_subscription(String, self.args.status_topic, self.on_bridge_status, 10)
        try:
            from quadrotor_msgs.msg import PositionCommand
            self.node.create_subscription(PositionCommand, self.args.cmd_topic, self.on_cmd, latched)
        except ImportError as exc:
            self.log("cannot subscribe %s (%s): the plan's liveness gate is DISABLED, and a dead plan box "
                     "would look like a finished tour again" % (self.args.cmd_topic, exc))
        from std_msgs.msg import Bool as _Bool
        try:
            self.node.create_subscription(_Bool, self.args.stall_topic, self.on_stall, 10)
        except Exception as exc:                                        # noqa: BLE001
            self.log("cannot subscribe %s (%s)" % (self.args.stall_topic, exc))
        self.trigger_pub = self.node.create_publisher(PoseStamped, self.args.trigger_topic, latched)
        self.phase_pub = self.node.create_publisher(String, "/fsm/phase", 10)
        self.rclpy = rclpy

    def on_lio(self, m):
        """THE PREFLIGHT'S LIVENESS GATE ONLY.  Nothing in flight reads this (design section 4.3)."""
        now = time.monotonic()
        dt = max(1e-3, now - self.lio["last_wall"])
        self.lio["t_us"] = int(m.header.stamp.sec) * 1_000_000 + int(m.header.stamp.nanosec) // 1000
        self.lio["p"] = np.array([m.pose.pose.position.x, m.pose.pose.position.y,
                                  m.pose.pose.position.z])
        self.lio["q"] = np.array([m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                                  m.pose.pose.orientation.y, m.pose.pose.orientation.z])
        self.lio["n"] += 1
        self.lio["last_wall"] = now
        self.lio["rate_slow"] = 1.0 / dt if self.lio["n"] == 1 else \
            (0.9 * self.lio["rate_slow"] + 0.1 * (1.0 / dt))

    def on_cmd(self, m):
        """The plan box is talking.  THIS is the fact that used to be indistinguishable from "finished"."""
        self.plan["n"] += 1
        self.plan["last_wall"] = time.monotonic()
        self.plan["flag"] = m.trajectory_flag
        if m.trajectory_id not in self.plan["ids"]:
            self.plan["ids"].append(m.trajectory_id)
            self.plan["last_tid"] = m.trajectory_id
            if self.phase.startswith("leg"):
                self.set_phase("leg%d" % m.trajectory_id)
        p_l = np.array([m.position.x, m.position.y, m.position.z])
        self.plan["pos"] = p_l
        self.plan["frame"] = m.header.frame_id
        # THE PHASE LOG'S `cmd_*` COLUMN, in W -- the same meaning it has had since before F1 ("the command
        # in force when the phase begins"), computed through the SAME calibration the bridge converts with.
        # The supervisor sends no setpoint (class 84 is the bridge's), so this is where the column comes
        # from now: one conversion, on the command the plan box is publishing.
        if m.header.frame_id in ("lio_world", "world"):
            self.cmd = self.calib.R_WL @ p_l + self.calib.t_WL
        else:
            self.cmd = p_l

    def on_bridge_status(self, m):
        """`bridge/bridge_node.py`'s own report -- the streams it is carrying, from the process that carries
        them.  The bridge does not DECIDE anything here; it reports, and this process decides."""
        for kv in m.data.split():
            if "=" not in kv:
                continue
            k, v = kv.split("=", 1)
            if k in ("nav_hz", "nav_age"):
                try:
                    self.bridge_status[k] = float(v)
                except ValueError:
                    pass
            elif k in ("relay", "sent", "armed"):
                try:
                    self.bridge_status[k] = int(v)
                except ValueError:
                    pass
            elif k == "mode":
                self.bridge_status["mode"] = v
        self.bridge_status["last_wall"] = time.monotonic()

    def on_stall(self, m):
        self.planner_stalled = bool(m.data)
        if self.planner_stalled and getattr(self, "planner_stall_since", None) is None:
            self.planner_stall_since = time.monotonic()
            self.log("PLANNER STALL SIGNALLED on %s" % self.args.stall_topic)
        elif not self.planner_stalled:
            self.planner_stall_since = None

    # ------------------------------------------------------------------ the phases
    def set_phase(self, phase):
        if phase == self.phase:
            return
        self.phase = phase
        self.log("phase -> %s" % phase)
        self.log_fh.write("%.3f,%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n" % (
            time.monotonic(), self.tap.t_us, phase, self.cmd[0], self.cmd[1], self.cmd[2],
            self.lio["p"][0], self.lio["p"][1], self.lio["p"][2],
            self.tap.p[0], self.tap.p[1], self.tap.p[2]))
        self.log_fh.flush()
        if self.phase_pub is not None:
            from std_msgs.msg import String
            m = String()
            m.data = phase
            self.phase_pub.publish(m)

    # ------------------------------------------------------------------ the loop
    def pump(self, seconds=None, until=None):
        period = 1.0 / GATES["rate_hz"]
        t_end = None if seconds is None else time.monotonic() + seconds
        while True:
            if t_end is not None and time.monotonic() >= t_end:
                break
            if until is not None and until():
                break
            self.bridge.poll()
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            self.check_gates()
            if self.abort_reason:
                return
            time.sleep(0.001)
        return None

    def check_gates(self):
        """THE FOUR GATES, all of them the supervisor's (design section 1.3's table).

        THE GATES ARE FOR A MISSION IN PROGRESS, so they are not evaluated once the sortie is ENDING.
        `finish` sets `gates_armed = False` before it lands, and this returns at once.  THAT IS THE FIX TO
        A MEASURED LOOP, not a tidy-up: the abort's own landing pumps through here, and every gate that
        fired is STILL TRUE while the aircraft comes down -- the plan box is still dead, so the tick after
        the landing starts sets `abort_reason` again and `pump` (whose exit condition that is) returns in
        one iteration.  MEASURED on `RECT27-kill-plan` and `RECT27b-kill-plan`, 2026-10-15: both exited with
        `landed: armed=True mode=LAND` 0.05 s after the LAND, the aircraft still at 2.0 m and still armed,
        and the second one did it with the reason explicitly cleared first -- the gate put it back.  An
        abort that keeps re-firing while it lands can never wait for the landing.
        """
        if not self.gates_armed:
            return
        st = self.bridge.state()
        reasons = []
        if self.tap.n > 0 and (abs(self.tap.p[0]) > self.args.geofence_m or
                               abs(self.tap.p[1]) > self.args.geofence_m):
            reasons.append("geofence: truth at [%+.2f %+.2f]" % (self.tap.p[0], self.tap.p[1]))
        if st.armed and st.mode == "LAND" and self.phase not in ("land", "done"):
            reasons.append("the FCU left the mission mode (%s)" % st.mode)
        # THE LIO'S LIVENESS IS NOT READ HERE ANY MORE.  It arrives as the BRIDGE's own report of the
        # stream it is carrying ("the stream I am transporting has stopped" is transport health, design
        # section 4.3), and the firmware's own EKF failsafe is the backstop.
        if self.plan["n"] > 0 and self.bridge_status["relay"] == 1:
            if self.bridge_status["nav_hz"] < self.args.nav_floor_hz:
                reasons.append("THE ESTIMATOR FEED HAS STOPPED: the bridge reports nav_hz=%.1f "
                               "(floor %.1f Hz, age %.2f s) -- the FCU is flying on a stale estimate"
                               % (self.bridge_status["nav_hz"], self.args.nav_floor_hz,
                                  self.bridge_status["nav_age"]))
        # THE PLAN BOX'S LIFELINE: silence is an error now that the end of the tour is a FIELD.
        if self.route_started and self.plan["last_wall"] is not None and self.args.plan_silence_s > 0:
            age = time.monotonic() - self.plan["last_wall"]
            if age > self.args.plan_silence_s:
                reasons.append("THE PLAN BOX HAS STOPPED TALKING: nothing on %s for %.1f s (limit %.1f s) "
                               "after %d messages, trajectory_id %s -- a plan box that is DONE says so in "
                               "the message (trajectory_flag = COMPLETED); SILENCE IS NOT COMPLETION"
                               % (self.args.cmd_topic, age, self.args.plan_silence_s, self.plan["n"],
                                  self.plan["last_tid"]))
        stall = self._stall_reason()
        if stall:
            reasons.append(stall)
        if reasons:
            self.abort_reason = "; ".join(reasons)

    def _stall_reason(self):
        """The legacy `--stall-topic` gate, CARRIED, not dropped (design section 8 item 8): it is the only
        NAMED cause for a dead source that any source could raise, and its replacement (the command stream's
        own liveness, above) is new."""
        if not getattr(self, "planner_stalled", False):
            return None
        up = time.monotonic() - getattr(self, "planner_stall_since", time.monotonic())
        quiet = time.monotonic() - (self.plan["last_wall"] or 0.0)
        if up < self.args.stall_abort_s or quiet < self.args.stall_abort_s:
            return None
        return ("THE PLANNER IS NOT PLANNING: %s has been up for %.1f s and nothing has arrived on %s for "
                "%.1f s -- this is NOT a plan-box timeout" % (self.args.stall_topic, up,
                                                              self.args.cmd_topic, quiet))

    # ------------------------------------------------------------------ preflight / takeoff / land
    def check_own_link(self):
        """CONDITION 1 OF THE F2 GATE, on THIS process's own channel: request, then MEASURE.

        TWO STAGES, because the bench measured that they are different facts.  MEASURED
        (`docs/metrics/f2-second-endpoint-20261016.txt`): the FCU ACCEPTS a 511 request, READS IT BACK as
        exactly what was asked (50000 us) and then sends NOTHING -- its intervals are served from a fixed set
        of buckets (`GCS_Common.cpp:1734-1770`), so the rate you asked for is not the rate you get.  And
        MEASURED while building the bench: before the EKF origin exists, Copter has no position to send and
        LOCAL_POSITION_NED does not exist at all (90 s of nothing), so a position check BEFORE the origin
        would refuse a healthy stack.  Hence: stage 1 = the heartbeat (always broadcast); stage 2 = the
        position stream, MEASURED AFTER the origin is set (`check_position_stream`).
        """
        st = self.bridge.state()
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.link_wait_s:
            self.bridge.poll()
            st = self.bridge.state()
            if st.last_heartbeat_us:
                break
            time.sleep(0.05)
        if not st.last_heartbeat_us:
            self.log("REFUSING: no heartbeat on %s (this process's OWN endpoint)" % self.args.mav)
            return False
        self.log("supervisor: stage 1 ok -- a heartbeat on %s (armed=%s mode=%s landed_state=%s, "
                 "alt AMSL %.1f m)"
                 % (self.args.mav, st.armed, st.mode, st.landed_state, st.alt_amsl_m))
        return True

    def check_position_stream(self, what):
        """Stage 2: the LOCAL_POSITION_NED this endpoint ASKED for, counted as it arrives.

        IT RE-ASKS.  MEASURED on this very endpoint (`logs/missionA-RECT25-kill-plan/supervisor.log`): at
        0.6 s of this process's life the 511 request reached an FCU that was still booting, and the stream
        was 0.0 Hz nine seconds later -- while the F2 bench measured the identical request delivering
        50.2 Hz when it was made after the FCU was up.  So the loop is request -> measure -> request again,
        and it refuses only when the rate is STILL below the floor after `--link-retries`.
        """
        for attempt in range(1, self.args.link_retries + 1):
            n0 = self.bridge.state().last_position_us
            n, t0 = 0, time.monotonic()
            while time.monotonic() - t0 < 2.0:
                self.bridge.poll()
                if self.bridge.state().last_position_us != n0:
                    n0 = self.bridge.state().last_position_us
                    n += 1
                time.sleep(0.002)
            hz = n / (time.monotonic() - t0)
            self.log("supervisor: stage 2 attempt %d/%d (%s) -- LOCAL_POSITION_NED %.1f Hz MEASURED on "
                     "this endpoint (floor %.1f Hz)"
                     % (attempt, self.args.link_retries, what, hz, self.args.link_floor_hz))
            if hz >= self.args.link_floor_hz:
                return True
            if attempt < self.args.link_retries:
                self.log("      re-asking this endpoint for its rates (a 511 sent into the FCU's boot is "
                         "accepted by nobody -- MEASURED)")
                self.bridge.request_rates()
                time.sleep(1.0)
        self.log("REFUSING TO CONTINUE: this endpoint's own position stream is still below the %.1f Hz "
                 "floor after %d attempts.  MEASURED on the bench, a 511 request can be ACCEPTED, read back "
                 "exactly as asked, and deliver nothing -- so 'I asked for it' is not evidence."
                 % (self.args.link_floor_hz, self.args.link_retries))
        return False

    def preflight(self):
        self.set_phase("preflight")
        self.log("supervisor: navigation=%s, this process's endpoint %s, the bridge's %s"
                 % (self.args.nav_source, self.args.mav, self.args.cmd_topic))
        if not self.check_own_link():
            return "this process's own MAVLink endpoint did not verify"
        self.log("waiting for the LIO on %s (PREFLIGHT ONLY: in flight the LIO is the plan box's and the "
                 "bridge's, never the supervisor's)" % self.args.lio_topic)
        self.pump(self.args.lio_wait_s, until=lambda: self.lio["n"] > 20)
        if self.lio["n"] <= 20:
            return "no LIO odometry on %s" % self.args.lio_topic
        self.log("LIO live: %d messages, rate %.1f Hz" % (self.lio["n"], self.lio["rate_slow"]))
        # the parked alignment, MEASURED AND PRINTED -- a CHECK now, not the frame the bridge converts with
        # (the bridge loads the same --calib-file the plan box publishes with, so the L->W->wire round trip
        # is exact; see `bridge/bridge_node.py::load_calib` for why that moved).
        samples = []
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            self.pump(0.02)
            if self.tap.n > 0 and self.lio["n"] > 0:
                samples.append((self.lio["p"].copy(), self.tap.p.copy()))
        if len(samples) < 20:
            return "not enough parked samples to check the alignment"
        L = np.array([s[0] for s in samples])
        W = np.array([s[1] for s in samples])
        t = W.mean(axis=0) - self.calib.R_WL @ L.mean(axis=0)
        resid = (self.calib.R_WL @ L.T).T + t - W
        self.log("      parked alignment CHECK: translation measured from %d samples is t = "
                 "[%+.4f %+.4f %+.4f] m, rms %.4f m; the bridge converts with %s"
                 % (len(samples), t[0], t[1], t[2],
                    float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))), self.calib.summary()))
        self.log("      (the parked mismatch to the file the bridge uses is [%+.3f %+.3f %+.3f] m in W; the "
                 "rotation gets checked on the flight data, where the path shape identifies it)"
                 % tuple(t - self.calib.t_WL))
        self.log("setting the EKF origin")
        if not self.bridge.set_ekf_origin(HOME_LAT, HOME_LON, HOME_ALT, pump=self.pump):
            return "EKF origin never accepted"
        self.pump(0.5)
        if not self.check_position_stream("after the EKF origin, which is what makes Copter able to send "
                                          "a position at all"):
            return "this process's own position stream never arrived after the EKF origin"
        st = self.bridge.state()
        self.log("FCU: armed=%s mode=%s pos=[%+.3f %+.3f %+.3f] alt AMSL %.1f m gps_fix=%d"
                 % (st.armed, st.mode, *st.pos, st.alt_amsl_m, st.gps_fix))
        self.log("      (this line is the stack's assertion that the EKF origin IS the pad, which is what "
                 "makes W and the FCU's local frame the same up to axis labels)")
        if st.gps_fix > 1:
            return "the FCU has a GPS fix (%d) -- mission A is defined as no-GPS" % st.gps_fix
        if not self.bridge.set_mode(Mode.GUIDED):
            return "could not enter GUIDED"
        if not self.bridge.calibrate_accel_simple():
            self.log("note: simple accel calibration not ACKed (may already be calibrated)")
        self.pump(0.5)
        if not self.bridge.arm(timeout_s=25.0):
            return "arming refused: %s" % _texts(self.bridge.state())
        self.pump(2.0)
        st = self.bridge.state()
        if not st.armed:
            return "the FCU does not report itself armed"
        self.log("armed; landed_state=%d" % st.landed_state)
        return None

    def takeoff(self):
        """THE TAKE-OFF, and WHICH CONTRACT IS FLOWN is decided by `--takeoff-command-m` and stated below.

        TWO MECHANISMS, BOTH IN THE CODE, ONE OF THEM FLOWN, and the log says which:

        * THE COMMAND HEIGHT.  `--takeoff-command-m` is what goes out as `packet.z` -- VERBATIM, with no
          datum added (`bridge/mavlink_impl.py::takeoff`; the bench settled that a 2.500 m request is
          ACCEPTED and ADOPTED as 2.5 m above the pad, CTUN DAlt peak +2.525).  The FCU's take-off
          controller then flies the aircraft to that height and its `landed_state` becomes flying.
        * THE SETPOINT-DRIVEN CLIMB (the OWNER'S design change, 2026-10-15: "起飞命令就飞20cm高，然后立马进定点").
          With `--takeoff-command-m 0.2` the command is only "leave the ground cleanly" and the CLIMB TO THE
          MISSION ALTITUDE IS THE PLAN'S: `--trigger-on-airborne` puts the go out the moment the vehicle is
          airborne, and the plan box's first target is a VERTICAL leg to the mission altitude
          (`rectangle_source.py --vertical-first-m`), held for the A4 window.  Position setpoints cancel
          GUIDED's take-off SubMode by design (STATUS row 25, trap 9), which is what hands the climb over.
        * `--climb-guard-m` STAYS ON either way and bounds the climb against the MISSION altitude
          (`--alt`), not against the command, so a 0.2 m command with a 2.5 m mission does not trip it.
        """
        self.set_phase("takeoff")
        if not self.bridge.takeoff(self.args.takeoff_command_m):
            return "takeoff refused: %s" % _texts(self.bridge.state())
        z0 = float(self.tap.p[2])
        if abs(self.args.takeoff_command_m - self.args.alt) < 1e-9:
            self.log("takeoff commanded to %.2f m, the mission altitude, VERBATIM (mechanism: the FCU's "
                     "own take-off controller flies the whole climb); the plan's setpoints are NOT on the "
                     "wire yet -- position setpoints cancel GUIDED's take-off SubMode"
                     % self.args.takeoff_command_m)
        else:
            self.log("takeoff commanded to only %.2f m (mechanism: SETPOINT-DRIVEN CLIMB -- the command "
                     "is just 'leave the ground', the mission altitude %.2f m is the PLAN's; the go goes "
                     "out on airborne=%s and the plan's first target is a vertical leg)"
                     % (self.args.takeoff_command_m, self.args.alt, self.args.trigger_on_airborne))
        # The airborne threshold is BELOW the command: with a 0.2 m command the FCU reaches its own target
        # and stops, so a threshold at the command height would never be met.  It is a "has it left the pad"
        # test, not a "has it reached the target" one.
        thr = min(0.25, max(0.10, 0.5 * self.args.takeoff_command_m))
        self.pump(12.0, until=lambda: self.tap.p[2] > z0 + thr)
        if self.tap.p[2] <= z0 + thr:
            return ("never left the ground (truth %.3f m after 12.0 s, threshold %.2f m above %.3f m)"
                    % (self.tap.p[2], thr, z0))
        self.log("airborne: truth altitude %.3f m (threshold %.2f m; the take-off command was %.2f m)"
                 % (self.tap.p[2], thr, self.args.takeoff_command_m))
        return None

    def hold(self, seconds):
        """The hold, and then THE GO.  Until the trigger, NO setpoint is on the wire, so the aircraft is
        held by the FCU's own take-off target and NOTHING the mission commands bounds it.

        THAT IS A DESIGN SMELL, MEASURED.  On `RECT25-kill-plan` the FCU's own take-off target was ~9.8 m
        while the supervisor had asked for 2.5 m, and for the 3 s of this hold plus the pre-trigger slack
        the only thing between the aircraft and that target was the FCU (`CTUN alt +9.757 / dalt +9.797`).
        The aircraft came back to 2.0 m the moment the plan's setpoints reached it.  Two responses, both
        implemented and NEITHER YET FLOWN:
          * `--climb-guard-m` (default 1.0, ON): if the TRUTH climbs more than this above the commanded
            take-off altitude before the trigger, the supervisor STOPS waiting and takes over with a NAMED
            abort -- this box is the one allowed to read the truth, so this is where the bound belongs;
          * `--trigger-on-airborne`: publish the go as soon as the vehicle is airborne instead of after this
            hold, so the plan's setpoints pin the altitude for the whole climb (at the cost of the A4
            window being a climb-and-transit rather than a hover; the default keeps the flown profile).
        """
        self.set_phase("hold")
        if self.args.trigger_on_airborne:
            self.log("the go goes out NOW (--trigger-on-airborne): the plan's setpoints, not the FCU's "
                     "take-off target, bound the climb")
            self.publish_trigger()
        self.log("holding for %.1f s before the trigger (the A4 window); the climb guard is %.2f m above "
                 "%.2f m" % (seconds, self.args.climb_guard_m, self.args.alt))
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if self.tap.n > 0 and self.tap.p[2] > self.args.alt + self.args.climb_guard_m:
                return ("THE TAKE-OFF ALTITUDE DID NOT MATCH THE REQUEST: the truth is at %.2f m for a "
                        "commanded %.2f m (guard %.2f m).  ArduPilot's GUIDED take-off GUARD compares the "
                        "requested number against its GLOBAL altitude while its EXECUTOR ADDS that same "
                        "number to the current up-estimate (`ArduCopter/takeoff.cpp`: "
                        "`do_user_takeoff_U_m`'s `takeoff_alt_m <= copter.current_loc.alt * 0.01` vs "
                        "`_TakeOff::start_m`'s `take_off_complete_alt_m = take_off_start_alt_m + alt_m`), "
                        "so a datum-compensated request can pass the guard and command a climb of that "
                        "whole datum.  MEASURED once: `RECT25-kill-plan` reached ~9.8 m on a 2.50 m command "
                        "(docs/metrics/f2-supervisor-20261016.txt section 4) -- this gate would have named "
                        "it in the first second"
                        % (self.tap.p[2], self.args.alt, self.args.climb_guard_m))
        return None

    def publish_trigger(self):
        from geometry_msgs.msg import PoseStamped
        m = PoseStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = "world_w"
        m.pose.orientation.w = 1.0
        self.trigger_pub.publish(m)
        self.trigger_sent = True
        self.log("THE GO: %s published (latched, once, when the vehicle is airborne and holding) -- the "
                 "bridge may now put the plan's setpoints on the wire" % self.args.trigger_topic)

    def route(self):
        """Watch the plan's statements until it declares COMPLETED, and window the phases it implies."""
        if not self.trigger_sent:
            self.publish_trigger()
        t0 = time.monotonic()
        while self.plan["n"] == 0:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if time.monotonic() - t0 > self.args.plan_wait_s:
                return ("THE PLAN BOX IS NOT TALKING: nothing on %s within %.1f s of the trigger"
                        % (self.args.cmd_topic, self.args.plan_wait_s))
        self.route_started = True
        self.log("the plan box is talking: %d messages, trajectory_id %s, frame %s"
                 % (self.plan["n"], self.plan["last_tid"], self.plan.get("frame")))
        self.set_phase("leg%d" % (self.plan["last_tid"] or 1))
        while True:
            self.pump(0.05)
            if self.abort_reason:
                return self.abort_reason
            if self.plan["flag"] == TRAJECTORY_STATUS_COMPLETED:
                self.log("the plan box has DECLARED THE TOUR COMPLETE (trajectory_flag = COMPLETED after "
                         "waypoint %s): %.1f s of route, %d setpoints relayed by the bridge"
                         % (self.plan["last_tid"], time.monotonic() - t0, self.bridge_status["sent"]))
                return None

    def land(self, settle_s=5.0):
        """NORMAL LAND.  The plan is already off the wire -- the BRIDGE takes it off when the stream says
        COMPLETED, and again when the mode leaves GUIDED -- so this is the authority acting alone.

        `settle_s` IS NOT A DELAY FOR ITS OWN SAKE; it is the answer to a MEASURED false end.  The wait
        below is "until the FCU says it is not armed", and `armed` is a LEVEL in the heartbeat, not an
        event: the 1 Hz heartbeat that was in flight when LAND was commanded can still carry `armed=True`
        while the mode has already changed.  MEASURED on `RECT27b-kill-plan` (2026-10-15): the abort's LAND
        was followed by `landed: armed=True mode=LAND` 0.05 s later and the process exited with the
        aircraft at 2.0 m, so a wait that starts by sampling the flag immediately can end on a STALE
        sample.  Settling first costs at most `settle_s` on a landing that cannot finish faster than that
        anyway (a LAND from 2.5 m takes ~12 s), and removes the whole class of "it said not-armed once".
        """
        self.set_phase("land")
        self.bridge.land()
        self.pump(settle_s)                     # let the commanded mode take effect before it is trusted
        self.pump(40.0, until=lambda: not self.bridge.state().armed)
        st = self.bridge.state()
        self.log("landed: armed=%s mode=%s" % (st.armed, st.mode))
        self.set_phase("done")
        return None if not st.armed else "still armed after LAND"

    def run(self):
        from bridge.mavlink_impl import MavlinkBridge
        if self.args.calib_file and os.path.exists(self.args.calib_file):
            import json
            d = json.load(open(self.args.calib_file))
            self.calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                                      t_WL=np.array(d["t_WL"], dtype=float))
        else:
            self.calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
        self.log_fh = open(self.args.phase_log, "w")
        self.log_fh.write("t_wall,t_sim_us,phase,cmd_x,cmd_y,cmd_z,lio_x,lio_y,lio_z,truth_x,truth_y,"
                          "truth_z\n")
        self.route_started = False
        self.planner_stalled = False
        self.planner_stall_since = None
        self.gates_armed = True                    # set in __init__ too; see the comment there
        self.start_ros()
        self.bridge = MavlinkBridge(url=self.args.mav, log=self.log)
        try:
            steps = [self.preflight, self.takeoff, lambda: self.hold(self.args.hold), self.route,
                     self.land]
            for step in steps:
                err = step()
                if err:
                    self.abort_reason = err
                    break
        finally:
            self.finish()
        return 0 if self.abort_reason is None else 1

    def finish(self):
        if self.abort_reason:
            # THE REASON IS CARRIED OUT OF THE PUMP LOOP, NOT LEFT IN IT.  `pump` STOPS as soon as
            # `abort_reason` is set (it is the loop's own exit), so a pump that runs while the reason is
            # still set is a ZERO-SECOND no-op.  The reason is restored before the final read, so the
            # summary and the process rc still name what ended the sortie.  WHERE ELSE IT HAS TO BE CLEARED
            # is a different occurrence of the same signature and is explained at the landing below.
            reason = self.abort_reason
            self.abort_reason = None
            self.gates_armed = False
            self.set_phase("abort")
            self.log("ABORT: %s" % reason)
            try:
                # RTL is available to the supervisor (it is a class-76 mode); the mission's own abort has
                # used LAND since S3, and the choice is a parameter rather than a rewrite.
                if self.args.abort_mode == "rtl":
                    self.bridge.set_mode(Mode.RTL)
                    self.pump(self.args.abort_land_s, until=lambda: not self.bridge.state().armed)
                else:
                    # THE LANDING IS THE SAME ACT AS THE NORMAL ONE, so it is the same code: `self.land()`
                    # commands LAND, pumps until the FCU reports DISARMED (bounded by the landing's own
                    # 40 s), names the phase `land` -- the phase log's ending is therefore the same on an
                    # abort as on a completed tour -- and logs `landed: armed=... mode=...`.
                    #
                    # MEASURED ON `RECT27-kill-plan`, 2026-10-15, WHICH IS WHY THIS IS NOT A
                    # COMMAND-AND-EXIT: the old abort path sent LAND and fell straight through to the final
                    # read 0.05 s later --
                    #     [  28.94] bridge/mavlink: mode -> LAND
                    #     [  28.94] final: armed=True mode=LAND alt AMSL 586.0 m
                    # -- and the FCU's own log for that sortie ends with the aircraft STILL ARMED AT
                    # 2.002 m (no MODE 9 until t 41.72, the FCU still in mode 4 when the log closed).  The
                    # abort had taken over the MODE and let go of the AIRCRAFT.  A process that exits before
                    # the vehicle is down has not ended the sortie cleanly, whatever its phase log says.
                    #
                    # WHAT IS DELIBERATELY NOT DONE: the landing's own "still armed after LAND" verdict is
                    # DISCARDED.  `reason` is why this sortie ended and what a reader must see; overwriting
                    # it with a post-abort outcome would lose the finding.  That outcome is not hidden --
                    # `landed: armed=...` and the `final:` line below carry it, and `tools/metrics.py`'s B4
                    # reads it off the phase log.
                    #
                    # AND THE REASON IS CLEARED FOR THE DURATION, WHICH IS NOT BOOKKEEPING.  `pump` returns
                    # the moment `abort_reason` is set, and `check_gates` sets it AGAIN during the landing:
                    # the tick after LAND arrives reads `st.armed and st.mode == "LAND" and self.phase not
                    # in ("land", "done")` -- "the FCU left the mission mode" -- which is TRUE of the very
                    # landing this block just commanded.  MEASURED on `RECT27b-kill-plan`: with the reason
                    # left set, the landing pump ran ONE iteration and the process again exited with the
                    # aircraft armed at 2.0 m.  The parking brake of the abort was switching itself back on.
                    self.abort_reason = None
                    self.land()
            except Exception as e:                                      # pragma: no cover
                self.log("could not command the abort mode: %s" % e)
            self.abort_reason = reason
        st = self.bridge.state()
        ctr = self.bridge.counters()
        self.log("final: armed=%s mode=%s alt AMSL %.1f m; %d setpoints, %d external-nav messages "
                 "(the SUPERVISOR sent none of them: class 84 is the bridge's)"
                 % (st.armed, st.mode, st.alt_amsl_m, ctr["positions"], ctr["external_nav"]))
        self.log("the FCU's last words: %s" % _texts(st))
        self.log_fh.close()
        try:
            self.bridge.close()
        except Exception:
            pass


def _texts(st):
    return " | ".join(t for _, t in st.messages[-4:]) or "(nothing)"


def main(argv=None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default="tcp:127.0.0.1:5761",
                    help="THIS process's own MAVLink endpoint.  The bridge is on serial0 (5760) and this is "
                         "serial1 (5761): one writer per message class, no arbitration (design section 4.2)")
    ap.add_argument("--truth-topic", default="/sim/truth/odom")
    ap.add_argument("--lio-topic", default="/lio/robo/odom",
                    help="read ONLY by the preflight liveness gate; never in flight")
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    ap.add_argument("--status-topic", default="/bridge/status")
    ap.add_argument("--trigger-topic", default="/plan/trigger")
    ap.add_argument("--stall-topic", default="/planner/stalled")
    ap.add_argument("--stall-abort-s", type=float, default=3.0)
    ap.add_argument("--nav-source", default="lio", choices=["lio"],
                    help="the truth-fed calibration flight is NOT ported to F2 -- see the evidence file, "
                         "section 'what this round could not do'")
    ap.add_argument("--calib-file", default="")
    ap.add_argument("--alt", type=float, default=2.5, help="MISSION altitude, m above the pad.  This is the "
                    "height the guard and the A4 window are judged against; it is NOT automatically what "
                    "the take-off command asks for (see --takeoff-command-m)")
    ap.add_argument("--takeoff-command-m", type=float, default=0.2,
                    help="WHAT GOES OUT AS packet.z, verbatim, m above the pad -- the OWNER'S take-off "
                         "design (2026-10-15): 0.2 m is 'just leave the ground cleanly', and the climb to "
                         "--alt is flown by the PLAN's setpoints once the go is out.  Set it equal to "
                         "--alt for the other mechanism, in which the FCU's own take-off controller flies "
                         "the whole climb (BENCH: a 2.50 m command is ACCEPTED and ADOPTED as 2.50 m above "
                         "the pad, CTUN DAlt peak +2.525).  THE FLOWN ONE IS PRINTED AT START-UP")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds at the take-off point before the go")
    ap.add_argument("--climb-guard-m", type=float, default=1.0,
                    help="ABORT if the truth climbs this far above the commanded take-off altitude "
                         "before the trigger: until the plan setpoints reach the FCU, nothing the "
                         "mission commands bounds the climb (MEASURED: 9.6 m on a 2.50 m command)")
    ap.add_argument("--trigger-on-airborne", dest="trigger_on_airborne", action="store_true",
                    default=True,
                    help="DEFAULT: publish the go the moment the vehicle is airborne, so the plan's "
                         "setpoints bound the climb.  MEASURED on RECT25-kill-plan: with the go held "
                         "back for the take-off hold, the FCU's own take-off target ran to ~9.8 m on "
                         "a 2.50 m command and only the arrival of the plan's setpoints stopped it "
                         "(docs/metrics/f2-supervisor-20261016.txt section 4).  NOT YET FLOWN in this "
                         "form: it changes the A4 window from a hover at the take-off point to a "
                         "climb-and-transit")
    ap.add_argument("--trigger-after-hold", dest="trigger_on_airborne", action="store_false",
                    help="the F1 sequencing (go after the hold), kept for the A/B -- it is the "
                         "sequence that let the take-off target run away")
    ap.add_argument("--geofence-m", type=float, default=4.60)
    ap.add_argument("--abort-mode", default="land", choices=["land", "rtl"])
    ap.add_argument("--abort-land-s", type=float, default=40.0,
                    help="wait up to this long for the FCU to DISARM on an RTL abort.  A LAND abort does "
                         "not use it: it goes through `self.land()`, which owns the same 40 s wait the "
                         "normal landing uses -- MEASURED on RECT27-kill-plan, where the old abort path "
                         "sent LAND and exited 0.05 s later with the aircraft armed at 2.0 m")
    ap.add_argument("--link-wait-s", type=float, default=30.0)
    ap.add_argument("--link-floor-hz", type=float, default=10.0,
                    help="REFUSE TO ARM below this MEASURED position rate on this endpoint")
    ap.add_argument("--link-retries", type=int, default=5,
                    help="ask -> measure -> ask AGAIN this many times before refusing (a 511 request sent "
                         "into the FCU's boot delivers 0 Hz forever; MEASURED, twice)")
    ap.add_argument("--nav-floor-hz", type=float, default=30.0,
                    help="abort when the bridge reports less than this estimator feed")
    ap.add_argument("--lio-wait-s", type=float, default=25.0)
    ap.add_argument("--plan-wait-s", type=float, default=20.0)
    ap.add_argument("--plan-silence-s", type=float, default=3.0,
                    help="A DEAD PLAN BOX IS AN ABORT, not a finished tour: the tour's end is a FIELD "
                         "(trajectory_flag = COMPLETED), so silence is an error.  0 disables")
    ap.add_argument("--phase-log", required=True)
    args = ap.parse_args(argv)
    GATES["geofence_m"] = args.geofence_m
    print("supervisor: endpoint %s (its OWN), phase log %s" % (args.mav, args.phase_log), flush=True)
    print("supervisor: gates -- geofence %.2f m on TRUTH, nav floor %.1f Hz, plan silence %.1f s, "
          "class 76 only (it never sends a setpoint)" % (args.geofence_m, args.nav_floor_hz,
                                                         args.plan_silence_s), flush=True)
    # WHICH TAKE-OFF CONTRACT IS FLOWN, IN THE RUN'S OWN WORDS.  Two mechanisms exist and they fly very
    # different profiles, so a reader must not have to infer which one from a number in the log.
    if abs(args.takeoff_command_m - args.alt) < 1e-9:
        print("supervisor: TAKE-OFF = FCU-TARGET: packet.z = %.3f m VERBATIM (= --alt); the FCU's own "
              "take-off controller flies the whole climb, the go goes out %s"
              % (args.takeoff_command_m,
                 "ON AIRBORNE (--trigger-on-airborne)" if args.trigger_on_airborne
                 else "AFTER THE HOLD (--trigger-after-hold)"), flush=True)
    else:
        print("supervisor: TAKE-OFF = SETPOINT-DRIVEN CLIMB (the owner's design): packet.z = %.3f m only "
              "('leave the ground'); the mission altitude %.3f m is the PLAN's, the go goes out %s, and the "
              "plan's first target must be a VERTICAL leg (rectangle_source.py --vertical-first-m)"
              % (args.takeoff_command_m, args.alt,
                 "ON AIRBORNE (--trigger-on-airborne)" if args.trigger_on_airborne
                 else "AFTER THE HOLD (--trigger-after-hold)"), flush=True)
    print("supervisor: the take-off's own guard -- abort if the TRUTH passes %.2f m (mission altitude "
          "%.2f + climb guard %.2f); nothing else bounds the flight before the trigger"
          % (args.alt + args.climb_guard_m, args.alt, args.climb_guard_m), flush=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.phase_log)), exist_ok=True)
    return Supervisor(args).run()


if __name__ == "__main__":
    sys.exit(main())
