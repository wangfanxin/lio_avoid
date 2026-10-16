#!/usr/bin/env python3
"""The BRIDGE's ROS-facing half: it consumes the PLAN box's command stream and drives MAVLink 84.

THE ARROW THIS FILE IS.  `_stage/refactor-bridge-plan.md` section 5, change 2: until F1 the thing that
streamed message 84 was the FSM (`fsm/mission_fsm.py::pump`, 50 Hz, from a `nav_msgs/Path` it had walked
itself).  F1 moves that to the transport, and the input is not a Path but the plan box's own command
stream: `/plan/cmd`, `quadrotor_msgs/msg/PositionCommand`, in L.  Nothing about the WIRE changes -- the
wire rate stays 50 Hz, because 50 Hz is what every mission-A sortie flew and `ARCHITECTURE.md:251-258`
explains that 50 is the firmware's own ceiling -- but the PUBLISHER may run faster, and the plan box's
does (100 Hz, its own timer).

WHY A RELAY AND NOT A SUBSCRIPTION THAT WRITES ON ARRIVAL.  Two independent numbers, deliberately:

  * the PUBLISHER owns its content rate (ego's `traj_server` is a 10 ms timer; `rectangle_source.py`'s
    stream is a 10 ms timer too);
  * the WIRE owns its rate, and it is 50 Hz.

A subscriber that wrote a setpoint per message would put the wire rate under the plan box's control -- so
a plan box that decided to publish at 400 Hz would quadruple the FCU's setpoint load, and one that
published at 4 Hz (which is exactly what the S3 shadow stream did, `_stage/refactor-S3-frames-and-discrepancies.txt`
section 1.3) would starve the position loop.  `tick()` therefore sends at most ONE setpoint per due tick,
taken from the NEWEST sample, and the achieved rate is counted here.

WHAT IT ALSO OWNS: the liveness of the stream it is carrying.  "The plan box has stopped talking" and "the
plan box says COMPLETED" are two different facts, and until F1 they were the SAME fact to the consumer
(`_await_next_leg` returned "no more legs" on a timeout, `legs()` returned `None` -- not an error -- and
the mission landed; `_stage/refactor-bridge-plan.md` section 2(e)).  The relay counts the samples, and
`stalled_reason()` turns silence into a NAMED abort the mission can act on.  It is **not** the supervisor:
the authority to abort is not the transport's (section 8 item 4), so this file reports and the mission
decides.

usage: imported by `fsm/mission_fsm.py` (F1: in the mission's process, one MAVLink link, one writer) and by
       `bridge/bridge_node.py`'s own main() (F2: its own process, its own endpoint)
"""
from __future__ import annotations

import json
import os
import sys
import time

import numpy as np

# THE REPO ROOT ON THE PATH: this file is run BY PATH (`python3 bridge/bridge_node.py`), so Python's first
# sys.path entry is `bridge/`, not the repository -- MEASURED the hard way, as a bridge process that died
# with `ModuleNotFoundError: No module named 'planner'` two seconds into a sortie.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from pymavlink import mavutil

from planner.cmd_stream import (CmdSample, FRAME_L, TRAJECTORY_STATUS_COMPLETED,
                                TRAJECTORY_STATUS_EMPTY, TRAJECTORY_STATUS_READY)

#: The frame names the relay accepts on `header.frame_id`.  `lio_world` is ours; `world` is ego's name for
#: its own planning world, which is the same frame under a different name (the adopted interface, section
#: 3.3: "a parameter, not a shim").  A message in any OTHER frame is REFUSED rather than converted by
#: guesswork -- a setpoint whose frame is unknown is the defect class this refactor exists to remove.
DEFAULT_FRAME_IDS = (FRAME_L, "world")


class CmdRelay:
    """Consumes `/plan/cmd` (L) and puts the newest sample on MAVLink 84 at `rate_hz`.

    The relay does not own a clock of its own: `tick(now)` is called from the mission's 50 Hz loop, so the
    wire is driven by the process that already runs at that rate, and the F2 split (a bridge process with
    its own loop) reuses the same object unchanged.
    """

    def __init__(self, bridge, calib, rate_hz=50.0, frame_ids=DEFAULT_FRAME_IDS, fields="pos",
                 yaw_rad=None, log=None, silence_abort_s=3.0, topic="/plan/cmd"):
        self.bridge = bridge
        self.calib = calib
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.frame_ids = tuple(frame_ids)
        self.fields = fields
        self.yaw_rad = yaw_rad
        self.log = log or (lambda s: None)
        self.topic = topic
        self.silence_abort_s = float(silence_abort_s)
        # ---- state
        self.enabled = False
        self.enabled_wall = None
        self.newest = None                  # the newest CmdSample received
        self.newest_wall = None
        self.n_rx = 0
        self.n_sent = 0
        self.n_refused = 0
        self.refused_why = ""
        self.last_sent = None               # the sample the wire last carried
        self.last_sent_w = None             # ... and where that was in W (what the FCU was told)
        self.last_sent_wall = None
        self.last_flag = None
        self.ids_seen = []
        self.next_t = None
        self._rate_t0 = None
        self._rate_n = 0
        self.rate_hz_achieved = 0.0
        self.rate_hz_rx = 0.0
        self._rx_t0 = None
        self._rx_n = 0

    # ---- the subscription ------------------------------------------------------------------------
    def attach(self, node):
        """Create the `/plan/cmd` subscription on an existing rclpy node.

        RELIABLE + TRANSIENT_LOCAL + a small depth, matching the publisher's own QoS: the plan box starts
        BEFORE this relay in the F2 order, so a late subscriber must still find the newest sample rather
        than wait for the next leg.
        """
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        from quadrotor_msgs.msg import PositionCommand
        qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._msg_type = PositionCommand
        return node.create_subscription(PositionCommand, self.topic, self.on_cmd, qos)

    def on_cmd(self, m):
        """One `PositionCommand` -> one `CmdSample` in L.  The frame is CHECKED, not assumed."""
        frame = m.header.frame_id
        if frame not in self.frame_ids:
            self.n_refused += 1
            if self.refused_why != frame:
                self.refused_why = frame
                self.log("bridge/relay: REFUSING %s messages whose header.frame_id is %r; this bridge "
                         "accepts %s (the plan box's L) and ego's name for the same frame.  A setpoint "
                         "whose frame is unknown is not converted by guesswork."
                         % (self.topic, frame, " or ".join(repr(f) for f in self.frame_ids)))
            return
        p = np.array([m.position.x, m.position.y, m.position.z], dtype=float)
        v = np.array([m.velocity.x, m.velocity.y, m.velocity.z], dtype=float)
        a = np.array([m.acceleration.x, m.acceleration.y, m.acceleration.z], dtype=float)
        s = CmdSample(p, v, a, m.yaw, m.yaw_dot, m.trajectory_id, m.trajectory_flag, frame)
        self.newest = s
        self.newest_wall = time.monotonic()
        self.n_rx += 1
        if self._rx_t0 is None:
            self._rx_t0, self._rx_n = self.newest_wall, 0
        self._rx_n += 1
        if self.newest_wall - self._rx_t0 > 1.0:
            self.rate_hz_rx = self._rx_n / (self.newest_wall - self._rx_t0)
        if not self.ids_seen or self.ids_seen[-1] != s.trajectory_id:
            if s.trajectory_id not in self.ids_seen:
                self.ids_seen.append(s.trajectory_id)

    # ---- the go, and the override ----------------------------------------------------------------
    def enable(self, why=""):
        """Start writing setpoints.  THE ONE-WAY GATE the mission drives: until this is called the relay
        sends NOTHING, which is what keeps position setpoints away from GUIDED's take-off SubMode
        (MEASURED: a position setpoint cancels a takeoff -- `mission_fsm.py:9-10`)."""
        if self.enabled:
            return
        self.enabled, self.enabled_wall = True, time.monotonic()
        self.log("bridge/relay: ENABLED%s -- the command stream reaches message 84 from now on"
                 % ("" if not why else " (%s)" % why))

    def disable(self, why=""):
        """Stop writing setpoints.  Called on LAND and on every abort: a setpoint streamed into LAND mode
        would be a second, silent command authority."""
        if not self.enabled:
            return
        self.enabled = False
        self.log("bridge/relay: DISABLED after %d setpoints%s -- nothing of the plan reaches the FCU"
                 % (self.n_sent, "" if not why else " (%s)" % why))

    def set_calib(self, calib):
        """Replace the L -> W calibration AFTER the mission has measured its own.

        WHY THIS EXISTS, and it is not optional: `fsm/mission_fsm.py::preflight` MEASURES the translation
        from a parked window and REPLACES `self.calib` with it (rotation from the construction -- MEASURED
        2026-10-13: a parked window cannot identify a rotation, and fitting one gave a 90 deg error and a
        runaway).  That measurement also feeds the EKF, so the COMMAND must use it too or the estimate and
        the command live in frames that differ by the measurement -- on the last flight logs, t = +3, +2,
        -12 mm (`logs/missionA-RECT21-tight-ekf/fsm.log:9-11`).  A relay that kept the constructor's
        reference would silently command the pre-measurement frame.
        """
        self.calib = calib

    # ---- the wire ---------------------------------------------------------------------------------
    def tick(self, now):
        """One wire tick.  Returns None when nothing was sent, else the (p_f, yaw, sample) that went out."""
        if not self.enabled or self.newest is None:
            return None
        if self.next_t is None:
            self.next_t = now
        if now < self.next_t:
            return None
        self.next_t += self.period
        if self.next_t < now:                    # a long stall must not build a backlog of setpoints
            self.next_t = now + self.period
        s = self.newest
        yaw = s.yaw if self.yaw_rad is None else float(self.yaw_rad)
        p_f = self.bridge.send_cmd(s, self.calib, yaw_rad=yaw, fields=self.fields)
        self.last_sent, self.last_sent_w, self.last_sent_wall = s, p_f, now
        self.last_flag = s.trajectory_flag
        self.n_sent += 1
        if self._rate_t0 is None:
            self._rate_t0, self._rate_n = now, 0
        self._rate_n += 1
        if now - self._rate_t0 > 1.0:
            self.rate_hz_achieved = self._rate_n / (now - self._rate_t0)
        return (p_f, yaw, s)

    # ---- what the mission reads -------------------------------------------------------------------
    @property
    def completed(self) -> bool:
        """The plan box has declared the tour FINISHED (and is still talking, by construction)."""
        return self.newest is not None and self.newest.trajectory_flag == TRAJECTORY_STATUS_COMPLETED

    def age_s(self, now=None):
        if self.newest_wall is None:
            return float("inf")
        return (time.monotonic() if now is None else now) - self.newest_wall

    def never_reason(self, wait_s, now=None):
        """A NAMED reason for "the plan box never said anything", or None."""
        if self.n_rx > 0 or self.enabled_wall is None:
            return None
        now = time.monotonic() if now is None else now
        if now - self.enabled_wall < wait_s:
            return None
        return ("THE PLAN BOX IS NOT TALKING: no message on %s in %.1f s (it accepts frames %s; %d "
                "refused) -- this is NOT a completed tour, it is a plan box that never produced a command"
                % (self.topic, now - self.enabled_wall,
                   " or ".join(repr(f) for f in self.frame_ids), self.n_refused))

    def stalled_reason(self, now=None):
        """A NAMED reason for "the plan box STOPPED talking", or None.

        THE DEFECT THIS FIXES (design sections 2(e) and R3): before F1 a dead producer and a finished tour
        looked identical to the consumer -- an empty path meant "no more legs", a timeout meant the same
        thing, and the mission landed and was scored as COMPLETE with no `abort` phase.  Now the tour's end
        is a FIELD (`trajectory_flag = COMPLETED`) and this is the other fact, told apart by silence.
        """
        if not self.enabled or self.n_rx == 0 or self.silence_abort_s <= 0.0:
            return None
        now = time.monotonic() if now is None else now
        age = now - (self.newest_wall or now)
        if age < self.silence_abort_s:
            return None
        return ("THE PLAN BOX HAS STOPPED TALKING: nothing on %s for %.1f s (limit %.1f s) after %d "
                "messages at %.1f Hz, trajectory_id %s, last flag %s -- a plan box that is DONE says so "
                "in the message (trajectory_flag = COMPLETED); silence is not completion"
                % (self.topic, age, self.silence_abort_s, self.n_rx, self.rate_hz_rx,
                   self.ids_seen[-1] if self.ids_seen else "-", self.last_flag))

    def status(self) -> str:
        return ("/plan/cmd: %d received (%.1f Hz), %d refused, %d waypoint ids %s, last flag %s; "
                "%d setpoints on the wire (%.2f Hz achieved of %.1f asked)"
                % (self.n_rx, self.rate_hz_rx, self.n_refused, len(self.ids_seen),
                   self.ids_seen[-1] if self.ids_seen else "-", self.last_flag,
                   self.n_sent, self.rate_hz_achieved, self.rate_hz))


# ======================================================================================================
# F2: THE BRIDGE AS ITS OWN PROCESS
# ======================================================================================================
#
# WHAT MOVED, AND WHAT DID NOT.  Until F2 the ROS half of the transport and the mission lived in one
# process (`fsm/mission_fsm.py`) sharing ONE MAVLink link.  F2 splits them: this process owns the LIO's
# estimator feed (331) and the plan's setpoints (84) on `--mav` (serial0), and `supervisor/supervisor.py`
# owns arm/takeoff/mode/LAND (class 76) on its OWN endpoint (serial1).  The rule that makes two writers safe
# without any runtime arbitration is BY MESSAGE CLASS (`_stage/refactor-bridge-plan.md` section 4.2), and the
# bench that proved two endpoints do not interfere is `tools/test-second-endpoint.py`.
#
# WHAT THIS PROCESS DELIBERATELY DOES NOT DO: it reads NO truth (`/sim/truth/odom` is the supervisor's), it
# makes NO decision about the mission, and it sends NO class-76 command.  It reports, and the supervisor
# decides (design section 8 item 4).

STATUS_TOPIC_DEFAULT = "/bridge/status"
TRIGGER_TOPIC_DEFAULT = "/plan/trigger"


class BridgeProcess:
    """The bridge's own process: external nav in, the plan's setpoints out, and its own health on a topic."""

    def __init__(self, args):
        self.args = args
        self.t_wall0 = time.monotonic()
        self.lio = {"t_us": 0, "p": np.zeros(3), "q": np.array([1.0, 0.0, 0.0, 0.0]),
                    "v": np.zeros(3), "n": 0, "rate": 0.0, "rate_slow": 0.0, "last_wall": 0.0,
                    "prev_p": None, "v_source": "diff", "v_seen": 0}
        self.nav_sent = 0
        #: the re-ask loop for the FCU's own streams (see `tick()`): without it a request sent before the EKF has
        #: an origin is dropped and the FCU never sends LOCAL_POSITION_NED at all
        self._reask_t = 0.0
        self._reask_n = 0
        self.nav_hz_measured = 0.0
        self._nav_t0 = None
        self._nav_n = 0
        self.calib = None
        self.node = None
        self.relay = None
        self.status_pub = None
        self.sp_log = None
        #: THE WIRE RECORD: one row per SETPOINT THE BRIDGE PUT ON MESSAGE 84, with the L input beside the
        #: W/wire output and the mask that went with it.  WHY IT EXISTS: the layer between "the plan box
        #: published a clean command" and "the FCU was told something" is where the `pos`/`posvel`/
        #: `posvelacc` choice lives, and without this file "the stream is clean but the bridge sent
        #: something else" stays invisible.
        self.wire_log = None
        self.next_nav = None
        self.last_status = 0.0

    def log(self, msg):
        print("[%7.2f] %s" % (time.monotonic() - self.t_wall0, msg), flush=True)

    # ---- the frame this process converts with -------------------------------------------------------
    def load_calib(self):
        """L -> W, loaded ONCE from the file the plan box also publishes with.

        WHY THE FILE AND NOT A MEASUREMENT.  Until F2 the mission MEASURED the translation from a parked
        window at preflight and used it for both the estimator feed and the commands.  F2 puts the
        estimator feed here and the measurement in the supervisor, and passing a calibration between
        processes would be a new interface whose only purpose is to carry a ~3 mm/12 mm correction
        (measured: +0.003 +0.002 -0.012 m on `logs/missionA-RECT21-tight-ekf/fsm.log:9-11`).  Loading the
        SAME FILE the plan box uses makes the round trip L -> W -> wire exact, and the supervisor still
        MEASURES the parked alignment and PRINTS the mismatch as a check.
        """
        from common import frames
        if self.args.calib_file and os.path.exists(self.args.calib_file):
            d = json.load(open(self.args.calib_file))
            self.calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float),
                                      t_WL=np.array(d["t_WL"], dtype=float))
            self.log("bridge/process: L -> W from %s: %s" % (self.args.calib_file, self.calib.summary()))
        else:
            self.calib = frames.Calib.predicted(spawn_heading_deg=0.0, world_axes="body")
            self.log("bridge/process: NO --calib-file: using the PREDICTED calibration (%s).  The plan box "
                     "must use the same one or every commanded point is offset by the difference."
                     % self.calib.summary())

    # ---- ROS ---------------------------------------------------------------------------------------
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
        self.node = Node("bridge_node")
        best_effort = QoSProfile(depth=200, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Odometry, self.args.lio_topic, self.on_lio, best_effort)
        self.node.create_subscription(Odometry, self.args.lio_vel_topic, self.on_lio_vel, best_effort)
        self.node.create_subscription(PoseStamped, self.args.trigger_topic, self.on_trigger, latched)
        self.status_pub = self.node.create_publisher(String, self.args.status_topic, 10)
        # the FCU's own estimate on a topic (see `tick()`); empty topic = off, which is what every sortie up to
        # round 78 used
        self._Odometry = Odometry
        self._fcu_odom_n = 0
        self._fcu_odom_last_us = 0
        self.fcu_odom_pub = None
        if self.args.fcu_odom_topic:
            self.fcu_odom_pub = self.node.create_publisher(Odometry, self.args.fcu_odom_topic, 10)
            self.log("bridge: the FCU's OWN estimate is published on %s (MAVLink LOCAL_POSITION_NED, W frame) -- "
                     "a DIFFERENT stream from the SITL truth tap" % self.args.fcu_odom_topic)
        self.relay = CmdRelay(self.bridge, self.calib, rate_hz=self.args.wire_hz,
                              fields=self.args.setpoint_mode, yaw_rad=self.args.yaw,
                              log=self.log, silence_abort_s=self.args.silence_abort_s,
                              topic=self.args.cmd_topic)
        self.relay.attach(self.node)
        self.rclpy = rclpy

    # ---- callbacks ---------------------------------------------------------------------------------
    def on_lio(self, m):
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
        if self.lio["v_source"] == "diff":
            if self.lio["prev_p"] is not None and dt > 1e-3:
                self.lio["v"] = 0.8 * self.lio["v"] + 0.2 * ((p - self.lio["prev_p"]) / dt)
            self.lio["prev_p"] = p.copy()

    def on_lio_vel(self, m):
        v = np.array([m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
        if float(np.linalg.norm(v)) > 1e-6:
            self.lio["v"] = v
            self.lio["v_source"] = "lio/odom"
            self.lio["v_seen"] += 1

    def on_trigger(self, m):                                            # noqa: ARG002
        """THE GO.  The supervisor publishes it (latched, once) when the vehicle is airborne and holding;
        until it arrives this process sends NO setpoint, which is what keeps position setpoints away from
        GUIDED's take-off SubMode (MEASURED: a position setpoint cancels a takeoff)."""
        self.relay.enable("the supervisor's %s" % self.args.trigger_topic)

    # ---- the loop ----------------------------------------------------------------------------------
    def tick(self, now):
        """One wire tick: the estimator feed at --nav-hz, then the relay's setpoint at --wire-hz."""
        # ASK AGAIN UNTIL THE FCU ANSWERS.  `MAV_CMD_SET_MESSAGE_INTERVAL` sent once, before the FCU's EKF has
        # an origin, is a request the firmware drops -- `mavlink_impl.py:86-92` records the measurement ("the
        # second channel received LOCAL_POSITION_NED at 50.2 Hz once it asked AFTER the first channel's preflight
        # had run") and calls the request a METHOD the consumer must verify and repeat.  MEASURED 2026-10-16 on
        # the parked Route B bench: without a re-ask the bridge received **no #32 at all** -- `st.t_us` stayed 0,
        # `/fcu/odom` published nothing, and ego printed "no odom." for the whole run while its subscription was
        # correctly wired.  This is the re-ask, bounded so a dead link cannot spam.
        if self.bridge is not None and not self.bridge.state().last_position_us \
                and (now - self._reask_t) > 2.0 and self._reask_n < 30:
            self._reask_t = now
            self._reask_n += 1
            self.bridge.request_rates()
            self.log("bridge: no LOCAL_POSITION_NED yet -- re-asking for the FCU's streams (%d)" % self._reask_n)
        if self.next_nav is None:
            self.next_nav = now
        if now >= self.next_nav:
            self.next_nav = now + 1.0 / self.args.nav_hz
            if self.lio["n"]:
                self.bridge.send_external_nav_l(self.lio["t_us"], self.lio["p"], self.lio["q"],
                                                self.lio["v"], self.calib)
                self.nav_sent += 1
                if self._nav_t0 is None:
                    self._nav_t0, self._nav_n = now, 0
                self._nav_n += 1
                if now - self._nav_t0 > 1.0:
                    self.nav_hz_measured = self._nav_n / (now - self._nav_t0)
        sent = self.relay.tick(now)
        # THE FCU'S OWN ESTIMATE, AS A ROS TOPIC.  `LocalState.pos/vel` is what the flight controller
        # BELIEVES -- parsed from MAVLink `LOCAL_POSITION_NED` (#32, requested at `--nav-hz`-ish rate in
        # `mavlink_impl.py:93` and already used by the FSM) -- and it is a DIFFERENT stream from the SITL truth
        # tap (`/sim/truth/odom`), which is the SIMULATOR's model state (`ardupilot/libraries/SITL/
        # SIM_Aircraft.cpp:708-716`) and does not exist on a real aircraft.  A planner that is fed the estimate
        # plans inside the same world the controller controls in, WITH the EKF's innovation gates in between;
        # that is the reason this topic exists (the owner's call, 2026-10-16).  It is published in W (ENU, z up),
        # the same convention as the tap, so ONE converter (`planner/fcu_pose_feed.py`) turns either into L.
        if self.fcu_odom_pub is not None and self.bridge is not None:
            st = self.bridge.state()
            # ONCE PER NEW SAMPLE, NOT ONCE PER TICK.  The first version of this block published the latest
            # state on every tick, and the tick runs at ~900 Hz (`tick()` is called from the node's own loop):
            # measured `ros2 topic hz /fcu/odom` = **800-918 Hz** for a stream whose source is MAVLink #32 at
            # ~48 Hz, i.e. 19 copies of every sample.  That is not harmless: ego's own odometry subscription is
            # `create_subscription<Odometry>("odom_world", 1, ...)` -- HISTORY DEPTH 1 -- and a 900 Hz flood of
            # duplicates against a depth-1 reader is a reader that can starve (measured: the FSM printed
            # "no odom." for a whole parked run while this publisher was at 900 Hz).  `st.t_us` advances once per
            # received #32, so keying the publication on it makes the topic carry exactly what the FCU said.
            if st.t_us and st.t_us != self._fcu_odom_last_us:
                self._fcu_odom_last_us = st.t_us
                if st.last_position_us and (int(now * 1e6) - st.last_position_us) < 1_000_000:
                    m = self._Odometry()
                    m.header.stamp = self.node.get_clock().now().to_msg()
                    m.header.frame_id = "world_w"
                    m.child_frame_id = "fcu_ekf"
                    m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(st.pos[0]),
                                                                                            float(st.pos[1]),
                                                                                            float(st.pos[2]))
                    m.pose.pose.orientation.w = 1.0
                    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = (float(st.vel[0]),
                                                                                              float(st.vel[1]),
                                                                                              float(st.vel[2]))
                    self.fcu_odom_pub.publish(m)
                    self._fcu_odom_n += 1
        if sent is not None and self.wire_log is not None:
            p_f, yaw_out, cmd = sent
            from bridge.mavlink_impl import (POS_YAW_MASK, POS_VEL_YAW_MASK,
                                             POS_VEL_ACC_YAW_MASK)
            mask = {"pos": POS_YAW_MASK, "posvel": POS_VEL_YAW_MASK,
                    "posvelacc": POS_VEL_ACC_YAW_MASK}[self.args.setpoint_mode]
            self.wire_log.write("%.4f,%s,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
                                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n"
                                % (now, self.args.setpoint_mode,
                                   mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                                   cmd.p[0], cmd.p[1], cmd.p[2],
                                   cmd.v[0], cmd.v[1], cmd.v[2],
                                   cmd.a[0], cmd.a[1], cmd.a[2],
                                   cmd.yaw, cmd.yaw_dot,
                                   -float(p_f[2]), p_f[0], p_f[1], p_f[2], yaw_out,
                                   cmd.trajectory_id, cmd.trajectory_flag))
            self.wire_log.flush()
        if sent is not None and self.sp_log is not None:
            p_f, yaw_out, cmd = sent
            st = self.bridge.state()
            self.sp_log.write(
                "%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"
                "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n" % (
                    now, self.lio["t_us"], p_f[0], p_f[1], p_f[2],
                    cmd.v[0], cmd.v[1], cmd.v[2], cmd.a[0], cmd.a[1], cmd.a[2],
                    st.pos[0], st.pos[1], st.pos[2], st.vel[0], st.vel[1], st.vel[2],
                    self.lio["p"][0], self.lio["p"][1], self.lio["p"][2], yaw_out))
            self.sp_log.flush()

    def status_line(self):
        st = self.bridge.state()
        return ("nav_hz=%.1f nav_age=%.2f nav_sent=%d lio_hz=%.1f relay=%d sent=%d mode=%s armed=%d "
                "plan_rx=%d plan_ids=%d plan_age=%.2f plan_flag=%s refused=%d"
                % (self.nav_hz_measured,
                   0.0 if not self.lio["n"] else time.monotonic() - self.lio["last_wall"],
                   self.nav_sent, self.lio.get("rate_slow", 0.0), 1 if self.relay.enabled else 0,
                   self.relay.n_sent, st.mode, 1 if st.armed else 0, self.relay.n_rx,
                   len(self.relay.ids_seen), self.relay.age_s(), self.relay.last_flag,
                   self.relay.n_refused))

    def check_own_links(self):
        """CONDITION 1 OF THE F2 GATE: VERIFY the streams this process depends on, do not assume them.

        MEASURED on the bench (`tools/test-second-endpoint.py`, and the three-point table in
        `docs/metrics/f2-second-endpoint-20261016.txt`): the FCU can ACCEPT a 511 rate request, READ IT BACK
        as exactly what was asked (50000 us) and then send NOTHING at all -- its intervals are served from a
        fixed set of buckets (`GCS_Common.cpp:1734-1770`), so a rate between buckets is silently not the
        rate you asked for.  "I requested it" is therefore not evidence of anything.  This method waits for
        the LIO input this process feeds the EKF with, and REFUSES to continue below the floor.
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.args.lio_wait_s:
            if self.lio["n"] > 20:
                break
            # IT MUST SPIN THE NODE WHILE IT WAITS.  MEASURED: without this the subscription's callback
            # never fires, `lio["n"]` stays 0 for the full timeout, the process exits 3 and the FCU gets NO
            # external nav -- which then makes the SUPERVISOR's own position stream 0 Hz (Copter sends no
            # LOCAL_POSITION_NED without a position) and the sortie aborts in preflight.  That chain ran
            # twice, in `logs/missionA-RECT25-kill-plan`.
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        if self.lio["n"] <= 20:
            self.log("REFUSING TO STREAM: no LIO odometry on %s after %.0f s -- the EKF would be fed "
                     "nothing and the plan's setpoints would have no vehicle behind them"
                     % (self.args.lio_topic, self.args.lio_wait_s))
            return False
        # ... and the RATE, measured over a second rather than declared
        t1 = time.monotonic()
        n0 = self.lio["n"]
        while time.monotonic() - t1 < 1.0:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
        hz = (self.lio["n"] - n0) / (time.monotonic() - t1)
        self.log("bridge/process: %s at %.1f Hz MEASURED (floor %.1f Hz), %s for the velocity"
                 % (self.args.lio_topic, hz, self.args.lio_floor_hz, self.lio["v_source"]))
        if hz < self.args.lio_floor_hz:
            self.log("REFUSING TO STREAM: the LIO input is %.1f Hz, below the %.1f Hz floor"
                     % (hz, self.args.lio_floor_hz))
            return False
        return True

    def run(self) -> int:
        from bridge.mavlink_impl import MavlinkBridge
        from std_msgs.msg import String                         # noqa: F401  (published in the loop)
        self.load_calib()
        # THE MAVLINK LINK FIRST, THEN ROS: `start_ros` builds the relay, and the relay needs the bridge.
        # MEASURED the hard way: the other order died with "'BridgeProcess' object has no attribute
        # 'bridge'" two seconds into a sortie -- the second startup failure of this process, and both were
        # process-boundary mistakes rather than transport ones.
        self.bridge = MavlinkBridge(url=self.args.mav, log=self.log)
        self.start_ros()
        if self.args.setpoint_log:
            self.sp_log = open(self.args.setpoint_log, "w")
            self.sp_log.write("t_wall,t_sim_us,cmd_x,cmd_y,cmd_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,"
                              "fcu_x,fcu_y,fcu_z,fcu_vx,fcu_vy,fcu_vz,lio_x,lio_y,lio_z,yaw_cmd\n")
        if self.args.wire_log:
            self.wire_log = open(self.args.wire_log, "w")
            # `frame_id` = the MAVLink frame the setpoint goes out in (1 = LOCAL_NED, the only one
            # `send_cmd` uses); `z_ned` = the z field as the FCU receives it (NED: UP IS NEGATIVE).
            # Without these, "the bridge wrote (0.5, 0.4, 2.0)" cannot be told apart from "it wrote a
            # RELATIVE offset of 2 m" or "the FCU read z as a velocity" -- which is the open question
            # about RECT25 (docs/metrics/f2-supervisor-20261016.txt).
            self.wire_log.write("t_wall,fields,frame_id,type_mask,p_l_x,p_l_y,p_l_z,v_l_x,v_l_y,"
                                "v_l_z,a_l_x,a_l_y,a_l_z,yaw_l,yaw_dot_l,z_ned,p_w_x,p_w_y,p_w_z,"
                                "yaw_out,trajectory_id,trajectory_flag\n")
        self.log("bridge/process: ready.  It owns the estimator feed (331) and the plan's setpoints (84) "
                 "on %s; it sends NO class-76 command -- arm/takeoff/mode/LAND are the supervisor's, on its "
                 "own endpoint." % self.args.mav)
        if not self.check_own_links():
            self.bridge.close()
            return 3
        stop = {"flag": False}

        def on_term(signum, frame):                                     # noqa: ARG001
            stop["flag"] = True

        import signal as _signal
        _signal.signal(_signal.SIGTERM, on_term)
        _signal.signal(_signal.SIGINT, on_term)
        period = 1.0 / self.args.wire_hz
        while self.rclpy.ok() and not stop["flag"]:
            self.bridge.poll()
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            now = time.monotonic()
            # ONE-WAY OVERRIDE, LOCALLY ENFORCED: if the FCU is no longer in GUIDED the setpoints this
            # process streams are inert, so it stops sending them.  The supervisor's mode change is the
            # authority; this is the transport refusing to talk into a mode that cannot hear it.
            st = self.bridge.state()
            if self.relay.enabled and st.armed and st.mode not in ("GUIDED", ""):
                self.relay.disable("the FCU left GUIDED (%s): the plan is off the wire" % st.mode)
            if self.relay.completed and self.relay.enabled:
                self.relay.disable("the plan declared the tour COMPLETE")
            self.tick(now)
            if now - self.last_status > 0.2:
                self.last_status = now
                if self.status_pub is not None:
                    msg = String()
                    msg.data = self.status_line()
                    self.status_pub.publish(msg)
            time.sleep(0.0005)
        self.log("bridge/process: stopping.  %s" % self.status_line())
        self.log("bridge/process: final: %d setpoints, %d external-nav messages"
                 % (self.bridge.counters()["positions"] if self.bridge else -1, self.nav_sent))
        if self.sp_log is not None:
            self.sp_log.close()
        if self.wire_log is not None:
            self.wire_log.close()
        try:
            self.bridge.close()
        except Exception:
            pass
        return 0


def bridge_main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="the bridge as its own process (F2)")
    ap.add_argument("--mav", default="tcp:127.0.0.1:5760", help="THIS process's endpoint (serial0)")
    ap.add_argument("--cmd-topic", default="/plan/cmd")
    ap.add_argument("--trigger-topic", default=TRIGGER_TOPIC_DEFAULT)
    ap.add_argument("--status-topic", default=STATUS_TOPIC_DEFAULT)
    ap.add_argument("--lio-topic", default="/lio/robo/odom")
    ap.add_argument("--lio-vel-topic", default="/lio/odom")
    ap.add_argument("--lio-wait-s", type=float, default=30.0)
    ap.add_argument("--lio-floor-hz", type=float, default=20.0,
                    help="REFUSE to stream below this measured input rate (condition 1 of the F2 gate)")
    ap.add_argument("--calib-file", default="")
    ap.add_argument("--wire-hz", type=float, default=50.0)
    ap.add_argument("--nav-hz", type=float, default=48.0)
    ap.add_argument("--fields", dest="setpoint_mode", default="pos", choices=["pos", "posvel", "posvelacc"])
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--silence-abort-s", type=float, default=3.0)
    ap.add_argument("--fcu-odom-topic", default="",
                    help="publish the FCU'S OWN estimate (MAVLink LOCAL_POSITION_NED #32, W frame) on this "
                         "topic; empty = off.  NOT the SITL truth tap: that one is the simulator's model state")
    ap.add_argument("--setpoint-log", default="")
    ap.add_argument("--wire-log", default="",
                    help="one row per setpoint PUT ON MESSAGE 84: the L input beside the W/wire "
                         "output and the field mask.  Without it, \"the plan published a clean "
                         "command but the bridge sent something else\" is invisible")
    args = ap.parse_args(argv)
    return BridgeProcess(args).run()


if __name__ == "__main__":
    sys.exit(bridge_main())
