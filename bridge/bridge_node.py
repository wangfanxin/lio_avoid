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

import time

import numpy as np

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
