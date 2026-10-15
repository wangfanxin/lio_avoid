#!/usr/bin/env python3
"""The PLAN box's command stream: `quadrotor_msgs/msg/PositionCommand` on `/plan/cmd`, in L.

WHY THIS FILE EXISTS (and why the leg/dwell state machine MOVED here rather than being duplicated).
Refactor-bridge-plan.md §3.3 fixes the PLAN box's output as a COMMAND STREAM, not a `nav_msgs/Path`:
it is the interface ego's own `traj_server` already speaks (`traj_server.cpp:244-250`: publisher
`/position_cmd`, `quadrotor_msgs/msg/PositionCommand`, a 10 ms timer, `header.frame_id = "world"`), and
adopting it verbatim is what deletes the adapter the previous ego integration needed
(`planner/ego_adapter.py`, "Why an adapter at all: the FSM must not change").

That makes the DWELL content rather than a convention: at the end of a leg the stream says
`position = corner, velocity = 0`, exactly as `traj_server.cpp:191-202` does at the end of a trajectory.
And the end of the tour becomes a FIELD -- `trajectory_flag = TRAJECTORY_STATUS_COMPLETED` -- instead of
the empty-Path convention (`traj_server.cpp` and `PositionCommand.msg:12-18` already define it).

The legs, the arrival gate and the dwell are the producer's own state machine, which worked
(`planner/rectangle_source.py`: RECT7/RECT9/RECT10 measured `4 of 4 corners hovered >= 5.0 s` from
`tap.csv`).  So this module does not re-implement it: `TourPacer` IS that state machine, lifted out of
`main()` unchanged so that ONE instance paces ONE tour and both outputs read it -- the Path publisher
(`publish(msgs[pacer.leg])`) and the command stream.  Two copies of a pacing rule is how the producer and
the consumer disagreed about arrival in the first place (§2(b) item 2: the gate exists twice, 0.10 m and
0.06 m, on purpose).

WHAT THIS MODULE DELIBERATELY DOES NOT DO
-----------------------------------------
* **No truth.**  `TourPacer.step()` reads the vehicle ONLY through the caller's `here_w()` -- one odometry
  sample, L -> W with the same calibration the legs were published with.  `/sim/truth/odom` is not
  imported, subscribed or named here, so the arrival decision cannot contain the truth tap.  That is the
  point of §2(b)/R2: a dropped-in ego can only see its own odometry (`ego_replan_fsm.cpp:103-104`).
* **No MAVLink, no modes, no LAND, no RTL.**  This is the PLAN box; it DECLARES that it is finished and
  stops.  Every mode change is the supervisor's (amendment 1 of this round: the split is by message class).
* **No ROS.**  This module is importable and testable with no ROS graph at all, which is what lets
  `tools/test-cmd-stream.py` prove the stream offline.

usage: imported by `planner/rectangle_source.py` (which owns the ROS node and the calibration file)
"""
from __future__ import annotations

import math

import numpy as np

# --- the trajectory_flag constants, spelled as the vendored message defines them ---------------------
# `ros_ws/src/quadrotor_msgs/msg/PositionCommand.msg:12-18`, itself a byte-identical copy of ego's.  The
# ABORT constant's upstream spelling is "TRAJECTROY_STATUS_ABORT" (a typo in the original) and is kept
# here for the same reason the .msg keeps it: the name has to be the one ego's code compiles against.
TRAJECTORY_STATUS_EMPTY = 0
TRAJECTORY_STATUS_READY = 1
TRAJECTORY_STATUS_COMPLETED = 3
TRAJECTROY_STATUS_ABORT = 4

#: The name of the frame the stream is in.  L is the LIO's world (`lio_world` is what every existing
#: consumer calls it: `tools/record-traj.py:18`, `fsm/mission_fsm.py`'s latched subscription).  The
#: bridge accepts the same frame by configuration under ego's name for it (`world`) -- §3.3: "the same
#: frame under two names, a parameter, not a shim".
FRAME_L = "lio_world"

#: kx/kv ARE PUBLISHED AS ZERO.  `traj_server.cpp:12-13` initialises them to zero and the only thing that
#: ever writes them is a parameter the launch does not set; upstream's consumer adopts them only when
#: they are > 1e-5 (`so3_control_component.cpp:104-110`).  MAVLink message 84 has NO gain fields at all
#: (`bridge/mavlink_impl.py:57`, `POS_VEL_ACC_YAW_MASK`), so nothing downstream can read them.  They are
#: carried because the type has them and ego sets them, and they are documented as IGNORED here so that
#: a future reader does not go looking for the gain that steers the vehicle -- the FCU's own PSC loop is
#: what steers it.
KX = (0.0, 0.0, 0.0)
KV = (0.0, 0.0, 0.0)


class CmdSample:
    """One `PositionCommand`'s worth of numbers, in L.  Frames of the fields, explicitly:

      position     L, metres
      velocity     L, m/s
      acceleration L, m/s^2
      yaw          L's +z, radians
      yaw_dot      rad/s

    `frame_id` rides along because the bridge has to be told which of the two names for L the producer
    used, and guessing it is the class of defect this refactor is removing.
    """

    __slots__ = ("p", "v", "a", "yaw", "yaw_dot", "kx", "kv", "trajectory_id", "trajectory_flag",
                 "frame_id")

    def __init__(self, p, v, a, yaw, yaw_dot, trajectory_id, trajectory_flag, frame_id=FRAME_L,
                 kx=KX, kv=KV):
        self.p = np.asarray(p, dtype=float)
        self.v = np.asarray(v, dtype=float)
        self.a = np.asarray(a, dtype=float)
        self.yaw = float(yaw)
        self.yaw_dot = float(yaw_dot)
        self.kx = tuple(float(x) for x in kx)
        self.kv = tuple(float(x) for x in kv)
        self.trajectory_id = int(trajectory_id)
        self.trajectory_flag = int(trajectory_flag)
        self.frame_id = frame_id

    @property
    def completed(self) -> bool:
        return self.trajectory_flag == TRAJECTORY_STATUS_COMPLETED

    def __repr__(self):
        return ("CmdSample(p=[%+.4f %+.4f %+.4f] v=[%+.4f %+.4f %+.4f] a=[%+.4f %+.4f %+.4f] "
                "yaw=%+.4f yaw_dot=%+.4f id=%d flag=%d)"
                % (*self.p, *self.v, *self.a, self.yaw, self.yaw_dot, self.trajectory_id,
                   self.trajectory_flag))


class TourPacer:
    """The corner tour's pacing: which leg is in hand, whether the vehicle has arrived, and the dwell.

    Lifted verbatim out of `planner/rectangle_source.py::main()` (the `while rclpy.ok()` / hold-ON branch)
    so that the decision exists ONCE.  The caller owns the clock, the logging sink and the odometry; this
    class owns only the decision, which is why it is testable offline.

    Signatures of the six measurements that decide everything, all of them from the flown configuration
    (`tools/run-mission.sh`, and `docs/metrics/` was scored against these numbers):

      arrive_m      0.10 m   the producer's own arrival gate -- ABOVE the worst measured corner residual
                             (71.0 mm on RECT1-noego) so that "arrived" here cannot lead the consumer
                             into the next leg before the consumer has settled (`:107-113`)
      leg_start_m   0.30 m   the leg's flight clock arms within this of the leg's START corner, because
                             leg 1 starts 0.64 m from the pad and preflight takes ~25 s (`:114-118`)
      leg_timeout_s 20.0 s   a NAMED advance-anyway so a missed arrival cannot hang the mission (`:119-124`)
      hold_s        5.0 s    the dwell, from `RECT_HOLD_S` / the world file's `mission.hold_s` (`:139`)
    """

    def __init__(self, corners_w, n_leg, hold_s, arrive_m, leg_start_m, leg_timeout_s, now,
                 log=None, leg_len_w=None, odom_label="", empty_mark_s=2.0, leg_speed=0.30):
        self.corners_w = np.asarray(corners_w, dtype=float)
        self.n_leg = int(n_leg)
        self.hold_s = float(hold_s)
        self.arrive_m = float(arrive_m)
        self.leg_start_m = float(leg_start_m)
        self.leg_timeout_s = float(leg_timeout_s)
        self.log = log or (lambda s: None)
        self.leg_len_w = leg_len_w or (lambda i: float("nan"))
        # only used to make the log lines the SAME strings they were before this class existed
        self.odom_label = odom_label
        self.odom_n = 0
        self.odom_wall = now
        self.empty_mark_s = float(empty_mark_s)
        self.leg_speed = float(leg_speed)
        # state -- identical to the producer's, and with its names
        self.i_leg = 0
        self.armed = False
        self.t_armed = None
        self.t_hold = None          # dwell start; None = not dwelling
        self.done = False
        self.t_done = None          # when the tour was declared complete
        self.tour_t0 = now
        self.t_warn = now
        # bookkeeping the log lines used, kept so the lines stay byte-identical
        self.arrival_residual_w = float("inf")

    # ---- what the two outputs read ------------------------------------------------------------------
    @property
    def dwelling(self) -> bool:
        """True while the corner just reached is being held -- i.e. the command stream's HOVER."""
        return self.t_hold is not None

    @property
    def corner_w(self):
        """The corner the vehicle is standing on while it dwells (the end of the leg in hand)."""
        return np.asarray(self.corners_w[(self.i_leg + 1) % len(self.corners_w)], dtype=float)

    def leg_start_w(self):
        return np.asarray(self.corners_w[self.i_leg][:2], dtype=float)

    def leg_end_w(self):
        return np.asarray(self.corners_w[(self.i_leg + 1) % len(self.corners_w)][:2], dtype=float)

    # ---- the decision -------------------------------------------------------------------------------
    def step(self, p_w, now):
        """One pacing tick.  `p_w` is the vehicle in W from the caller's OWN odometry, or None.

        Returns nothing: the caller reads `dwelling` / `done` / `i_leg` afterwards.  The log lines are
        the SAME strings the producer printed before this class existed, because those lines are the
        record of which leg armed when, and a flight's log is evidence.
        """
        if self.done:
            return
        start_w = self.corners_w[self.i_leg][:2]
        end_w = self.corners_w[(self.i_leg + 1) % len(self.corners_w)][:2]
        if self.t_hold is not None:
            # IN THE DWELL.  The stream says "position = corner, velocity = 0"; the next leg is published
            # only when the dwell is over.  Publishing it earlier would end the dwell, because the
            # consumer advances as soon as a new corner is in hand (`:358-361`).
            if now - self.t_hold >= self.hold_s:
                if self.i_leg >= self.n_leg - 1:
                    self.done, self.t_done = True, now
                    self.log("TOUR COMPLETE: leg %d held corner %d for %.1f s (total %.1f s of tour).  "
                             "Publishing an EMPTY path for %.1f s -- the consumer ends the tour on that "
                             "statement, not on a timeout."
                             % (self.i_leg, (self.i_leg + 1) % len(self.corners_w), now - self.t_hold,
                                now - self.tour_t0, self.empty_mark_s))
                else:
                    self.i_leg += 1
                    self.armed, self.t_armed, self.t_hold = False, None, None
                    self.log("LEG %d (corner %d -> %d, %.3f m) published after the %.1f s hold"
                             % (self.i_leg, self.i_leg, (self.i_leg + 1) % len(self.corners_w),
                                self.leg_len_w(self.i_leg), self.hold_s))
            return
        if not self.armed:
            d_start = float("inf") if p_w is None else float(np.linalg.norm(p_w[:2] - start_w))
            if d_start <= self.leg_start_m:
                self.armed, self.t_armed = True, now
                self.log("LEG %d (corner %d -> %d, %.3f m): the vehicle is %.3f m from the leg's start "
                         "-- flight clock ARMED (arrival gate %.3f m, timeout %.1f s, predicted %.1f s "
                         "at %.2f m/s)"
                         % (self.i_leg, self.i_leg, (self.i_leg + 1) % len(self.corners_w),
                            self.leg_len_w(self.i_leg), d_start, self.arrive_m, self.leg_timeout_s,
                            self.leg_len_w(self.i_leg) / self.leg_speed, self.leg_speed))
            elif now - self.t_warn > 15.0:
                self.t_warn = now
                self.log("LEG %d has not started: the vehicle is %s from the leg's start corner %d "
                         "(needs <= %.2f m); %d pose samples on %s%s"
                         % (self.i_leg, "not known" if p_w is None else "%.3f m" % d_start, self.i_leg,
                            self.leg_start_m, self.odom_n, self.odom_label,
                            "" if self.odom_n else "  <-- NOTHING ON THAT TOPIC: the arrival cannot be "
                                                   "measured, and the FSM's own 25 s corner gate is what "
                                                   "will end this"))
            return
        # ARMED: waiting for the arrival.  THE ONLY INPUT IS p_w -- one odometry sample.
        d = float("inf") if p_w is None else float(np.linalg.norm(p_w[:2] - end_w))
        self.arrival_residual_w = d
        if d <= self.arrive_m:
            self.t_hold = now
            self.log("ARRIVED at corner %d: the vehicle is %.3f m from it (gate %.3f m) after "
                     "%.1f s of flight; holding %.1f s, then the next leg is published"
                     % ((self.i_leg + 1) % len(self.corners_w), d, self.arrive_m, now - self.t_armed,
                        self.hold_s))
        elif now - self.t_armed > self.leg_timeout_s:
            self.log("LEG %d TIMEOUT: THE VEHICLE NEVER ARRIVED -- it is %s from the commanded "
                     "corner %d after %.1f s of flight (gate %.3f m), %d pose samples on %s, the "
                     "newest %.1f s old.  Advancing to the next leg so a missed arrival cannot "
                     "hang the mission; the FSM's own corner gate will abort if it disagrees."
                     % (self.i_leg, "not known" if p_w is None else "%.3f m" % d,
                        (self.i_leg + 1) % len(self.corners_w), now - self.t_armed, self.arrive_m,
                        self.odom_n, self.odom_label, now - self.odom_wall))
            if self.i_leg >= self.n_leg - 1:
                self.done, self.t_done = True, now
            else:
                self.i_leg += 1
                self.armed, self.t_armed, self.t_hold = False, None, None


class CmdStream:
    """Publishes the command stream's CONTENT: one `CmdSample` per leg, at the declared rate.

    SELF-CLOCKED, and that is a property the test asserts rather than a convention: `advance(now)`
    returns the wall time of the NEXT sample, so the rate is a number this object owns and not a number
    the caller happens to sleep for.  `tools/test-cmd-stream.py` drives it with a clock it controls and
    counts the samples per simulated second.

    THE RATE.  Ego's `traj_server` runs at **100 Hz** (`traj_server.cpp:248-250`, a 10 ms
    `create_wall_timer`).  Section 3.3 prices the interface as "publisher <= 100 Hz; consumed at 50 Hz --
    the wire rate stays 50 Hz".  This round implements the PUBLISHER side only, so 100 Hz is the default
    (`--cmd-rate`); nothing consumes it yet (`bridge/bridge_node.py` is F2's, and routing it to message
    84 at 50 Hz is F1's).  Raising the WIRE rate to 100 Hz is explicitly not this refactor's
    (refactor-bridge-plan.md section 8 item 10).

    THE FRAME, and the one thing that makes this class safe to hand to a consumer: the positions it emits
    are built FROM THE L LEGS, so what it says is in L is in L by construction and not by a second
    computation.  (`legs_l` is what the Path is published from; the W legs are the same geometry before
    the producer's W->L conversion.)
    """

    def __init__(self, pacer, rate_hz, legs_l, trajectory_id=1, log=None, frame_id=FRAME_L):
        self.pacer = pacer
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.legs_l = [np.asarray(leg, dtype=float) for leg in legs_l]
        # the corners the stream commands, in L: the START of each leg, in leg order.  A tour visits
        # corner 0, then the END of leg 0 == the START of leg 1, and so on -- which is why this list and
        # the pacer's own `corners_w` index the same way.
        self.corners_l = np.array([leg[0] for leg in self.legs_l], dtype=float)
        # per-leg yaw, in L, computed from the LEG's own tangent -- the same quantity the Path's
        # per-point orientation carries (`rectangle_source.py:221-225`).  A stream has no point list, so
        # it carries the leg's yaw as one number.
        self.leg_yaw_l = [self._leg_yaw(leg) for leg in self.legs_l]
        self.trajectory_id = int(trajectory_id)
        self.frame_id = frame_id
        self.log = log or (lambda s: None)
        self.n = 0
        self.next_t = None
        self.last = None
        self.first_flag = TRAJECTORY_STATUS_EMPTY

    @staticmethod
    def _leg_yaw(leg_l):
        """One leg's yaw in L: the direction at its START point.

        The Path carries a yaw PER POINT (`rectangle_source.py:221-225`, the tangent at the point,
        `atan2(dy, dx)`).  A command stream carries ONE yaw, so it has to be the leg's own direction --
        and for a straight leg the tangent at any interior point is the same number.  At the START point
        is the convention, because that is the heading the vehicle holds as it leaves the corner it was
        parked on, and it is what the Path says at the point the vehicle is standing on.
        """
        pts = np.asarray(leg_l, dtype=float)
        for i in range(len(pts) - 1):
            d = pts[i + 1] - pts[i]
            if float(np.hypot(d[0], d[1])) > 1e-9:
                return math.atan2(d[1], d[0])
        return float("nan")

    # ---- the content -------------------------------------------------------------------------------
    def sample(self):
        """The sample the stream should carry RIGHT NOW, given the pacer's state.

        Three cases, and they are the whole difference between a Path and a command stream:

          dwell                  position = the corner reached, velocity = 0, acceleration = 0
                                 yaw = the INCOMING leg's tangent, i.e. the heading it arrived on, held
          tour complete          the same, with `trajectory_flag = COMPLETED`
          leg in hand            position = the leg's END corner, yaw = THAT leg's tangent

        `acceleration` is zero because a geometric leg has no time parameterisation: the producer knows
        WHERE it wants the vehicle and in WHICH direction, not how hard it may accelerate.  Sending a
        made-up acceleration would be a claim the rectangle cannot support; the FCU's own PSC limits are
        what govern it.  (This is exactly the field `--setpoint-mode posvelacc` was A/B-ing in the FSM,
        and the honest value from a geometric source is zero, not a guess.)
        """
        i = self.pacer.i_leg
        if self.pacer.done or self.pacer.dwelling:
            # THE DWELL IS CONTENT.  `p = the corner the leg in hand ENDS on`, which is where the
            # vehicle is standing: `corners_l[(i + 1) % n]` is leg i's end corner, because leg i starts
            # at `corners_l[i]`.
            p = self.corners_l[(i + 1) % len(self.corners_l)]
            yaw = self.leg_yaw_l[i % len(self.leg_yaw_l)]
            flag = TRAJECTORY_STATUS_COMPLETED if self.pacer.done else TRAJECTORY_STATUS_READY
            s = CmdSample(p, np.zeros(3), np.zeros(3), yaw, 0.0, self.trajectory_id, flag,
                          self.frame_id)
        else:
            p = self.corners_l[(i + 1) % len(self.corners_l)]
            # BEFORE THE FIRST SAMPLE the flag is EMPTY, which is what the type documents for "no
            # trajectory yet" (`PositionCommand.msg:12`: `TRAJECTORY_STATUS_EMPTY = 0`, and the ID
            # comment at :20 explains why 0 is comparable).  A consumer that joins before the tour is
            # armed reads "there is nothing yet" rather than "here is a leg".
            flag = self.first_flag if self.n == 0 else TRAJECTORY_STATUS_READY
            s = CmdSample(p, np.zeros(3), np.zeros(3), self.leg_yaw_l[i % len(self.leg_yaw_l)], 0.0,
                          self.trajectory_id, flag, self.frame_id)
        self.last = s
        return s

    def advance(self, now):
        """Record one emitted sample and return the wall time of the next one."""
        s = self.sample()
        self.n += 1
        self.next_t = (now + self.period) if self.next_t is None else (self.next_t + self.period)
        return self.next_t

    def due(self, now):
        """True when a sample is due -- the loop's own test, so the CLOCK is this class's."""
        return self.next_t is None or now >= self.next_t

    # ---- the verdict, for the record ---------------------------------------------------------------
    def summary(self) -> str:
        return ("/plan/cmd: %d samples at %.1f Hz declared, last flag=%d (%s), frame %s, %d legs"
                % (self.n, self.rate_hz, self.last.trajectory_flag if self.last else -1,
                   "COMPLETED" if (self.last and self.last.completed) else "not completed",
                   self.frame_id, len(self.leg_yaw_l)))
