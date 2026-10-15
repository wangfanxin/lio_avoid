#!/usr/bin/env python3
"""The PLAN box's executor and its command stream: `quadrotor_msgs/msg/PositionCommand` on `/plan/cmd`, in L.

WHAT CHANGED IN F1 (2026-10-16), AND WHY THE DWELL RULE CHANGED WITH IT
----------------------------------------------------------------------
Until F1 this file held `TourPacer` -- the producer's leg/dwell state machine, lifted out of
`planner/rectangle_source.py` unchanged -- and `CmdStream` published a SHADOW stream beside `nav_msgs/Path`.
The Path was what flew: the FSM consumed it, walked the corners and owned the arrival gate, the settle and
the dwell (`fsm/mission_fsm.py::_fly_to_corner` / `_await_next_leg`, deleted in F1).

F1 moves that ownership into this file, because the PLAN box is the box that gets REPLACED (amendment A1 of
`_stage/refactor-bridge-plan.md`): its output is a command stream, not a Path that someone else executes.
So `TourPacer` is no longer "which leg of the published polyline is in hand" but **which waypoint the plan
box is flying to**, and it decides arrival, settle, dwell and completion itself:

  * the plan box's own arrival gate -- ESTIMATE-ONLY.  `step()` reads the vehicle through the caller's
    `here_w()` (one odometry sample, L -> W with the same calibration the targets were converted with) and
    nothing else.  `/sim/truth/odom` is not imported, subscribed or named here, so the arrival decision
    cannot contain the truth tap -- which is what the old gate did
    (`fsm/mission_fsm.py` ANDed `h_truth < 0.06` into it) and what a dropped-in ego could not do
    (`ego_replan_fsm.cpp:103-104`: its only vehicle input is its own odometry).
  * **THE DWELL CLOCK STARTS ON SETTLED, NOT ON THE FIRST RADIUS CROSSING.**  MEASURED TWICE, and it is
    the defect change 8 of the F1 brief exists for: with the old rule the aircraft was declared ARRIVED at
    0.055-0.073 m while still ringing from a 1.7 m/s arrival, the ring carried it OUTSIDE the 0.15 m
    judging radius and back, and one visit split in two -- RECT11 c3 4.96 s and RECT21 c3 4.80 s, both
    missing the 5.0 s bar by a hair (`docs/metrics/rect-hold5-ratio-20261015.txt`, STATUS rows 48-49).
    `arrive_m` (inside) AND `settle_speed_mps` (slow) are now both required, BOTH are printed at arrival,
    and the settle has the same bounded cap the FSM's own settle loop had (`settle_s`, after which the
    arrival is declared anyway so a noisy estimate cannot stall the mission).
  * the tour's end is a FIELD: `trajectory_flag = TRAJECTORY_STATUS_COMPLETED`, published AFTER the last
    dwell -- not an EMPTY Path, and not a guess (`PositionCommand.msg:12-18`).

THE WAYPOINT LIST, AND WHY IT IS FIVE TARGETS AND NOT FOUR LEGS.  The pre-F1 tour produced FIVE `leg`
phases and the metric window depends on exactly that: `tools/metrics.py:171-183` selects the route window
as every phase whose name starts with `leg`, and `:426-435` anchors B1's window at the **leg2** entry.  The
FSM's corner walk got five from four published legs because it visited BOTH ends of each leg and dropped
the one it was standing on (`_legs_left`).  The plan box now commands waypoints directly, so the same tour
is the five targets `[c1, c2, c3, c4, c1]`: the first is the ENTRY (pad -> corner 1, no dwell -- the tour's
own dwells belong to the corners of the tour), and the remaining four are the square's corners, each with
one dwell.  That is the same route, in the same order, with the same five boundaries
(`_stage/refactor-bridge-plan.md` risk R1: renaming a phase or changing the number of leg phases silently
redefines A1-A3/B1-B3/B4/C1).

WHAT THIS MODULE DELIBERATELY DOES NOT DO
-----------------------------------------
* **No truth** (see above).
* **No MAVLink, no modes, no LAND, no RTL, no arm.**  This is the PLAN box; it DECLARES that it is
  finished and keeps publishing the hover.  Every mode change is the supervisor's (message class 76).
* **No ROS.**  Importable and testable with no ROS graph, which is what lets `tools/test-cmd-stream.py`
  prove the stream offline.

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
#: consumer calls it).  The bridge accepts the same frame by configuration under ego's name for it
#: (`world`) -- section 3.3: "the same frame under two names, a parameter, not a shim".
FRAME_L = "lio_world"

#: kx/kv ARE PUBLISHED AS ZERO.  `traj_server.cpp:12-13` initialises them to zero and the only thing that
#: ever writes them is a parameter the launch does not set; upstream's consumer adopts them only when they
#: are > 1e-5 (`so3_control_component.cpp:104-110`).  MAVLink message 84 has NO gain fields at all, so
#: nothing downstream can read them.  They are carried because the type has them and ego sets them, and
#: they are documented as IGNORED here so that a future reader does not go looking for the gain that steers
#: the vehicle -- the FCU's own PSC loop is what steers it.
KX = (0.0, 0.0, 0.0)
KV = (0.0, 0.0, 0.0)

#: THE YAW FIELD'S CONVENTION, stated because the type does not state it.  `yaw` is the HEADING THE FCU
#: READS: 0 = North, +90 deg = East, i.e. the LOCAL_NED yaw field's own sense, which is also `--yaw`'s and
#: `atan2(East, North)`'s.  It is NOT measured from L's +x axis; the adopted type's `header.frame_id`
#: therefore covers position/velocity/acceleration and not this one field.  Recorded rather than converted
#: because the bridge's pass-through of `yaw` is what the `atol = 0.0` proof
#: (`tools/test-s4-l-to-wire.py`) covers, and because the flown configuration overrides it
#: (`--yaw-mode fixed`, every mission-A sortie streamed `--yaw 0.0`).  A planner that wants its own heading
#: on the wire sets it in this convention.
YAW_IS_FCU_HEADING = True


class CmdSample:
    """One `PositionCommand`'s worth of numbers, in L.  Frames of the fields, explicitly:

      position     L, metres
      velocity     L, m/s
      acceleration L, m/s^2
      yaw          the FCU's heading sense (see YAW_IS_FCU_HEADING above), radians
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
    """The tour's pacing: which WAYPOINT is in hand, whether the vehicle has ARRIVED AND SETTLED, the dwell.

    This is the plan box's execution, and the caller owns the clock, the logging sink and the odometry --
    this class owns only the decision, which is why it is testable offline.

    THE SIX NUMBERS THAT DECIDE EVERYTHING, with where each comes from:

      arrive_m          0.06 m   the arrival tolerance EVERY flight in `docs/metrics/` was scored against
                                 (B3's own 100 mm bar is 40 of 40 corners <= 68 mm at this gate;
                                 `planner/rectangle_source.py`'s old 0.10 m producer gate existed only so
                                 that the producer could not lead the consumer into the next leg before the
                                 consumer had settled -- with one owner there is one number, and it is the
                                 one the evidence was measured at)
      settle_speed_mps  0.08 m/s the FSM's own settle threshold, moved here with the settle
      settle_s          3.0 s    and its bound: after this the arrival is declared anyway
      hold_s            5.0 s    the dwell, from `RECT_HOLD_S` (the owner's requirement)
      entry_hold_s      0.0 s    the entry leg's dwell: the tour's dwells belong to the tour's corners
      leg_timeout_s     20.0 s   a NAMED advance-anyway so a missed arrival cannot hang the mission

    and one more that is new here because the plan box owns the clock from ITS start-up (about 25 s
    before take-off): the leg clocks do not arm until the vehicle has LEFT THE PAD (`_arm`).
    """

    def __init__(self, targets_w, hold_s, arrive_m, settle_speed_mps, settle_s, leg_timeout_s, now,
                 log=None, seg_len_w=None, odom_label="", leg_speed=0.30, entry_hold_s=0.0,
                 arm_above_m=0.50, arm_move_m=0.50, arm_near_m=0.30, warn_period_s=15.0,
                 vertical_arrive_slack_m=0.20, holds=None):
        self.targets_w = np.asarray(targets_w, dtype=float)
        self.n_leg = int(len(self.targets_w))
        self.hold_s = float(hold_s)
        self.entry_hold_s = float(entry_hold_s)
        self.arrive_m = float(arrive_m)
        self.settle_speed_mps = float(settle_speed_mps)
        self.settle_s = float(settle_s)
        #: extra gate for a VERTICAL leg only -- see the arrival block in `step`.  0.20 m covers this stack's
        #: measured ~0.1 m altitude-datum term with room to spare, and it is applied to NO other leg.
        self.vertical_arrive_slack_m = float(vertical_arrive_slack_m)
        #: per-waypoint dwells, or None for "entry_hold_s at the entry, hold_s at every other waypoint"
        self.holds = None if holds is None else [float(h) for h in holds]
        self.leg_timeout_s = float(leg_timeout_s)
        self.arm_above_m = float(arm_above_m)
        self.arm_move_m = float(arm_move_m)
        self.arm_near_m = float(arm_near_m)
        self.warn_period_s = float(warn_period_s)
        self.log = log or (lambda s: None)
        self.seg_len_w = seg_len_w or (lambda i: float("nan"))
        # only used to make the log lines the SAME strings they were before this class existed
        self.odom_label = odom_label
        self.odom_n = 0
        self.odom_wall = now
        self.leg_speed = float(leg_speed)
        # state
        self.i_leg = 0                 # the index of the WAYPOINT being flown to (0-based)
        self.armed = False
        self.t_armed = None
        self.t_hold = None             # dwell start -- on the SETTLED arrival; None = not dwelling
        self.t_inside = None           # when the vehicle first came inside arrive_m without being slow
        self.done = False
        self.t_done = None
        self.tour_t0 = now
        self.t_warn = now
        #: which leg's arrival rule has already been printed (see the arrival block in `step`)
        self._leg_logged = -1
        # bookkeeping the log lines use
        self.arrival_residual_w = float("inf")
        self.settle_speed_seen = float("nan")
        self.p_first = None            # where the plan box first saw the vehicle (the pad)
        self.p_start_w = None          # the START of the leg in hand, for the yaw and the log

    # ---- what the output reads ----------------------------------------------------------------------
    @property
    def dwelling(self) -> bool:
        """True while the corner just reached is being held -- i.e. the command stream's HOVER."""
        return self.t_hold is not None

    @property
    def target_w(self):
        """The waypoint the plan box is commanding right now (the leg in hand's END)."""
        return np.asarray(self.targets_w[self.i_leg], dtype=float)

    def hold_for(self, i):
        """The dwell required after arriving at waypoint `i`.

        `holds`, when the caller gives one, is a per-waypoint dwell in the order of `targets_w`.  It exists
        for APPROACH SHAPING (2026-10-15): an inserted waypoint short of a corner must be STOPPED AT but not
        HELD (the tour's dwells belong to the tour's corners), and the entry leg keeps its own 0.0.  Without
        `holds` this is exactly the two-value rule it always was.
        """
        if self.holds is not None and 0 <= i < len(self.holds):
            return self.holds[i]
        return self.entry_hold_s if i == 0 else self.hold_s

    def leg_start_w(self):
        """Where the leg in hand started: the previous waypoint, or the pad for the entry leg."""
        if self.i_leg == 0:
            return self.p_first if self.p_first is not None else None
        return np.asarray(self.targets_w[self.i_leg - 1], dtype=float)

    def vertical_leg(self, tol=None):
        """True when the leg in hand has NO HORIZONTAL EXTENT -- its question is the HEIGHT.

        ONE RULE, TWO USES.  It decides which quantity the arrival is measured against (the height, not the
        horizontal distance) and it decides whether the leg may arm on `arm_near_m`.  Both are the same
        question about the same pair of points, so they are asked in one place; the tolerance is the same
        fraction of the arrival gate the arrival block documents (see the long note there).
        """
        start = self.leg_start_w()
        tgt = np.asarray(self.target_w, dtype=float)
        if start is None or tgt.shape[0] < 3:
            return False
        t = float(self.arrive_m / 10.0 if tol is None else tol)
        return (abs(float(tgt[0]) - float(start[0])) < t and
                abs(float(tgt[1]) - float(start[1])) < t)

    # ---- the decision -------------------------------------------------------------------------------
    def _arm(self, p_w, now):
        """Start the leg's flight clock -- but only once the vehicle has LEFT THE PAD.

        WHY THIS IS NOT JUST "the clock starts at start-up".  The plan box runs from before preflight
        (about 25 s of EKF origin, GUIDED, arming and take-off), so a timeout armed at start-up would fire
        on a healthy mission and advance the tour while the aircraft was still on the ground -- the exact
        shape of the corner-hold defect this project already paid for once.  Three conditions, any of which
        arms it, and each of them is a fact the plan box can see in its own odometry:

          * the vehicle is `arm_move_m` from where the plan box first saw it (it has taken off);
          * or its W altitude is above `arm_above_m` (the same fact, stated absolutely);
          * or it is already within `arm_near_m` of the waypoint (the original `leg_start_m` rule, which
            covers a plan box that starts after the aircraft is already flying).

        AND `arm_near_m` IS NOT ASKED OF A VERTICAL LEG.  MEASURED 2026-10-15 -- this is the third distinct
        defect the take-off's climb leg produced (`RECT31`, `RECT32`, `RECT33-accept`), and the only one that
        survived the arrival-rule fix:

          * the climb leg's target is the PAD, so "already within `arm_near_m` (0.30 m) of the waypoint" is
            true while the aircraft is still parked.  `RECT33-accept`'s own log arms it on the pad
            (`LEG 1 ... flight clock ARMED`, `leg_start_w=[-0.0026 -0.002 0.1128]`);
          * the clock then ran through the whole of pre-flight and take-off and the leg TIMED OUT 0.9 s after
            the relay opened: the first class-84 packet reached the FCU at sim t 30.301 s (`wire.csv` row 1,
            `p_w=(0,0,2.5)`), the target changed to waypoint 2 at 31.186 s, and the flight's own record says
            `it is 1.817 m from the commanded waypoint 1 after 20.0 s` -- 2.50 - 1.817 = 0.683 m, the height
            the aircraft had reached in those 0.9 s;
          * the two remaining rules are facts about having LEFT THE PAD, which is what the clock waits for:
            `arm_move_m` (0.50 m of motion since the plan box first saw it) and `arm_above_m` (0.50 m of
            absolute W altitude, which covers a plan box that starts while the aircraft is already flying).
        """
        if p_w is None:
            return False
        if not self.vertical_leg() and \
                float(np.linalg.norm(p_w[:2] - self.target_w[:2])) <= self.arm_near_m:
            return True
        if float(p_w[2]) > self.arm_above_m:
            return True
        if self.p_first is not None and float(np.linalg.norm(p_w - self.p_first)) > self.arm_move_m:
            return True
        return False

    def _advance(self, now):
        if self.i_leg >= self.n_leg - 1:
            self.done, self.t_done = True, now
            self.log("TOUR COMPLETE: waypoint %d held for %.1f s (total %.1f s of tour).  The command "
                     "stream now carries trajectory_flag = COMPLETED and the hover is kept: the plan box "
                     "does not stop talking, it STOPS ASKING FOR MORE."
                     % (self.i_leg, now - (self.t_hold if self.t_hold is not None else now),
                        now - self.tour_t0))
            return
        self.i_leg += 1
        self.armed, self.t_armed, self.t_hold, self.t_inside = False, None, None, None
        self.p_start_w = None
        self.log("LEG %d (waypoint %d, %.3f m from waypoint %d): the dwell is over, the next waypoint is "
                 "commanded" % (self.i_leg + 1, self.i_leg + 1, self.seg_len_w(self.i_leg), self.i_leg))

    def step(self, p_w, v_h, now):
        """One pacing tick.  `p_w` is the vehicle in W from the caller's OWN odometry, or None; `v_h` its
        horizontal speed in m/s, or None when the caller has no velocity source.

        Returns nothing: the caller reads `dwelling` / `done` / `i_leg` afterwards.  The log lines are the
        record of which leg armed when and of WHAT THE ARRIVAL MEASURED, because a flight's log is evidence.
        """
        if self.done:
            return
        if self.p_first is None and p_w is not None:
            self.p_first = np.asarray(p_w, dtype=float).copy()
        target_w = self.target_w[:2]
        if not self.armed:
            if self._arm(p_w, now):
                self.armed, self.t_armed = True, now
                self.p_start_w = None if p_w is None else np.asarray(p_w, dtype=float).copy()
                start = self.leg_start_w()
                d_start = float("inf") if start is None else float(np.linalg.norm(p_w[:2] - start[:2]))
                self.log("LEG %d (waypoint %d -> %d ): the vehicle is %.3f m from the leg's start "
                         "-- flight clock ARMED (arrival gate %.3f m, settle %.3f m/s within %.1f s, "
                         "timeout %.1f s, predicted %.1f s at %.2f m/s)"
                         % (self.i_leg + 1, self.i_leg, self.i_leg + 1, d_start, self.arrive_m,
                            self.settle_speed_mps, self.settle_s, self.leg_timeout_s,
                            self.seg_len_w(self.i_leg) / self.leg_speed, self.leg_speed))
            elif now - self.t_warn > self.warn_period_s:
                self.t_warn = now
                self.log("LEG %d has not started: the vehicle has not left the pad (altitude %.3f m, "
                         "needs > %.2f m or %.2f m of motion) -- %d pose samples on %s"
                         % (self.i_leg + 1, 0.0 if p_w is None else float(p_w[2]), self.arm_above_m,
                            self.arm_move_m, self.odom_n, self.odom_label))
            return
        if self.t_hold is not None:
            # IN THE DWELL, and its clock started on the SETTLED arrival.  The stream says "position =
            # the waypoint, velocity = 0"; the next waypoint is commanded only when the dwell is over.
            if now - self.t_hold >= self.hold_for(self.i_leg):
                self._advance(now)
            return
        # ARMED: waiting for the arrival.  THE ONLY INPUT IS p_w AND v_h -- the plan box's own estimate.
        #
        # THE ARRIVAL TEST IS HORIZONTAL, EXCEPT ON A LEG THAT HAS NO HORIZONTAL EXTENT, WHICH IS TESTED
        # VERTICALLY.  That exception is not a tweak, it is what makes the VERTICAL FIRST TARGET possible at
        # all, and it was MEASURED -- by flying it wrong twice:
        #   * `RECT31-accept` (2026-10-15) used the horizontal test on the climb leg and declared ARRIVAL
        #     while the aircraft was pinned over the pad at the ~0.2 m the take-off command asked for
        #     ("ARRIVED at waypoint 1 ... 0.004 m", with the next log line reporting altitude 0.111 m), so the
        #     climb never appeared in the command stream and corner 1's 2.0 m became the first setpoint;
        #   * `RECT32-accept` used the full 3-D distance and the leg TIMED OUT at 20 s -- it never arrived --
        #     because the aircraft was 1.831 m away with a DAlt of 2.152 m: the horizontal term contributed
        #     centimetres of drift that a climb straight up from the pad is not asked to remove, and no gate
        #     worth setting would absorb them.
        # A vertical leg's question is "has the CLIMB finished", so its test is the ALTITUDE ERROR ALONE.
        # Every leg with horizontal extent keeps the horizontal test unchanged, which is what B3's corner
        # arrivals and the arrival gate every flight in `docs/metrics/` was scored against depend on.
        #
        # THE ENDPOINTS ARE READ FROM `self.targets_w` AND NOT FROM `target_w`.  `target_w` a few lines above
        # is `self.target_w[:2]` -- the 2-D target the horizontal test uses -- so comparing against IT tests
        # "is the target's X and Y zero", which is TRUE of the pad-anchored vertical leg AND of nothing else,
        # and is a different question from "does this leg have horizontal extent".  That mistake was made once
        # here and caught by the diagnostic below, on a replay of the flown odometry: the rule looked right,
        # the printed line said `target_w=[0. 0.]`, and the branch taken was still the 2-D one.
        #
        # AND THE EXTENT TEST IS A TOLERANCE, NOT AN EQUALITY.  "No horizontal extent" cannot be `== 0` on
        # this stack: the pad is not at the plan's origin in the vehicle's own frame -- the measured
        # calibration puts it at (-0.9, -4.2) mm, and the LIO's own estimate moves by millimetres while the
        # aircraft stands still.  An exact-zero test classified the vertical leg as horizontal BY 0.9 mm,
        # which is how the first version of this fix failed on the replay.  The bound is a fraction of the
        # arrival gate: a leg whose endpoints are within `arrive_m / 10` horizontally cannot be the leg that
        # carries the arrival, so it is the vertical one.  For the 0.06 m gate that is 6 mm.
        if p_w is None:
            d = float("inf")
        else:
            start_w = self.leg_start_w()
            tgt = np.asarray(self.target_w, dtype=float)
            vertical_leg = self.vertical_leg()
            # A CLIMB LEG'S GATE IS NOT A CORNER'S GATE.  `arrive_m` (0.06 m) is the number B3's corner
            # arrivals were scored against; on a CLIMB the quantity that is flown and measured is the
            # HEIGHT, and this stack's altitude datum carries a measured ~0.1 m term (the L->W calibration's
            # t_z is +0.1156 m; A4's own datum line reads D -0.098 m in flight), so a 0.06 m gate on a 2.5 m
            # climb would ask for the datum to be perfect.  A vertical leg therefore gets
            # `vertical_arrive_slack_m` on top of the gate.  No other leg's gate changes.
            gate = self.arrive_m + (self.vertical_arrive_slack_m if vertical_leg else 0.0)
            if vertical_leg:
                d = abs(float(p_w[2]) - float(tgt[2]))            # the climb's height error
            else:
                d = float(np.linalg.norm(p_w[:2] - tgt[:2]))      # the horizontal error, as before
            # THE DECISION IS PRINTED, ONCE PER LEG.  This line exists because the rule below was BELIEVED
            # to be in force and `RECT31-accept`'s own log said otherwise; a replay of that flight's recorded
            # odometry through this module is what settled it, and `RECT32-accept` then measured the second
            # version's own failure.  A reader should not have to infer which test a leg used.
            if self._leg_logged != self.i_leg:
                self._leg_logged = self.i_leg
                self.log("LEG %d arrival rule: TESTING %s; horizontal error %.4f m, 3-D error %.4f m, "
                         "height error %.4f m, gate %.3f m; leg_start_w=%s target_w=%s p_w=%s"
                         % (self.i_leg + 1,
                            "the HEIGHT (this leg has no horizontal extent)" if vertical_leg
                            else "the HORIZONTAL distance (this leg has horizontal extent)",
                            float(np.linalg.norm(p_w[:2] - tgt[:2])),
                            float(np.linalg.norm(np.asarray(p_w, dtype=float) - tgt)),
                            abs(float(p_w[2]) - float(tgt[2])), gate,
                            None if start_w is None else np.round(np.asarray(start_w, float), 4),
                            np.round(tgt, 4), np.round(np.asarray(p_w, float), 4)))
        self.arrival_residual_w = d
        self.settle_speed_seen = v_h
        if d <= gate and v_h is not None and v_h < self.settle_speed_mps:
            self._arrive(d, now, capped=False)
            return
        if d <= gate:
            if self.t_inside is None:
                self.t_inside = now
            elif now - self.t_inside >= self.settle_s:
                self._arrive(d, now, capped=True)
            return
        self.t_inside = None
        if now - self.t_armed > self.leg_timeout_s:
            self.log("LEG %d TIMEOUT: THE VEHICLE NEVER ARRIVED -- it is %s from the commanded waypoint "
                     "%d after %.1f s of flight (gate %.3f m), %d pose samples on %s, the newest %.1f s "
                     "old.  Advancing to the next waypoint so a missed arrival cannot hang the mission."
                     % (self.i_leg + 1, "not known" if p_w is None else "%.3f m" % d, self.i_leg + 1,
                        now - self.t_armed, self.arrive_m, self.odom_n, self.odom_label,
                        now - self.odom_wall))
            self._advance(now)

    def _arrive(self, d, now, capped):
        """The arrival is declared: inside `arrive_m` and (settled, or the settle cap expired)."""
        self.t_inside = None
        hold = self.hold_for(self.i_leg)
        flight_s = "n/a" if self.t_armed is None else "%.1f" % (now - self.t_armed)
        self.log("ARRIVED at waypoint %d: the vehicle is %.3f m from it (gate %.3f m) after %s s of "
                 "flight; SETTLED to %.3f m/s (threshold %.3f m/s%s) -- %s"
                 % (self.i_leg + 1, d, self.arrive_m, flight_s,
                    0.0 if self.settle_speed_seen is None else self.settle_speed_seen,
                    self.settle_speed_mps,
                    ", settle cap %.1f s expired" % self.settle_s if capped else "",
                    "the %.1f s dwell starts NOW, on the settled arrival, not on the first radius "
                    "crossing" % hold if hold > 0 else
                    "this is the ENTRY leg, so there is no dwell and the next waypoint is commanded at "
                    "once"))
        if hold <= 0.0:
            self._advance(now)
            return
        self.t_hold = now


class CmdStream:
    """The command stream's CONTENT: the waypoint in hand, at the declared rate.

    SELF-CLOCKED, and that is a property the test asserts rather than a convention: `advance(now)` returns
    the wall time of the NEXT sample, so the rate is a number this object owns and not a number the caller
    happens to sleep for.  `tools/test-cmd-stream.py` drives it with a clock it controls and counts the
    samples per simulated second.

    THE RATE.  Ego's `traj_server` runs at **100 Hz** (`traj_server.cpp:248-250`, a 10 ms
    `create_wall_timer`); section 3.3 prices the interface as "publisher <= 100 Hz; consumed at 50 Hz --
    the wire rate stays 50 Hz", and that is what F1 ships: this object's own 100 Hz clock in the plan box,
    and `bridge/bridge_node.py`'s 50 Hz relay on the way to the FCU.

    THE FRAME: the positions it emits are built FROM THE L WAYPOINTS, so what it says is in L is in L by
    construction and not by a second computation.

    AFTER COMPLETION IT KEEPS PUBLISHING.  `traj_server.cpp:191-202` keeps a hover at the end of a
    trajectory and the F2 supervisor needs the difference between "the plan box has STOPPED TALKING"
    (dead -- abort) and "the plan box says COMPLETED" (finished -- land).  A stream that goes silent when
    it is finished cannot make that distinction, so this one does not.
    """

    def __init__(self, pacer, rate_hz, targets_l, trajectory_id=1, log=None, frame_id=FRAME_L):
        self.pacer = pacer
        self.rate_hz = float(rate_hz)
        self.period = 1.0 / self.rate_hz
        self.targets_l = [np.asarray(t, dtype=float) for t in targets_l]
        self.trajectory_id = int(trajectory_id)
        self.frame_id = frame_id
        self.log = log or (lambda s: None)
        self.n = 0
        self.next_t = None
        self.last = None

    # ---- the content -------------------------------------------------------------------------------
    def leg_yaw(self):
        """The heading of the leg in hand, in the FCU's own sense (0 = North, +90 deg = East).

        Computed from the leg's L start and L target, then expressed as a heading: the adopted type's
        `yaw` field is what LOCAL_NED reads, and this module states that convention at the top rather
        than leaving it to be inferred from a sign.
        """
        tgt = self.targets_l[self.pacer.i_leg]
        start = None
        if self.pacer.i_leg > 0:
            start = self.targets_l[self.pacer.i_leg - 1]
        elif self.pacer.p_start_w is not None:
            # the entry leg: from where the plan box saw the vehicle start, put into L with the same
            # calibration the targets were -- the pacer stores it in W, and does not own the calibration
            start = getattr(self.pacer, "p_start_l", None)
        if start is None:
            return 0.0
        d = np.asarray(tgt, dtype=float) - np.asarray(start, dtype=float)
        if float(np.hypot(d[0], d[1])) < 1e-9:
            return 0.0
        # L: x = East, y = North (the same axes as W, PLAN.md decision 7/14), so the heading the FCU reads
        # is atan2(East, North).
        return float(math.atan2(d[0], d[1]))

    def sample(self):
        """The sample the stream should carry RIGHT NOW, given the pacer's state.

        Two cases, and they are the whole difference between a Path and a command stream:

          dwell / tour complete   position = the waypoint reached, velocity = 0, acceleration = 0
          leg in hand             position = the waypoint being flown to

        `velocity` and `acceleration` are zero because a geometric source has no time parameterisation: the
        producer knows WHERE it wants the vehicle, not how fast it may go or how hard it may accelerate --
        and MEASURED 2026-10-14 (sortie G2) an acceleration field of zero in the type mask is the one thing
        that BROKE holding, which is why `--setpoint-mode` exists and why `send_cmd` defaults to the
        position-only mask the corner tour has always flown.
        """
        i = min(self.pacer.i_leg, len(self.targets_l) - 1)
        p = self.targets_l[i]
        flag = TRAJECTORY_STATUS_COMPLETED if self.pacer.done else TRAJECTORY_STATUS_READY
        s = CmdSample(p, np.zeros(3), np.zeros(3), self.leg_yaw(), 0.0,
                      self.trajectory_id + i, flag, self.frame_id)
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
        return ("/plan/cmd: %d samples at %.1f Hz declared, last flag=%d (%s), frame %s, %d waypoints"
                % (self.n, self.rate_hz, self.last.trajectory_flag if self.last else -1,
                   "COMPLETED" if (self.last and self.last.completed) else "not completed",
                   self.frame_id, len(self.targets_l)))
