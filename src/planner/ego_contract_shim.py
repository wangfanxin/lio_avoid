#!/usr/bin/env python3
"""THE CONTRACT SHELL FOR ROUTE B: ego's OWN traj_server samples the plan; this only says WHO/WHERE/WHEN.

WHY THIS EXISTS AT ALL.  `traj_server` (ego's own, unmodified) turns `planning/bspline` into a 100 Hz
`quadrotor_msgs/PositionCommand` on `/position_cmd`, carrying **position AND velocity AND acceleration** -- the
reference ego actually wrote.  What it does NOT carry is our MISSION semantics, and it never will: it hard-codes
`trajectory_flag = READY` (`traj_server.cpp:211`) and forwards ego's per-replan `traj_id`, which changes about
once a second.  Two consumers need the other behaviour:

  * the SUPERVISOR ends the sortie on `trajectory_flag = COMPLETED` ("the plan box has declared the tour
    complete") and lands;
  * the acceptance report counts `trajectory_id` per WAYPOINT (`docs/PLANNERS.md`), which is what makes "which
    leg is this" answerable in a recording.

SO THIS SHELL DOES EXACTLY TWO THINGS, and nothing else:
  1. it OWNS THE TOUR -- sends one goal at a time on `/move_base_simple/goal` (in L, re-sent every 1 s until the
     aircraft arrives within `--arrive-m`, then a `--hold-s` dwell) and counts the waypoint index;
  2. it REWRITES THE CONTRACT on the way through -- copies `/position_cmd` to `/plan/cmd` verbatim, except
     `trajectory_id = waypoint index + 1` and `trajectory_flag = READY` while flying, `COMPLETED` once the last
     waypoint is reached and settled -- and it KEEPS TALKING after COMPLETED, because that rule is what let the
     supervisor tell "done" from "dead" in the first place.  **From COMPLETED on it also HOLDS**: the forwarded
     point is pinned to the last waypoint with zero velocity, zero acceleration and a latched yaw.  The contract
     asks the stream to keep TALKING, not to keep FLYING, and the sortie that showed the difference spent ~20 s of
     its landing window being thrown around by ego's own sweeping reference (round 81).

IT DOES NOT SAMPLE, FILTER, RATE-LIMIT OR PREDICT ANYTHING -- before COMPLETED.  If the numbers on the wire differ
from ego's own there, that is a bug in this file, not a design choice -- which is why every forwarded message is
also written to a CSV (`--trace-csv`), so the forwarding can be checked against `/position_cmd` rather than
trusted.  The post-COMPLETED hold is the ONE deliberate exception, it is a few lines, and the bench measures it:
after the flag turns COMPLETED the forwarded point is a single coordinate with |v| = 0 (`_stage/s12-rig.sh`).

THE POSE IT USES for arrival/dwell is the same state the planner is fed (`/ego/odom_fcu`, the FCU's own estimate
in L) -- the owner's call: the mission must not be sequenced on a different estimate from the one it plans on.

usage: ego_contract_shim.py --world worlds/diagonal.yaml --calib-file _stage/calib.json \
                            --odom-topic /ego/odom_fcu [--hold-s 5.0] [--arrive-m 0.30] [--trace-csv PATH]
       ego_contract_shim.py --selftest
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from common import frames                                                       # noqa: E402

FRAME_L = "lio_world"
DEFAULT_CMD_TOPIC = "/plan/cmd"
DEFAULT_IN_TOPIC = "/position_cmd"          # traj_server's own publisher (traj_server.cpp:244)
DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"
TRAJECTORY_STATUS_READY = 1                 # ego's own constants (traj_server.cpp:211)
TRAJECTORY_STATUS_COMPLETED = 3


class TourSequencer:
    """The mission's bookkeeping, with NO ROS IN IT, so `--selftest` can assert it with no graph and no flight.

    The rule for arrival is the adapter's, unchanged, because it is the rule the accepted sorties used: the
    vehicle must be within `arrive_m` AND slow (|v| < `settle_mps`), then it must STAY there for `hold_s` before
    the next waypoint is released.  `traj_server` has no opinion about any of this -- it flies whatever spline
    arrives, and when the spline runs out it hovers at its end (`traj_server.cpp:191-202`).
    """

    def __init__(self, targets_l, arrive_m=0.30, hold_s=5.0, settle_mps=0.08, goal_resend_s=1.0,
                 log=None, ego_driven=False, ref_clamp_m=0.0, leg_timeout_s=0.0, leg_abort_s=0.0,
                 arrive_z_m=1.00, start_alt_m=0.0, ref_z_clamp_m=0.0, ref_slew_mps=0.0,
                 corner_m=0.0, corner_mps=1.0, ref_z_floor_m=0.0, ref_acc_max=0.0,
                 corridor_m=0.0, corridor_end_m=0.0, corridor_open_m=2.0):
        self.targets = np.asarray(targets_l, dtype=float).reshape(-1, 3)
        self.arrive_m = float(arrive_m)
        self.hold_s = float(hold_s)
        self.settle_mps = float(settle_mps)
        self.goal_resend_s = float(goal_resend_s)
        self.log = log or (lambda s: None)
        #: PLAN X: when True the TOUR IS EGO'S OWN -- `wp_index` drives the index and the end, this shell
        #: sends no goals and runs no arrival gate of its own.  Default False = the flown behaviour.
        self.ego_driven = bool(ego_driven)
        self.i_goal = 0
        self.t_dwell = None
        self.done = False
        self.p = None
        self.v = None
        self.t_last_goal = None
        self.goals_sent = 0
        self.n_ready = 0
        self.n_completed = 0
        #: how many `/wp_index` messages were seen (PLAN X's own health signal)
        self.wp_index_msgs = 0
        #: the yaw latched when the tour completed, so the post-COMPLETED hold cannot command a spin
        self._hold_yaw = None
        #: how many forwarded messages were rewritten to the hold (round 81)
        self.n_held_after = 0
        #: HOW FAR THE REFERENCE MAY BE FROM THE VEHICLE (metres; 0 = off = the flown behaviour until 2026-10-16).
        #: This is the "continuity at the source" fix -- see `rewrite()`'s docstring for the measurements that
        #: justify it and for why this process is the only one that can apply it.
        self.ref_clamp_m = float(ref_clamp_m)
        self.n_clamped = 0
        self.t_last_clamp_log = None
        #: THE PER-LEG WATCHDOG (seconds; 0 = off).  Flight E spent 374.3 s frozen at waypoint 7 -- the FSM was
        #: stuck in GEN_NEW_TRAJ ("First 3 control points in obstacles": the odometry start inside the inflated
        #: map) while `traj_server` kept publishing a HELD point, so the stream looked alive and the supervisor's
        #: 3.0 s plan-silence rule never fired.  A frozen reference and a hover are the same bytes; the only
        #: thing that can tell them apart is the CLOCK, and this is where the mission's clock lives.
        self.leg_timeout_s = float(leg_timeout_s)
        self.leg_abort_s = float(leg_abort_s)
        self.t_leg_start = None
        self.t_last_leg_warn = None
        self.n_leg_warn = 0
        #: True once a leg has timed out and the stream has been stopped on purpose.
        self.abort = False
        #: the last point forwarded on `/plan/cmd`, so the watchdog can print WHERE the reference is stuck
        self.last_ref = (0.0, 0.0, 0.0)
        #: THE SLEW LIMITER'S OWN DECISION, recorded so the trace and the selftest can see it
        self.n_slewed = 0
        self.t_last_fwd = None
        self.last_fwd = None
        #: HOW FAR ABOVE THE CORNER THE VEHICLE MAY BE AND STILL COUNT AS ARRIVED (metres).  Flight F MEASURED
        #: why a purely spherical gate cannot work on a tour whose legs are 5-8 m of HORIZONTAL travel:
        #: `metrics.txt` scored "corner 1: closest horizontal approach 84.4 mm PASS, vertical +0.834 m" -- the
        #: aircraft stood 8 cm from the corner and 83 cm high, so the 3-D distance 0.838 m never fell under
        #: `arrive_m` and the tour could not advance at all.  The vertical is a tolerance, not a target.
        self.arrive_z_m = float(arrive_z_m)        #: START-ALTITUDE GATE for the TOUR: no goal is published until the vehicle's z reaches this (0 = off).
        #: This is PATCH 0008's `fsm/waypoint_start_alt` idea applied to Route B, where the SHELL owns the tour
        #: and ego's gate (which only arms `flight_type 2`) does not apply.  MEASURED Flight F: goals were sent
        #: from mission t=1.9 s, so ego spent the whole 19 s take-off replanning a climb that nothing was
        #: following -- 87 "First 3 control points in obstacles", 83 "A star error", and at the trigger the FCU
        #: inherited a plan already 19 s into its clock.
        self.start_alt_m = float(start_alt_m)
        self.t_first_goal = None
        #: AN UPPER CLAMP ON THE FORWARDED ALTITUDE, metres above the tour's own z (0 = off).  One-sided on
        #: purpose: the climb off the pad must be free to be BELOW the tour's altitude, but MEASURED in Flight F
        #: the reference ran to z = 3.25 m for a 1.884 m tour (the aircraft followed it to 3.32 m, 0.44 m under
        #: the supervisor's 3.00 m climb guard) and the flight never came back down.
        self.ref_z_clamp_m = float(ref_z_clamp_m)
        self.n_z_clamped = 0
        #: THE ALTITUDE FLOOR (see `rewrite`): never forward a z more than this far BELOW the tour's own, but
        #: only once the vehicle has been up at that altitude, so the climb off the pad is untouched.
        self.ref_z_floor_m = float(ref_z_floor_m)
        self.n_z_floored = 0
        self.z_floor_armed = False
        #: THE ACCELERATION CLAMP (see `rewrite`): the bound on the acceleration field the FCU is given.
        self.ref_acc_max = float(ref_acc_max)
        self.n_acc_clamped = 0
        #: True the first time a forwarded message asks for motion: see `_leg_watchdog`.
        self.motion_seen = False
        #: THE REFERENCE'S OWN SPEED LIMIT (m/s; 0 = off).  A position reference must be CONTINUOUS, and this
        #: one is not: `t_cur = now - start_time` restarts at every 1 Hz re-plan, so the point jumps (MEASURED
        #: Flight A: median 0.281 m, max 0.927 m).  `--ref-clamp-m` bounds how FAR the jump may land from the
        #: vehicle; this bounds how FAST the forwarded point may move, which is the part the flight controller
        #: actually feels -- MEASURED Flight H at 3 m/s: a 0.139 m median tracking error but p90 0.532 and max
        #: 0.840 m, i.e. the transients, and the aircraft's 0.10-0.20 m of real margin is gone exactly there.
        #: A legitimate 3 m/s reference advances 0.03 m per 100 Hz message; a re-plan step is 20x that.
        self.ref_slew_mps = float(ref_slew_mps)
        self.n_slewed = 0
        self.t_last_fwd = None
        self.last_fwd = None
        #: the 1 Hz "waiting for the first pose" note (see `goal_to_send`)
        self.t_last_no_odom = None
        #: THE CORNER GOVERNOR (see `rewrite`): within `corner_m` of the corner the reference may not move
        #: faster than `corner_mps`, and the velocity it carries is capped with it.
        self.corner_m = float(corner_m)
        self.corner_mps = float(corner_mps)
        self.n_corner_capped = 0
        #: the corner the tour has just LEFT: the governor holds through the turn (see `rewrite`)
        self.prev_target = None
        #: THE ROUTE CORRIDOR (metres; 0 = off = every sortie before 2026-10-17).  MEASURED why it exists
        #: (`_stage/leg-clearance.py` on `worlds/rect68.yaml`): the pillars are r = 0.30 on a 1.0 m staggered
        #: checkerboard, so all FOUR route edges are exactly +0.200 m from the nearest pillar surface over their
        #: whole interior (the corners have +0.407 m) -- the centrelines are the MAXIMUM-clearance paths of the
        #: field, and no offset from them leaves more.  The plan does not fly them: the AD top view shows plan and
        #: truth snaking about the line with +-0.4-0.5 m amplitude, and the east edge's cut-in to x = 2.67 at
        #: y = 2.5 is 0.13 m inside the (2.5,2.5) pillar.  So the reference is held inside this half-width of the
        #: CURRENT leg's centreline, which converts the error budget from "lateral snake" (which spends clearance)
        #: into "along-track lag" (which spends none).
        self.corridor_m = float(corridor_m)
        #: how far from the leg's ENDS the corridor opens back up (metres).  The corners are the safe places --
        #: +0.407 m against the edges' +0.200 -- so the turn is left free rather than clamped into a 90 degree
        #: kink; the clamp is full strength in the interior and ramps to `corridor_m + corridor_open_m` at the ends.
        self.corridor_end_m = float(corridor_end_m)
        self.corridor_open_m = float(corridor_open_m)
        self.n_corridor = 0
        self.n_corridor_v = 0
    # ---- inputs ---------------------------------------------------------------------------------------
    def on_odom(self, p_l, v_l=None, now=0.0):
        self.p = np.asarray(p_l, dtype=float).reshape(3)
        self.v = None if v_l is None else np.asarray(v_l, dtype=float).reshape(3)
        self._sequence(now)

    def _sequence(self, now: float) -> None:
        if self.ego_driven:
            return                      # PLAN X: ego's `wp_index` is the only thing that advances this tour
        if self.done or self.p is None:
            return
        if self._leg_watchdog(now):
            return
        # ARRIVAL IS HORIZONTAL, WITH A VERTICAL TOLERANCE -- see `arrive_z_m`.  The speed test is skipped
        # entirely when `settle_mps` is 0, which is what "fly THROUGH the waypoint" means: a 3 m/s aircraft
        # spends ~0.2 s inside a 0.35 m disc and a gate that also demands |v| < 0.30 m/s never fires at all
        # (Flight F: the aircraft passed 0.314 m from corner 1 at 1.6 m/s and the tour did not advance).
        d_xy = float(np.linalg.norm((self.p - self.target)[:2]))
        d_z_ok = abs(float(self.p[2] - self.target[2])) <= self.arrive_z_m
        speed = float(np.linalg.norm(self.v)) if self.v is not None else float("inf")
        slow_ok = (self.settle_mps <= 0.0) or (speed < self.settle_mps)
        if self.t_dwell is None:
            if d_xy <= self.arrive_m and d_z_ok and slow_ok:
                self.t_dwell = now
                self.log("shim: ARRIVED at waypoint %d of %d (%.3f m horizontal, %.3f m vertical, |v| %.2f "
                         "m/s), dwelling %.1f s"
                         % (self.i_goal + 1, len(self.targets), d_xy, float(self.p[2] - self.target[2]),
                            speed if np.isfinite(speed) else -1.0, self.hold_s))
            return
        if now - self.t_dwell >= self.hold_s:
            if self.i_goal >= len(self.targets) - 1:
                self.done = True
                self.log("shim: TOUR COMPLETE -- trajectory_flag becomes COMPLETED and the stream KEEPS TALKING")
                return
            self.i_goal += 1
            self.t_dwell = None
            self.t_leg_start = now
            self.t_last_leg_warn = None
            self.prev_target = self.targets[self.i_goal - 1].copy()
            self.log("shim: next waypoint %d of %d" % (self.i_goal + 1, len(self.targets)))

    def _leg_watchdog(self, now: float) -> bool:
        """SAY IT, AND THEN STOP TALKING.  Returns True when the leg has been given up on.

        The rule is the mission's own clock, not a guess about the planner: a leg that has not advanced in
        `leg_timeout_s` is announced once a second with the distance still to go, and at `leg_abort_s` this
        process STOPS PUBLISHING -- which is the one signal the supervisor already treats as fatal (its 3.0 s
        plan-silence rule).  The tour is NOT advanced: a leg that was not flown must never be scored as
        reached, and the missing `ARRIVED` line is the evidence.
        """
        if self.leg_timeout_s <= 0.0:
            return False
        # A LEG CLOCK ONLY MEANS SOMETHING ONCE THE PLAN BOX HAS COMMANDED MOTION AT ALL.  MEASURED Flight F:
        # the tour is released at mission t=1.9 s, the FCU is not allowed to see the plan until the supervisor's
        # trigger at t=20.4 s, and the watchdog counted those 19 s of take-off hold -- it aborted a healthy
        # flight at t=27.5 s while the aircraft was still on its first leg.  `motion_seen` is set by `rewrite()`
        # the first time a message asks for more than 0.20 m/s, i.e. when the flight has demonstrably begun.
        if not self.motion_seen:
            self.t_leg_start = now
            self.t_last_leg_warn = None
            return False
        if self.t_leg_start is None:
            self.t_leg_start = now
        held = now - self.t_leg_start
        if held < self.leg_timeout_s:
            return False
        if self.abort:
            return True
        if self.t_last_leg_warn is None or (now - self.t_last_leg_warn) >= 1.0:
            self.t_last_leg_warn = now
            self.n_leg_warn += 1
            d = float(np.linalg.norm(self.p - self.target)) if self.p is not None else float("nan")
            self.log("shim: LEG %d/%d HAS NOT ADVANCED FOR %.1f s -- the reference is at (%.3f, %.3f, %.3f), the "
                     "vehicle is %.3f m from the corner, dwelling=%s (this is a STALL, not a hover: see the "
                     "planner's PLANNER-STALL / EMERGENCY_STOP lines)"
                     % (self.i_goal + 1, len(self.targets), held,
                        self.last_ref[0], self.last_ref[1], self.last_ref[2],
                        d, "yes" if self.t_dwell is not None else "no"))
        if self.leg_abort_s > 0.0 and held >= self.leg_abort_s:
            self.abort = True
            self.log("shim: LEG %d/%d TIMEOUT after %.1f s -- STOPPING THE STREAM (no more messages are "
                     "published) so the supervisor's plan-silence rule ends the sortie.  The leg is NOT counted "
                     "as reached and the tour does NOT advance." % (self.i_goal + 1, len(self.targets), held))
            return True
        return False

    # ---- PLAN X: the tour driven by EGO's own sequencer -------------------------------------------------
    def on_wp_index(self, idx: int, now: float = 0.0) -> None:
        """`/wp_index` from PATCH 0011: the waypoint the mission is on, or -1 when the tour is finished.

        This is the whole of Plan X's shell: the id and the COMPLETED transition follow EGO, and nothing else
        in this file has an opinion about when the mission advances.  Note what is deliberately NOT done --
        the shell does not check an arrival of its own here, and does not require the vehicle to be anywhere:
        if ego says the tour is over, the contract says COMPLETED, and if ego's index is wrong that is a fact
        about ego which the trace should show rather than one this shell should hide.
        """
        self.wp_index_msgs += 1
        if int(idx) < 0:
            if not self.done:
                self.done = True
                self.i_goal = len(self.targets) - 1
                self.log("shim: TOUR COMPLETE reported BY EGO (wp_index -1 after %d wp_index message(s)) -- "
                         "trajectory_flag becomes COMPLETED and the stream KEEPS TALKING, held at waypoint %d"
                         % (self.wp_index_msgs, len(self.targets)))
            return
        new_i = max(0, min(int(idx), len(self.targets) - 1))
        if new_i != self.i_goal:
            self.log("shim: ego's wp_index advanced %d -> %d of %d: trajectory_id follows it"
                     % (self.i_goal + 1, new_i + 1, len(self.targets)))
            self.i_goal = new_i

    # ---- outputs --------------------------------------------------------------------------------------
    @property
    def target(self):
        return self.targets[min(self.i_goal, len(self.targets) - 1)]

    def goal_to_send(self, now: float):
        """The one goal ego should be working on, or None when the tour is over -- re-sent at `goal_resend_s`
        because ego's FSM falls back to WAIT_TARGET when it decides a plan has stalled, and only a NEW goal
        pulls it out (the same reason `planner/ego_source.py:480-493` re-sends).

        PLAN X: under `ego_driven` there is NOTHING to send -- ego's own sequencer owns the tour, and a goal
        sent to a FSM that is not subscribed to `/move_base_simple/goal` would be a message with no reader.
        The re-send cadence that used to drive the mission is exactly what Plan X removes.

        AND NOTHING IS SENT AFTER A TIMEOUT either: the watchdog's whole signal is the silence."""
        if self.ego_driven or self.done or self.abort:
            return None
        # THE FIRST GOAL WAITS FOR THE FIRST POSE.  MEASURED Flight K (2026-10-17): the shell published goal 1
        # about 0.2 s after it started, the FSM was still in INIT with no odometry, its own guard logged
        # "waypoint ignored: no odometry yet (REPLAN_TRAJ with no trajectory segfaults)", the state went
        # INIT -> REPLAN_TRAJ, and NOTHING ever planned again: 20 s later the supervisor's own "the plan box is
        # not talking" rule ended the sortie at t=40.7 s.  The vehicle state needed to sequence a tour is the
        # same state the shell already subscribes to, so the gate is free -- and it is NOT the same thing as
        # `start_alt_m` (Flight G), which withheld goals for 20 s and starved the one topic this box publishes.
        if self.p is None:
            if self.t_last_no_odom is None or (now - self.t_last_no_odom) >= 1.0:
                self.t_last_no_odom = now
                self.log("shim: no pose on the odometry topic yet -- goal %d/%d is being HELD (the FSM must not "
                         "be handed a target before it has odometry; Flight K lost a whole sortie to that race)"
                         % (self.i_goal + 1, len(self.targets)))
            return None
        # THE START-ALTITUDE GATE (see `start_alt_m`): nothing is sent while the vehicle is still on the pad,
        # so ego's first plan is made from an AIRBORNE state instead of a 19 s-old climb nobody was following.
        if self.start_alt_m > 0.0:
            if self.p is None or float(self.p[2]) < self.start_alt_m:
                return None
            if self.t_first_goal is None:
                self.t_first_goal = now
                self.log("shim: START-ALTITUDE GATE OPEN -- the vehicle is at z = %.3f m (gate %.2f m); the tour "
                         "starts NOW, so ego's first plan is made from the air" % (float(self.p[2]),
                                                                                   self.start_alt_m))
        if self.t_last_goal is not None and (now - self.t_last_goal) < self.goal_resend_s:
            return None
        self.t_last_goal = now
        self.goals_sent += 1
        return self.target

    def rewrite(self, msg, now: float):
        """Apply the CONTRACT to one forwarded message: id per waypoint, READY -> COMPLETED.  In place.

        `trajectory_id` is `i_goal + 1`, so it advances exactly when the MISSION advances and at no other time
        (the adapter's rule, `ego_source.py:195`).  Everything else in the message is ego's own and is not
        touched -- including `header.frame_id`, which stays what `traj_server` wrote (`world`); the bridge
        accepts both names for this frame (`bridge/bridge_node.py:54-57`).

        ONE EXCEPTION, AND IT IS MEASURED (round 81): **after the tour is complete the message is rewritten to a
        HOLD.**  The contract requires the stream to keep TALKING after COMPLETED, not to keep FLYING, and the
        sortie that showed the difference (`DIAG34-routeb-pv`) spent **~20 s of the landing window being thrown
        around**: `setpoints.csv` records the commanded point going to (-2.5,-1.2), then (+0.4,-1.25), then
        (+1.1,-2.0) and back at 1.4-2.1 m/s, with the command's implied speed between messages reaching 4.25 m/s
        at the 95th percentile and 14.5 m/s at the maximum (median 0).  That was ego's own age rule sweeping and
        snapping (`traj_server.cpp:171-172`) -- motion the mission was over.  So from the moment `done` is set,
        the forwarded position is pinned to the LAST WAYPOINT (the point the aircraft has just arrived at, within
        the arrival gate), the velocity and acceleration are zeroed, and the yaw is latched at whatever the first
        post-COMPLETED message carried, so nothing spins during the landing.  Before COMPLETED the forwarding is
        still verbatim.

        A SECOND EXCEPTION, ADDED 2026-10-16 AND ALSO MEASURED: **the reference is CLAMPED to `--ref-clamp-m`
        of the vehicle** (0 = off).  This is the "continuity at the source" fix, and this process is the only one
        that can do it: `traj_server` has NO odometry subscription at all (`traj_server.cpp:289-306`), and the
        FSM's check tests the plan.  MEASURED in `logs/missionA-FLIGHTA-routeb-hold`: `t_cur = now - start_time`
        restarts at every 1 Hz replan, so the sampled point JUMPS -- 14 of 16 replans step more than 0.05 m,
        median 0.281 m, max 0.927 m on the wire -- and at the moment the aircraft entered a pillar the reference
        was 0.623 m ahead of it.  A position reference is a place to go, not a schedule, so capping its distance
        from the vehicle costs the mission nothing (the vehicle moves, the cap follows, the reference advances)
        while removing the steps that the flight controller then has to chase.  `n_clamped` counts them, and
        every clamp is logged, so the fix is measurable rather than assumed.
        """
        msg.trajectory_id = int(self.i_goal) + 1
        self.last_ref = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
        if float(np.hypot(msg.velocity.x, msg.velocity.y)) > 0.20 or abs(float(msg.velocity.z)) > 0.20:
            self.motion_seen = True
        if self.done:
            if self._hold_yaw is None:
                self._hold_yaw = float(msg.yaw)          # latch once: a yaw jump mid-landing is a spin command
            p = self.targets[-1]
            msg.position.x, msg.position.y, msg.position.z = float(p[0]), float(p[1]), float(p[2])
            msg.velocity.x = msg.velocity.y = msg.velocity.z = 0.0
            msg.acceleration.x = msg.acceleration.y = msg.acceleration.z = 0.0
            msg.yaw, msg.yaw_dot = self._hold_yaw, 0.0
            msg.trajectory_flag = TRAJECTORY_STATUS_COMPLETED
            self.n_completed += 1
            self.n_held_after += 1
        else:
            if self.ref_clamp_m > 0.0 and self.p is not None:
                d = np.array([msg.position.x, msg.position.y, msg.position.z]) - self.p
                dist = float(np.linalg.norm(d))
                if dist > self.ref_clamp_m:
                    q = self.p + d * (self.ref_clamp_m / dist)
                    msg.position.x, msg.position.y, msg.position.z = float(q[0]), float(q[1]), float(q[2])
                    self.n_clamped += 1
                    if self.t_last_clamp_log is None or (now - self.t_last_clamp_log) >= 1.0:
                        self.t_last_clamp_log = now
                        self.log("shim: REFERENCE CLAMPED -- the plan asked for a point %.3f m from the vehicle; "
                                 "forwarded %.3f m along that direction instead (clamp %d, %.1f %% of the "
                                 "stream)" % (dist, self.ref_clamp_m, self.n_clamped,
                                              100.0 * self.n_clamped / max(1, self.n_clamped + self.n_ready)))
            msg.trajectory_flag = TRAJECTORY_STATUS_READY
            self.n_ready += 1
        # ---- THE ONE-SIDED ALTITUDE CLAMP, applied to BOTH branches (see `ref_z_clamp_m`) ----
        # It is deliberately AFTER the COMPLETED hold as well as before it: the hold pins z to the last
        # waypoint, which is already the tour's altitude, so it is a no-op there -- but keeping it in one place
        # means the forwarded z can never exceed the tour's z by more than the cap, whatever branch wrote it.
        if self.ref_z_clamp_m > 0.0:
            z_cap = float(self.targets[-1][2]) + self.ref_z_clamp_m
            if float(msg.position.z) > z_cap:
                msg.position.z = z_cap
                self.n_z_clamped += 1
        # ---- AND A FLOOR, which is NOT the same guard as the ceiling ----
        # MEASURED Flight O, leg 5 (`_stage/leg5-forensics.py`): the reference FROZE for four seconds at
        # (-2.037, 4.738, **z = 1.099**) -- 0.8 m below the tour's 1.884 m -- and the truth's worst sample of the
        # entire flight is at t = 66.4 s, clearance -0.143 m, at (-4.57, 1.36) with **altitude 0.18 m**: the
        # aircraft was dragged down to the ground and out into the outermost pillar column while the mission
        # carried on and completed.  A one-sided CEILING cannot stop that, and the 21.35 s leg is that event.
        # The floor is LATCHED on the vehicle having been up at the tour's altitude, so the climb off the pad --
        # which is legitimately far below it -- is never clipped.
        if self.ref_z_floor_m > 0.0 and self.p is not None:
            if float(self.p[2]) > float(self.targets[-1][2]) - 0.30:
                self.z_floor_armed = True
            if self.z_floor_armed:
                z_floor = float(self.targets[-1][2]) - self.ref_z_floor_m
                if float(msg.position.z) < z_floor:
                    msg.position.z = z_floor
                    self.n_z_floored += 1
        # ---- THE ACCELERATION CLAMP, added 2026-10-17 with the mask opened ----
        # Flight D's only `posvelacc` trial was refuted for the wrong reason -- its LIO had diverged 3.65 m -- but
        # the review's own diagnosis of it stands: the clamp that protects the POSITION left the ACCELERATION
        # field riding unclamped in the same message, so the sampler's 1 Hz spike went into a channel ArduPilot
        # adds to its velocity target with `limit_total=false` (unshaped).  That pre-condition -- a continuous
        # reference -- is met now (`--ref-clamp-m` and `--ref-slew-mps`), so the acceleration is put on the wire
        # with a bound of its own.  MEASURED reason to want it at all (Flight Z, `_stage/tracking-z.py`): the
        # tracking error is CROSS-TRACK -- ALONG the plan p50 0.081 m, ACROSS it p50 0.210 / p90 0.456 m -- at
        # matched speed, which is the signature of a missing lateral feed-forward, and the contacts are exactly
        # where that cross-track error is largest (gap p50 0.532 m with the reference itself +0.157 m clear).
        if self.ref_acc_max > 0.0:
            a = np.array([float(msg.acceleration.x), float(msg.acceleration.y), float(msg.acceleration.z)])
            an = float(np.linalg.norm(a))
            if an > self.ref_acc_max:
                a = a * (self.ref_acc_max / an)
                msg.acceleration.x, msg.acceleration.y, msg.acceleration.z = float(a[0]), float(a[1]), float(a[2])
                self.n_acc_clamped += 1
        # ---- THE ROUTE CORRIDOR, added 2026-10-17: hold the reference on the leg's centreline ----
        # WHY HERE, between the acceleration clamp and the speed limiter: it is a POSITION correction, so the slew
        # limiter below must see the corrected point (it would otherwise ramp back out of the corridor), and the
        # velocity field must be corrected with the position -- a reference that is told "stay on the line" while
        # carrying the plan's ACROSS-the-line velocity is the very definition of the cross-track error this fixes.
        if self.corridor_m > 0.0 and self.p is not None and self.prev_target is not None:
            a_xy = np.asarray(self.prev_target[:2], dtype=float)
            b_xy = np.asarray(self.target[:2], dtype=float)
            seg = b_xy - a_xy
            seg_len = float(np.linalg.norm(seg))
            if seg_len > 1e-6:
                u = seg / seg_len
                n = np.array([-u[1], u[0]])                  # the leg's normal, i.e. the ACROSS direction
                q = np.array([float(msg.position.x), float(msg.position.y)]) - a_xy
                along = float(np.dot(q, u))
                across = float(np.dot(q, n))
                allow = self.corridor_m
                if self.corridor_end_m > 1e-6:
                    # ramp open at BOTH ends: full strength in the interior, `corridor_open_m` of freedom at the
                    # corner, so the 90 degree turn is not clamped into a kink (the corners are where the field
                    # leaves +0.407 m instead of the edges' +0.200, so this is also where the room is).
                    w = min(1.0, max(0.0, min(along / self.corridor_end_m,
                                                (seg_len - along) / self.corridor_end_m)))
                    allow += (1.0 - w) * self.corridor_open_m
                if 0.0 <= along <= seg_len and abs(across) > allow:
                    sign = 1.0 if across >= 0.0 else -1.0
                    q = u * along + n * (sign * allow)
                    msg.position.x, msg.position.y = float(a_xy[0] + q[0]), float(a_xy[1] + q[1])
                    self.n_corridor += 1
                    v_xy = np.array([float(msg.velocity.x), float(msg.velocity.y)])
                    v_across = float(np.dot(v_xy, n))
                    if abs(v_across) > 1e-9:
                        v_xy = v_xy - v_across * n        # the wall of the corridor carries no lateral velocity
                        msg.velocity.x, msg.velocity.y = float(v_xy[0]), float(v_xy[1])
                        self.n_corridor_v += 1
                    a_xy_acc = np.array([float(msg.acceleration.x), float(msg.acceleration.y)])
                    a_across = float(np.dot(a_xy_acc, n))
                    if abs(a_across) > 1e-9:
                        a_xy_acc = a_xy_acc - a_across * n
                        msg.acceleration.x, msg.acceleration.y = float(a_xy_acc[0]), float(a_xy_acc[1])
        # ---- THE REFERENCE'S OWN SPEED LIMIT, last, so it sees exactly what would have gone out ----
        if self.ref_slew_mps > 0.0:
            p = np.array([float(msg.position.x), float(msg.position.y), float(msg.position.z)])
            # THE CORNER GOVERNOR.  MEASURED (Flights J and K, `_stage/corner-forensics.py`): every pillar
            # contact of both sorties is OUTSIDE the rectangle -- (4.58,-1.55), (4.68,0.54), (4.30,2.45),
            # (2.53,4.49), (3.50,-4.50) against corners at (+-3,+-4) -- i.e. the aircraft OVERSHOOTS each corner
            # by 1-1.7 m, out into the pillar ring, at |v_ref| 1.8-2.4 m/s.  Slow the reference down as it
            # arrives: within `corner_m` of the corner the forwarded point may not move faster than
            # `corner_mps`, and the velocity it carries is capped with it, so the flight controller is not
            # told "2.4 m/s" while being handed a point that is standing still.
            rate = self.ref_slew_mps
            if self.corner_m > 0.0 and self.p is not None:
                # THROUGH THE TURN, not only into it.  MEASURED Flight L: two of the five contact episodes
                # happened at |v_ref| 2.02 and 2.80 m/s, i.e. just OUTSIDE the governor's radius -- the moment
                # the tour advances, the vehicle is far from the NEW corner, the governor releases, and the
                # reference is back at full speed while the aircraft is still carrying momentum through a 90
                # degree turn.  The cap therefore applies within `corner_m` of the corner the tour is ON OR the
                # one it has just LEFT, which is the same physical place during the turn.
                near = float(np.linalg.norm((self.p - self.target)[:2]))
                if self.prev_target is not None:
                    near = min(near, float(np.linalg.norm((self.p - self.prev_target)[:2])))
                if near < self.corner_m:
                    # A TAPER, NOT A STEP.  The cap used to switch on at exactly `corner_m`, which puts a STEP
                    # in the reference's own speed at that radius -- and the tracking error's tail (p90 0.178
                    # against a p50 of 0.078 in Flight Q, 0.231/0.094 in S) is made of exactly those transients.
                    # The cap now ramps from `corner_mps` AT the corner to the unrestricted slew at the radius,
                    # so the reference decelerates and re-accelerates smoothly and nothing has to chase a step.
                    frac = near / max(self.corner_m, 1e-6)
                    rate = min(rate, self.corner_mps + (self.ref_slew_mps - self.corner_mps) * frac)
                    # ... but the VELOCITY field keeps the hard cap: it is the feed-forward, and a value above
                    # the reference's own speed is what makes the aircraft lead the plan.
                    v = np.array([float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)])
                    vn = float(np.linalg.norm(v))
                    if vn > self.corner_mps:
                        v = v * (self.corner_mps / vn)
                        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(v[0]), float(v[1]), float(v[2])
                        self.n_corner_capped += 1
            if self.last_fwd is not None and self.t_last_fwd is not None:
                dt = max(now - self.t_last_fwd, 1e-4)
                d = p - self.last_fwd
                dist = float(np.linalg.norm(d))
                cap = rate * dt
                if dist > cap:
                    p = self.last_fwd + d * (cap / dist)
                    msg.position.x, msg.position.y, msg.position.z = float(p[0]), float(p[1]), float(p[2])
                    self.n_slewed += 1
            self.last_fwd = p
            self.t_last_fwd = now
        return msg

    def summary(self):
        s = ("shim: %d waypoint(s), %d goal(s) sent, %d READY and %d COMPLETED messages forwarded "
             "(%d of them held at the last waypoint), %s, tour source %s%s"
             % (len(self.targets), self.goals_sent, self.n_ready, self.n_completed, self.n_held_after,
                "TOUR COMPLETE" if self.done else "waypoint %d" % (self.i_goal + 1),
                "EGO (wp_index)" if self.ego_driven else "this shell",
                " (%d wp_index message(s))" % self.wp_index_msgs if self.ego_driven else ""))
        # THE CORRIDOR'S OWN COUNT, because it is invisible in the flight's other logs: a clamp that never fired
        # and a clamp that fired on every message are the same bytes on the wire otherwise.
        if self.corridor_m > 0.0:
            s += (" | route corridor %.3f m: %d point(s) and %d velocity/acceleration field(s) clamped"
                  % (self.corridor_m, self.n_corridor, self.n_corridor_v))
        return s


# -----------------------------------------------------------------------------------------------------------
def _targets_from_world(path, calib, z_default=2.0):
    w = yaml.safe_load(open(path))
    corners = w["mission"]["corners"]
    tw = np.array([[c[0], c[1], c[2] if len(c) > 2 else z_default] for c in corners], dtype=float)
    return (calib.R_WL.T @ (tw - calib.t_WL).T).T


def _selftest() -> int:
    bad = []
    t0 = 1000.0
    seq = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.30, hold_s=1.0, log=lambda s: print("   " + s))

    class Vec:                                      # the shape of PositionCommand's x/y/z triples
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class M:                                        # a stand-in for PositionCommand, fields and all: the hold
        def __init__(self):                         # rewrite touches position/velocity/acceleration/yaw, so a
            self.trajectory_id = 0                  # two-field stub cannot exercise it (it raised AttributeError
            self.trajectory_flag = 0                # the first time this test ran, which is the test working)
            self.position, self.velocity, self.acceleration = Vec(), Vec(), Vec()
            self.yaw = 0.0
            self.yaw_dot = 0.0

    m = M()
    # THE FLIGHT'S OWN ORDER, because the first version of this test teleported the vehicle and asserted a
    # completion that a real sequence only reaches after the SECOND arrival:  arrive at WP1 -> dwell -> advance
    # -> arrive at WP2 -> dwell -> the tour is DONE.
    seq.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)          # at WP1's target: arrival + dwell start
    seq.rewrite(m, now=t0)
    print("   at waypoint 1: id %d flag %d (READY=%d)" % (m.trajectory_id, m.trajectory_flag,
                                                          TRAJECTORY_STATUS_READY))
    if (m.trajectory_id, m.trajectory_flag) != (1, TRAJECTORY_STATUS_READY):
        bad.append("the first waypoint must be id 1 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 0.5)   # mid-dwell, vehicle moved on: id must NOT move
    seq.rewrite(m, now=t0 + 0.5)
    if m.trajectory_id != 1:
        bad.append("the dwell changed the id to %d -- the consumer would think the mission advanced"
                   % m.trajectory_id)
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.6)   # dwell over: the MISSION advances
    seq.rewrite(m, now=t0 + 1.6)
    print("   after waypoint 1's dwell: id %d flag %d (still READY)" % (m.trajectory_id, m.trajectory_flag))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_READY):
        bad.append("advancing to waypoint 2 must give id 2 READY, got id %d flag %d" % (m.trajectory_id,
                                                                                       m.trajectory_flag))
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 1.7)   # arrival at WP2 starts its dwell
    seq.on_odom([5.0, 0.0, 2.0], [0.01, 0.0, 0.0], now=t0 + 2.8)   # ... and the dwell ends the tour
    seq.rewrite(m, now=t0 + 2.8)
    print("   after the LAST waypoint's dwell: id %d flag %d (COMPLETED=%d)"
          % (m.trajectory_id, m.trajectory_flag, TRAJECTORY_STATUS_COMPLETED))
    if (m.trajectory_id, m.trajectory_flag) != (2, TRAJECTORY_STATUS_COMPLETED):
        bad.append("the last waypoint's dwell must complete the tour: got id %d flag %d"
                   % (m.trajectory_id, m.trajectory_flag))
    if seq.goal_to_send(now=t0 + 3.0) is not None:
        bad.append("a goal was offered after the tour was complete")
    # IT KEEPS TALKING: further messages are still forwarded, still COMPLETED -- AND IT NO LONGER MOVES (round 81)
    msg = m                                          # the same stand-in, reused with its new fields
    msg.position.x, msg.position.y, msg.position.z = 9.0, 9.0, 9.0
    msg.velocity.x, msg.velocity.y, msg.velocity.z = 3.0, 3.0, 3.0
    msg.acceleration.x, msg.acceleration.y = 5.0, 5.0
    msg.yaw = 1.234
    seq.rewrite(msg, now=t0 + 4.0)
    if msg.trajectory_flag != TRAJECTORY_STATUS_COMPLETED:
        bad.append("the stream stopped saying COMPLETED -- the supervisor needs it on every message")
    last_wp = seq.targets[-1]
    held = (float(msg.position.x), float(msg.position.y), float(msg.position.z))
    if max(abs(held[k] - float(last_wp[k])) for k in range(3)) > 1e-9:
        bad.append("after COMPLETED the forwarded point is not the last waypoint: %s vs %s"
                   % (held, tuple(float(x) for x in last_wp)))
    if any(abs(v) > 1e-12 for v in (msg.velocity.x, msg.velocity.y, msg.velocity.z)):
        bad.append("after COMPLETED the forwarded VELOCITY is not zero: %s"
                   % (msg.velocity.x, msg.velocity.y, msg.velocity.z))
    if any(abs(a) > 1e-12 for a in (msg.acceleration.x, msg.acceleration.y, msg.acceleration.z)):
        bad.append("after COMPLETED the forwarded ACCELERATION is not zero")
    yaw1 = float(msg.yaw)
    msg.position.x = -7.0
    msg.yaw = -3.0
    seq.rewrite(msg, now=t0 + 4.1)
    print("   after COMPLETED: point pinned to the last waypoint %s, |v| = 0, yaw latched at %.3f (second"
          " message keeps %.3f)"
          % (tuple(round(float(x), 3) for x in last_wp), yaw1, float(msg.yaw)))
    if abs(float(msg.yaw) - yaw1) > 1e-12:
        bad.append("the post-COMPLETED hold let the yaw move (%.3f -> %.3f)" % (yaw1, float(msg.yaw)))
    if seq.n_held_after < 2:
        bad.append("the hold counter did not move (%d)" % seq.n_held_after)
    # and the goal cadence
    seq2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], goal_resend_s=1.0, log=lambda s: None)
    seq2.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)
    n_sent = sum(1 for k in range(30) if seq2.goal_to_send(now=t0 + 0.1 * k) is not None)
    print("   3 s of ticks at 10 Hz, goal_resend_s 1.0 -> %d goal(s) (expect 3-4)" % n_sent)
    if not 3 <= n_sent <= 4:
        bad.append("the goal cadence is %d in 3 s, expected 3-4" % n_sent)

    # ---- PLAN X: THE TOUR DRIVEN BY EGO'S OWN `wp_index` ------------------------------------------------
    # The assertions are the MODE'S CONTRACT, not a copy of the shell's: no goal may ever be sent, the id must
    # follow ego's index (including BACKWARDS, which the shell's own sequencer can never do and which would
    # otherwise be silently ignored), and -1 must complete the tour -- after which the hold pins the point.
    seq3 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0], [9.0, 0.0, 2.0]],
                         log=lambda s: print("   " + s), ego_driven=True)
    if any(seq3.goal_to_send(now=t0 + 0.1 * k) is not None for k in range(30)):
        bad.append("PLAN X sent a goal -- ego's sequencer owns the tour, and /move_base_simple/goal has no "
                   "subscriber under flight_type 2")
    # the shell's own arrival gate must be DEAD in this mode: park the vehicle exactly on target 1 and tick
    # past any dwell -- the tour must NOT advance, because ego has not said so.
    for k in range(30):
        seq3.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.5 * k)
    m3 = M()
    seq3.rewrite(m3, now=t0 + 15.0)
    if (m3.trajectory_id, seq3.done) != (1, False):
        bad.append("PLAN X advanced on the SHELL's arrival gate (id %d, done %s) -- the gate must be inert "
                   "when ego owns the tour" % (m3.trajectory_id, seq3.done))
    seq3.on_wp_index(1, now=t0 + 16.0)
    seq3.rewrite(m3, now=t0 + 16.1)
    if m3.trajectory_id != 2:
        bad.append("PLAN X did not follow ego's wp_index 1 -> id %d" % m3.trajectory_id)
    seq3.on_wp_index(0, now=t0 + 17.0)          # a BACKWARDS index: a wrong index must be SHOWN, not hidden
    seq3.rewrite(m3, now=t0 + 17.1)
    if m3.trajectory_id != 1:
        bad.append("PLAN X ignored a backwards wp_index (id %d, expected 1)" % m3.trajectory_id)
    seq3.on_wp_index(2, now=t0 + 18.0)
    seq3.on_wp_index(-1, now=t0 + 19.0)
    seq3.rewrite(m3, now=t0 + 19.1)
    hold = seq3.targets[-1]
    print("   PLAN X: ego drove the id to %d, then wp_index -1 -> flag %d, point pinned to %s, %d wp_index "
          "message(s)" % (m3.trajectory_id, m3.trajectory_flag,
                          tuple(round(float(x), 3) for x in hold), seq3.wp_index_msgs))
    if m3.trajectory_flag != TRAJECTORY_STATUS_COMPLETED:
        bad.append("PLAN X did not turn ego's wp_index -1 into COMPLETED (flag %d)" % m3.trajectory_flag)
    if max(abs(float(getattr(m3.position, c)) - float(hold[i])) for i, c in enumerate("xyz")) > 1e-9:
        bad.append("PLAN X's post-COMPLETED hold is not on the LAST waypoint")
    if seq3.n_held_after != 1:
        bad.append("PLAN X's hold counter is %d, expected 1" % seq3.n_held_after)
    if seq3.wp_index_msgs != 4:
        bad.append("PLAN X counted %d wp_index messages, expected 4" % seq3.wp_index_msgs)

    # ---- THE REFERENCE CLAMP (2026-10-16): a message far from the vehicle must be pulled back TO the cap ----
    # The assertion is the behaviour's own definition: the forwarded point keeps the DIRECTION of what the plan
    # asked for and its distance becomes exactly the cap; and with the cap off nothing is touched (the flown
    # behaviour stays byte-identical).
    veh = [0.0, 0.0, 2.0]
    seq4 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_clamp_m=0.35)
    seq4.on_odom(veh, [0.0, 0.0, 0.0], now=t0)
    m4 = M()
    m4.position.x, m4.position.y, m4.position.z = 2.0, 0.0, 2.0        # the plan asks for a point 2 m away
    seq4.rewrite(m4, now=t0)
    d4 = float(np.linalg.norm([m4.position.x - veh[0], m4.position.y - veh[1], m4.position.z - veh[2]]))
    print("   reference clamp 0.35 m: a point 2.000 m from the vehicle was forwarded at %.3f m, direction %s"
          % (d4, "kept" if (m4.position.x > 0 and abs(m4.position.y) < 1e-9) else "CHANGED"))
    if abs(d4 - 0.35) > 1e-9 or abs(float(m4.position.y) - 0.0) > 1e-9 or float(m4.position.x) <= 0:
        bad.append("the clamp did not pull a 2 m reference back to 0.35 m along the same direction (got %.3f m "
                   "at (%.3f, %.3f))" % (d4, float(m4.position.x), float(m4.position.y)))
    if seq4.n_clamped != 1:
        bad.append("the clamp counter is %d, expected 1" % seq4.n_clamped)
    seq5 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None)   # cap OFF
    seq5.on_odom(veh, [0.0, 0.0, 0.0], now=t0)
    m5 = M()
    m5.position.x = 2.0
    m5.position.z = 2.0
    seq5.rewrite(m5, now=t0)
    if abs(float(m5.position.x) - 2.0) > 1e-12 or seq5.n_clamped != 0:
        bad.append("with the clamp OFF the reference was changed (x %.6f, clamped %d) -- the flown behaviour "
                   "must stay byte-identical when the knob is 0" % (float(m5.position.x), seq5.n_clamped))

    # ---- THE ARGUMENT SURFACE AND THE START-UP BANNERS ---------------------------------------------------
    # WHY THIS IS IN THE SELFTEST AT ALL: on 2026-10-16 the P3-2 banner referenced `args.goal_resend_s`, which
    # is not an argument, so the shell died at start-up with `AttributeError` in the FLOWN mode
    # (`--tour-source shell`) while this selftest passed -- it never built the banner.  A whole bench arm went
    # into finding that.  So: parse the real parser, and build BOTH banners from it.
    print("   -- the argument surface and the banners, built from the REAL parser")
    a = _parser().parse_args([])
    for field in ("world", "calib_file", "z", "hold_s", "arrive_m", "settle_mps", "in_topic", "cmd_topic",
                  "goal_topic", "odom_topic", "trace_csv", "tour_source", "wp_index_topic",
                  "leg_timeout_s", "leg_abort_s", "arrive_z_m", "start_alt_m", "ref_z_clamp_m",
                  "ref_slew_mps", "corner_m", "corner_mps", "ref_z_floor_m", "ref_acc_max"):
        if not hasattr(a, field):
            bad.append("the start-up banner's argument surface has no %r -- main() would raise AttributeError "
                       "at start-up, in the flown mode, before the node spins" % field)
    if a.tour_source != "shell":
        bad.append("the default tour source is %r, not 'shell' (the flown behaviour must be the default)"
                   % a.tour_source)
    try:
        _print_p3_2_banner(a.arrive_m, a.settle_mps, a.hold_s, a.arrive_z_m)
        _print_tour_source_banner(a.tour_source, a.goal_topic, TourSequencer([[0, 0, 2]]).goal_resend_s,
                                  a.arrive_m, a.settle_mps, a.hold_s, a.wp_index_topic)
        _print_tour_source_banner("ego", a.goal_topic, 1.0, a.arrive_m, a.settle_mps, a.hold_s,
                                  a.wp_index_topic)
    except Exception as e:                                                       # noqa: BLE001
        bad.append("building the start-up banners raised %s: %s -- this is the exact failure that killed the "
                   "shell at start-up on 2026-10-16" % (type(e).__name__, e))
    # ---- THE PER-LEG WATCHDOG (2026-10-16): a leg that never advances must not be silent ----
    # The assertions are the behaviour's own definition: the first timeout is ANNOUNCED, the tour does NOT
    # advance, and at the abort threshold the sequencer refuses goals AND sets the flag the node uses to stop
    # publishing.  The last one is the whole point -- Flight E's 374.3 s freeze looked alive on the wire.
    wlog = []
    w = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.30, hold_s=0.3, settle_mps=0.3,
                      log=lambda s: wlog.append(s), leg_timeout_s=2.0, leg_abort_s=5.0)
    wm = M()                                                  # the plan box commands motion: the flight has begun
    wm.velocity.x = 1.5
    w.rewrite(wm, now=t0)
    for k in range(80):                                       # 8 s parked 1 m from waypoint 1 (never arrives)
        w.on_odom([1.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)
    announced = [s for s in wlog if "HAS NOT ADVANCED" in s]
    print("   watchdog: %d announcement(s) over 8 s at timeout 2.0 / abort 5.0; abort=%s; %d goal(s) after"
          % (len(announced), w.abort, sum(1 for k in range(20) if w.goal_to_send(now=t0 + 9.0 + 0.1 * k) is not None)))
    if not announced:
        bad.append("the leg watchdog never announced a stall")
    if not w.abort:
        bad.append("the leg watchdog did not abort at the abort threshold")
    if w.i_goal != 0:
        bad.append("the watchdog advanced the tour to waypoint %d -- a leg that was not flown must never be "
                   "scored as reached" % (w.i_goal + 1))
    if any(w.goal_to_send(now=t0 + 9.0 + 0.1 * k) is not None for k in range(20)):
        bad.append("a goal was still offered after the leg timeout -- the silence IS the signal")
    w2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0], [9.0, 0.0, 2.0]], arrive_m=0.30, hold_s=0.3,
                       settle_mps=0.3, log=lambda s: None, leg_timeout_s=2.0, leg_abort_s=5.0)
    w2.rewrite(wm, now=t0)
    # THE FLIGHT'S OWN ORDER, leg by leg, each one inside the 2 s timeout: the vehicle must arrive, dwell,
    # advance -- and a leg that DOES advance must get a FRESH clock, or the watchdog would kill a healthy tour.
    for k in range(5):
        w2.on_odom([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)         # at waypoint 1
    for k in range(10, 15):
        w2.on_odom([5.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)         # at waypoint 2
    for k in range(20, 25):
        w2.on_odom([9.0, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)         # at waypoint 3
    print("   watchdog: after a real arrival the tour is at waypoint %d of 3, abort=%s (a fresh leg gets a fresh"
          " clock)" % (w2.i_goal + 1, w2.abort))
    if w2.abort or w2.i_goal != 2:
        bad.append("the watchdog killed a tour that was arriving normally (waypoint %d of 3, abort %s)"
                   % (w2.i_goal + 1, w2.abort))
    w3 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.30, hold_s=0.3, settle_mps=0.3,
                       log=lambda s: None, leg_timeout_s=1.0, leg_abort_s=3.0)
    for k in range(50):                                       # 5 s of PRE-FLIGHT hold: no message ever moves
        w3.on_odom([0.0, 0.0, 0.1], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)
    print("   watchdog: 5 s of pre-flight hold with the plan box silent-but-talking -> abort=%s (the clock must "
          "not run before the flight does)" % w3.abort)
    if w3.abort:
        bad.append("the watchdog counted the pre-flight hold -- MEASURED Flight F: that aborted a healthy flight "
                   "at t=27.5 s, 19 s of which was the take-off the FCU is not even allowed to see")

    # ---- THE ARRIVAL GATE: HORIZONTAL, WITH A VERTICAL TOLERANCE, AND OPTIONALLY NO SPEED TEST ----
    # Flight F's own numbers: "corner 1: closest horizontal approach 84.4 mm PASS, vertical +0.834 m", and the
    # tour never advanced because a SPHERICAL 0.35 m gate was 0.838 m away.  The assertions are that geometry.
    g1 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.35, hold_s=0.3, settle_mps=0.0,
                       log=lambda s: None, arrive_z_m=1.00)
    g1.on_odom([0.084, 0.0, 2.834], [1.60, 0.0, 0.0], now=t0)   # 84 mm away horizontally, 834 mm HIGH, moving
    g1.on_odom([0.084, 0.0, 2.834], [1.60, 0.0, 0.0], now=t0 + 0.4)
    print("   arrival gate: 84 mm horizontal / 834 mm vertical at 1.60 m/s -> waypoint %d of 2 (expect 2)"
          % (g1.i_goal + 1))
    if g1.i_goal != 1:
        bad.append("the horizontal+vertical gate did not accept Flight F's own arrival geometry (waypoint %d)"
                   % (g1.i_goal + 1))
    g2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], arrive_m=0.35, hold_s=0.3, settle_mps=0.0,
                       log=lambda s: None, arrive_z_m=1.00)
    g2.on_odom([0.10, 0.0, 3.20], [0.0, 0.0, 0.0], now=t0)      # same corner, but 1.2 m high
    for k in range(10):
        g2.on_odom([0.10, 0.0, 3.20], [0.0, 0.0, 0.0], now=t0 + 0.1 * k)
    if g2.i_goal != 0:
        bad.append("the vertical tolerance let a 1.2 m altitude error count as an arrival")
    # and the z clamp: a reference 1.4 m above a 2.0 m tour must be pulled to the cap
    g3 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_z_clamp_m=0.50)
    m6 = M()
    m6.position.x, m6.position.z = 4.0, 3.25
    g3.rewrite(m6, now=t0)
    print("   altitude clamp: a reference at z = 3.250 m for a 2.000 m tour was forwarded at %.3f m (%d clamp(s))"
          % (float(m6.position.z), g3.n_z_clamped))
    if abs(float(m6.position.z) - 2.50) > 1e-9 or g3.n_z_clamped != 1:
        bad.append("the altitude clamp is not 2.000 + 0.500 (got %.3f m, %d clamp(s))"
                   % (float(m6.position.z), g3.n_z_clamped))
    m7 = M()
    m7.position.x, m7.position.z = 1.0, 1.10                      # BELOW the tour: must be left alone
    g3.rewrite(m7, now=t0 + 0.1)
    if abs(float(m7.position.z) - 1.10) > 1e-9:
        bad.append("the altitude clamp touched a reference BELOW the tour (%.3f m) -- it must be one-sided, or "
                   "the climb off the pad is clipped" % float(m7.position.z))
    # and the start-altitude gate
    g4 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, start_alt_m=0.50)
    g4.on_odom([0.0, 0.0, 0.10], [0.0, 0.0, 0.0], now=t0)
    pre = sum(1 for k in range(10) if g4.goal_to_send(now=t0 + 0.1 * k) is not None)
    g4.on_odom([0.0, 0.0, 0.90], [0.0, 0.0, 0.0], now=t0 + 2.0)
    post = sum(1 for k in range(10) if g4.goal_to_send(now=t0 + 2.0 + 0.1 * k) is not None)
    print("   start-altitude gate 0.50 m: %d goal(s) on the pad, %d once at 0.90 m" % (pre, post))
    if pre != 0 or post == 0:
        bad.append("the start-altitude gate is not gating (on the pad %d goal(s), airborne %d)" % (pre, post))

    # ---- THE SLEW LIMITER: a step must become a ramp ----
    sl = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_slew_mps=2.0)
    m8 = M()
    m8.velocity.x = 1.0
    m8.position.x = 0.0
    m8.position.z = 2.0
    sl.rewrite(m8, now=t0)                                   # the first message: taken as it is
    m8.position.x = 3.0                                      # a 3 m re-plan step, 0.1 s later
    sl.rewrite(m8, now=t0 + 0.1)
    print("   slew limiter 2.0 m/s: a 3.000 m step in 0.100 s was forwarded as %.3f m (%d slew(s))"
          % (float(m8.position.x), sl.n_slewed))
    if abs(float(m8.position.x) - 0.2) > 1e-6 or sl.n_slewed != 1:
        bad.append("the slew limiter did not turn a 3 m step into 0.2 m in one 0.1 s message (got %.3f m, %d)"
                   % (float(m8.position.x), sl.n_slewed))
    for k in range(1, 300):                                  # and it must CONVERGE on the requested point
        m8.position.x = 3.0                                  # a FRESH message every time (the node's real case)
        sl.rewrite(m8, now=t0 + 0.1 + 0.01 * k)
    print("   slew limiter: after 3 s of messages the forwarded point is at x = %.3f m (requested 3.000; at "
          "2.0 m/s a 3 m step takes 1.5 s, which is the point -- it is a RAMP, not a refusal)"
          % float(m8.position.x))
    if abs(float(m8.position.x) - 3.0) > 1e-6:
        bad.append("the slew limiter never converged on the requested point (%.3f m)" % float(m8.position.x))
    sl2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None)   # limiter OFF
    m9 = M()
    m9.position.x = 3.0
    sl2.rewrite(m9, now=t0)
    if abs(float(m9.position.x) - 3.0) > 1e-12 or sl2.n_slewed != 0:
        bad.append("with the slew limiter OFF the reference was changed -- the flown behaviour must stay "
                   "byte-identical when the knob is 0")

    g5 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None)
    pre = sum(1 for k in range(10) if g5.goal_to_send(now=t0 + 0.1 * k) is not None)
    g5.on_odom([0.0, 0.0, 0.1], [0.0, 0.0, 0.0], now=t0 + 1.0)
    post = sum(1 for k in range(10) if g5.goal_to_send(now=t0 + 1.0 + 0.1 * k) is not None)
    print("   first-goal gate: %d goal(s) before any pose, %d once the pose arrives" % (pre, post))
    if pre != 0 or post == 0:
        bad.append("a goal was published before the first pose (before %d, after %d) -- MEASURED Flight K: the "
                   "FSM was in INIT, ignored the goal with 'no odometry yet', went to REPLAN_TRAJ and never "
                   "planned again, and the sortie died on the supervisor's 20 s rule" % (pre, post))

    # ---- THE CORNER GOVERNOR: the reference must SLOW as it arrives, not arrive at speed ----
    cg = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_slew_mps=6.0,
                       corner_m=1.50, corner_mps=0.60)
    cg.on_odom([1.00, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)          # 1.00 m from waypoint 1 (0,0,2)
    mc = M()
    mc.trajectory_id, mc.trajectory_flag = 0, 0
    mc.position, mc.velocity, mc.acceleration = Vec(), Vec(), Vec()
    mc.position.x, mc.position.z = 1.2, 2.0
    mc.velocity.x = 2.40                                          # the speed Flight K arrived at
    cg.rewrite(mc, now=t0 + 0.1)
    print("   corner governor: at 1.00 m from the corner a 2.40 m/s reference was forwarded at %.3f m/s and "
          "x = %.3f m (%d cap(s))" % (float(mc.velocity.x), float(mc.position.x), cg.n_corner_capped))
    if abs(float(mc.velocity.x) - 0.60) > 1e-9 or cg.n_corner_capped != 1:
        bad.append("the corner governor did not cap a 2.40 m/s reference to 0.60 (got %.3f m/s, %d cap(s))"
                   % (float(mc.velocity.x), cg.n_corner_capped))
    cg2 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_slew_mps=6.0,
                        corner_m=1.50, corner_mps=0.60)
    cg2.on_odom([3.00, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)         # 3.00 m out: outside the radius
    mc2 = M()
    mc2.trajectory_id, mc2.trajectory_flag = 0, 0
    mc2.position, mc2.velocity, mc2.acceleration = Vec(), Vec(), Vec()
    mc2.position.x, mc2.position.z = 2.5, 2.0
    mc2.velocity.x = 2.40
    cg2.rewrite(mc2, now=t0 + 0.1)
    print("   corner governor: at 3.00 m out the same reference is forwarded at %.3f m/s (%d cap(s))"
          % (float(mc2.velocity.x), cg2.n_corner_capped))
    if abs(float(mc2.velocity.x) - 2.40) > 1e-9 or cg2.n_corner_capped != 0:
        bad.append("the corner governor capped the reference FAR from the corner (%.3f m/s) -- it must only act "
                   "inside its radius" % float(mc2.velocity.x))
    # ... AND IT MUST HOLD THROUGH THE TURN: once the tour has advanced, the vehicle is far from the NEW corner
    # and inside the radius of the one it has just LEFT (MEASURED Flight L: two of five contacts at 2.02 and
    # 2.80 m/s, just outside a radius that only looked at the current corner).
    cg3 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_slew_mps=6.0,
                        corner_m=1.50, corner_mps=0.60, arrive_m=0.35, hold_s=0.1, settle_mps=0.0)
    cg3.on_odom([0.10, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)          # arrive at waypoint 1 (0,0,2)
    cg3.on_odom([0.10, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.2)    # ... dwell 0.1 s -> advance to (5,0,2)
    moved = (cg3.i_goal == 1)
    cg3.on_odom([0.60, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.3)    # 0.6 m from the corner LEFT, 4.4 from the new
    mc3 = M()
    mc3.trajectory_id, mc3.trajectory_flag = 0, 0
    mc3.position, mc3.velocity, mc3.acceleration = Vec(), Vec(), Vec()
    mc3.position.x, mc3.position.z = 1.0, 2.0
    mc3.velocity.x = 2.40
    cg3.rewrite(mc3, now=t0 + 0.4)
    print("   corner governor THROUGH the turn: tour advanced=%s, vehicle 0.6 m past the old corner (4.4 m from "
          "the new one) -> forwarded at %.3f m/s (%d cap(s))"
          % (moved, float(mc3.velocity.x), cg3.n_corner_capped))
    if not moved:
        bad.append("the through-the-turn test did not advance the tour, so it proves nothing")
    elif abs(float(mc3.velocity.x) - 0.60) > 1e-9 or cg3.n_corner_capped != 1:
        bad.append("the corner governor released at the moment the tour advanced -- a 2.40 m/s reference was "
                   "forwarded at %.3f m/s while the vehicle was still inside the corner's radius"
                   % float(mc3.velocity.x))

    # ---- THE ALTITUDE FLOOR: Flight O lost 21 s of leg 5 to a reference that went 0.8 m low ----
    fl = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, ref_z_floor_m=0.60)
    fl.on_odom([0.0, 0.0, 0.10], [0.0, 0.0, 0.0], now=t0)      # on the pad: the floor must be INERT
    mf = M()
    mf.position.z = 0.20
    fl.rewrite(mf, now=t0 + 0.1)
    pad_z, armed0, floored0 = float(mf.position.z), fl.z_floor_armed, fl.n_z_floored
    fl.on_odom([0.0, 0.0, 1.80], [0.0, 0.0, 0.0], now=t0 + 0.2)  # up at the tour's altitude: the floor ARMS
    mf.position.z = 1.10                                        # Flight O's frozen 1.099 m
    fl.rewrite(mf, now=t0 + 0.3)
    print("   altitude floor 0.60 m: on the pad z = 0.200 m was forwarded as %.3f m (armed=%s); once up, a "
          "1.100 m reference was forwarded as %.3f m (%d floor(s))"
          % (pad_z, armed0, float(mf.position.z), fl.n_z_floored))
    if abs(pad_z - 0.20) > 1e-9 or armed0 or floored0 != 0:
        bad.append("the altitude floor clipped the CLIMB OFF THE PAD (z %.3f -> %.3f) -- it must be latched on "
                   "the vehicle having been up there" % (0.20, pad_z))
    if abs(float(mf.position.z) - 1.40) > 1e-9 or fl.n_z_floored != 1:
        bad.append("the altitude floor did not hold a 1.100 m reference at 2.000 - 0.600 = 1.400 m (got %.3f m, "
                   "%d floor(s))" % (float(mf.position.z), fl.n_z_floored))

    # ---- THE ROUTE CORRIDOR: the reference must be held on the leg's centreline, and opened at the corners ----
    # The world's own numbers, from `_stage/leg-clearance.py`: every route edge is +0.200 m from the nearest pillar
    # surface, the corners are +0.407 m, and the plan snakes about the line by +-0.4-0.5 m.  The clamp is the thing
    # that spends the 0.200 m on along-track lag instead of on the snake.
    rc = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, arrive_m=0.35, hold_s=0.0,
                       settle_mps=0.0, corridor_m=0.15, corridor_end_m=1.20, corridor_open_m=2.0)
    rc.on_odom([0.05, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)             # arrive at waypoint 1 (0,0,2)
    rc.on_odom([0.05, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1)       # hold 0 -> advance: the leg is now y = 0
    advanced = (rc.i_goal == 1 and rc.prev_target is not None)
    mr = M()
    mr.position.x, mr.position.y, mr.position.z = 2.50, 0.50, 2.0     # MID-LEG, 0.50 m off the line
    mr.velocity.x, mr.velocity.y = 1.00, 0.80
    mr.acceleration.y = 0.50
    rc.rewrite(mr, now=t0 + 0.2)
    print("   route corridor 0.15 m: advanced=%s; mid-leg the reference (y = 0.500, vy = 0.800) was forwarded at "
          "y = %.3f, vy = %.3f, ay = %.3f (%d point(s), %d velocity field(s) clamped)"
          % (advanced, float(mr.position.y), float(mr.velocity.y), float(mr.acceleration.y),
             rc.n_corridor, rc.n_corridor_v))
    if not advanced:
        bad.append("the corridor test did not advance the tour, so it proves nothing")
    if abs(float(mr.position.y) - 0.150) > 1e-9 or rc.n_corridor != 1:
        bad.append("the route corridor did not hold a 0.500 m-across reference at 0.150 m (got %.3f m, %d clamp(s))"
                   % (float(mr.position.y), rc.n_corridor))
    if abs(float(mr.velocity.x) - 1.00) > 1e-9 or abs(float(mr.velocity.y)) > 1e-9:
        bad.append("the route corridor left the ACROSS-the-line velocity on the wire (vx %.3f, vy %.3f) -- that is "
                   "the cross-track demand it exists to remove" % (float(mr.velocity.x), float(mr.velocity.y)))
    if abs(float(mr.acceleration.y)) > 1e-9:
        bad.append("the route corridor left the across-the-line ACCELERATION on the wire (ay %.3f)"
                   % float(mr.acceleration.y))
    mr2 = M()
    mr2.position.x, mr2.position.y, mr2.position.z = 2.50, -0.50, 2.0    # the OTHER side of the same line
    mr2.velocity.x = 1.00
    rc.rewrite(mr2, now=t0 + 0.3)
    print("   route corridor: the other side (y = -0.500) was forwarded at y = %.3f" % float(mr2.position.y))
    if abs(float(mr2.position.y) + 0.150) > 1e-9:
        bad.append("the route corridor is one-sided (y = -0.500 went out at %.3f) -- the centrelines are the "
                   "maximum-clearance paths, so both sides are equally expensive" % float(mr2.position.y))
    # ... AND IT MUST OPEN AT THE CORNERS, which is where the field leaves twice the room (0.407 m against 0.200):
    mr3 = M()
    mr3.position.x, mr3.position.y, mr3.position.z = 4.60, 0.50, 2.0    # 0.40 m from the leg's END
    mr3.velocity.x = 1.00
    rc.rewrite(mr3, now=t0 + 0.4)
    print("   route corridor: 0.40 m from the corner the same 0.500 m offset was forwarded at y = %.3f "
          "(%d clamp(s) total)" % (float(mr3.position.y), rc.n_corridor))
    if abs(float(mr3.position.y) - 0.50) > 1e-9:
        bad.append("the route corridor stayed at full strength at the leg's END (y = 0.500 -> %.3f) -- the turn "
                   "would be clamped into a 90 degree kink at the one place with room to take it"
                   % float(mr3.position.y))
    rc0 = TourSequencer([[0.0, 0.0, 2.0], [5.0, 0.0, 2.0]], log=lambda s: None, arrive_m=0.35, hold_s=0.0,
                        settle_mps=0.0)                                  # the knob at 0 = the flown behaviour
    rc0.on_odom([0.05, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0)
    rc0.on_odom([0.05, 0.0, 2.0], [0.0, 0.0, 0.0], now=t0 + 0.1)
    mr4 = M()
    mr4.position.x, mr4.position.y, mr4.position.z = 2.50, 0.50, 2.0
    mr4.velocity.y = 0.80
    rc0.rewrite(mr4, now=t0 + 0.2)
    print("   route corridor OFF: the same reference went out at y = %.3f, vy = %.3f (%d clamp(s))"
          % (float(mr4.position.y), float(mr4.velocity.y), rc0.n_corridor))
    if abs(float(mr4.position.y) - 0.50) > 1e-12 or abs(float(mr4.velocity.y) - 0.80) > 1e-12 or rc0.n_corridor:
        bad.append("with the corridor at 0 the reference was changed -- the flown behaviour must stay "
                   "byte-identical when the knob is 0")

    if bad:
        print("\nCONTRACT SHIM SELFTEST FAILED (%d problem(s)):" % len(bad))
        for b in bad:
            print("   - %s" % b)
        return 1
    print("\nCONTRACT SHIM SELFTEST PASSED: id per waypoint, COMPLETED keeps talking WITHOUT moving, yaw latched, "
          "goal cadence")
    return 0


def _parser() -> "argparse.ArgumentParser":
    """THE ARGUMENT SURFACE, in one place, so `--selftest` can parse it and touch every field the start-up
    banner uses.  That is not tidiness: on 2026-10-16 the banner referenced `args.goal_resend_s`, which is NOT
    an argument (the sequencer's own constructor default is), so the shell died with `AttributeError` at
    start-up in `--tour-source shell` -- i.e. in the FLOWN configuration -- while `--selftest` passed, because
    the selftest never built the banner.  The next Plan X bench caught it only because it ran a control arm."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--world", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    "worlds", "diagonal.yaml"))
    ap.add_argument("--calib-file", default="/mnt/d/distill_self/_stage/calib.json")
    ap.add_argument("--z", type=float, default=2.0)
    ap.add_argument("--hold-s", type=float, default=5.0)
    ap.add_argument("--arrive-m", type=float, default=0.30)
    ap.add_argument("--settle-mps", type=float, default=0.08,
                    help="the speed under which the vehicle counts as ARRIVED; 0 disables the speed test, which "
                         "is what 'fly THROUGH the waypoint' needs -- a 3 m/s aircraft spends ~0.2 s inside a "
                         "0.35 m disc, so a gate that also demands it be slow never fires")
    ap.add_argument("--arrive-z-m", type=float, default=1.00,
                    help="SECOND arrival tolerance, VERTICAL (metres).  Flight F MEASURED why it is needed: the "
                         "aircraft stood 84.4 mm from corner 1 horizontally and 0.834 m high, so a spherical "
                         "gate never opened however close it got -- while ego's OWN 3-D arrival (1.0 m) had "
                         "already fired.  The tour is a horizontal one; the altitude is reported, not gated.")
    ap.add_argument("--start-alt-m", type=float, default=0.0,
                    help="no goal is published until the vehicle's z reaches this (0 = off).  The Route B twin of "
                         "PATCH 0008's `fsm/waypoint_start_alt`, which only arms ego's own `flight_type 2` tour")
    ap.add_argument("--ref-z-clamp-m", type=float, default=0.0,
                    help="never forward an altitude more than this far ABOVE the tour's own z (0 = off, one-sided "
                         "on purpose so the climb off the pad is untouched).  Flight F's reference ran to 3.25 m "
                         "for a 1.884 m tour and the aircraft followed it")
    ap.add_argument("--corner-m", type=float, default=0.0,
                    help="THE CORNER GOVERNOR (0 = off), with --corner-mps: within this distance of the tour's "
                         "current corner the forwarded reference may not move faster than --corner-mps and the "
                         "velocity it carries is capped with it.  MEASURED Flights J and K: every pillar contact "
                         "is OUTSIDE the rectangle, 1-1.7 m past a corner, at |v_ref| 1.8-2.4 m/s.")
    ap.add_argument("--corner-mps", type=float, default=1.0,
                    help="the reference's speed limit inside the corner governor's radius (m/s)")
    ap.add_argument("--ref-acc-max", type=float, default=0.0,
                    help="the bound on the ACCELERATION field the FCU is given (0 = off).  Added with the mask "
                         "opened: Flight D's trial of `posvelacc` was refuted for the wrong reason, but the review "
                         "was right that the position clamp leaves this channel unclamped, and ArduPilot adds it "
                         "to its velocity target unshaped.  The pre-condition -- a continuous reference -- is met "
                         "by --ref-clamp-m and --ref-slew-mps")
    ap.add_argument("--ref-z-floor-m", type=float, default=0.0,
                    help="never forward an altitude more than this far BELOW the tour's own z, once the vehicle "
                         "has been up there (0 = off).  MEASURED Flight O: the reference froze 0.8 m low for four "
                         "seconds and the aircraft's worst sample of the whole flight is at altitude 0.18 m, "
                         "0.143 m inside a pillar of the outermost column")
    ap.add_argument("--ref-slew-mps", type=float, default=0.0,
                    help="the forwarded reference's own SPEED LIMIT (0 = off).  The sampler restarts its clock at "
                         "every re-plan, so the point jumps; this turns each jump into a short, continuous ramp "
                         "instead of a step the flight controller has to chase.  0.06 m at 100 Hz = 6 m/s, which "
                         "is 20x a legitimate 3 m/s reference's 0.03 m step")
    ap.add_argument("--leg-timeout-s", type=float, default=0.0,
                    help="a leg that has not advanced in this many seconds is ANNOUNCED once a second (0 = off). "
                         "A frozen reference produces exactly the same bytes as a hover, so only the clock can "
                         "tell them apart -- Flight E lost 374.3 s to the difference.")
    ap.add_argument("--leg-abort-s", type=float, default=0.0,
                    help="at this many seconds the shell STOPS PUBLISHING, which trips the supervisor's 3.0 s "
                         "plan-silence rule and ends the sortie loudly (0 = off).  The leg is not scored as "
                         "reached and the tour does not advance.")
    ap.add_argument("--in-topic", default=DEFAULT_IN_TOPIC)
    ap.add_argument("--cmd-topic", default=DEFAULT_CMD_TOPIC)
    ap.add_argument("--goal-topic", default=DEFAULT_GOAL_TOPIC)
    ap.add_argument("--odom-topic", default="/ego/odom_fcu")
    ap.add_argument("--trace-csv", default="")
    ap.add_argument("--tour-source", choices=("shell", "ego"), default="shell",
                    help="WHO OWNS THE TOUR.  `shell` (default, the flown behaviour): this process sends the "
                         "goals and gates the arrivals.  `ego` (Plan X): ego's own waypoint sequencer owns it, "
                         "`--wp-index-topic` drives the id and the COMPLETED transition, and no goal is sent.")
    ap.add_argument("--wp-index-topic", default="/wp_index",
                    help="`std_msgs/Int32`, published by PATCH 0011: the current waypoint, -1 when finished")
    ap.add_argument("--corridor-m", type=float, default=0.0,
                    help="THE ROUTE CORRIDOR (0 = off): the forwarded horizontal reference is held within this "
                         "distance of the CURRENT leg's centreline, and the across-the-line component of the "
                         "velocity and acceleration fields is removed with it.  MEASURED justification "
                         "(`_stage/leg-clearance.py`): all four route edges are exactly +0.200 m from the nearest "
                         "pillar surface -- they are the field's maximum-clearance paths -- while the plan snakes "
                         "about them by +-0.4-0.5 m, which is the entire collision deficit")
    ap.add_argument("--corridor-end-m", type=float, default=0.0,
                    help="how far from each leg's ends the corridor opens back up (metres; needs --corridor-m).  "
                         "The corners are the safe places -- +0.407 m of clearance against the edges' +0.200 -- so "
                         "the turn is left free instead of being clamped into a 90 degree kink")
    ap.add_argument("--corridor-open-m", type=float, default=2.0,
                    help="how much freedom (metres) the corridor gives at the very ends of a leg; the allowance "
                         "ramps from --corridor-m in the interior to corridor-m + this at the ends")
    ap.add_argument("--ref-clamp-m", type=float, default=0.0,
                    help="CONTINUITY AT THE SOURCE: never forward a reference point more than this far from the "
                         "vehicle (0 = off, the behaviour every sortie before 2026-10-16 flew).  A position "
                         "reference is a place to go, not a schedule, so clamping its distance costs the mission "
                         "nothing while removing the 0.28 m median / 0.93 m max steps the sampler produces at "
                         "every 1 Hz replan -- steps the flight controller then chases.")
    return ap


def _print_p3_2_banner(arrive_m: float, settle_mps: float, hold_s: float, arrive_z_m: float = 1.0) -> None:
    """P3-2 OF THE MISSION-B REVIEW: "任务结束" is decided in THREE places whose thresholds differ, and two of
    them were only discoverable by reading three files.  They are printed HERE, once, at start-up, with the
    source of each -- so a run's own log says which rule ended it.  (The fourth number, the supervisor's
    SILENCE window, is a FAILURE rule, not an arrival rule, and is listed with the others so the difference is
    visible.)  It takes plain FLOATS rather than an `argparse.Namespace`, which is what makes it callable from
    `--selftest` -- see `_parser`."""
    print("shim: ARRIVAL RULES IN THIS STACK -- (1) EGO's own: `plan_done && wp_d_veh < "
          "fsm/waypoint_advance_m` (`ego_replan_fsm.cpp:879`, launch default 1.000 m, PATCH 0008's advance); "
          "(2) THIS SHELL's: d_xy <= %.3f m AND |dz| <= %.3f m AND |v| < %.3f m/s (0 = no speed test, i.e. fly "
          "THROUGH), held %.1f s (`--arrive-m/--arrive-z-m/--settle-mps/--hold-s`); "
          "(3) the SUPERVISOR's: the contract's `trajectory_flag = COMPLETED` (`supervisor.py:295`) -- silence "
          "is NOT completion.  The supervisor's 3.0 s plan-silence window is a FAILURE rule, not an arrival "
          "rule." % (arrive_m, arrive_z_m, settle_mps, hold_s))


def _print_tour_source_banner(tour_source: str, goal_topic: str, goal_resend_s: float, arrive_m: float,
                              settle_mps: float, hold_s: float, wp_index_topic: str) -> None:
    """WHO OWNS THE TOUR, said out loud -- `shell` re-sends goals, `ego` (Plan X) sends none."""
    if tour_source == "ego":
        print("shim: TOUR SOURCE = EGO -- subscribed to %s (PATCH 0011); no goals are sent, and the id/flag "
              "follow ego's index.  Set --tour-source shell to restore the flown behaviour." % wp_index_topic)
    else:
        print("shim: TOUR SOURCE = THIS SHELL -- goals are re-sent on %s at %.1f Hz and the arrival gate is "
              "ours (%.3f m, |v| < %.3f m/s, held %.1f s)"
              % (goal_topic, 1.0 / goal_resend_s, arrive_m, settle_mps, hold_s))


def main() -> int:
    args = _parser().parse_args()
    if args.selftest:
        return _selftest()

    import json                                                                  # noqa: E402
    import rclpy                                                                 # noqa: E402
    from geometry_msgs.msg import PoseStamped                                    # noqa: E402
    from nav_msgs.msg import Odometry                                            # noqa: E402
    from quadrotor_msgs.msg import PositionCommand                               # noqa: E402
    from rclpy.node import Node                                                  # noqa: E402
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402

    d = json.load(open(args.calib_file))
    calib = frames.Calib(R_WL=np.array(d["R_WL"], dtype=float), t_WL=np.array(d["t_WL"], dtype=float))
    targets_l = _targets_from_world(args.world, calib, args.z)
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    except Exception:                                                            # noqa: BLE001
        rclpy.init(args=[])
    node = Node("ego_contract_shim")
    # LINE-BUFFERED STDOUT, because this process is normally ended by a signal and its banner is not flushed by
    # then: a bench arm on 2026-10-16 was diagnosed as "the shell printed nothing" when in fact it had printed
    # a full banner that was sitting in a 4 KB buffer.  An empty log must mean the process said nothing.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:                                                            # noqa: BLE001
        pass
    ego_driven = (args.tour_source == "ego")
    seq = TourSequencer(targets_l, arrive_m=args.arrive_m, hold_s=args.hold_s,
                        settle_mps=args.settle_mps, log=lambda s: print(s, flush=True),
                        ego_driven=ego_driven, ref_clamp_m=args.ref_clamp_m,
                        leg_timeout_s=args.leg_timeout_s, leg_abort_s=args.leg_abort_s,
                        arrive_z_m=args.arrive_z_m, start_alt_m=args.start_alt_m,
                        ref_z_clamp_m=args.ref_z_clamp_m, ref_slew_mps=args.ref_slew_mps,
                        corner_m=args.corner_m, corner_mps=args.corner_mps,
                        ref_z_floor_m=args.ref_z_floor_m, ref_acc_max=args.ref_acc_max,
                        corridor_m=args.corridor_m, corridor_end_m=args.corridor_end_m,
                        corridor_open_m=args.corridor_open_m)
    print("shim: %d waypoint(s) in L from %s" % (len(targets_l), args.world))
    for i, t in enumerate(targets_l):
        print("shim: waypoint %d -> %s (L)" % (i + 1, np.round(t, 3)))
    print("shim: TRAJ_SERVER's %s is forwarded to %s with trajectory_id per waypoint and a real COMPLETED"
          % (args.in_topic, args.cmd_topic))
    _print_p3_2_banner(args.arrive_m, args.settle_mps, args.hold_s, args.arrive_z_m)

    # the CONTRACT's QoS is not negotiable: RELIABLE + KEEP_LAST(5) + TRANSIENT_LOCAL, exactly as
    # `planner/ego_source.py:421-424` publishes it (a latched subscriber must still find the newest sample)
    cmd_qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PositionCommand, args.cmd_topic, cmd_qos)
    goal_pub = node.create_publisher(PoseStamped, args.goal_topic, 10)
    # traj_server publishes with a plain depth-50 RELIABLE profile (`traj_server.cpp:244`)
    sub_qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE)
    trace = open(args.trace_csv, "w") if args.trace_csv else None
    if trace:
        trace.write("t_wall,wp,flag,px,py,pz,vx,vy,vz,ax,ay,az,yaw,jump_m,frame\n")
    last = [None]
    n = {"fwd": 0}

    def on_odom(m: Odometry):
        # A REPORTED ZERO IS "SLOW", NOT "UNKNOWN".  The first version passed `None` whenever the twist was
        # (0,0,0), and the sequencer reads `None` as "speed unknown" -- which BLOCKS the arrival and therefore the
        # whole tour.  In flight it worked only because the FCU's velocity noise is never exactly zero; a
        # stationary vehicle reporting a clean zero would have stalled the mission.  A message callback always
        # HAS a twist, so it is passed through as it is.
        v = [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z]
        seq.on_odom([m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z],
                    v, now=node.get_clock().now().nanoseconds * 1e-9)

    def on_cmd(m: PositionCommand):
        if seq.abort:
            # THE WATCHDOG'S OWN ACTUATION.  After a leg times out the contract asks the supervisor to end the
            # sortie by going SILENT -- a frozen reference is otherwise indistinguishable from a hover, which is
            # exactly how Flight E spent 374.3 s with the stream looking healthy.
            return
        seq.rewrite(m, now=node.get_clock().now().nanoseconds * 1e-9)
        p = np.array([m.position.x, m.position.y, m.position.z])
        jump = 0.0 if last[0] is None else float(np.linalg.norm(p - last[0]))
        last[0] = p
        pub.publish(m)
        n["fwd"] += 1
        if trace:
            trace.write("%.4f,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s\n"
                        % (node.get_clock().now().nanoseconds * 1e-9, m.trajectory_id, m.trajectory_flag,
                           p[0], p[1], p[2], m.velocity.x, m.velocity.y, m.velocity.z,
                           m.acceleration.x, m.acceleration.y, m.acceleration.z, m.yaw, jump,
                           m.header.frame_id))

    node.create_subscription(Odometry, args.odom_topic, on_odom, sub_qos)
    node.create_subscription(PositionCommand, args.in_topic, on_cmd, sub_qos)
    if ego_driven:
        # PLAN X (PATCH 0011 publishes it): ego's own sequencer says which waypoint the mission is on, and -1
        # says it is over.  With this topic live the shell sends NO goals and runs NO arrival gate.
        from std_msgs.msg import Int32                                                # noqa: E402

        def on_wp_index(m: Int32):
            seq.on_wp_index(m.data, now=node.get_clock().now().nanoseconds * 1e-9)

        node.create_subscription(Int32, args.wp_index_topic, on_wp_index, sub_qos)
    _print_tour_source_banner(args.tour_source, args.goal_topic, seq.goal_resend_s, args.arrive_m,
                              args.settle_mps, args.hold_s, args.wp_index_topic)

    def tick():
        g = seq.goal_to_send(now=node.get_clock().now().nanoseconds * 1e-9)
        if g is None:
            return
        gm = PoseStamped()
        gm.header.stamp = node.get_clock().now().to_msg()
        gm.header.frame_id = FRAME_L
        gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = float(g[0]), float(g[1]), float(g[2])
        gm.pose.orientation.w = 1.0
        goal_pub.publish(gm)
        # SAY IT.  A goal that is published and never seen by the FSM (wrong topic, no subscriber, the node
        # still in a constructor wait) looks EXACTLY like a planner that refuses to plan -- a whole bench arm
        # was spent on that ambiguity on 2026-10-16 (the control arm of `_stage/s13-planx.sh` never planned and
        # nothing in either log said whether a goal had been sent).  One line per send, at 1 Hz, is cheap.
        print("shim: goal %d/%d -> (%.3f, %.3f, %.3f) L on %s"
              % (seq.i_goal + 1, len(seq.targets), float(g[0]), float(g[1]), float(g[2]), args.goal_topic),
              flush=True)

    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(seq.summary(), flush=True)
    if trace:
        trace.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
