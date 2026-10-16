#!/usr/bin/env bash
# Mission A, one flight: SITL + the simulated MID-360 + the LIO + the PLAN box + the BRIDGE + the
# SUPERVISOR.  Since F2 there is no FSM: the plan box executes, the bridge transports, the supervisor
# owns the flight commands.  Three processes, three jobs, one writer per MAVLink message class.
# There is no planner process: the rectangle IS the trajectory, and the truth tap is what judges it.
#
# Everything is recorded, because the metrics are computed offline from these files:
#   tap.csv      the truth (1200 Hz)
#   lio.csv      exactly what the FCU was fed (/lio/robo/odom)
#   phase.csv    the FSM's phase boundaries and the commanded point in force during each
#   sitl.log     the FCU's console, and logs/*.BIN for its own view
#
# usage: tools/run-mission.sh [RUN_TAG]
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
ROOT=/mnt/d/distill_self/lio_avoid        # the project: params/, worlds/, docs/, frozen/, rviz/, firmware/
REPO="$ROOT/src"                          # the code: bridge/, common/, lio/, planner/, sim/, supervisor/, tools/
WS=/home/fanxin/lio_avoid/lio_ws
TAP=14560
FAN=14561
TAG="${1:-$(date +%Y%m%d_%H%M%S)}"
OUT="/home/fanxin/lio_avoid/logs/missionA-$TAG"
mkdir -p "$OUT"
# THE SIMULATED AIRFRAME MUST MATCH THE FIRMWARE'S MIXER.  MEASURED 2026-10-14 (the user's question
# "等会，你用的不是quad x吗？"): this script passed `--model quad`, and in the simulator's own table
# (libraries/SITL/SIM_Frame.cpp:414) `quad` is `quad_plus_motors` -- motors at 90/-90/0/180 degrees --
# while params/sitl_basics.parm:12 sets FRAME_TYPE 1, whose mixer is X (motors at 45/-135/-45/135; the
# firmware build default, ArduCopter/config.h:630 HAL_FRAME_TYPE_DEFAULT = MOTOR_FRAME_TYPE_X).
#
# Upstream never makes that pair: sim_vehicle.py takes the model FROM the frame (`--frame X` -> model X ->
# default_params/copter-X.parm, which sets FRAME_TYPE 1; `--frame +` -> model +), and
# AutoTestCopter.default_frame() is "+" for exactly that reason.  With an X mixer driving a plus airframe,
# a roll demand also produces pitch of the same magnitude, so the attitude loop fights a 1:1 cross-coupling
# -- what "the aircraft is too heavy, its attitudes answer too late" looks like in the tap.
# `--model x` is quad_x_motors: motor 1 front-right, 2 rear-left, 3 front-left, 4 rear-right.
SITL_MODEL="${SITL_MODEL:-x}"
if grep -q "^FRAME_TYPE[[:space:]]*1" "$ROOT/params/sitl_basics.parm" 2>/dev/null; then
    case "$SITL_MODEL" in
        x|X) : ;;
        *) echo "SITL_MODEL=$SITL_MODEL while sitl_basics.parm sets FRAME_TYPE 1 (X): the mixer and the" \
                "simulated motor geometry would disagree.  Use SITL_MODEL=x or change FRAME_TYPE." >&2
           exit 1 ;;
    esac
fi
# ALT_MODE selects the altitude source and therefore WHICH PHASE this flight is evidence for:
#   baro = step (a), the stepping stone (EK3_SRC1_POSZ = 1 / VELZ = 0)
#   lio  = step (b), the configuration mission A is defined by (POSZ = 6 / VELZ = 6)
ALT_MODE="${ALT_MODE:-baro}"
case "$ALT_MODE" in
    baro) ALT_PARM="$ROOT/params/phase_a_posz_baro.parm" ;;
    lio)  ALT_PARM="$ROOT/params/phase_a_posz_lio.parm" ;;
    *) echo "ALT_MODE must be baro or lio, not '$ALT_MODE'" >&2; exit 1 ;;
esac
# WORLD and MISSION_PARM select the mission itself: the same pipeline flies the slow 2.0 x 1.2 m square in
# the dense forest (default) or the fast 6.0 x 4.0 m square in the sparse one, and every geometric number
# -- the trajectory, the corner reference the metrics judge against, the geofence -- comes from the world
# file through tools/world-info.py, so a reference cannot drift from what is flown (STATUS.md row 44).
WORLD="${WORLD:-$ROOT/worlds/pillars.yaml}"
MISSION_PARM="${MISSION_PARM:-$ROOT/params/phase_a_mission.parm}"
[ -f "$WORLD" ] || { echo "no world file at $WORLD" >&2; exit 1; }
# GEOFENCE_M may be overridden for a mission whose goal is ON the arena wall: the gate exists to catch a
# runner, and the arena itself is already enforced by the planner's own arena check.
#
# THE CALLER'S VALUE IS CAPTURED *BEFORE* THE EVAL, AND THAT ORDER IS THE WHOLE FIX.  `world-info.py
# --shell` exports GEOFENCE_M, so anything read after the eval is the world's value, not the caller's.
# MEASURED 2026-10-14: sortie E1 aborted with "geofence: truth at [+6.00 +5.39]" although GEOFENCE_M=6.25
# had been exported (its own goal was the corner (6,6), ON the world's 6.00 m wall).  The first repair kept
# the bug -- it assigned GEOFENCE_M from $GEOFENCE_M *after* the eval -- and sortie F1 (20:36) printed
# "geofence : 6.00 m" with GEOFENCE_M=6.35 in its environment, which is how the wrong order was caught.
GEOFENCE_ENV="${GEOFENCE_M:-}"
eval "$(python3 "$REPO/tools/world-info.py" --world "$WORLD" --shell)"
if [ -n "$GEOFENCE_ENV" ]; then GEOFENCE_M="$GEOFENCE_ENV"; fi
MISSION_NAME="$(basename "$MISSION_PARM" .parm)"
# EKF_PARM selects HOW MUCH THE EKF TRUSTS THE LIO: `params/ekf_trust_lio.parm` sets every measurement and
# process noise to the value that has been measured, instead of the firmware's aircraft-scale defaults.
# Empty (the default) keeps the historical configuration, so old arms stay reproducible.
EKF_PARM="${EKF_PARM:-}"
EKF_ARG=""
EKF_NAME="firmware defaults"
if [ -n "$EKF_PARM" ]; then
    [ -f "$EKF_PARM" ] || { echo "no EKF parameter file at $EKF_PARM" >&2; exit 1; }
    EKF_ARG=",$EKF_PARM"
    EKF_NAME="$(basename "$EKF_PARM" .parm)"
fi
# ONE-VARIABLE EXPERIMENTS WITHOUT TOUCHING A FROZEN FILE.  `EXTRA_PARM` is a SINGLE parameter file
# appended LAST to `--defaults` (the firmware applies them in order, so the last wins), and it is how a
# parameter A/B is flown while `params/*.parm` stays byte-for-byte what the frozen manifest lists: the file
# lives outside the repo and its PATH IS PRINTED in the revision block, so the sortie's own log names the
# configuration it overrode.  MEASURED 2026-10-13 and kept: two `--defaults` arguments do not accumulate
# (the second REPLACES the first) -- which is why this is ONE comma-joined list and not a second flag.
EXTRA_PARM="${EXTRA_PARM:-}"
EXTRA_ARG=""
EXTRA_NAME="none"
if [ -n "$EXTRA_PARM" ]; then
    [ -f "$EXTRA_PARM" ] || { echo "no EXTRA_PARM file at $EXTRA_PARM" >&2; exit 1; }
    EXTRA_ARG=",$EXTRA_PARM"
    EXTRA_NAME="$EXTRA_PARM ($(grep -c . "$EXTRA_PARM") line(s))"
fi
# THE DWELL, AND WHY THE HARNESS SETS IT (F1, 2026-10-16).  The owner's requirement is a hover at every
# corner, and the number is 5.0 s (`docs/PLAN.md`).  It used to be passed only when the caller exported
# RECT_HOLD_S; with it unset the producer's `--hold-s -1` read `mission.hold_s` out of the FROZEN world
# file, which is **3.0** -- and STATUS row 42 is the measured consequence of that value: the default
# configuration flew one leg and landed.  The default is now the requirement, and which one was used is
# PRINTED at start-up rather than inferred from the log afterwards.
if [ -n "${RECT_HOLD_S:-}" ]; then
    RECT_HOLD_FROM="the CALLER (RECT_HOLD_S=$RECT_HOLD_S)"
else
    RECT_HOLD_S=5.0
    RECT_HOLD_FROM="THE HARNESS DEFAULT (RECT_HOLD_S was not set)"
fi
WORLD_HOLD_S="$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["mission"].get("hold_s", "unset"))' "$WORLD" 2>/dev/null || echo unreadable)"

# THE TAKE-OFF PROFILE, AND IT IS PRINTED BECAUSE IT CHANGED WHAT IS FLOWN.  Two mechanisms are in the
# code; this harness flies the OWNER'S (2026-10-15): the take-off command asks for only ~0.2 m ("just leave
# the ground cleanly"), the go goes out the moment the vehicle is airborne, and the CLIMB TO THE MISSION
# ALTITUDE IS THE PLAN'S -- a vertical first target held for the A4 window.  `TAKEOFF_COMMAND_M` equal to
# the mission altitude restores the other profile (the FCU's own take-off controller flies the whole climb),
# and `VERTICAL_FIRST=0` removes the plan's climb leg with it.
TAKEOFF_COMMAND_M="${TAKEOFF_COMMAND_M:-0.2}"
VERTICAL_FIRST="${VERTICAL_FIRST:-1}"
VERTICAL_HOLD_S="${VERTICAL_HOLD_S:-3.0}"
# APPROACH SHAPING (the B1 remedy, 2026-10-15): 0.0 = off, the mission as it has always been flown.  A
# positive value inserts ONE extra waypoint that far before each tour corner, ON the leg, with no dwell of
# its own -- the vehicle must come to a SETTLED stop there (the settle gate is unchanged), so the final
# approach into the corner starts from rest instead of braking from 1.7 m/s.  WHY: B1's box is made by the
# under-damped ring that braking excites, measured at 35-93 mm past each wall with the extremes at ~0 m/s
# (`docs/metrics/b1-mechanism-20261016.txt`).
SHAPING_M="${SHAPING_M:-0.0}"
SHAPING_ARG=""
if awk "BEGIN{exit !($SHAPING_M > 0)}"; then
    SHAPING_ARG="--approach-shaping-m $SHAPING_M"
fi
if awk "BEGIN{exit !($TAKEOFF_COMMAND_M == $TAKEOFF_ALT)}"; then
    TAKEOFF_PROFILE="FCU-TARGET -- packet.z = ${TAKEOFF_COMMAND_M} m VERBATIM, the FCU's own take-off controller flies the whole climb"
    VERTICAL_ARG=""
    SUP_TRIGGER="--trigger-after-hold"
else
    TAKEOFF_PROFILE="SETPOINT-DRIVEN CLIMB (the owner's design) -- packet.z = ${TAKEOFF_COMMAND_M} m only; the ${TAKEOFF_ALT} m mission altitude is the PLAN's, the go goes out ON AIRBORNE"
    if [ "$VERTICAL_FIRST" = "1" ]; then
        VERTICAL_ARG="--vertical-first-m $TAKEOFF_ALT --vertical-hold-s $VERTICAL_HOLD_S"
    else
        VERTICAL_ARG=""
    fi
    SUP_TRIGGER="--trigger-on-airborne"
fi

# WHICH NAVIGATION SOURCE, AND THEREFORE WHICH CALIBRATION.  This decides the L->W file the plan box, the
# bridge and the plan recorder all take, so it is settled HERE -- before anything is started and before the
# sortie lock is taken -- rather than in the middle of the start-up sequence where it used to live.  It moved
# because the planner's own argv is built below and must carry the SAME `--calib-file` as the rest of the
# stack: with the predicted transform in one place and the measured one in another, every commanded corner
# moved 51 mm in N.
NAV_SOURCE="${NAV_SOURCE:-lio}"
CALIB_ARG=""
TRAJ_CALIB_ARG=""
if [ "$NAV_SOURCE" = "lio" ]; then
    CALIB="$ROOT/../_stage/calib.json"
    if [ ! -f "$CALIB" ]; then
        echo "no calibration file at $CALIB -- run once with NAV_SOURCE=truth first"
        exit 1
    fi
    CALIB_ARG="--calib-file $CALIB"
    TRAJ_CALIB_ARG="--calib-file $CALIB"
fi

# WHO PLANS.  The planner is a PLUGIN: everything downstream -- the bridge, the supervisor, the FCU --
# reads only `/plan/cmd` (quadrotor_msgs/PositionCommand, frame L, `trajectory_id` per waypoint,
# `trajectory_flag` 1 while flying and 3 when the mission is over).  `PLANNER=` chooses the producer, and
# which one ran is PRINTED in the banner below, because a sortie must never be readable as "some planner".
#   rectangle  the Mission A rectangle we fly and score (`planner/rectangle_source.py`)
#   ego        ego-planner behind the same contract    (`planner/ego_source.py`)
#   <path.py>  any file that publishes the contract
# Nothing in the bridge or the supervisor changes with this switch: `tools/test-planner-swap.py` drives the
# SAME consumer code with three different planners and asserts exactly that.
PLANNER="${PLANNER:-rectangle}"
PLANNER_TOPIC="${PLANNER_TOPIC:-/plan/cmd}"
#: the limits ego's optimiser is told to respect, when the ego stack is started (see `EGO_STACK` below).
#: They are SOFT quadratic penalties in that optimiser (`bspline_optimizer.cpp:1118-1149`), not constraints,
#: so they shape the plan rather than bound the flight -- the FCU is what bounds the flight.
EGO_MAX_VEL="${EGO_MAX_VEL:-2.0}"
EGO_MAX_ACC="${EGO_MAX_ACC:-3.0}"
#: HOW FAT THE PLANNER'S OBSTACLE IS ON TOP OF THE MARKED CELLS (`grid_map/obstacles_inflation`).  0.099 above is
#: one 0.1 m cell and has been the flown value since DIAG14.  MEASURED consequence, Flight E's own DIAG/2 slices
#: via `_stage/map-inflation.py`: the map's obstacle reaches 0.540 m from a pillar's axis, i.e. 0.240 m past the
#: true 0.300 m surface, while the 6 x 8 m rectangle's legs run 0.500 m from those axes.  Every leg so far has
#: therefore been INSIDE the planner's obstacle -- the 1-2 m detours, the 0.33 m map corridor and the
#: "First 3 control points in obstacles" deadlock all follow from it.  0.0 removes the 27-cell cube (`inf_step`
#: is an INTEGER cell count, so 0.099 and 0.0 are the only two values this parameter can express).
EGO_INFLATION="${EGO_INFLATION:-0.099}"
#: `fsm/thresh_replan_time`: how long a committed spline is executed before the FSM re-plans from its own tail.
#: Every re-plan also restarts `traj_server`'s clock, so this is "how much of each spline the aircraft flies".
EGO_REPLAN_S="${EGO_REPLAN_S:-1.0}"
#: THE ARRIVAL GATE AN EGO PLANNER CAN ACTUALLY MEET, and the number is not a preference.
#: `EGOPlannerManager::reboundReplan` REFUSES to plan when the replan's start is within 0.2 m of its local
#: target -- `planner_manager.cpp:55`, whose own patch comment calls it "a DELIBERATE refusal to plan when the
#: replan's start is within 0.2 m of its local target -- i.e. the last metre of every arrival".  So an ego
#: planner cannot place the vehicle closer than ~0.2 m by design, and a contract that demands 0.060 m waits
#: for a precision nobody is producing any more.  MEASURED on EGO2 and EGO3: the vehicle reached 0.12 m from
#: waypoint 1, the refusal line fired **148 780 times**, and the tour never advanced.
#: NOT a silent loosening: this is the ego planner's OWN floor, the rectangle keeps 0.060 m, and the
#: consequence -- ego cannot meet the 0.060 m corner-arrival rows without a patch to that line -- is recorded
#: rather than hidden.
EGO_ARRIVE_M="${EGO_ARRIVE_M:-0.30}"
#: HOW SLOW THE VEHICLE MUST BE FOR THE SHELL TO CALL IT "ARRIVED" (m/s), and how long the tour then holds.
#: Both are the contract shell's own numbers (`--settle-mps`, `--hold-s`).  Exposed on 2026-10-16 because the
#: owner's requirement is now a TIME PER WAYPOINT (<=6 s), and MEASURED in Flight E the two of them together
#: cost **9-10 s of standstill at every corner**: 5.0 s of dwell (`RECT_HOLD_S`) plus 4-5 s of waiting for a
#: 0.08 m/s settle -- on a 6 m leg that is more than the flight itself.  `--settle-mps` 0.30 lets the tour
#: advance while the aircraft is still rolling, which is what "fly through the waypoint" means.
EGO_SETTLE_MPS="${EGO_SETTLE_MPS:-0.08}"
#: THE VERTICAL TOLERANCE OF THE SHELL'S ARRIVAL GATE (metres) and an UPPER CLAMP ON THE FORWARDED ALTITUDE.
#: Both exist because of ONE measurement (Flight F, 2026-10-17): the aircraft arrived at corner 1 **84.4 mm away
#: horizontally and 0.834 m high**, the 3-D gate therefore never opened in 21 s, and the reference itself had
#: meanwhile run to z = 3.25 m for a 1.884 m tour and taken the aircraft with it (truth 3.32 m, 0.44 m under the
#: supervisor's 3.00 m climb guard).  The tour is a horizontal one, so the arrival test is horizontal with a
#: vertical tolerance, and `EGO_REF_Z_CLAMP_M` stops a reference that climbs away from the mission altitude.
EGO_ARRIVE_Z_M="${EGO_ARRIVE_Z_M:-1.00}"
EGO_REF_Z_CLAMP_M="${EGO_REF_Z_CLAMP_M:-0.0}"
#: THE FORWARDED REFERENCE'S OWN SPEED LIMIT (m/s; 0 = off).  `EGO_REF_CLAMP_M` bounds how FAR a re-plan's step
#: may land from the vehicle; this bounds how FAST the point may move, which is what the controller feels.
#: MEASURED Flight H at 3 m/s: tracking error p50 0.139 m but p90 0.532 and max 0.840 m -- the transients, and
#: they are exactly where a 0.10-0.20 m real margin disappears.  A legitimate 3 m/s reference advances 0.03 m
#: per 100 Hz message, so a 6 m/s cap touches nothing but the steps.
EGO_REF_SLEW_MPS="${EGO_REF_SLEW_MPS:-0.0}"
#: NO GOAL IS PUBLISHED UNTIL THE VEHICLE IS THIS HIGH (metres; 0 = off).  Route B's twin of PATCH 0008's
#: `fsm/waypoint_start_alt`; without it ego plans a climb for the whole take-off that nothing is following
#: (Flight F: goals from t=1.9 s, 87 "First 3 control points in obstacles" before the trigger at t=20.4 s).
EGO_START_ALT_M="${EGO_START_ALT_M:-0.0}"
#: THE PER-LEG WATCHDOG (seconds; 0 = off = the behaviour of every sortie up to and including Flight E).
#: `--leg-timeout-s` is when the shell starts SAYING so, once a second; `--leg-abort-s` is when it STOPS
#: PUBLISHING, which trips the supervisor's own 3.0 s plan-silence rule and ends the sortie loudly.  It exists
#: because Flight E spent **374.3 s frozen at waypoint 7** and nothing in any log said why: the FSM was stuck in
#: GEN_NEW_TRAJ ("First 3 control points in obstacles", the odometry start inside the inflated map) while
#: `traj_server` kept publishing a held point, so the stream looked alive and the silence rule never fired.
EGO_LEG_TIMEOUT_S="${EGO_LEG_TIMEOUT_S:-0.0}"
EGO_LEG_ABORT_S="${EGO_LEG_ABORT_S:-0.0}"
#: WHETHER THE EGO STACK IS STARTED, decided HERE so that the banner below can tell the truth about it.
#: `PLANNER=ego` implies it; `EGO_STACK=0` overrides it for an ego that is already running elsewhere.
EGO_STACK="${EGO_STACK:-$([ "$PLANNER" = "ego" ] && echo 1 || echo 0)}"
#: WHOSE SAMPLER PRODUCES `/plan/cmd` WHEN THE EGO STACK RUNS (the owner's call, round 76).  Defined HERE, in the
#: parameter block, so the planner's argv and the banner below both read the SAME decision.
#:   * `adapter` -- `planner/ego_source.py` samples ego's B-spline itself (every sortie up to round 75);
#:   * `shim`    -- ROUTE B: ego's OWN `traj_server` samples it -- the reference ego wrote, position AND velocity
#:                  AND acceleration -- and `planner/ego_contract_shim.py` owns only the tour and rewrites
#:                  `trajectory_id`/`trajectory_flag` on the way through.  It also brings two processes:
#:                  `traj_server` (`time_forward:=1.0`; its own default is -1.0, a yaw look-ahead into the PAST)
#:                  and `planner/fcu_pose_feed.py`, which feeds ego the FCU'S OWN estimate (W -> L) so that the
#:                  planner and the controller close their loops on the same state instead of on the raw LIO.
#: DEFAULT IS `shim` SINCE ROUND 83, because that is the route the owner asked for and the shell is now
#: verified: bench-checked end to end (round 76: 100 Hz, verbatim forwarding, ids/flags; round 82: the
#: post-COMPLETED hold, a pinned point with |v| = 0) and flown in `DIAG34`.  The `adapter` default existed
#: only while the shim was unverified; `EGO_CONTRACT=adapter` still selects it.
EGO_CONTRACT="${EGO_CONTRACT:-shim}"
case "$EGO_CONTRACT" in
    adapter|shim) ;;
    *) echo "EGO_CONTRACT='$EGO_CONTRACT' -- use adapter or shim" >&2; exit 4 ;;
esac
EGO_CLOUD_TOPIC="${EGO_CLOUD_TOPIC:-/ego/cloud}"
#: HOW MANY SECONDS OF SCANS THE RELAY UNIONS INTO EACH MESSAGE (0 = one scan, the behaviour every round up to
#: and including 8 was flown with).  This is the fix round 8's measurement asks for: ego's map is a single-scan
#: shell that held 0 % of the pillar the vehicle was inside (`docs/metrics/diagonal-map-truth-20261016.txt`), so
#: the memory the planner assumes it has is restored here, on our side.  ~3 s is what the geometry asks for: the
#: measured closing speed on the diagonal is ~0.4 m/s, so 3 s spans the whole approach the map forgets.
EGO_CLOUD_WINDOW="${EGO_CLOUD_WINDOW:-0.0}"
#: THE RELAY'S DECIMATION CELL (m).  0.10 has been the flown value since the relay existed, and it is HALF of
#: the measured reason the 6 x 8 m rectangle cannot be flown straight: `_stage/map-inflation.py` on Flight I's
#: own DIAG/2 slices (inflation 0.0, so NOTHING is added by ego) measures the map's obstacle at **0.720 m from a
#: 0.300 m pillar's axis -- 0.420 m past the true surface**, which CLOSES the 1.414 m corridor between two
#: diagonal pillars (it computes to -0.03 m) and leaves the planner no route but AROUND the whole field -- which
#: is exactly what Flight I's top view shows.  Of that 0.420: the map's own 0.1 m cell is ~0.05-0.10, this
#: decimation is up to 0.087, and the REST is the union below.  0.03 keeps the points where the lidar put them.
EGO_CLOUD_VOXEL="${EGO_CLOUD_VOXEL:-0.10}"
#: RADIUS OF THE SELF-BLIND BUBBLE the relay clears around the vehicle (0 = off, the behaviour of every round
#: up to 13).  ego's hard collision check treats its OWN POSITION inside the inflated map as fatal
#: (`bspline_optimizer.cpp:1341-1345`), and MEASURED with two cells of inflation that produced 112 729
#: 'the drone is in obstacle' and a hung sortie (DIAG13).  Two cells is the only way to give the C2 row its
#: 0.20 m footprint, so this is its enabler, not a convenience: with the bubble, marked cells are at least this
#: far away and the map cannot mark the machine itself.
EGO_SELF_BLIND="${EGO_SELF_BLIND:-0.0}"
#: HOW FAR THE REFERENCE MAY BE FROM THE VEHICLE, metres (0 = off = the behaviour of every sortie before
#: 2026-10-16).  This is the "continuity at the source" fix, and it lives in the contract shell because that is
#: the only process in the chain that HAS the vehicle's state: `traj_server` subscribes to nothing but the spline,
#: and the FSM's own check tests the PLAN, never the vehicle.  MEASURED (`docs/metrics/why-collide-20261016.txt`):
#: `t_cur = now - start_time` restarts at every 1 Hz replan, so the sampled point jumps -- 14 of 16 replans step
#: more than 0.05 m, median 0.281 m, max 0.927 m on the wire -- and at the moment the aircraft entered a pillar
#: the reference was 0.623 m ahead of it while the plan's own margin there was 5-17 cm.  A position reference is a
#: place to go, not a schedule: capping its distance from the vehicle costs the mission nothing and removes the
#: steps the controller otherwise chases.
EGO_REF_CLAMP_M="${EGO_REF_CLAMP_M:-0.0}"
case "$PLANNER" in
    rectangle)
        PLANNER_FILE="$REPO/planner/rectangle_source.py"
        PLANNER_ARGS="--world $WORLD $TRAJ_CALIB_ARG --hold-s $RECT_HOLD_S $VERTICAL_ARG $SHAPING_ARG ${CMD_ARGS:-}"
        PLANNER_WHAT="the Mission A tour on the rectangle (our own plan box)"
        ;;
    ego)
        PLANNER_FILE="$REPO/planner/ego_source.py"
        # THE ARRIVAL GATE IS PASSED EXPLICITLY, and it is looser than the rectangle's 0.060 m on purpose:
        # an ego planner stops planning 0.2 m out (`planner_manager.cpp:55`), so 0.060 m can never be met.
        # See `EGO_ARRIVE_M` in the parameter block.
        # PATCH REFERENCE-TELEMETRY/1 (round 73): ego runs also write a PER-SAMPLE trace of /plan/cmd, because
        # round 72 could not tell "the planner went quiet and the adapter held" from "the sampler clamped at the
        # spline's end" -- the counters that separate them were printed only at exit, and a killed stack never
        # gets there.  `EGO_SAMPLE_CSV` turns it off by setting it empty; the rectangle arm is untouched.
        EGO_SAMPLE_CSV="${EGO_SAMPLE_CSV-$OUT/ego_samples.csv}"
        EGO_SAMPLE_ARG=""
        [ -n "$EGO_SAMPLE_CSV" ] && EGO_SAMPLE_ARG="--sample-csv $EGO_SAMPLE_CSV"
        if [ "${EGO_CONTRACT:-adapter}" = "shim" ]; then
            # ROUTE B (the owner's call): ego's OWN `traj_server` produces the reference; this process owns only
            # the tour and the contract's `trajectory_id` / `trajectory_flag` (see the knob block above).
            PLANNER_FILE="$REPO/planner/ego_contract_shim.py"
            PLANNER_ARGS="--world $WORLD $TRAJ_CALIB_ARG --hold-s $RECT_HOLD_S --arrive-m $EGO_ARRIVE_M \
--settle-mps $EGO_SETTLE_MPS --leg-timeout-s $EGO_LEG_TIMEOUT_S --leg-abort-s $EGO_LEG_ABORT_S \
--arrive-z-m $EGO_ARRIVE_Z_M --start-alt-m $EGO_START_ALT_M --ref-z-clamp-m $EGO_REF_Z_CLAMP_M \
--ref-slew-mps $EGO_REF_SLEW_MPS \
--odom-topic /ego/odom_fcu --ref-clamp-m $EGO_REF_CLAMP_M --trace-csv $OUT/shim_trace.csv ${CMD_ARGS:-}"
            PLANNER_WHAT="ego's OWN traj_server samples the plan (p+v+a) + the contract shell (tour, ids, COMPLETED)"
            PLANNER_TOPIC="/plan/cmd"
        else
            PLANNER_ARGS="--world $WORLD $TRAJ_CALIB_ARG --hold-s $RECT_HOLD_S --arrive-m $EGO_ARRIVE_M $EGO_SAMPLE_ARG ${CMD_ARGS:-}"
            PLANNER_WHAT="ego-planner-swarm behind the /plan/cmd contract (adapter; arrival gate ${EGO_ARRIVE_M} m = the planner's own floor)"
        fi
        ;;
    */*|*.py)
        PLANNER_FILE="$PLANNER"
        PLANNER_ARGS="${CMD_ARGS:-}"
        PLANNER_WHAT="a planner named by path -- the harness knows nothing about its internals"
        ;;
    *)
        echo "unknown PLANNER='$PLANNER' -- use rectangle, ego, or a path to a .py" >&2
        exit 4
        ;;
esac
if [ ! -f "$PLANNER_FILE" ]; then
    # the NAME AS GIVEN, not an internal label: a caller who typed a path must read that path back
    echo "PLANNER='$PLANNER': no file at $PLANNER_FILE" >&2
    exit 4
fi
# only now, so the message above can quote what the caller actually typed
case "$PLANNER" in
    rectangle|ego) ;;
    *) PLANNER="custom" ;;
esac
# ------------------------------------------------------------------ WHAT THE WIRE CARRIES (owner's call, round 76)
# `--fields` selects which of position / velocity / acceleration the FCU receives.  The bridge has always
# supported all three (`bridge/mavlink_impl.py:455-469`, masks POS_YAW / POS_VEL_YAW / POS_VEL_ACC_YAW) and the
# FCU uses them -- this ArduPilot has `ModeGuided::set_pos_vel_accel_NED_m(pos, vel, accel, ...)`
# (`ArduCopter/mode_guided.cpp:618`) and `AC_PosControl::input_pos_vel_accel_NE_m(pos, vel, accel, ...)`
# (`AC_AttitudeControl/AC_PosControl.h:163`), i.e. the acceleration is a controller INPUT, not a field that is
# accepted and dropped.  The default was `pos`, which is why every sortie so far fed the FCU a position with no
# feed-forward -- measured cost, round 72: the aircraft ran ~1 m off its reference at the 95th percentile.
#
# WHY IT IS NOT GLOBAL: the rectangle planner publishes ZERO velocity and acceleration by construction
# (`planner/rectangle_source.py` has no velocity model), so `--fields posvelacc` on THAT arm would order the FCU
# to hold v = 0, a = 0 while chasing a position -- a brake.  The frozen mission's behaviour must not change, so
# ego gets feed-forward and the rectangle keeps pos, both overridable:
#     BRIDGE_FIELDS=posvelacc  bash _stage/diagonal-fly.sh ego <TAG>      (the script lives in _stage/, not
#                                                                        in src/tools/ -- corrected round 83)
#
# AND WHY EGO GETS `posvel` RATHER THAN `posvelacc`, WHICH IS A MEASUREMENT AND NOT A PREFERENCE.  MEASURED
# 2026-10-16 on `DIAG33-routeb`, the first sortie whose wire carried pva: the aircraft left the 6.60 m geofence
# and the supervisor aborted BY NAME ("ABORT: geofence: truth at [-5.65 -6.61]") 32.7 s in.  The cause is in the
# reference's own numbers (`_stage/s9-pv-vs-pva.py`, 2091 consecutive messages of ego's traj_server):
#     position      step: median 0.0103 m, 99th 0.029 m, BUT max 3.40 m   (the age rule switches trajectories)
#     velocity      step: median 0.023 m/s, 99th 0.18 m/s   -- 0.9% of steps above 0.5 m/s
#     acceleration  step: median 0.037, 99th 1.58, MAX 22.2 m/s^2  -- 20.2% of steps above 0.1 m/s^2
# i.e. the velocity is a feed-forward a controller can use and the acceleration is not: the second derivative is
# the most sensitive to sampling two different splines in consecutive messages.  Feeding ±20 m/s^2 with
# alternating sign is what threw the aircraft out of the arena.  The acceleration goes back on the wire when the
# SAMPLER is continuous (the `traj_server` patch the owner has been asked about), not before.
if [ "${PLANNER:-}" = "ego" ]; then
    BRIDGE_FIELDS="${BRIDGE_FIELDS:-posvel}"
else
    BRIDGE_FIELDS="${BRIDGE_FIELDS:-pos}"
fi
case "$BRIDGE_FIELDS" in
    pos|posvel|posvelacc) ;;
    *) echo "BRIDGE_FIELDS='$BRIDGE_FIELDS' -- use pos, posvel or posvelacc" >&2; exit 4 ;;
esac
BRIDGE_ARGS="--fields $BRIDGE_FIELDS ${BRIDGE_ARGS:-}"
# THE FCU'S OWN ESTIMATE ON A TOPIC, but only when the Route B contract is in use: `fcu_pose_feed` converts it
# into L for ego.  NOT the SITL truth tap -- see `planner/fcu_pose_feed.py`'s header (round 79's correction).
if [ "$EGO_CONTRACT" = "shim" ]; then
    BRIDGE_ARGS="--fcu-odom-topic /fcu/odom ${BRIDGE_ARGS:-}"
fi

# A KNOB THAT CANNOT BE CHECKED BREAKS SILENTLY, and this one did.  MEASURED 2026-10-16 while writing it:
# the first version of the block above stood ~40 lines EARLIER, where `$VERTICAL_ARG` and `$TRAJ_CALIB_ARG`
# did not exist yet, so under `set -u` the harness died on the first unbound variable -- `PLANNER=rectangle`,
# the DEFAULT, could not start at all, and nothing in the tree would have noticed until a sortie failed to
# launch.  `tools/test-planner-knob.py` now runs this script with PLANNER_DRY_RUN=1 for every value and
# asserts the argv below, and the dry run exits BEFORE the sortie lock is taken so it cannot leave one.
if [ "${PLANNER_DRY_RUN:-0}" = "1" ]; then
    echo "PLANNER_DRY_RUN=1 -- nothing started, no lock taken, no SITL, no ROS"
    echo "   planner        : $PLANNER -- $PLANNER_WHAT"
    echo "   file           : $PLANNER_FILE"
    echo "   topic          : $PLANNER_TOPIC (quadrotor_msgs/PositionCommand, frame lio_world)"
    echo "   argv           : $PLANNER_ARGS"
    echo "   wire fields    : $BRIDGE_FIELDS (position, velocity, acceleration the FCU receives)"
    echo "   what would run : python3 $PLANNER_FILE $PLANNER_ARGS"
    echo "   ego stack      : $EGO_STACK (contract $EGO_CONTRACT; cloud_relay /lio/cloud_world -> $EGO_CLOUD_TOPIC; ego max_vel $EGO_MAX_VEL max_acc $EGO_MAX_ACC; inflation $EGO_INFLATION; replan $EGO_REPLAN_S s; cloud window ${EGO_CLOUD_WINDOW}s, self-blind ${EGO_SELF_BLIND}m)"
    if [ "$EGO_STACK" = "1" ]; then
        echo "                    would also run: ros2 launch ego_planner ego_on_lio.launch.py"
        echo "                                   python3 $REPO/planner/cloud_relay.py --out $EGO_CLOUD_TOPIC --window $EGO_CLOUD_WINDOW --self-blind $EGO_SELF_BLIND"
        if [ "$EGO_CONTRACT" = "shim" ]; then
            echo "                    ROUTE B: ros2 run ego_planner traj_server -p traj_server/time_forward:=1.0"
            echo "                             python3 $REPO/planner/fcu_pose_feed.py --in /fcu/odom --out /ego/odom_fcu  (the FCU's OWN estimate, #32)"
            echo "                             the bridge publishes --fcu-odom-topic /fcu/odom for it"
        fi
    fi
    exit 0
fi

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1

# ------------------------------------------------------------------ PROCESS OWNERSHIP (audit F2)
# NO UNANCHORED pkill, NO -9 BY PROGRAM NAME, AND NO SILENT TAKE-OVER.  The block that used to stand here
# killed `sim/lidar_imu.py`, `super_lio_node`, `mission_fsm.py` ... by name -- i.e. it killed whatever else
# happened to be running those programs, which in this project has ALREADY killed another agent's in-flight
# sortie (`pkill -9 -f arducopter`), and it never touched the PLANNER chain, so a stale adapter could go on
# publishing /planner/traj into the next sortie while that sortie's FSM overwrote its own plan without
# asking.  The rule now:
#   * THIS script is started with `setsid` and is therefore the leader of its own process group; the group is
#     the sortie's identity, and teardown kills the GROUP, never a name;
#   * a LOCK names the live sortie.  If its leader is alive the new sortie REFUSES TO START (exit 3) with the
#     other tag in the message -- one flight at a time, enforced by the harness rather than by good manners;
#   * if the leader is GONE, its group's leftovers are killed by GROUP ID (precise) and the run continues.
LOCK=/tmp/lio_avoid_sortie.lock
PGID_SELF="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"
if [ -e "$LOCK" ]; then
    OLD="$(cat "$LOCK" 2>/dev/null || true)"
    OLD_PID="${OLD%%:*}"
    OLD_TAG="${OLD##*:}"
    if [ -n "$OLD_PID" ] && kill -0 "$OLD_PID" 2>/dev/null; then
        echo "REFUSING TO START: a sortie is already live -- pid $OLD_PID, tag '$OLD_TAG' ($LOCK)." >&2
        echo "  ONE FLIGHT AT A TIME is the project's rule, and this refusal is how it is enforced:" >&2
        echo "  wait for that sortie to finish, or stop it deliberately.  Nothing has been killed." >&2
        exit 3
    fi
    if [ -n "$OLD_PID" ]; then
        echo "clearing the leftovers of the DEAD sortie '$OLD_TAG' (process group $OLD_PID):"
        pgrep -g "$OLD_PID" 2>/dev/null | sed 's/^/   pid /' || true
        kill -9 -- "-$OLD_PID" 2>/dev/null || true
        sleep 1
    fi
fi
echo "$$:$TAG" > "$LOCK"
echo "sortie lock : $LOCK ($TAG, pid $$, process group ${PGID_SELF:-?})"
sleep 1

echo "== revisions under test =="
# THIS REPO'S OWN REVISION FIRST.  MEASURED 2026-10-15: a sortie's log named the ArduPilot revision, the
# LIO's revision and every parameter file, but NOT the commit that produced the PLAN box and the BRIDGE --
# so "which code flew" could only be reconstructed from the wall clock, and a fix that was in the tree but
# not in the commit looked identical to one that was.  A dirty tree is stated here, not hidden.
echo "   this repo : $(git -C "$REPO" rev-parse --short HEAD) $(git -C "$REPO" log -1 --format=%s | cut -c1-58)$(mt=$(git -C "$REPO" status --porcelain --untracked-files=no 2>/dev/null | wc -l); ut=$(git -C "$REPO" status --porcelain --untracked-files=all 2>/dev/null | grep -c '^??'); if [ "${mt:-0}" -gt 0 ]; then echo "  *** TREE DIRTY: $mt tracked path(s) modified ***"; else echo "  (tracked tree clean${ut:+, $ut untracked path(s)})"; fi)"
echo "   ardupilot : $(git -C "$AP" describe --tags --always) $(git -C "$AP" rev-parse --short HEAD) (tap applied: $(git -C "$AP" --no-pager diff --stat | tail -1 | tr -s ' '))"
echo "   super_lio : $(git -C /home/fanxin/lio_avoid/src/Super-LIO rev-parse --short HEAD)  liblio $(md5sum "$WS/install/super_lio/lib/liblio.so" | cut -c1-16)"
echo "   config    : lio/config/livox_360.yaml + apm_lio_only,sitl_basics,${ALT_MODE}_posz,$MISSION_NAME,$EKF_NAME"
echo "   override  : $EXTRA_NAME   (appended LAST to --defaults: the last value wins)"
echo "   planner   : $PLANNER -- $PLANNER_WHAT"
echo "               $PLANNER_FILE  ->  $PLANNER_TOPIC (quadrotor_msgs/PositionCommand, frame lio_world)"
echo "               (the BRIDGE and the SUPERVISOR do not know which planner this is: they read the topic)"
if [ "${EGO_STACK:-0}" = "1" ]; then
    echo "   ego stack : ON -- ego_planner_node (src/planner/ego_on_lio.launch.py) + cloud_relay"
    echo "               /lio/cloud_world -> ${EGO_CLOUD_TOPIC:-/ego/cloud} at 0.10 m; ego max_vel ${EGO_MAX_VEL:-2.0} max_acc ${EGO_MAX_ACC:-3.0} (SOFT); inflation ${EGO_INFLATION:-0.099} m"
fi
echo "   world     : $WORLD  ($CORNERS)"
echo "   airframe  : sim model '$SITL_MODEL' against FRAME_TYPE 1 (X) -- the two must agree (see SITL_MODEL above)"
printf 'model=%s\nfirmware=FRAME_CLASS 1 / FRAME_TYPE 1 (X mixer)\nrule=the SITL model names the motor geometry; it must match FRAME_TYPE\n' \
    "$SITL_MODEL" > "$OUT/airframe.txt"
echo "   sensor    : the SAME world file ($REPO/sim/lidar_imu.py --world \"$WORLD\")"
echo "   geofence  : $GEOFENCE_M m, take-off altitude $TAKEOFF_ALT m"
echo "   dwell     : ${RECT_HOLD_S} s at every corner -- from ${RECT_HOLD_FROM}; the world file's"
echo "               mission.hold_s is ${WORLD_HOLD_S} and is deliberately NOT what a default sortie flies"
if [ "${EGO_CONTRACT:-shim}" = "shim" ]; then
    echo "   arrival   : the shell's gate is d_xy <= ${EGO_ARRIVE_M} m, |dz| <= ${EGO_ARRIVE_Z_M} m AND |v| < ${EGO_SETTLE_MPS} m/s, then ${RECT_HOLD_S} s"
    echo "   altitude  : the forwarded z is capped at tour_z + ${EGO_REF_Z_CLAMP_M} m (0 = off); goals withheld below ${EGO_START_ALT_M} m"
    echo "   watchdog  : a leg that has not advanced in ${EGO_LEG_TIMEOUT_S} s is announced; at ${EGO_LEG_ABORT_S} s the stream"
    echo "               STOPS so the supervisor's plan-silence rule ends the sortie (0 = off)"
fi
echo "   take-off  : ${TAKEOFF_PROFILE}"
# THE CLIMB-LEG LINES DESCRIBE THE PLANNER THAT IS ACTUALLY RUNNING, not the one the take-off profile was
# written for.  MEASURED while preparing the first `PLANNER=ego` sortie: `VERTICAL_ARG` is built for the
# rectangle's argv and the ego argv does not carry it, so this used to print "the plan's first target is a
# VERTICAL leg to 2.50 m" for a tour whose first target is corner 1 -- a banner that lies about the flight is
# worse than no banner, because the banner is what a pilot reads.
case "$PLANNER_ARGS" in
    *--vertical-first-m*)
        echo "               the plan's first target is a VERTICAL leg to ${TAKEOFF_ALT} m, held ${VERTICAL_HOLD_S} s"
        echo "               (that hold IS the A4 window, which is what keeps A4 comparable with docs/metrics/)"
        ;;
    *)
        if awk "BEGIN{exit !($TAKEOFF_COMMAND_M != $TAKEOFF_ALT)}"; then
            echo "               NO vertical leg in $PLANNER's argv: the first setpoint is the planner's own"
            echo "               first target ($PLANNER_WHAT)"
            echo "               and the aircraft climbs IN TRANSIT -- the A4 window is then a climb, not a hover"
        fi
        ;;
esac
echo "   run dir   : $OUT"
echo

cd "$OUT" || exit 1
# TWO ENDPOINTS SINCE F2, and that is a capability this repository proved on the bench BEFORE using it:
# `tools/test-second-endpoint.py` (run by `_stage/f2-endpoint-bench.sh`) connects two clients with their own
# 511 rate requests and shows no interference, one vehicle, and a command on one visible as state on the
# other.  serial0 (5760) is the BRIDGE's -- setpoints 84 and the estimator feed 331; serial1 (5761) is the
# SUPERVISOR's -- every class-76 command.  One writer per message class is the rule that makes this safe
# without any runtime arbitration (`_stage/refactor-bridge-plan.md` section 4.2).
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model "$SITL_MODEL" --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 --serial1 tcp:5761 \
    --defaults "$ROOT/params/apm_lio_only.parm,$ROOT/params/sitl_basics.parm,$ALT_PARM,$MISSION_PARM$EKF_ARG$EXTRA_ARG" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null

# STOP THE FLIGHT STACK, AND WAIT FOR THE TWO PROCESSES THAT *WRITE EVIDENCE* TO FINISH WRITING IT.
#
# THE ORDERING IS THE ROW (MEASURED 2026-10-15).  `tools/metrics.py` used to run while `truth_rx.py` was
# still in its receive loop and while SITL was still appending the FCU's BIN, so metrics.txt was computed
# against inputs that were still growing: the run-summary sample count and the whole A4 block were quoted
# from a TRUNCATED tap and a SHORT BIN.  Every acceptance row reproduced anyway -- A1/A2/B1/B2/B3/C1/C2 are
# computed over the route window and the missing tail is after it -- but those two lines are read by a
# human, and "the evidence file does not agree with the evidence" is the failure mode this project treats
# as the worst one.
#
# MEASURED, on 7 committed sorties: metrics.txt quotes 2.9-3.6 s less tap than the file now holds (RECT9
# 90294 vs its own truth_report.txt's "of 94020 seen"; RECT1 62516 vs 65092).  A per-row flush in the
# recorder is NECESSARY BUT NOT SUFFICIENT, and that was measured too: with `buffering=1` in place, a
# passive watcher on a real sortie caught metrics.txt being written 1.1 s BEFORE the tap stopped growing
# (RECT13).  The recorder stops when SITL stops sending, so the stop must come first.  RECT14 proves the
# result: summary 61289 = truth_report's "of 61289 seen" = the closed tap's own row count, and metrics.txt
# mtime AFTER tap.csv instead of before it (this function's A4 row needed the same treatment for the BIN).
stop_stack() {
    # THE EGO STACK IS STOPPED FIRST when it exists, because `ros2 launch` owns a child of its own and the
    # planner it feeds is the process that must not be left publishing into a dead stack.
    for p in "${EGO_LAUNCH_PID:-}" "${EGO_RELAY_PID:-}" "${EGO_TRAJ_SERVER_PID:-}" "${EGO_FCU_POSE_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    for p in "${FSM_PID:-}" "${BRIDGE_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RECT_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$REC_PID" 2>/dev/null
    kill "$RECC_PID" 2>/dev/null
    # SITL is stopped BEFORE the tap recorder is waited for, because the recorder's own stop condition is
    # "SITL has stopped sending"; and its BIN must be closed before A4 reads it, which is why it is waited
    # for in its own right.  5 s is ~5x what SITL's clean exit has been measured to take; the SIGKILL below
    # is the backstop, not the plan.
    if [ -n "${SITL:-}" ]; then
        kill "$SITL" 2>/dev/null
        for _ in $(seq 1 50); do
            kill -0 "$SITL" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$SITL" 2>/dev/null; then
            kill -9 "$SITL" 2>/dev/null
            echo "WARNING: SITL did not stop within 5 s and was killed -- the FCU's BIN may be short"
        fi
    fi
    # SIGTERM is handled gracefully by the recorder (`_install_stop_handler` sets a flag; the loop then
    # closes the CSV and writes truth_report.txt), so no sample is lost to a signal -- and if it does not
    # stop, this says so instead of silently truncating the tap.  MEASURED (s26-stop-ab): with the silent-tap
    # stop check in sim/truth_rx.py the recorder leaves 0.22 s after SIGTERM and writes its report; without
    # it, it was still running 15 s later -- which is why the check had to be fixed in the same round as
    # this ordering, and why the two are one commit.
    if [ -n "${RX_PID:-}" ]; then
        kill "$RX_PID" 2>/dev/null
        for _ in $(seq 1 50); do
            kill -0 "$RX_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$RX_PID" 2>/dev/null; then
            kill -9 "$RX_PID" 2>/dev/null
            echo "WARNING: the truth recorder did not stop within 5 s and was killed -- tap.csv may be short"
        else
            echo "truth recorder and SITL stopped; tap.csv and the FCU's BIN are complete"
        fi
    fi
    # anything that ignored SIGTERM gets SIGKILL; `cleanup` repeats this on the way out, so nothing is left
    # behind whether the script is exiting normally or not.
    for p in "${EGO_LAUNCH_PID:-}" "${EGO_RELAY_PID:-}" \
             "${FSM_PID:-}" "${BRIDGE_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${SITL:-}" "${RX_PID:-}" \
             "${REC_PID:-}" "${RECC_PID:-}" "${RECT_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    return 0
}

cleanup() {
    stop_stack
    # F2: AND ANYTHING ELSE THIS SORTIE LEFT IN ITS OWN PROCESS GROUP.  `ap6-fly.sh` starts this script with
    # `setsid`, so `pgrep -g $$` is exactly "the processes this sortie started" -- no name matching, and no
    # reach into another sortie's group.  A killed sortie therefore cannot leave an adapter behind publishing
    # plans into the next one.
    for p in $(pgrep -g "$$" 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        kill -9 "$p" 2>/dev/null
    done
    if [ -e "$LOCK" ] && [ "$(cat "$LOCK" 2>/dev/null || true)" = "$$:$TAG" ]; then
        rm -f "$LOCK"
    fi
}
trap cleanup EXIT

for i in $(seq 1 24); do
    grep -q TruthTap "$OUT/sitl.log" 2>/dev/null && break
    kill -0 "$SITL" 2>/dev/null || { echo "SITL died:"; tail -12 "$OUT/sitl.log"; exit 1; }
    sleep 0.5
done
grep -q TruthTap "$OUT/sitl.log" || { echo "tap never reported"; exit 1; }
echo "SITL stepping"

python3 "$REPO/sim/truth_rx.py" --ros --moving --duration 0 --port $TAP --fanout $FAN \
    --fanout-decimate 6 --csv "$OUT/tap.csv" --save "$OUT/truth_report.txt" \
    > "$OUT/truth_rx.log" 2>&1 &
RX_PID=$!
sleep 2
python3 "$REPO/sim/lidar_imu.py" --port $FAN --world "$WORLD" --duration 0 > "$OUT/lidar_imu.log" 2>&1 &
SENSOR_PID=$!
sleep 2
ros2 run super_lio super_lio_node --ros-args --params-file "$REPO/lio/config/livox_360.yaml" \
    > "$OUT/lio.log" 2>&1 &
LIO_PID=$!
sleep 5
kill -0 "$LIO_PID" 2>/dev/null || { echo "the LIO died:"; tail -20 "$OUT/lio.log"; exit 1; }

python3 "$REPO/tools/record-odom.py" --topic /lio/robo/odom --csv "$OUT/lio.csv" \
    --seconds 0 > "$OUT/rec.log" 2>&1 &
REC_PID=$!
# The LIO's own registered map is recorded next to the odometry, so a flight's clearance number can be
# checked against the map the trajectory was built from rather than taken on trust.
python3 "$REPO/tools/record-cloud.py" --topic /lio/cloud_world --csv "$OUT/cloud.csv" \
    --decimate 40 --scans 3 > "$OUT/reccloud.log" 2>&1 &
RECC_PID=$!
# WHICH NAVIGATION SOURCE, AND THEREFORE WHICH CALIBRATION -- MOVED.  `NAV_SOURCE`, `CALIB`,
# `CALIB_ARG` and `TRAJ_CALIB_ARG` are decided in the parameter block near the top of this script, before
# anything is started or locked, because the planner's own argv is built there and needs the same
# `--calib-file`.  Nothing here re-decides them; this comment stays so the next reader does not look for
# the decision where it used to be.
# THE PLAN, recorded next to the truth.  Without it "the gate cannot see the truth" is an argument, not a
# measurement: MEASURED 2026-10-14 the truth spent 55.9 % of a 12-stop flight inside the 0.20 m contract
# while every published path had been verified clear, and there was no record of those paths to compare
# against.  tools/record-traj.py writes every plan message plus its own minimum clearance -- and since
# 2026-10-15 it takes the SAME --calib-file as the FSM and the producer, because the plan arrives in L and
# the world file is in W: without it `clr_m` read -0.2972 m for a plan that is +0.6000 m clear.
#
# F1 (2026-10-16): THE TOPIC MOVED WITH THE INTERFACE, IN THE SAME CHANGE.  The plan box no longer
# publishes `nav_msgs/Path`; its output is the command stream on /plan/cmd, and this recorder now reads
# that (`--msg position_cmd`).  Design risk R5 is exactly this line: had the recorder stayed on the Path,
# every run would have lost its plan-clearance column and nobody would have noticed until an audit.
# ... and `traj.csv.cmd.csv` beside it: EVERY message, no de-duplication, so the REFERENCE's own
# smoothness is measurable at the rate it is published (the de-duplicated points file holds one
# point per distinct command, which is what the Path-era readers need and is NOT enough to answer
# "is the reference wobbling, or is the aircraft not tracking it?").
python3 "$REPO/tools/record-traj.py" --topic /plan/cmd --msg position_cmd --csv "$OUT/traj.csv" \
    --world "$WORLD" $TRAJ_CALIB_ARG > "$OUT/rectraj.log" 2>&1 &
RECT_PID=$!
sleep 3

echo "== flying =="
# THE EGO STACK, WHEN THE PLANNER NEEDS ONE, AND IT IS TWO MORE PROCESSES.  This is the piece that made
# `PLANNER=ego` a label rather than a capability: `ego_source.py` produces the CONTRACT, but it needs ego's
# own `ego_planner_node` running and fed, and the harness's planner slot starts exactly one process.  Both
# are started here, BEFORE the plan box, so the planner is not publishing for a planner that is not up:
#
#   * `ros2 launch ego_planner ego_on_lio.launch.py` -- one node, wired to our odometry and our scan
#     (`src/planner/ego_on_lio.launch.py`; installed into the ego checkout by
#     `src/tools/install-ego-launch.py`, which also checks every parameter name against the node's own
#     declared list);
#   * `planner/cloud_relay.py` -- `/lio/cloud_world` -> `/ego/cloud`, decimated to the 0.10 m lattice ego's
#     map stores anyway.  MEASURED: our scan is 5.0 Hz at ~19480 points per scan, and ego's map writes 27
#     voxels per accepted point INLINE in its cloud callback, in a callback group it shares with the
#     odometry it needs (`plan_env/grid_map.cpp:845-887`, `:860-885`).
#
# ego's map subscription is RELIABLE (`grid_map.cpp:168-172`) and Super-LIO's publisher is RELIABLE too
# (`ROSWrapper.cpp:338-340`), so nothing here is about QoS; it is about how much work one callback does.
# `EGO_STACK` can be forced to 0 to run `PLANNER=ego` with a hand-started ego (e.g. one already running
# outside the harness).  The decision itself is made in the parameter block, so the banner states it.
EGO_LAUNCH_PID=""; EGO_RELAY_PID=""; EGO_TRAJ_SERVER_PID=""; EGO_FCU_POSE_PID=""
if [ "$EGO_STACK" = "1" ]; then
    set +u
    source "$WS/install/setup.bash" 2>/dev/null || true
    source /home/fanxin/lio_avoid/planner_ws/install/setup.bash
    set -u
    export ROS_LOCALHOST_ONLY=1
    python3 "$REPO/planner/cloud_relay.py" --in /lio/cloud_world --out "$EGO_CLOUD_TOPIC" \
        --voxel "$EGO_CLOUD_VOXEL" \
        --window "$EGO_CLOUD_WINDOW" --self-blind "$EGO_SELF_BLIND" \
        > "$OUT/ego_relay.log" 2>&1 &
    EGO_RELAY_PID=$!
    EGO_ODOM_TOPIC="/lio/robo/odom"
    EGO_ODOM_VEL_TOPIC="/lio/odom"
    if [ "$EGO_CONTRACT" = "shim" ]; then
        EGO_ODOM_TOPIC="/ego/odom_fcu"
        EGO_ODOM_VEL_TOPIC="/ego/odom_fcu"
        python3 "$REPO/planner/fcu_pose_feed.py" --calib-file "$CALIB" \
            --in /fcu/odom --out /ego/odom_fcu > "$OUT/fcu_pose_feed.log" 2>&1 &
        EGO_FCU_POSE_PID=$!
        ros2 run ego_planner traj_server --ros-args -p traj_server/time_forward:=1.0 \
            > "$OUT/traj_server.log" 2>&1 &
        EGO_TRAJ_SERVER_PID=$!
        sleep 2
        kill -0 "$EGO_TRAJ_SERVER_PID" 2>/dev/null || { echo "traj_server died:"; tail -20 "$OUT/traj_server.log"; exit 1; }
    fi
    ros2 launch ego_planner ego_on_lio.launch.py \
        odom_vel_topic:="$EGO_ODOM_VEL_TOPIC" odom_topic:="$EGO_ODOM_TOPIC" cloud_topic:="$EGO_CLOUD_TOPIC" \
        max_vel:="$EGO_MAX_VEL" max_acc:="$EGO_MAX_ACC" \
        obstacles_inflation:="$EGO_INFLATION" thresh_replan_time:="$EGO_REPLAN_S" \
        > "$OUT/ego_planner.log" 2>&1 &
    EGO_LAUNCH_PID=$!
    echo "   ego stack : contract $EGO_CONTRACT; cloud_relay pid $EGO_RELAY_PID -> $EGO_CLOUD_TOPIC;"
    echo "               ego_planner pid $EGO_LAUNCH_PID, odom $EGO_ODOM_TOPIC (twist: $EGO_ODOM_VEL_TOPIC)"
    [ -n "$EGO_TRAJ_SERVER_PID" ] && echo "               traj_server pid $EGO_TRAJ_SERVER_PID (time_forward 1.0)"
    [ -n "$EGO_FCU_POSE_PID" ] && echo "               fcu_pose_feed pid $EGO_FCU_POSE_PID (/fcu/odom = the FCU's OWN estimate -> /ego/odom_fcu)"
    sleep 6
    kill -0 "$EGO_RELAY_PID" 2>/dev/null || { echo "the cloud relay died:"; tail -20 "$OUT/ego_relay.log"; exit 1; }
    kill -0 "$EGO_LAUNCH_PID" 2>/dev/null || { echo "the ego launch died:"; tail -20 "$OUT/ego_planner.log"; exit 1; }
fi

# THE PLAN BOX, which since F1 is ALSO THE EXECUTOR: `planner/rectangle_source.py` publishes the mission's
# five waypoints as a `quadrotor_msgs/PositionCommand` stream on /plan/cmd (100 Hz, its own wall timer),
# decides ARRIVAL + SETTLE + DWELL itself from `/lio/robo/odom`, and declares the tour complete with
# `trajectory_flag = COMPLETED`.  With PLANNER=ego this process is `planner/ego_source.py`: it asks the ego
# node for one goal at a time and samples the B-spline it gets back.
# PLANNER=$PLANNER (see the knob block): the file is chosen there and named in the banner above.
python3 "$PLANNER_FILE" $PLANNER_ARGS > "$OUT/traj.log" 2>&1 &
TRAJ_PID=$!

# THE BRIDGE, ITS OWN PROCESS AND ITS OWN ENDPOINT (serial0).  It carries the estimator feed (331) from the
# LIO and the plan's setpoints (84), and it sends NO class-76 command.  It waits for the supervisor's
# /plan/trigger before it puts a setpoint on the wire, and it takes the plan OFF the wire when the stream
# says COMPLETED or when the FCU leaves GUIDED.
# THE HEADING, PER ARM, STATED HERE RATHER THAN LEFT TO A DEFAULT.  `bridge_node.py`'s `--yaw` used to default to
# 0.0, so every setpoint carried a FIXED heading and the plan's own yaw -- which `traj_server` computes toward the
# point `time_forward_` (1.0 s) ahead along the spline, i.e. toward the next waypoint -- was discarded on every
# message (MEASURED: `wire.csv` `yaw_l` swept +-3.14 rad while `yaw_out` was 0.000000 for all 1664 rows of the
# last flight).  The default is now None = "use the plan's yaw", and the RECTANGLE arm states its 0.0 EXPLICITLY
# because that is what every mission-A sortie flew and what its acceptance numbers were measured under
# (`planner/cmd_stream.py` says so).  So: the ego arm follows its path, mission A is byte-identical to before.
if [ "$PLANNER" = "rectangle" ]; then
    YAW_ARG="--yaw 0.0"
else
    YAW_ARG=""
fi
python3 "$REPO/bridge/bridge_node.py" --mav "tcp:127.0.0.1:5760" $CALIB_ARG \
    --nav-hz 48 --wire-hz 50 $YAW_ARG ${BRIDGE_ARGS:-} \
    --setpoint-log "$OUT/setpoints.csv" --wire-log "$OUT/wire.csv" \
    > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
kill -0 "$BRIDGE_PID" 2>/dev/null || { echo "the bridge process died:"; tail -20 "$OUT/bridge.log"; exit 1; }

# THE SUPERVISOR, ITS OWN PROCESS AND ITS OWN ENDPOINT (serial1).  Preflight, arm, take-off, the go, the
# four gates, LAND and the NAMED abort.  Its inputs are the truth and the LINK, never the LIO in flight.
# It is last because its exit is the sortie's result, and because the two processes it watches must already
# be publishing when it starts.
python3 "$REPO/supervisor/supervisor.py" --mav "tcp:127.0.0.1:5761" $CALIB_ARG \
    --alt "$TAKEOFF_ALT" --takeoff-command-m "$TAKEOFF_COMMAND_M" \
    --hold 3.0 --geofence-m "$GEOFENCE_M" $SUP_TRIGGER ${SUP_ARGS:-} \
    --phase-log "$OUT/phase.csv" > "$OUT/supervisor.log" 2>&1
SUP_RC=$?
cat "$OUT/supervisor.log"
FSM_RC=$SUP_RC

# ALL THREE PRODUCER PROCESSES ARE STOPPED AND WAITED FOR BEFORE THE METRICS RUN (see stop_stack above).
# This is the whole fix: after it, tap.csv, the FCU's BIN and lio.csv are FROZEN, so the file this script
# writes and the file a reader can re-run against are the same file.
stop_stack
echo
echo "== metrics =="
BINFILE=$(ls "$OUT"/logs/*.BIN 2>/dev/null | tail -1)
# the corner reference comes from the SAME world file the trajectory was built from.  `--corners=` with
# the `=` is not style: a mission whose corners start at a negative coordinate (the 6 x 4 m square is
# centred on the pad, so they do) made argparse read "-3.0000,..." as another option and refuse to run.
python3 "$REPO/tools/metrics.py" --tap "$OUT/tap.csv" --odom "$OUT/lio.csv" \
    --phase "$OUT/phase.csv" --bin "${BINFILE:-}" --corners="$CORNERS" --world "$WORLD" \
    | tee "$OUT/metrics.txt"
MET_RC=${PIPESTATUS[0]}

echo
echo "== the FCU's own log =="
if [ -n "$BINFILE" ]; then
    python3 "$REPO/tools/bin-triage.py" "$BINFILE" | tail -25
    cp "$BINFILE" "$OUT/flight.BIN" 2>/dev/null || true
fi
echo
echo "run dir: $OUT   (supervisor rc=$SUP_RC, metrics rc=$MET_RC)"
exit $SUP_RC
