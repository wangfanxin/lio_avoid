#!/usr/bin/env bash
# Mission A, one flight: SITL + the simulated MID-360 + the LIO + the trajectory source + the FSM.
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
REPO=/mnt/d/distill_self/lio_avoid
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
if grep -q "^FRAME_TYPE[[:space:]]*1" "$REPO/params/sitl_basics.parm" 2>/dev/null; then
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
    baro) ALT_PARM="$REPO/params/phase_a_posz_baro.parm" ;;
    lio)  ALT_PARM="$REPO/params/phase_a_posz_lio.parm" ;;
    *) echo "ALT_MODE must be baro or lio, not '$ALT_MODE'" >&2; exit 1 ;;
esac
# WORLD and MISSION_PARM select the mission itself: the same pipeline flies the slow 2.0 x 1.2 m square in
# the dense forest (default) or the fast 6.0 x 4.0 m square in the sparse one, and every geometric number
# -- the trajectory, the corner reference the metrics judge against, the geofence -- comes from the world
# file through tools/world-info.py, so a reference cannot drift from what is flown (STATUS.md row 44).
WORLD="${WORLD:-$REPO/worlds/pillars.yaml}"
MISSION_PARM="${MISSION_PARM:-$REPO/params/phase_a_mission.parm}"
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
echo "   ardupilot : $(git -C "$AP" describe --tags --always) $(git -C "$AP" rev-parse --short HEAD) (tap applied: $(git -C "$AP" --no-pager diff --stat | tail -1 | tr -s ' '))"
echo "   super_lio : $(git -C /home/fanxin/lio_avoid/src/Super-LIO rev-parse --short HEAD)  liblio $(md5sum "$WS/install/super_lio/lib/liblio.so" | cut -c1-16)"
echo "   config    : lio/config/livox_360.yaml + apm_lio_only,sitl_basics,${ALT_MODE}_posz,$MISSION_NAME,$EKF_NAME"
echo "   world     : $WORLD  ($CORNERS)"
echo "   airframe  : sim model '$SITL_MODEL' against FRAME_TYPE 1 (X) -- the two must agree (see SITL_MODEL above)"
echo "   plan speed: manager/max_vel ${EGO_MAX_VEL:-<from the params file>}"
printf 'model=%s\nfirmware=FRAME_CLASS 1 / FRAME_TYPE 1 (X mixer)\nrule=the SITL model names the motor geometry; it must match FRAME_TYPE\n' \
    "$SITL_MODEL" > "$OUT/airframe.txt"
echo "   sensor    : the SAME world file ($REPO/sim/lidar_imu.py --world \"$WORLD\")"
echo "   geofence  : $GEOFENCE_M m, take-off altitude $TAKEOFF_ALT m"
echo "   run dir   : $OUT"
echo

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model "$SITL_MODEL" --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 \
    --defaults "$REPO/params/apm_lio_only.parm,$REPO/params/sitl_basics.parm,$ALT_PARM,$MISSION_PARM$EKF_ARG" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null

cleanup() {
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}" \
             "${RECC_PID:-}" "${RECT_PID:-}" "${RECB_PID:-}" "${RECM_PID:-}" "${SEQ_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL" 2>/dev/null
    sleep 1
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}" \
             "${RECC_PID:-}" "${RECT_PID:-}" "${RECB_PID:-}" "${RECM_PID:-}" "${SEQ_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    kill -9 "$SITL" 2>/dev/null
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
# The map the PLANNER sees is recorded next to the odometry, because the last Mission B link was measured
# to be the map (offline the same scenario plans at 0.498 m clearance, live at 0.308 m): tools/map-vs-world.py
# compares this cloud with the world file's cylinders and names the pillars the live map lacks.
python3 "$REPO/tools/record-cloud.py" --topic /lio/cloud_world --csv "$OUT/cloud.csv" \
    --decimate 40 --scans 3 > "$OUT/reccloud.log" 2>&1 &
RECC_PID=$!
# THE PLAN, recorded next to the truth.  Without it "the gate cannot see the truth" is an argument, not a
# measurement: MEASURED 2026-10-14 the truth spent 55.9 % of a 12-stop flight inside the 0.20 m contract
# while every published path had been verified clear, and there was no record of those paths to compare
# against.  tools/record-traj.py writes every Path message plus its own minimum clearance.
python3 "$REPO/tools/record-traj.py" --topic /planner/traj --csv "$OUT/traj.csv" \
    --world "$WORLD" > "$OUT/rectraj.log" 2>&1 &
RECT_PID=$!
# EGO'S OWN OUTPUT, recorded next to the plan that survived the gate -- the difference between the two is
# where a whole sortie can disappear.  MEASURED 2026-10-14 (sortie F1): the aircraft hovered for the 60 s its
# first stop was commanded because ego never committed a new trajectory, and nothing in the run dir could
# show that: the adapter's log had ~600 identical WITHHELD lines quoting the SAME expired plan, and ego's own
# FSM had logged `from REPLAN_TRAJ to REPLAN_TRAJ` 5031 times (10 ms tick = 50.3 s) with no timestamped
# failure line.  tools/record-bspline.py writes every planning/bspline with its start_time, duration, control
# points and traj_id, so "is ego committing anything?" is a column and not an inference.
#
# THE PLANNER WORKSPACE IS SOURCED IN A SUBSHELL, and that is not tidiness: `traj_utils` (the package that
# defines the Bspline message) is built into planner_ws, and the parent script sources only ROS and lio_ws.
# MEASURED 2026-10-14 (sorties F2/F3): without this the recorder exited with "cannot import
# traj_utils/Bspline" and bspline.csv was never created -- the instrument was missing exactly when the
# question it exists for came up.
python3 "$REPO/tools/record-bspline.py" --help > /dev/null 2>&1 || true
( set +u; source /opt/ros/humble/setup.bash; source /home/fanxin/lio_avoid/planner_ws/install/setup.bash; \
  exec python3 "$REPO/tools/record-bspline.py" --topic planning/bspline --csv "$OUT/bspline.csv" ) \
    > "$OUT/recbspline.log" 2>&1 &
RECB_PID=$!
# EGO'S OWN OCCUPANCY GRID, recorded next to the plan it refused.  MEASURED 2026-10-14 (sorties L1/N1): the
# adapter refused 13 plans per sortie with "pillar clearance -0.172 .. -0.219 m", i.e. those plans cut 17-22 cm
# INTO a 0.40 m pillar, while every PUBLISHED plan held the contract (min 0.278 m).  ego treats UNKNOWN cells
# as free (occupancy_buffer_inflate_ = 0), so either that pillar is not in its grid (a map-update problem) or
# it is in the grid and the plan cuts it anyway (an optimiser problem).  `grid_map/occupancy_inflate` is ego's
# own answer to which, so this file turns "is the pillar in ego's map?" into a measurement.
python3 "$REPO/tools/record-cloud.py" --topic grid_map/occupancy_inflate --csv "$OUT/ego_map.csv" \
    --decimate 20 --scans 400 > "$OUT/recogomap.log" 2>&1 &
RECM_PID=$!
sleep 3

echo "== flying =="
NAV_SOURCE="${NAV_SOURCE:-lio}"
CALIB_ARG=""
TRAJ_CALIB_ARG=""
if [ "$NAV_SOURCE" = "lio" ]; then
    CALIB="$REPO/../_stage/calib.json"
    if [ ! -f "$CALIB" ]; then
        echo "no calibration file at $CALIB -- run once with NAV_SOURCE=truth first"
        exit 1
    fi
    CALIB_ARG="--calib-file $CALIB"
    # the trajectory source must invert the SAME transform the FSM applies (see planner/rectangle_source.py):
    # with the predicted one here and the measured one there, every commanded corner moved 51 mm in N
    TRAJ_CALIB_ARG="--calib-file $CALIB"
fi
# TRAJ_SOURCE selects WHO publishes /planner/traj: `rect` (the rectangle, mission A) or `planner`
# (ego-planner through tools/run-planner.sh, mission B).  The FSM does not know or care which -- that is
# the point of the frozen interface -- but a planner's trajectory is ASYNCHRONOUS, so the FSM is switched
# to path following (with its own wait) and the planner gets the goal from the world file.
TRAJ_SOURCE="${TRAJ_SOURCE:-rect}"
# STATIONS > 0 flies a SEQUENCE of stops: planner/goal_sequencer.py publishes them one at a time (the
# user's division of labour: "ego只需要大概路径规划点吧，然后他会自己规划路径") and the FSM follows one
# trajectory per stop (`--stations N`).  The stop list comes from the world file through tools/world-circle.py,
# including the rule that a stop landing inside a pillar moves forward to that pillar's next exit.
STATIONS="${STATIONS:-0}"
# STATION_MODE=ego (patch 0008): THE STATION LIST BECOMES EGO'S OWN.  tools/mission-params.py writes
# fsm/waypoint* -- already converted into the LIO frame with the measured calibration -- into
# $OUT/stations.yaml, run-planner.sh passes that file to the planner as a second --params-file, and ego
# plans the tour ITSELF with its advance test on the VEHICLE.  The sequencer then keeps an OBSERVER role
# only (--observe-wp-index /wp_index): it mirrors ego's own wp_index onto the stop topic and still SCORES
# every stop from truth, so the numbers this script's metrics come from are unchanged.
#
# WHY THE SWITCH.  MEASURED sortie AO2: with the stops driven from OUTSIDE, the adapter had already
# converted the NEXT goal while ego's own end_pt_ was still the PREVIOUS stop's -- two sources of truth for
# "where the mission is".  MEASURED sortie AO1: the external sequencer walked stops 2..11 on its own
# timeout while the vehicle hovered at stop 1.  Both are one defect and it cannot be repaired from outside
# ego, which is why the sequence moved inside it.
#
# UNSET = every earlier sortie's behaviour, unchanged: no stations.yaml is written, the planner gets no
# station params and keeps its own auto-goal, and the sequencer publishes the goals itself.
STATION_MODE="${STATION_MODE:-}"
STATION_PARAMS="${STATION_PARAMS:-}"
STATION_GOAL=""
STATION_ARGS_PLANNER=""
STATION_ARGS_SEQ=""
if [ "$STATION_MODE" = "ego" ]; then
    if [ "$TRAJ_SOURCE" != "planner" ]; then
        echo "STATION_MODE=ego needs TRAJ_SOURCE=planner (the stations are EGO's params, and nothing else" \
             "reads them): TRAJ_SOURCE=$TRAJ_SOURCE" >&2
        exit 1
    fi
    STATION_PARAMS="$OUT/stations.yaml"
    # --start-alt 1.28 m = DO NOT PLAN THE FIRST TRAJECTORY FROM THE PAD.  MEASURED earlier: planning from
    # the ground left ego in GEN_NEW_TRAJ for ever -- 599 replans, 0 plan_success -- and the sortie had to be
    # killed; the external sequencer's own workaround was `goal_sequencer.py --min-alt`, whose default is
    # max_stop_z - 0.60 = 1.28 m.  With fsm/flight_type 2 (PRESET_TARGET) ego starts the tour in its
    # CONSTRUCTOR, so that delay can no longer be imposed from outside: --start-alt writes
    # fsm/waypoint_start_alt and the gate arms the tour from ego's own FSM loop once odom z >= 1.28 m.
    echo "== the station list ego will own (patch 0008) =="
    # ... AND THE SAME LIST IN W FOR THE OBSERVER (see mission-params.py --stops-w-out).  MEASURED AP6: the
    # observer scored the circle's 12 stops while ego flew the four corners, so its "N of N" was meaningless.
    python3 "$REPO/tools/mission-params.py" --world "$WORLD" --calib-file "$CALIB" \
        --mission "${MISSION:-rect}" \
        --out "$STATION_PARAMS" --stops-w-out "$OUT/stations_w.csv" \
        --advance-m "${STATION_ADVANCE_M:-0.30}" \
        --start-alt "${STATION_START_ALT:-1.28}" || {
        echo "tools/mission-params.py FAILED: ego would fly with NO station list -- refusing to take off" >&2
        exit 1
    }
    # GOAL=none AND --no-goal, because the ENV FORM ALONE IS A SILENT NO-OP: tools/run-planner.sh assigns
    # `GOAL=""` unconditionally near its top and only its own argument parser can set it to "none".  With
    # ego owning the tour, an auto-goal published by the planner chain would be a second source of truth
    # again -- the very defect this mode removes -- so the enforcing form is passed as well.
    STATION_GOAL="none"
    STATION_ARGS_PLANNER="--no-goal"
    STATION_ARGS_SEQ="--observe-wp-index /wp_index"
    # THE OBSERVER'S STOP LIST = EGO'S STOP LIST (see the --stops-w-out above).  If the file is missing the
    # run REFUSES rather than scoring a different mission in silence.
    if [ ! -s "$OUT/stations_w.csv" ]; then
        echo "REFUSING TO FLY: $OUT/stations_w.csv is empty -- the observer would score a DIFFERENT stop list" \
             "than the one ego was given (MEASURED AP6: it scored the circle's 12 while ego flew 4 corners)" >&2
        exit 1
    fi
    SEQ_STOPS="$(paste -sd';' "$OUT/stations_w.csv")"
    echo "   observer   : scoring ego's OWN $STATIONS stops from $OUT/stations_w.csv"
    echo "                $SEQ_STOPS"
fi
SEQ_PID=""
if [ "$TRAJ_SOURCE" = "planner" ]; then
    # run-planner.sh's world default is now worlds/forest_b.yaml (mission B's own world), and passing
    # WORLD here keeps the PLANNER's map, its goal and its clearance gate on the SAME file the simulator
    # renders and the metrics judge -- the "two worlds" bug of STATUS.md row 50.
    # EGO_MAX_VEL is passed through rather than defaulted here: empty means "let the params file's own
    # manager/max_vel govern", which is the only way `PARAMS=...fast3.yaml` can actually reach 3 m/s.
    # MEASURED 2026-10-14: run-planner.sh used to hard-code `-p manager/max_vel:=1.0` on top of
    # `--params-file` (ROS 2 `-p` wins), so every fast3 attempt flew a 1.0 m/s time allocation.
    WORLD="$WORLD" EGO_MAX_VEL="${EGO_MAX_VEL:-}" STATION_PARAMS="$STATION_PARAMS" \
        GOAL="$STATION_GOAL" bash "$REPO/tools/run-planner.sh" $STATION_ARGS_PLANNER \
        > "$OUT/planner.log" 2>&1 &
    TRAJ_PID=$!
    sleep 1
    if [ "$STATIONS" -gt 0 ]; then
        # `--stops=VALUE`, not `--stops VALUE`: a stop list starts with a coordinate, and argparse reads a
        # leading "-5,..." as another option -- exactly the trap that made `--corners=` necessary in
        # tools/metrics.py.  MEASURED 2026-10-14: with the space form the sequencer died on
        # "argument --stops: expected one argument" and the flight silently flew the world's circle instead.
        python3 "$REPO/planner/goal_sequencer.py" --world "$WORLD" --calib-file "$CALIB" \
            --arrive "${SEQ_ARRIVE:-0.35}" --timeout "${SEQ_TIMEOUT:-35}" \
            ${SEQ_STOPS:+--stops="$SEQ_STOPS"} ${SEQ_DWELL:+--dwell-s="$SEQ_DWELL"} \
            --log "$OUT/sequencer.log" $STATION_ARGS_SEQ > "$OUT/sequencer.out" 2>&1 &
        SEQ_PID=$!
        echo "   stations   : $STATIONS, fed by planner/goal_sequencer.py (arrive ${SEQ_ARRIVE:-0.35} m)"
        FSM_ARGS="${FSM_ARGS:-} --stations $STATIONS"
    fi
else
    # RECT_HOLD_S=5 -> hover 5 s at every corner (the producer then publishes one leg per message and keeps
    # publishing the leg past its flight time, so the FSM holds at the corner; 0/unset = one continuous path)
    python3 "$REPO/planner/rectangle_source.py" --world "$WORLD" $TRAJ_CALIB_ARG \
        ${RECT_HOLD_S:+--hold-s "$RECT_HOLD_S"} > "$OUT/traj.log" 2>&1 &
    TRAJ_PID=$!
fi
python3 "$REPO/fsm/mission_fsm.py" --nav-source "$NAV_SOURCE" $CALIB_ARG \
    --tap-port 0 --alt "$TAKEOFF_ALT" --hold 3.0 --geofence-m "$GEOFENCE_M" ${FSM_ARGS:-} \
    ${YAW_MODE:+--yaw-mode "$YAW_MODE"} \
    --phase-log "$OUT/phase.csv" --setpoint-log "$OUT/setpoints.csv" > "$OUT/fsm.log" 2>&1
FSM_RC=$?
cat "$OUT/fsm.log"

kill "$SEQ_PID" 2>/dev/null
kill "$REC_PID" 2>/dev/null
kill "$RECC_PID" 2>/dev/null
kill "$RECT_PID" 2>/dev/null
sleep 1
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
echo "== the planner's health, from this run's own artifacts (audit F3) =="
python3 - "$OUT" <<'PYHEALTH'
import os, re, sys
out = sys.argv[1]
log = os.path.join(out, "planner.log")
text = open(log, errors="replace").read() if os.path.exists(log) else ""
crash = [s for s in ("terminate called", "std::bad_alloc", "what():", "[ros2run]: Aborted",
                     "Traceback (most recent call last)", "NameError") if s in text]
status = re.findall(r"ego_adapter STATUS: .*", text)
bsplines = re.findall(r"bsplines (\d+)", text)
withheld = re.findall(r"withheld (\d+)", text)
ages = [float(v) for v in re.findall(r"committed ([0-9.]+) s ago", text)]
print("   adapter's last STATUS : %s" % (status[-1] if status else "(none)"))
print("   bsplines received     : %s" % (bsplines[-1] if bsplines else "?"))
print("   plans withheld        : %s   (the highest 'committed X s ago' seen: %s)"
      % (withheld[-1] if withheld else "?", ("%.1f s" % max(ages)) if ages else "n/a"))
print("   crash strings in the planner log: %s"
      % (", ".join(crash) if crash else "none -- the planner chain did not report a crash"))
if ages and max(ages) > 10.0:
    print("   *** PLANNER LIVENESS FAIL: a plan was withheld while the last committed trajectory was %.1f s old"
          % max(ages))
    print("       (healthy sorties measured <= 4.5 s; AP5's dead node reached 69.8 s) -- treat every")
    print("       'no new trajectory' abort in this run as a PLANNER FAILURE, not a trajectory timeout.")
PYHEALTH
echo
echo "== the FCU's own log =="
if [ -n "$BINFILE" ]; then
    python3 "$REPO/tools/bin-triage.py" "$BINFILE" | tail -25
    cp "$BINFILE" "$OUT/flight.BIN" 2>/dev/null || true
fi
echo
echo "run dir: $OUT   (fsm rc=$FSM_RC, metrics rc=$MET_RC)"
exit $FSM_RC
