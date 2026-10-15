#!/usr/bin/env bash
# Mission A, one flight: SITL + the simulated MID-360 + the LIO + planner/rectangle_source.py + the FSM.
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
             "${RECC_PID:-}" "${RECT_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL" 2>/dev/null
    sleep 1
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}" \
             "${RECC_PID:-}" "${RECT_PID:-}"; do
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
# The LIO's own registered map is recorded next to the odometry, so a flight's clearance number can be
# checked against the map the trajectory was built from rather than taken on trust.
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
# THE TRAJECTORY SOURCE.  `planner/rectangle_source.py` publishes the mission's corners on
# /planner/traj -- the same interface an external planner used to publish on -- and the FSM does not know
# or care who produces it.  It is the only source there is: the ego-planner line was dropped on
# 2026-10-15 and its adapter, its station list and its switches went with it.
#
# RECT_HOLD_S=5 -> hover 5 s at every corner.  The producer then publishes ONE LEG PER MESSAGE, ADVANCES ON
# ARRIVAL (it subscribes /lio/robo/odom and converts L->W with the SAME calibration it publishes with) and
# holds each corner for that long before it publishes the next leg; the FSM accepts the next leg
# mid-mission and keeps commanding the corner it is parked on while it waits -- that interval is the hover.
# 0/unset = one continuous path (the old behaviour, RECT6's configuration).
#
# THE TWO CLOCKS ARE ONE NUMBER.  The FSM must not give up on the tour while the producer is legitimately
# dwelling, so its wait for the next leg is derived from the same RECT_HOLD_S (+10 s of arrival detection
# and publish slack) instead of being a second default in a second file.
NEXT_LEG_ARG=""
if [ -n "${RECT_HOLD_S:-}" ]; then
    NEXT_LEG_ARG="--next-leg-wait-s $(awk -v h="$RECT_HOLD_S" 'BEGIN{printf "%.1f", h + 10.0}')"
fi
python3 "$REPO/planner/rectangle_source.py" --world "$WORLD" $TRAJ_CALIB_ARG \
    ${RECT_HOLD_S:+--hold-s "$RECT_HOLD_S"} > "$OUT/traj.log" 2>&1 &
TRAJ_PID=$!
python3 "$REPO/fsm/mission_fsm.py" --nav-source "$NAV_SOURCE" $CALIB_ARG \
    --tap-port 0 --alt "$TAKEOFF_ALT" --hold 3.0 --geofence-m "$GEOFENCE_M" ${FSM_ARGS:-} \
    ${NEXT_LEG_ARG} \
    ${YAW_MODE:+--yaw-mode "$YAW_MODE"} \
    --phase-log "$OUT/phase.csv" --setpoint-log "$OUT/setpoints.csv" > "$OUT/fsm.log" 2>&1
FSM_RC=$?
cat "$OUT/fsm.log"

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
echo "== the FCU's own log =="
if [ -n "$BINFILE" ]; then
    python3 "$REPO/tools/bin-triage.py" "$BINFILE" | tail -25
    cp "$BINFILE" "$OUT/flight.BIN" 2>/dev/null || true
fi
echo
echo "run dir: $OUT   (fsm rc=$FSM_RC, metrics rc=$MET_RC)"
exit $FSM_RC
