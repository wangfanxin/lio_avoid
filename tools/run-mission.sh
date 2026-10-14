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
eval "$(python3 "$REPO/tools/world-info.py" --world "$WORLD" --shell)"
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

pkill -f "sim/lidar_imu.py" 2>/dev/null
pkill -f "sim/truth_rx.py" 2>/dev/null
pkill -f "super_lio_node" 2>/dev/null
pkill -f "rectangle_source.py" 2>/dev/null
pkill -f "mission_fsm.py" 2>/dev/null
sleep 1

echo "== revisions under test =="
echo "   ardupilot : $(git -C "$AP" describe --tags --always) $(git -C "$AP" rev-parse --short HEAD) (tap applied: $(git -C "$AP" --no-pager diff --stat | tail -1 | tr -s ' '))"
echo "   super_lio : $(git -C /home/fanxin/lio_avoid/src/Super-LIO rev-parse --short HEAD)  liblio $(md5sum "$WS/install/super_lio/lib/liblio.so" | cut -c1-16)"
echo "   config    : lio/config/livox_360.yaml + apm_lio_only,sitl_basics,${ALT_MODE}_posz,$MISSION_NAME,$EKF_NAME"
echo "   world     : $WORLD  ($CORNERS)"
echo "   sensor    : the SAME world file ($REPO/sim/lidar_imu.py --world \"$WORLD\")"
echo "   geofence  : $GEOFENCE_M m, take-off altitude $TAKEOFF_ALT m"
echo "   run dir   : $OUT"
echo

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 \
    --defaults "$REPO/params/apm_lio_only.parm,$REPO/params/sitl_basics.parm,$ALT_PARM,$MISSION_PARM$EKF_ARG" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null

cleanup() {
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}" \
             "${SEQ_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL" 2>/dev/null
    sleep 1
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}" \
             "${SEQ_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    kill -9 "$SITL" 2>/dev/null
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
SEQ_PID=""
if [ "$TRAJ_SOURCE" = "planner" ]; then
    # run-planner.sh's world default is now worlds/forest_b.yaml (mission B's own world), and passing
    # WORLD here keeps the PLANNER's map, its goal and its clearance gate on the SAME file the simulator
    # renders and the metrics judge -- the "two worlds" bug of STATUS.md row 50.
    WORLD="$WORLD" bash "$REPO/tools/run-planner.sh" > "$OUT/planner.log" 2>&1 &
    TRAJ_PID=$!
    sleep 1
    if [ "$STATIONS" -gt 0 ]; then
        python3 "$REPO/planner/goal_sequencer.py" --world "$WORLD" --calib-file "$CALIB" \
            --arrive "${SEQ_ARRIVE:-0.35}" --timeout "${SEQ_TIMEOUT:-35}" \
            --log "$OUT/sequencer.log" > "$OUT/sequencer.out" 2>&1 &
        SEQ_PID=$!
        echo "   stations   : $STATIONS, fed by planner/goal_sequencer.py (arrive ${SEQ_ARRIVE:-0.35} m)"
        FSM_ARGS="${FSM_ARGS:-} --stations $STATIONS"
    fi
else
    python3 "$REPO/planner/rectangle_source.py" --world "$WORLD" $TRAJ_CALIB_ARG > "$OUT/traj.log" 2>&1 &
    TRAJ_PID=$!
fi
python3 "$REPO/fsm/mission_fsm.py" --nav-source "$NAV_SOURCE" $CALIB_ARG \
    --tap-port 0 --alt "$TAKEOFF_ALT" --hold 3.0 --geofence-m "$GEOFENCE_M" ${FSM_ARGS:-} \
    --phase-log "$OUT/phase.csv" > "$OUT/fsm.log" 2>&1
FSM_RC=$?
cat "$OUT/fsm.log"

kill "$SEQ_PID" 2>/dev/null
kill "$REC_PID" 2>/dev/null
kill "$RECC_PID" 2>/dev/null
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
