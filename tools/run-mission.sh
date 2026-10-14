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
echo "   config    : lio/config/livox_360.yaml + apm_lio_only,sitl_basics,${ALT_MODE}_posz,phase_a_mission"
echo "   run dir   : $OUT"
echo

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model quad --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 \
    --defaults "$REPO/params/apm_lio_only.parm,$REPO/params/sitl_basics.parm,$ALT_PARM,$REPO/params/phase_a_mission.parm" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null

cleanup() {
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL" 2>/dev/null
    sleep 1
    for p in "${FSM_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}" "${REC_PID:-}"; do
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
python3 "$REPO/sim/lidar_imu.py" --port $FAN --duration 0 > "$OUT/lidar_imu.log" 2>&1 &
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
python3 "$REPO/planner/rectangle_source.py" $TRAJ_CALIB_ARG > "$OUT/traj.log" 2>&1 &
TRAJ_PID=$!
sleep 1
python3 "$REPO/fsm/mission_fsm.py" --nav-source "$NAV_SOURCE" $CALIB_ARG \
    --tap-port 0 --alt 2.0 --hold 3.0 ${FSM_ARGS:-} \
    --phase-log "$OUT/phase.csv" > "$OUT/fsm.log" 2>&1
FSM_RC=$?
cat "$OUT/fsm.log"

kill "$REC_PID" 2>/dev/null
sleep 1
echo
echo "== metrics =="
BINFILE=$(ls "$OUT"/logs/*.BIN 2>/dev/null | tail -1)
python3 "$REPO/tools/metrics.py" --tap "$OUT/tap.csv" --odom "$OUT/lio.csv" \
    --phase "$OUT/phase.csv" --bin "${BINFILE:-}" \
    --corners "0.5,0.4,2.0;2.5,0.4,2.0;2.5,1.6,2.0;0.5,1.6,2.0" \
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
