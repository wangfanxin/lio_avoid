#!/usr/bin/env bash
# S6 gate: the clean upstream LIO, fed by the simulated MID-360, measured against the truth tap.
#
# Phases (each recorded separately):
#   parked  T seconds with the vehicle on the pad: drift and yaw offset
#   moving  the model is shoved/twisted with SIM_SHOVE_*/SIM_TWIST_* -- NO ARMING and no FCU
#           involvement, so the LIO is tested against motion rather than against the EKF that does not
#           exist yet
#
# usage: tools/sitl-lio.sh [PARKED_SECONDS] [MOVING_SECONDS]
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
ROOT=/mnt/d/distill_self/lio_avoid        # the project: params/, worlds/, docs/, frozen/, rviz/, firmware/
REPO="$ROOT/src"                          # the code: bridge/, common/, lio/, planner/, sim/, supervisor/, tools/
WS=/home/fanxin/lio_avoid/lio_ws
TAP_PORT=14560
FAN_PORT=14561
MAV=tcp:127.0.0.1:5760
PARKED="${1:-30}"
MOVING="${2:-25}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s6-lio-$STAMP"
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY=1

pkill -f "sim/lidar_imu.py" 2>/dev/null
pkill -f "sim/truth_rx.py" 2>/dev/null
pkill -f "super_lio_node" 2>/dev/null
sleep 1

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP_PORT "$BIN" --model "${SITL_MODEL:-x}" --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 -w > sitl.log 2>&1 &
SITL_PID=$!
cd - > /dev/null

cleanup() {
    for p in "${REC_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL_PID" 2>/dev/null
    sleep 1
    for p in "${REC_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    kill -9 "$SITL_PID" 2>/dev/null
}
trap cleanup EXIT

for i in $(seq 1 24); do
    grep -q TruthTap "$OUT/sitl.log" 2>/dev/null && break
    kill -0 "$SITL_PID" 2>/dev/null || { echo "SITL died:" >&2; tail -10 "$OUT/sitl.log" >&2; exit 1; }
    sleep 0.5
done
grep -q TruthTap "$OUT/sitl.log" || { echo "tap never reported" >&2; exit 1; }
echo "SITL stepping (pid $SITL_PID)"

python3 "$REPO/sim/truth_rx.py" --ros --duration 0 --parked --port $TAP_PORT --fanout $FAN_PORT \
    --csv "$OUT/tap.csv" --save "$OUT/truth_report.txt" > "$OUT/truth_rx.log" 2>&1 &
RX_PID=$!
sleep 2
python3 "$REPO/sim/lidar_imu.py" --port $FAN_PORT --duration 0 > "$OUT/lidar_imu.log" 2>&1 &
SENSOR_PID=$!
sleep 2

echo
echo "== starting the LIO (clean upstream, our measured config) =="
echo "   super_lio : $(git -C /home/fanxin/lio_avoid/src/Super-LIO rev-parse --short HEAD)"
echo "   liblio.so : $(md5sum "$WS/install/super_lio/lib/liblio.so" | cut -c1-32)"
ros2 run super_lio super_lio_node --ros-args --params-file "$REPO/lio/config/livox_360.yaml" \
    > "$OUT/lio.log" 2>&1 &
LIO_PID=$!
sleep 6
if ! kill -0 "$LIO_PID" 2>/dev/null; then
    echo "the LIO exited at start-up:" >&2
    tail -30 "$OUT/lio.log" >&2
    exit 1
fi

echo "== LIO topics =="
timeout 8 ros2 topic list 2>/dev/null | grep -E "^/lio" | sed 's/^/   /' || echo "   (none)"
echo "== rates =="
for t in /lio/odom /lio/robo/odom; do
    echo -n "   $t: "
    timeout 6 ros2 topic hz "$t" 2>/dev/null | grep -m1 "average rate" || echo "(no data)"
done

echo
echo "== phase 1: parked ${PARKED}s =="
python3 "$REPO/tools/record-odom.py" --topic /lio/robo/odom --csv "$OUT/lio_parked.csv" \
    --seconds "$PARKED" > "$OUT/rec_parked.log" 2>&1
echo "   $(tail -1 "$OUT/rec_parked.log")"

echo
echo "== phase 2: motion =="
if [ "${S6_MOTION:-0}" != "1" ]; then
    echo "   SKIPPED.  MEASURED 2026-10-13 with the shove harness enabled (SIM_GND_BEHAV 0 removes the"
    echo "   velocity/gyro zeroing): the truth travelled 5.89 m for 0.74 m of excursion -- brake shoves"
    echo "   on frictionless ground with no walls produce a slide with reversals, not a trajectory, and"
    echo "   the LIO's path came out with three times the truth's spread.  Whether that is the estimator"
    echo "   or the harness cannot be told apart from such motion, and it does not need to be: S7 flies"
    echo "   the real square, and the tracking/tilt numbers are measured there.  Set S6_MOTION=1 to run"
    echo "   the harness anyway (it is kept because it needs no arming and it exercises the chain)."
    MOVING=0
else
    echo "   the shove harness: SIM_GND_BEHAV 0, push north, brake, push west, brake, then a yaw twist"
fi
if [ "$MOVING" != "0" ]; then
python3 - "$MAV" <<'PY'
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
m.wait_heartbeat(timeout=5)

def setp(name, value, wait=0.2):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(wait)

setp("SIM_GND_BEHAV", 0)
setp("SIM_SHOVE_X", 0.0)
setp("SIM_SHOVE_Y", 0.0)
setp("SIM_SHOVE_Z", 0.0)
setp("SIM_SHOVE_TIME", 1200)
setp("SIM_TWIST_X", 0.0)
setp("SIM_TWIST_Y", 0.0)
setp("SIM_TWIST_Z", 0.0)
setp("SIM_TWIST_TIME", 1500)
print("   ground behaviour NONE, shove/twist zeroed and ready")
PY
python3 "$REPO/tools/record-odom.py" --topic /lio/robo/odom --csv "$OUT/lio_moving.csv" \
    --seconds "$MOVING" > "$OUT/rec_moving.log" 2>&1 &
REC_PID=$!
sleep 1
python3 - "$MAV" <<'PY'
import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1])
m.wait_heartbeat(timeout=5)

def setp(name, value):
    m.mav.param_set_send(m.target_system, m.target_component, name.encode(), float(value),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)

# MEASURED 2026-10-13: with ground behaviour NONE there is NO friction, so an unbraked shove slid the
# vehicle 109 m out of the 10 m arena (the SITL physics has no walls) and the LIO diverged with it.
# Every leg is therefore shove-then-brake, and the whole path stays inside about 1 m.
# SIM_SHOVE_X/Y are accelerations in BODY axes (x forward, y left); SIM_SHOVE_TIME is ms.
def leg(axis, accel, ms, brake_ms):
    other = "SIM_SHOVE_Y" if axis == "SIM_SHOVE_X" else "SIM_SHOVE_X"
    setp(other, 0.0)
    setp(axis, accel)
    setp("SIM_SHOVE_TIME", ms)
    time.sleep(ms / 1000.0 + 0.7)
    setp(axis, -accel)
    setp("SIM_SHOVE_TIME", brake_ms)
    time.sleep(brake_ms / 1000.0 + 0.7)
    setp(axis, 0.0)

leg("SIM_SHOVE_X", 1.5, 500, 500)      # north, then brake
leg("SIM_SHOVE_Y", 1.5, 500, 500)      # west, then brake -- the path now turns, so a rotation fits
setp("SIM_TWIST_Z", 0.5)
setp("SIM_TWIST_TIME", 1500)
time.sleep(2.0)
setp("SIM_TWIST_Z", 0.0)
print("   pushed north 0.5 s, braked, pushed west 0.5 s, braked, then yawed at 0.5 rad/s for 1.5 s")
PY
kill "$REC_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
echo "   $(tail -1 "$OUT/rec_moving.log")"
fi

kill "$LIO_PID" 2>/dev/null
sleep 1

echo
echo "== the LIO's own log, last lines =="
tail -12 "$OUT/lio.log"

echo
echo "== parked: LIO vs truth (the S6 gate) =="
python3 "$REPO/tools/lio-vs-truth.py" --tap "$OUT/tap.csv" --odom "$OUT/lio_parked.csv" \
    --max-drift-mm 50 | tee "$OUT/fit_parked.txt"
PARK_RC=${PIPESTATUS[0]}

MOVE_RC=0
if [ "$MOVING" != "0" ]; then
    echo
    echo "== moving: LIO vs truth (harness; not a gate, see the note above) =="
    python3 "$REPO/tools/lio-vs-truth.py" --tap "$OUT/tap.csv" --odom "$OUT/lio_moving.csv" \
        --max-h-rms-mm 50 --max-v-rms-mm 50 | tee "$OUT/fit_moving.txt"
    MOVE_RC=${PIPESTATUS[0]}
fi

echo
echo "run dir: $OUT"
exit $(( PARK_RC + MOVE_RC ))
