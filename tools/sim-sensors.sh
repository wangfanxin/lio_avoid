#!/usr/bin/env bash
# S4 part 1: SITL + truth_rx (fan-out) + lidar_imu, live, for a few scans.
#
# usage: tools/sim-sensors.sh [SCANS]
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
REPO=/mnt/d/distill_self/lio_avoid
TAP_PORT=14560
FAN_PORT=14561
HOME_POS=-35.363261,149.165230,584,0
SCANS="${1:-5}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s4-sensors-$STAMP"
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP_PORT "$BIN" --model "${SITL_MODEL:-x}" --speedup 1 -I0 --home "$HOME_POS" \
    --serial0 tcp:5760 -w > sitl.log 2>&1 &
SITL_PID=$!
cd - > /dev/null

cleanup() {
    [ -n "${RX_PID:-}" ] && kill "$RX_PID" 2>/dev/null
    [ -n "${SENSOR_PID:-}" ] && kill "$SENSOR_PID" 2>/dev/null
    kill "$SITL_PID" 2>/dev/null
    sleep 1
    kill -9 "$SITL_PID" 2>/dev/null
    [ -n "${RX_PID:-}" ] && kill -9 "$RX_PID" 2>/dev/null
    [ -n "${SENSOR_PID:-}" ] && kill -9 "$SENSOR_PID" 2>/dev/null
}
trap cleanup EXIT

for i in $(seq 1 20); do
    grep -q TruthTap "$OUT/sitl.log" 2>/dev/null && break
    kill -0 "$SITL_PID" 2>/dev/null || { echo "SITL died:" >&2; tail -10 "$OUT/sitl.log" >&2; exit 1; }
    sleep 0.5
done
grep -q TruthTap "$OUT/sitl.log" || { echo "tap never reported" >&2; exit 1; }
echo "SITL is stepping (pid $SITL_PID)"

python3 "$REPO/sim/truth_rx.py" --ros --duration 0 --parked --port $TAP_PORT --fanout $FAN_PORT \
    --save "$OUT/truth.txt" > "$OUT/truth_rx.log" 2>&1 &
RX_PID=$!
sleep 2
# The sensor node runs long enough to be MEASURED: an earlier version ran 5 scans in 0.6 s and then
# every topic looked unpublished ("does not appear to be published yet"), which reads like a broken
# publisher when it is only a stopped one.
python3 "$REPO/sim/lidar_imu.py" --port $FAN_PORT --duration 20 \
    > "$OUT/lidar_imu.log" 2>&1 &
SENSOR_PID=$!
sleep 5

echo "== topics while running =="
timeout 8 ros2 topic list 2>/dev/null | grep -E "^/livox|^/clock|^/sim" || echo "(none)"
echo
echo "== rates =="
timeout 8 ros2 topic hz /livox/lidar 2>/dev/null | grep -m1 "average rate" || true
timeout 8 ros2 topic hz /livox/imu 2>/dev/null | grep -m1 "average rate" || true
echo
echo "== one cloud message: header, field list, width =="
timeout 8 ros2 topic echo --once --field header /livox/lidar 2>/dev/null | head -8
timeout 8 ros2 topic echo --once --field width /livox/lidar 2>/dev/null | head -3
timeout 8 ros2 topic echo --once --field fields /livox/lidar 2>/dev/null | head -30
echo
echo "== one imu message =="
timeout 8 ros2 topic echo --once /livox/imu 2>/dev/null | head -24

kill "$SENSOR_PID" 2>/dev/null
wait "$SENSOR_PID" 2>/dev/null
SENSOR_RC=$?
echo
echo "== lidar_imu log =="
cat "$OUT/lidar_imu.log"
echo
echo "== truth_rx tail =="
tail -8 "$OUT/truth.txt" 2>/dev/null || tail -8 "$OUT/truth_rx.log"

kill "$RX_PID" 2>/dev/null
sleep 1
echo
echo "run dir: $OUT"
exit $SENSOR_RC
