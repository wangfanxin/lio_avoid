#!/usr/bin/env bash
# S3 gate, part 3: does the receiver publish what S4-S7 will consume?
#
# Starts SITL parked, runs sim/truth_rx.py in ROS mode, and then measures the topics from outside:
# the achieved rates, /sim/rtf, the TF broadcast, and -- the interesting one -- the CONTENT of one
# /sim/truth/odom message, which is the whole wire -> W conversion exercised end to end on live data.
#
# usage: tools/sim-ros-check.sh [SECONDS]
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
ROOT=/mnt/d/distill_self/lio_avoid        # the project: params/, worlds/, docs/, frozen/, rviz/, firmware/
REPO="$ROOT/src"                          # the code: bridge/, common/, lio/, planner/, sim/, supervisor/, tools/
PORT=14560
HOME_POS=-35.363261,149.165230,584,0
SECS="${1:-40}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s3-ros-$STAMP"
mkdir -p "$OUT"

export ROS_LOCALHOST_ONLY=1
# ROS's setup.bash reads unset variables on purpose, so `set -u` has to come off around it.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || { echo "no ROS 2 at /opt/ros/humble" >&2; exit 1; }
set -u

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$PORT "$BIN" --model "${SITL_MODEL:-x}" --speedup 1 -I0 --home "$HOME_POS" \
    --serial0 tcp:5760 -w > sitl.log 2>&1 &
SITL_PID=$!
cd - > /dev/null

cleanup() {
    [ -n "${RX_PID:-}" ] && kill "$RX_PID" 2>/dev/null
    kill "$SITL_PID" 2>/dev/null
    sleep 1
    kill -9 "$SITL_PID" 2>/dev/null
    [ -n "${RX_PID:-}" ] && kill -9 "$RX_PID" 2>/dev/null
}
trap cleanup EXIT

for i in $(seq 1 20); do
    grep -q TruthTap "$OUT/sitl.log" 2>/dev/null && break
    kill -0 "$SITL_PID" 2>/dev/null || { echo "SITL died:" >&2; tail -10 "$OUT/sitl.log" >&2; exit 1; }
    sleep 0.5
done
grep -q TruthTap "$OUT/sitl.log" || { echo "tap never reported" >&2; exit 1; }
echo "SITL is stepping (pid $SITL_PID)"
echo

python3 "$REPO/sim/truth_rx.py" --ros --duration "$SECS" --parked --port $PORT \
    --save "$OUT/report.txt" > "$OUT/receiver.log" 2>&1 &
RX_PID=$!
sleep 4

echo "== topics =="
timeout 10 ros2 topic list 2>/dev/null | grep -E "^/clock|^/sim/|^/tf$" || echo "(none of ours)"
echo
echo "== rates =="
echo "-- /clock";        timeout 6 ros2 topic hz /clock 2>/dev/null | grep -m1 "average rate"
echo "-- /sim/truth/odom"; timeout 6 ros2 topic hz /sim/truth/odom 2>/dev/null | grep -m1 "average rate"
echo "-- /sim/rtf";       timeout 6 ros2 topic echo --once /sim/rtf 2>/dev/null | head -2
echo
echo "== one odometry message, and what it must be =="
timeout 6 ros2 topic echo --once /sim/truth/odom 2>/dev/null | head -22
echo "   (at rest on the pad the tap reports NED (0, 0, -0.1), so W must be (0, 0, +0.1) with the"
echo "    body's x pointing North: quaternion (0.7071, 0, 0, 0.7071), i.e. W yaw +90 deg)"
echo
echo "== TF =="
timeout 6 ros2 topic echo --once /tf 2>/dev/null | grep -E "frame_id|child_frame_id|translation|^  [xyz]:" | head -12
echo
wait "$RX_PID" 2>/dev/null
echo "== receiver report tail =="
tail -14 "$OUT/report.txt"
echo
echo "run dir: $OUT"
