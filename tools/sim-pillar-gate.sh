#!/usr/bin/env bash
# S4 gate: SITL parked + truth_rx (fan-out) + lidar_imu + the pillar fit.
#
# This is the gate that validates the mount, the real pattern, the per-point timing and the frame
# chain BEFORE any LIO exists: if the accumulated clouds do not land on the world file's surfaces, no
# estimator result downstream can be trusted.
#
# usage: tools/sim-pillar-gate.sh [SECONDS]
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
REPO=/mnt/d/distill_self/lio_avoid
TAP_PORT=14560
FAN_PORT=14561
HOME_POS=-35.363261,149.165230,584,0
SECS="${1:-8}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s4-pillar-$STAMP"
mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

# Clear strays of OUR OWN scripts only (never anything else the user runs), then refuse to start if a
# leftover sensor is still alive.  MEASURED 2026-10-13: when two sim/lidar_imu.py processes overlap,
# both publish /livox/lidar and the fitter silently collects a MIXTURE of two mounts -- the correct
# configuration failed with the wrong mount's profile, which looks exactly like a simulator bug.
pkill -f "sim/lidar_imu.py" 2>/dev/null
pkill -f "sim/truth_rx.py" 2>/dev/null
pkill -f "tools/pillar-fit.py" 2>/dev/null
sleep 1
if pgrep -f "sim/lidar_imu.py" > /dev/null; then
    echo "REFUSING to run: a stray sim/lidar_imu.py is still alive" >&2
    exit 1
fi

stop_pid() {
    local pid="$1"
    kill "$pid" 2>/dev/null
    for _ in $(seq 1 25); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -9 "$pid" 2>/dev/null
    for _ in $(seq 1 10); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.2
    done
    if kill -0 "$pid" 2>/dev/null; then
        echo "WARNING: pid $pid did not die" >&2
    fi
}

cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$TAP_PORT "$BIN" --model quad --speedup 1 -I0 --home "$HOME_POS" \
    --serial0 tcp:5760 -w > sitl.log 2>&1 &
SITL_PID=$!
cd - > /dev/null

cleanup() {
    for p in "${PILLAR_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$SITL_PID" 2>/dev/null
    sleep 1
    for p in "${PILLAR_PID:-}" "${SENSOR_PID:-}" "${RX_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    kill -9 "$SITL_PID" 2>/dev/null
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

run_case() {
    # $1 = tag, $2 = mount pitch for the SENSOR node, $3 = seconds, $4 = extra sensor args
    local tag="$1" pitch="$2" secs="$3" extra="${4:-}"
    if pgrep -f "sim/lidar_imu.py" > /dev/null; then
        echo "REFUSING case $tag: a sensor from the previous case is still alive" >&2
        return 1
    fi
    # shellcheck disable=SC2086
    python3 "$REPO/sim/lidar_imu.py" --port $FAN_PORT --duration 0 --mount-pitch "$pitch" $extra \
        > "$OUT/lidar_imu_$tag.log" 2>&1 &
    local sensor_pid=$!
    sleep 2
    if [ "$(pgrep -c -f 'sim/lidar_imu.py')" != "1" ]; then
        echo "REFUSING case $tag: $(pgrep -c -f 'sim/lidar_imu.py') sensor processes are alive" >&2
        stop_pid "$sensor_pid"
        return 1
    fi
    python3 "$REPO/tools/pillar-fit.py" --seconds "$secs" --mount-pitch "$pitch" \
        --dump "$OUT/fit_$tag.npz" --split-json "$OUT/split_$tag.json" \
        > "$OUT/pillar_fit_$tag.log" 2>&1
    local rc=$?
    stop_pid "$sensor_pid"
    echo "----- case $tag (mount pitch $pitch deg${extra:+, $extra}) rc=$rc -----"
    grep -E "surface residual|surface split|pillar fit|direction test|^\[|GATE" "$OUT/pillar_fit_$tag.log"
    echo
    return $rc
}

# The real configuration, then two deliberately WRONG ones: a level mount (the split must notice) and
# a mirrored azimuth (the direction test must notice).  A check that cannot fail is not a check.
run_case right 45 "$SECS"
RC_RIGHT=$?
run_case wrong 0 "$SECS"
run_case mirror 45 "$SECS" "--mirror-azimuth"

echo "== power check 1: the mount must be visible in the surface split =="
python3 - "$OUT/split_right.json" "$OUT/split_wrong.json" <<'PY'
import json, sys
right = json.load(open(sys.argv[1]))
wrong = json.load(open(sys.argv[2]))
print("  45 deg nose-down (real): floor %.3f, ceiling %.3f, pillars %.3f" %
      (right["floor"], right["ceiling"], right["pillars"]))
print("   0 deg (deliberately wrong): floor %.3f, ceiling %.3f, pillars %.3f" %
      (wrong["floor"], wrong["ceiling"], wrong["pillars"]))
# The tilted dome reaches the floor well beyond the ceiling and keeps a third of its returns off the
# pillars; a LEVEL housing looks mostly sideways at the forest (measured 74.3 % pillars, 4.7 % floor).
# Both inequalities are what the measurement actually shows, not what a 2D argument predicted: the
# first version of this check demanded a ceiling-dominated level case and was simply wrong.
ok = (right["floor"] >= 0.15 and right["floor"] > right["ceiling"] and
      wrong["pillars"] >= 0.60 and wrong["floor"] <= 0.10)
print("  [%s] the real mount is floor-rich, the wrong one is pillar-dominated" %
      ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
PY
POWER=$?

echo
echo "== power check 2 (what the direction test can and cannot see) =="
python3 - "$OUT/split_right.json" "$OUT/split_wrong.json" <<'PY'
import json, sys
right = json.load(open(sys.argv[1]))
wrong = json.load(open(sys.argv[2]))
r, w = right["direction_worst_deg"], wrong["direction_worst_deg"]
print("  direction test, correct mount: worst %.2f deg" % r)
print("  direction test, wrong  mount: worst %.2f deg" % w)
print("  [INFO] the wrong-mount value is NOT a gate: MEASURED 2026-10-13 it read 87.6 deg in one run")
print("         and 3.1 deg in another, because the fitter and the caster share the same mount")
print("         constant, so a wrong constant cancels; what is left is which surfaces got sampled.")
print("         The check has power over a mount applied in the WRONG PLACE (the LIO's config rather")
print("         than the simulator), and that is what it is kept for.  The mount DIRECTION is gated by")
print("         the surface split above, which separated the two cases reproducibly.")
PY

echo
echo "== documented experiment: a mirrored azimuth is UNOBSERVABLE (not a gate, a finding) =="
python3 - "$OUT/split_right.json" "$OUT/split_mirror.json" <<'PY'
import json, sys
right = json.load(open(sys.argv[1]))
mirror = json.load(open(sys.argv[2]))
print("  correct azimuth : floor %.3f, pillars %.3f, direction worst %.2f deg, quadrants %s" %
      (right["floor"], right["pillars"], right["direction_worst_deg"],
       ["%.3f" % x for x in right["quadrant_pillar_fraction"]]))
print("  mirrored azimuth: floor %.3f, pillars %.3f, direction worst %.2f deg, quadrants %s" %
      (mirror["floor"], mirror["pillars"], mirror["direction_worst_deg"],
       ["%.3f" % x for x in mirror["quadrant_pillar_fraction"]]))
# WHY this is not a gate: mirroring az maps ray j (az) to -az, and the ray that then lands on a given
# pillar is the mirror of the ray that landed on the mirrored pillar -- so the S-frame direction of
# the point on a given pillar is the same either way, and the only difference is which ELEVATION and
# which timestamp that ray carried.  A 360-degree-per-scan fan is therefore statistically its own
# mirror, and no statistic of the clouds can tell the two apart (measured: the numbers above).
# Consequence, recorded here so nobody "fixes" it later: the azimuth SIGN is a convention fixed by the
# right-handed housing frame, not a measured quantity; nothing downstream reads azimuth (the LIO
# consumes x,y,z,time), so a wrong sign would only mirror the world the estimator maps.
diff = abs(right["floor"] - mirror["floor"]) + abs(right["pillars"] - mirror["pillars"])
print("  [INFO] |difference| in the two statistics: %.4f -- indistinguishable, as the argument says"
      % diff)
PY

echo
echo "== lidar_imu (real mount) =="
tail -6 "$OUT/lidar_imu_right.log"
echo
echo "== /sim/rtf =="
timeout 6 ros2 topic echo --once /sim/rtf 2>/dev/null | head -2
echo
echo "run dir: $OUT"
if [ $RC_RIGHT -eq 0 ] && [ $POWER -eq 0 ]; then
    echo "S4 GATE: PASS (right case green; the mount is visible in the surface split)"
    exit 0
else
    echo "S4 GATE: FAIL (right=$RC_RIGHT power1=$POWER)"
    exit 1
fi
