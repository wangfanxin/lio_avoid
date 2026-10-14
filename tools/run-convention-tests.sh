#!/usr/bin/env bash
# S5 gate, live: the convention tests at three spawn headings, plus a mutation run that must FAIL.
#
# Nothing here arms or flies: the simulator is restarted per heading, the tests read the truth tap and
# the FCU's own telemetry, and the whole thing is a few seconds per case.
#
# usage: tools/run-convention-tests.sh
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
REPO=/mnt/d/distill_self/lio_avoid
TAP_PORT=14560
FAN_PORT=14561
LATLON=-35.363261,149.165230,584
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s5-conventions-$STAMP"
mkdir -p "$OUT"

pkill -f "sim/truth_rx.py" 2>/dev/null
sleep 0.5

start_sitl() {
    local heading="$1"
    setsid nohup env LIO_TRUTH_TAP=$TAP_PORT "$BIN" --model "${SITL_MODEL:-x}" --speedup 1 -I0 \
        --home "$LATLON,$heading" --serial0 tcp:5760 -w > "$OUT/sitl_$heading.log" 2>&1 &
    SITL_PID=$!
    for _ in $(seq 1 24); do
        grep -q TruthTap "$OUT/sitl_$heading.log" 2>/dev/null && return 0
        kill -0 "$SITL_PID" 2>/dev/null || return 1
        sleep 0.5
    done
    return 1
}

stop_sitl() {
    kill "$SITL_PID" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$SITL_PID" 2>/dev/null || break; sleep 0.2; done
    kill -9 "$SITL_PID" 2>/dev/null
}

PASS=0
FAIL=0
for H in 0 90 270; do
    echo "================ spawn heading $H deg ================"
    if ! start_sitl "$H"; then echo "SITL did not start" >&2; FAIL=$((FAIL+1)); continue; fi
    python3 "$REPO/sim/truth_rx.py" --duration 0 --parked --port $TAP_PORT --fanout $FAN_PORT \
        > "$OUT/truth_rx_$H.log" 2>&1 &
    RX=$!
    sleep 1.5
    python3 "$REPO/tools/convention-tests.py" --tap-port $FAN_PORT --spawn-heading "$H" \
        --seconds 3 > "$OUT/tests_$H.log" 2>&1
    RC=$?
    cat "$OUT/tests_$H.log"
    echo "case rc=$RC"
    [ $RC -eq 0 ] && PASS=$((PASS+1)) || FAIL=$((FAIL+1))
    stop_sitl
    kill "$RX" 2>/dev/null
    sleep 1
    echo
done

echo "================ live mutation: the tests must FAIL ================"
if start_sitl 90; then
    python3 "$REPO/sim/truth_rx.py" --duration 0 --parked --port $TAP_PORT --fanout $FAN_PORT \
        > "$OUT/truth_rx_mut.log" 2>&1 &
    RX=$!
    sleep 1.5
    python3 "$REPO/tools/convention-tests.py" --tap-port $FAN_PORT --spawn-heading 90 \
        --mutation flip-enu --seconds 3 > "$OUT/tests_mutation.log" 2>&1
    MRC=$?
    cat "$OUT/tests_mutation.log"
    echo "mutation rc=$MRC (must be non-zero)"
    stop_sitl
    kill "$RX" 2>/dev/null
else
    echo "SITL did not start for the mutation case" >&2
    MRC=0
fi

echo
echo "================ S5 verdict ================"
echo "cases passed: $PASS of 3, failed: $FAIL"
if [ $FAIL -eq 0 ] && [ $MRC -ne 0 ]; then
    echo "S5 GATE: PASS (three headings correct, and a wrong conversion fails)"
    echo "run dir: $OUT"
    exit 0
else
    echo "S5 GATE: FAIL (failed=$FAIL, mutation rc=$MRC)"
    echo "run dir: $OUT"
    exit 1
fi
