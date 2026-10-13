#!/usr/bin/env bash
# S3 gate, parked: run SITL with the truth tap enabled, receive 30 s of it, and print the verdict.
#
#   tools/sitl-parked.sh [SECONDS] [--defaults PARM]
#
# Everything is written under logs/s3-parked-<stamp>/: the SITL console, the tap CSV, the report.
# The report's PASS/FAIL is this script's exit status, so a caller cannot mistake it for success.
set -u

SECONDS_SIM="${1:-30}"
DEFAULTS="${2:-}"

AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
REPO=/mnt/d/distill_self/lio_avoid
PORT=14560
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s3-parked-$STAMP"
mkdir -p "$OUT"
cd /home/fanxin/lio_avoid || exit 1

if [ ! -x "$BIN" ]; then
    echo "no SITL binary at $BIN -- build first (wsl ./waf copter)" >&2
    exit 1
fi

# The world's origin is the pad; SITL needs *a* latitude/longitude for its origin.  It is arbitrary
# (no GPS, no compass, no map) but it must be pinned, so it is written down here and asserted later
# from the tap: home = lat,lon,alt,yaw.  yaw 0 = North = PLAN.md decision 14.
HOME_POS="-35.363261,149.165230,584,0"
echo "binary    : $(md5sum "$BIN" | cut -c1-32)"
echo "revision  : $(git -C "$AP" describe --tags --always)"
echo "run dir   : $OUT"
echo "home      : $HOME_POS  (yaw 0 = North)"
echo "tap port  : $PORT"
echo "sim time  : ${SECONDS_SIM} s"
echo

# --model quad: the built-in quadcopter model.  -w wipes the (SITL) EEPROM so the run starts from
# firmware defaults rather than from whatever a previous run left behind.
#
# --serial0 is spelled out WITHOUT `:wait`.  The default is `tcp:5760:wait`, and `wait` makes the
# firmware block in accept() before the sketch's main loop ever runs -- the model does not step, so
# the tap sends nothing and no BIN is written.  Without `wait` the socket still listens and accepts a
# client whenever mavros or a script connects, which is what we want.
#
# setsid + nohup: the simulator gets its own session, so nothing that happens to this script's
# process group can take it down quietly, and its console keeps running to sitl.log.
cd "$OUT" || exit 1
setsid nohup env LIO_TRUTH_TAP=$PORT "$BIN" --model quad --speedup 1 -I0 --home "$HOME_POS" \
    --serial0 tcp:5760 ${DEFAULTS:+--defaults "$DEFAULTS"} -w > sitl.log 2>&1 &
SITL_PID=$!
cd - > /dev/null

# Never leave the simulator behind, whatever happens to the receiver.
cleanup() {
    kill "$SITL_PID" 2>/dev/null
    sleep 1
    kill -9 "$SITL_PID" 2>/dev/null
}
trap cleanup EXIT

# Wait for the model to actually be stepping: the tap prints its banner on its first call, so the
# banner is proof that update_dynamics() ran.  Without this check a simulator that never starts looks
# exactly like a receiver that cannot bind (both are "no samples"), and that cost a round.
for i in $(seq 1 20); do
    if grep -q "TruthTap" "$OUT/sitl.log" 2>/dev/null; then break; fi
    if ! kill -0 "$SITL_PID" 2>/dev/null; then
        echo "SITL died during startup:" >&2
        tail -20 "$OUT/sitl.log" >&2
        exit 1
    fi
    sleep 0.5
done
if ! grep -q "TruthTap" "$OUT/sitl.log"; then
    echo "the tap never reported in (10 s): LIO_TRUTH_TAP was not honoured?" >&2
    tail -20 "$OUT/sitl.log" >&2
    exit 1
fi
echo "SITL is stepping (pid $SITL_PID)"
echo

python3 "$REPO/sim/truth_rx.py" --print --duration "$SECONDS_SIM" --parked \
    --port $PORT \
    --csv "$OUT/tap.csv" \
    --save "$OUT/report.txt" | tee "$OUT/receiver.log"
RC=${PIPESTATUS[0]}

kill "$SITL_PID" 2>/dev/null
sleep 1
kill -9 "$SITL_PID" 2>/dev/null

echo
echo "== SITL console, tap line =="
grep -m2 -i "truthtap" "$OUT/sitl.log" || echo "(the tap banner is missing -- was LIO_TRUTH_TAP passed?)"
echo
echo "== SITL console, tail =="
tail -6 "$OUT/sitl.log"
echo
echo "== BIN log =="
ls -l "$OUT"/logs/*.BIN 2>/dev/null | tail -2 || echo "(no BIN written)"
echo
echo "report: $OUT/report.txt"
echo "csv   : $OUT/tap.csv ($(wc -l < "$OUT/tap.csv" 2>/dev/null) lines)"
exit $RC
