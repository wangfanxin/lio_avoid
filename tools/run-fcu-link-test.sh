#!/usr/bin/env bash
# Isolated FCU-link test: SITL with the LIO-only parameter set, the truth tap fed straight to the FCU
# as external nav, GUIDED + arm + takeoff + hold.  No LIO involved -- see tools/test-fcu-link.py.
set -u
AP=/home/fanxin/lio_avoid/src/ardupilot
BIN="$AP/build/sitl/bin/arducopter"
REPO=/mnt/d/distill_self/lio_avoid
TAP=14560
FAN=14561
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="/home/fanxin/lio_avoid/logs/s7-fcu-link-$STAMP"
mkdir -p "$OUT"

pkill -f "sim/truth_rx.py" 2>/dev/null
pkill -f "sim/lidar_imu.py" 2>/dev/null
sleep 1

cd "$OUT" || exit 1
# MEASURED 2026-10-13: passing --defaults TWICE does not accumulate -- the second file REPLACES the
# first (the SITL log shows a single "Loaded defaults from ..." line, and the FCU reported
# EK3_SRC1_POSXY 3 (GPS) / VISO_TYPE 0 / GUID_OPTIONS 0 while claiming to have loaded the baro file).
# The parameter check below is what caught it.  One comma-separated list is the documented form.
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model "${SITL_MODEL:-x}" --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 \
    --defaults "$REPO/params/apm_lio_only.parm,$REPO/params/sitl_basics.parm,$REPO/params/phase_a_posz_baro.parm" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null
trap 'kill $SITL 2>/dev/null; sleep 1; kill -9 $SITL 2>/dev/null' EXIT

for i in $(seq 1 24); do
    grep -q TruthTap "$OUT/sitl.log" 2>/dev/null && break
    kill -0 $SITL 2>/dev/null || { echo "SITL died:"; tail -15 "$OUT/sitl.log"; exit 1; }
    sleep 0.5
done
grep -q TruthTap "$OUT/sitl.log" || { echo "tap never reported"; exit 1; }
echo "SITL stepping (pid $SITL)"

# verify the parameters actually reached the FCU (PLAN.md rule 3): the source map is the whole point
python3 - "$OUT" <<'PY'
import sys, time
from pymavlink import mavutil
out = sys.argv[1]
m = mavutil.mavlink_connection("tcp:127.0.0.1:5760")
m.wait_heartbeat(timeout=10)
want = {"VISO_TYPE": 1.0, "EK3_SRC1_YAW": 6.0, "EK3_SRC1_POSXY": 6.0, "EK3_SRC1_POSZ": 1.0,
        "EK3_SRC1_VELZ": 0.0, "EK3_VIS_VERR_MAX": 0.9, "GUID_OPTIONS": 8.0, "GPS1_TYPE": 0.0}
got = {}
for name in want:
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
t_end = time.monotonic() + 8
while time.monotonic() < t_end and len(got) < len(want):
    msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
    if msg is not None:
        n = msg.param_id.decode() if isinstance(msg.param_id, bytes) else msg.param_id
        if n in want:
            got[n] = msg.param_value
print("parameter check (what the FCU reports, not what the file says):")
bad = 0
for n, v in want.items():
    have = got.get(n)
    ok = have is not None and abs(have - v) < 1e-6
    bad += 0 if ok else 1
    print("   [%s] %-18s want %-6s got %s" % ("OK" if ok else "BAD", n, v, have))
m.close()
sys.exit(1 if bad else 0)
PY
PARAM_RC=$?
echo "parameter check rc=$PARAM_RC (0 = every source-map value reached the FCU)"

# --print, not --ros: this test needs the fan-out and the CSV, and NOT a ROS graph (an earlier version
# used --ros without sourcing /opt/ros/humble, so the receiver died on import and the test then reported
# "no tap samples" -- a harness fault wearing the costume of a broken tap).
python3 "$REPO/sim/truth_rx.py" --print --duration 0 --parked --port $TAP --fanout $FAN \
    --csv "$OUT/tap.csv" --save "$OUT/truth_report.txt" > "$OUT/truth_rx.log" 2>&1 &
RX=$!
sleep 2
if ! kill -0 "$RX" 2>/dev/null; then
    echo "truth_rx died at startup:"; tail -10 "$OUT/truth_rx.log"; exit 1
fi
python3 "$REPO/tools/test-fcu-link.py" --tap-port $FAN --alt 0.8 --hold 15 \
    > "$OUT/fcu_link.log" 2>&1
LINK_RC=$?
cat "$OUT/fcu_link.log"
kill "$RX" 2>/dev/null
echo
echo "== SITL console, last lines =="
tail -6 "$OUT/sitl.log"
echo
echo "run dir: $OUT"
exit $(( PARAM_RC + LINK_RC ))
