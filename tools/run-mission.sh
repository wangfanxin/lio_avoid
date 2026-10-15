#!/usr/bin/env bash
# Mission A, one flight: SITL + the simulated MID-360 + the LIO + the PLAN box + the BRIDGE + the
# SUPERVISOR.  Since F2 there is no FSM: the plan box executes, the bridge transports, the supervisor
# owns the flight commands.  Three processes, three jobs, one writer per MAVLink message class.
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
# ONE-VARIABLE EXPERIMENTS WITHOUT TOUCHING A FROZEN FILE.  `EXTRA_PARM` is a SINGLE parameter file
# appended LAST to `--defaults` (the firmware applies them in order, so the last wins), and it is how a
# parameter A/B is flown while `params/*.parm` stays byte-for-byte what the frozen manifest lists: the file
# lives outside the repo and its PATH IS PRINTED in the revision block, so the sortie's own log names the
# configuration it overrode.  MEASURED 2026-10-13 and kept: two `--defaults` arguments do not accumulate
# (the second REPLACES the first) -- which is why this is ONE comma-joined list and not a second flag.
EXTRA_PARM="${EXTRA_PARM:-}"
EXTRA_ARG=""
EXTRA_NAME="none"
if [ -n "$EXTRA_PARM" ]; then
    [ -f "$EXTRA_PARM" ] || { echo "no EXTRA_PARM file at $EXTRA_PARM" >&2; exit 1; }
    EXTRA_ARG=",$EXTRA_PARM"
    EXTRA_NAME="$EXTRA_PARM ($(grep -c . "$EXTRA_PARM") line(s))"
fi
# THE DWELL, AND WHY THE HARNESS SETS IT (F1, 2026-10-16).  The owner's requirement is a hover at every
# corner, and the number is 5.0 s (`docs/PLAN.md`).  It used to be passed only when the caller exported
# RECT_HOLD_S; with it unset the producer's `--hold-s -1` read `mission.hold_s` out of the FROZEN world
# file, which is **3.0** -- and STATUS row 42 is the measured consequence of that value: the default
# configuration flew one leg and landed.  The default is now the requirement, and which one was used is
# PRINTED at start-up rather than inferred from the log afterwards.
if [ -n "${RECT_HOLD_S:-}" ]; then
    RECT_HOLD_FROM="the CALLER (RECT_HOLD_S=$RECT_HOLD_S)"
else
    RECT_HOLD_S=5.0
    RECT_HOLD_FROM="THE HARNESS DEFAULT (RECT_HOLD_S was not set)"
fi
WORLD_HOLD_S="$(python3 -c 'import sys, yaml; print(yaml.safe_load(open(sys.argv[1]))["mission"].get("hold_s", "unset"))' "$WORLD" 2>/dev/null || echo unreadable)"

# THE TAKE-OFF PROFILE, AND IT IS PRINTED BECAUSE IT CHANGED WHAT IS FLOWN.  Two mechanisms are in the
# code; this harness flies the OWNER'S (2026-10-15): the take-off command asks for only ~0.2 m ("just leave
# the ground cleanly"), the go goes out the moment the vehicle is airborne, and the CLIMB TO THE MISSION
# ALTITUDE IS THE PLAN'S -- a vertical first target held for the A4 window.  `TAKEOFF_COMMAND_M` equal to
# the mission altitude restores the other profile (the FCU's own take-off controller flies the whole climb),
# and `VERTICAL_FIRST=0` removes the plan's climb leg with it.
TAKEOFF_COMMAND_M="${TAKEOFF_COMMAND_M:-0.2}"
VERTICAL_FIRST="${VERTICAL_FIRST:-1}"
VERTICAL_HOLD_S="${VERTICAL_HOLD_S:-3.0}"
# APPROACH SHAPING (the B1 remedy, 2026-10-15): 0.0 = off, the mission as it has always been flown.  A
# positive value inserts ONE extra waypoint that far before each tour corner, ON the leg, with no dwell of
# its own -- the vehicle must come to a SETTLED stop there (the settle gate is unchanged), so the final
# approach into the corner starts from rest instead of braking from 1.7 m/s.  WHY: B1's box is made by the
# under-damped ring that braking excites, measured at 35-93 mm past each wall with the extremes at ~0 m/s
# (`docs/metrics/b1-mechanism-20261016.txt`).
SHAPING_M="${SHAPING_M:-0.0}"
SHAPING_ARG=""
if awk "BEGIN{exit !($SHAPING_M > 0)}"; then
    SHAPING_ARG="--approach-shaping-m $SHAPING_M"
fi
if awk "BEGIN{exit !($TAKEOFF_COMMAND_M == $TAKEOFF_ALT)}"; then
    TAKEOFF_PROFILE="FCU-TARGET -- packet.z = ${TAKEOFF_COMMAND_M} m VERBATIM, the FCU's own take-off controller flies the whole climb"
    VERTICAL_ARG=""
    SUP_TRIGGER="--trigger-after-hold"
else
    TAKEOFF_PROFILE="SETPOINT-DRIVEN CLIMB (the owner's design) -- packet.z = ${TAKEOFF_COMMAND_M} m only; the ${TAKEOFF_ALT} m mission altitude is the PLAN's, the go goes out ON AIRBORNE"
    if [ "$VERTICAL_FIRST" = "1" ]; then
        VERTICAL_ARG="--vertical-first-m $TAKEOFF_ALT --vertical-hold-s $VERTICAL_HOLD_S"
    else
        VERTICAL_ARG=""
    fi
    SUP_TRIGGER="--trigger-on-airborne"
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
# THIS REPO'S OWN REVISION FIRST.  MEASURED 2026-10-15: a sortie's log named the ArduPilot revision, the
# LIO's revision and every parameter file, but NOT the commit that produced the PLAN box and the BRIDGE --
# so "which code flew" could only be reconstructed from the wall clock, and a fix that was in the tree but
# not in the commit looked identical to one that was.  A dirty tree is stated here, not hidden.
echo "   this repo : $(git -C "$REPO" rev-parse --short HEAD) $(git -C "$REPO" log -1 --format=%s | cut -c1-58)$(mt=$(git -C "$REPO" status --porcelain --untracked-files=no 2>/dev/null | wc -l); ut=$(git -C "$REPO" status --porcelain --untracked-files=all 2>/dev/null | grep -c '^??'); if [ "${mt:-0}" -gt 0 ]; then echo "  *** TREE DIRTY: $mt tracked path(s) modified ***"; else echo "  (tracked tree clean${ut:+, $ut untracked path(s)})"; fi)"
echo "   ardupilot : $(git -C "$AP" describe --tags --always) $(git -C "$AP" rev-parse --short HEAD) (tap applied: $(git -C "$AP" --no-pager diff --stat | tail -1 | tr -s ' '))"
echo "   super_lio : $(git -C /home/fanxin/lio_avoid/src/Super-LIO rev-parse --short HEAD)  liblio $(md5sum "$WS/install/super_lio/lib/liblio.so" | cut -c1-16)"
echo "   config    : lio/config/livox_360.yaml + apm_lio_only,sitl_basics,${ALT_MODE}_posz,$MISSION_NAME,$EKF_NAME"
echo "   override  : $EXTRA_NAME   (appended LAST to --defaults: the last value wins)"
echo "   world     : $WORLD  ($CORNERS)"
echo "   airframe  : sim model '$SITL_MODEL' against FRAME_TYPE 1 (X) -- the two must agree (see SITL_MODEL above)"
printf 'model=%s\nfirmware=FRAME_CLASS 1 / FRAME_TYPE 1 (X mixer)\nrule=the SITL model names the motor geometry; it must match FRAME_TYPE\n' \
    "$SITL_MODEL" > "$OUT/airframe.txt"
echo "   sensor    : the SAME world file ($REPO/sim/lidar_imu.py --world \"$WORLD\")"
echo "   geofence  : $GEOFENCE_M m, take-off altitude $TAKEOFF_ALT m"
echo "   dwell     : ${RECT_HOLD_S} s at every corner -- from ${RECT_HOLD_FROM}; the world file's"
echo "               mission.hold_s is ${WORLD_HOLD_S} and is deliberately NOT what a default sortie flies"
echo "   take-off  : ${TAKEOFF_PROFILE}"
if [ -n "$VERTICAL_ARG" ]; then
    echo "               the plan's first target is a VERTICAL leg to ${TAKEOFF_ALT} m, held ${VERTICAL_HOLD_S} s"
    echo "               (that hold IS the A4 window, which is what keeps A4 comparable with docs/metrics/)"
elif awk "BEGIN{exit !($TAKEOFF_COMMAND_M != $TAKEOFF_ALT)}"; then
    echo "               VERTICAL_FIRST=0: the plan has NO climb leg, so the first setpoint is corner 1 and"
    echo "               the aircraft climbs IN TRANSIT -- the A4 window is then a climb, not a hover"
fi
echo "   run dir   : $OUT"
echo

cd "$OUT" || exit 1
# TWO ENDPOINTS SINCE F2, and that is a capability this repository proved on the bench BEFORE using it:
# `tools/test-second-endpoint.py` (run by `_stage/f2-endpoint-bench.sh`) connects two clients with their own
# 511 rate requests and shows no interference, one vehicle, and a command on one visible as state on the
# other.  serial0 (5760) is the BRIDGE's -- setpoints 84 and the estimator feed 331; serial1 (5761) is the
# SUPERVISOR's -- every class-76 command.  One writer per message class is the rule that makes this safe
# without any runtime arbitration (`_stage/refactor-bridge-plan.md` section 4.2).
setsid nohup env LIO_TRUTH_TAP=$TAP "$BIN" --model "$SITL_MODEL" --speedup 1 -I0 \
    --home -35.363261,149.165230,584,0 --serial0 tcp:5760 --serial1 tcp:5761 \
    --defaults "$REPO/params/apm_lio_only.parm,$REPO/params/sitl_basics.parm,$ALT_PARM,$MISSION_PARM$EKF_ARG$EXTRA_ARG" \
    -w > sitl.log 2>&1 &
SITL=$!
cd - > /dev/null

# STOP THE FLIGHT STACK, AND WAIT FOR THE TWO PROCESSES THAT *WRITE EVIDENCE* TO FINISH WRITING IT.
#
# THE ORDERING IS THE ROW (MEASURED 2026-10-15).  `tools/metrics.py` used to run while `truth_rx.py` was
# still in its receive loop and while SITL was still appending the FCU's BIN, so metrics.txt was computed
# against inputs that were still growing: the run-summary sample count and the whole A4 block were quoted
# from a TRUNCATED tap and a SHORT BIN.  Every acceptance row reproduced anyway -- A1/A2/B1/B2/B3/C1/C2 are
# computed over the route window and the missing tail is after it -- but those two lines are read by a
# human, and "the evidence file does not agree with the evidence" is the failure mode this project treats
# as the worst one.
#
# MEASURED, on 7 committed sorties: metrics.txt quotes 2.9-3.6 s less tap than the file now holds (RECT9
# 90294 vs its own truth_report.txt's "of 94020 seen"; RECT1 62516 vs 65092).  A per-row flush in the
# recorder is NECESSARY BUT NOT SUFFICIENT, and that was measured too: with `buffering=1` in place, a
# passive watcher on a real sortie caught metrics.txt being written 1.1 s BEFORE the tap stopped growing
# (RECT13).  The recorder stops when SITL stops sending, so the stop must come first.  RECT14 proves the
# result: summary 61289 = truth_report's "of 61289 seen" = the closed tap's own row count, and metrics.txt
# mtime AFTER tap.csv instead of before it (this function's A4 row needed the same treatment for the BIN).
stop_stack() {
    for p in "${FSM_PID:-}" "${BRIDGE_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${RECT_PID:-}"; do
        [ -n "$p" ] && kill "$p" 2>/dev/null
    done
    kill "$REC_PID" 2>/dev/null
    kill "$RECC_PID" 2>/dev/null
    # SITL is stopped BEFORE the tap recorder is waited for, because the recorder's own stop condition is
    # "SITL has stopped sending"; and its BIN must be closed before A4 reads it, which is why it is waited
    # for in its own right.  5 s is ~5x what SITL's clean exit has been measured to take; the SIGKILL below
    # is the backstop, not the plan.
    if [ -n "${SITL:-}" ]; then
        kill "$SITL" 2>/dev/null
        for _ in $(seq 1 50); do
            kill -0 "$SITL" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$SITL" 2>/dev/null; then
            kill -9 "$SITL" 2>/dev/null
            echo "WARNING: SITL did not stop within 5 s and was killed -- the FCU's BIN may be short"
        fi
    fi
    # SIGTERM is handled gracefully by the recorder (`_install_stop_handler` sets a flag; the loop then
    # closes the CSV and writes truth_report.txt), so no sample is lost to a signal -- and if it does not
    # stop, this says so instead of silently truncating the tap.  MEASURED (s26-stop-ab): with the silent-tap
    # stop check in sim/truth_rx.py the recorder leaves 0.22 s after SIGTERM and writes its report; without
    # it, it was still running 15 s later -- which is why the check had to be fixed in the same round as
    # this ordering, and why the two are one commit.
    if [ -n "${RX_PID:-}" ]; then
        kill "$RX_PID" 2>/dev/null
        for _ in $(seq 1 50); do
            kill -0 "$RX_PID" 2>/dev/null || break
            sleep 0.1
        done
        if kill -0 "$RX_PID" 2>/dev/null; then
            kill -9 "$RX_PID" 2>/dev/null
            echo "WARNING: the truth recorder did not stop within 5 s and was killed -- tap.csv may be short"
        else
            echo "truth recorder and SITL stopped; tap.csv and the FCU's BIN are complete"
        fi
    fi
    # anything that ignored SIGTERM gets SIGKILL; `cleanup` repeats this on the way out, so nothing is left
    # behind whether the script is exiting normally or not.
    for p in "${FSM_PID:-}" "${BRIDGE_PID:-}" "${TRAJ_PID:-}" "${LIO_PID:-}" "${SENSOR_PID:-}" "${SITL:-}" "${RX_PID:-}" \
             "${REC_PID:-}" "${RECC_PID:-}" "${RECT_PID:-}"; do
        [ -n "$p" ] && kill -9 "$p" 2>/dev/null
    done
    return 0
}

cleanup() {
    stop_stack
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
# WHICH NAVIGATION SOURCE, AND THEREFORE WHICH CALIBRATION -- decided HERE, before the recorders start,
# because tools/record-traj.py needs the same L->W calibration the FSM and the producer use (it writes
# every geometric column in W; without it clr_m is nan and says so).
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
# THE PLAN, recorded next to the truth.  Without it "the gate cannot see the truth" is an argument, not a
# measurement: MEASURED 2026-10-14 the truth spent 55.9 % of a 12-stop flight inside the 0.20 m contract
# while every published path had been verified clear, and there was no record of those paths to compare
# against.  tools/record-traj.py writes every plan message plus its own minimum clearance -- and since
# 2026-10-15 it takes the SAME --calib-file as the FSM and the producer, because the plan arrives in L and
# the world file is in W: without it `clr_m` read -0.2972 m for a plan that is +0.6000 m clear.
#
# F1 (2026-10-16): THE TOPIC MOVED WITH THE INTERFACE, IN THE SAME CHANGE.  The plan box no longer
# publishes `nav_msgs/Path`; its output is the command stream on /plan/cmd, and this recorder now reads
# that (`--msg position_cmd`).  Design risk R5 is exactly this line: had the recorder stayed on the Path,
# every run would have lost its plan-clearance column and nobody would have noticed until an audit.
# ... and `traj.csv.cmd.csv` beside it: EVERY message, no de-duplication, so the REFERENCE's own
# smoothness is measurable at the rate it is published (the de-duplicated points file holds one
# point per distinct command, which is what the Path-era readers need and is NOT enough to answer
# "is the reference wobbling, or is the aircraft not tracking it?").
python3 "$REPO/tools/record-traj.py" --topic /plan/cmd --msg position_cmd --csv "$OUT/traj.csv" \
    --world "$WORLD" $TRAJ_CALIB_ARG > "$OUT/rectraj.log" 2>&1 &
RECT_PID=$!
sleep 3

echo "== flying =="
# THE PLAN BOX, which since F1 is ALSO THE EXECUTOR: `planner/rectangle_source.py` publishes the mission's
# five waypoints as a `quadrotor_msgs/PositionCommand` stream on /plan/cmd (100 Hz, its own wall timer),
# decides ARRIVAL + SETTLE + DWELL itself from `/lio/robo/odom`, and declares the tour complete with
# `trajectory_flag = COMPLETED`.
python3 "$REPO/planner/rectangle_source.py" --world "$WORLD" $TRAJ_CALIB_ARG \
    --hold-s "$RECT_HOLD_S" $VERTICAL_ARG $SHAPING_ARG \
    ${CMD_ARGS:-} > "$OUT/traj.log" 2>&1 &
TRAJ_PID=$!

# THE BRIDGE, ITS OWN PROCESS AND ITS OWN ENDPOINT (serial0).  It carries the estimator feed (331) from the
# LIO and the plan's setpoints (84), and it sends NO class-76 command.  It waits for the supervisor's
# /plan/trigger before it puts a setpoint on the wire, and it takes the plan OFF the wire when the stream
# says COMPLETED or when the FCU leaves GUIDED.
python3 "$REPO/bridge/bridge_node.py" --mav "tcp:127.0.0.1:5760" $CALIB_ARG \
    --nav-hz 48 --wire-hz 50 ${BRIDGE_ARGS:-} \
    --setpoint-log "$OUT/setpoints.csv" --wire-log "$OUT/wire.csv" \
    > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2
kill -0 "$BRIDGE_PID" 2>/dev/null || { echo "the bridge process died:"; tail -20 "$OUT/bridge.log"; exit 1; }

# THE SUPERVISOR, ITS OWN PROCESS AND ITS OWN ENDPOINT (serial1).  Preflight, arm, take-off, the go, the
# four gates, LAND and the NAMED abort.  Its inputs are the truth and the LINK, never the LIO in flight.
# It is last because its exit is the sortie's result, and because the two processes it watches must already
# be publishing when it starts.
python3 "$REPO/supervisor/supervisor.py" --mav "tcp:127.0.0.1:5761" $CALIB_ARG \
    --alt "$TAKEOFF_ALT" --takeoff-command-m "$TAKEOFF_COMMAND_M" \
    --hold 3.0 --geofence-m "$GEOFENCE_M" $SUP_TRIGGER ${SUP_ARGS:-} \
    --phase-log "$OUT/phase.csv" > "$OUT/supervisor.log" 2>&1
SUP_RC=$?
cat "$OUT/supervisor.log"
FSM_RC=$SUP_RC

# ALL THREE PRODUCER PROCESSES ARE STOPPED AND WAITED FOR BEFORE THE METRICS RUN (see stop_stack above).
# This is the whole fix: after it, tap.csv, the FCU's BIN and lio.csv are FROZEN, so the file this script
# writes and the file a reader can re-run against are the same file.
stop_stack
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
echo "run dir: $OUT   (supervisor rc=$SUP_RC, metrics rc=$MET_RC)"
exit $SUP_RC
