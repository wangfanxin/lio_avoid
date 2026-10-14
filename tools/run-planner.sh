#!/usr/bin/env bash
# Bring up the ego-planner chain and NOTHING ELSE.
#
# It starts exactly two processes -- the planner and planner/ego_adapter.py -- and prints what they
# publish.  It does NOT start the simulator, the LIO, RViz or the FSM: whoever is flying starts those,
# and `/planner/traj` is the same interface `planner/rectangle_source.py` publishes for mission A, so
# the FSM does not know or care which of the two is producing it.
#
#   1. ego_planner_node   subscribes  /cloud_registered (L)  +  /lio/robo/odom (L)
#                         publishes   planning/bspline (traj_utils/Bspline, L)
#   2. ego_adapter.py     subscribes  /goal_pose (W)  +  planning/bspline (L)  +  /lio/robo/odom (L)
#                         publishes   /planner/traj (nav_msgs/Path, frame lio_world)
#                                     /move_base_simple/goal (L, the planner's goal input)
#
# WHAT IT NEEDS UPSTREAM: /cloud_registered and /lio/robo/odom from the LIO.  Without odometry the
# adapter defers goals (and the planner, unpatched, would segfault on one -- see patches/0002).
#
# usage: tools/run-planner.sh [--goal E,N,U] [--no-goal] [--rate HZ] [--max-vel V] [--check]
#   --goal E,N,U   goal in W (ENU, origin = the pad).  Default: the world file's mission_b.goal.
#                  It is published ONCE, --goal-timeout seconds after start, and only if nothing has
#                  published /goal_pose by then, so a real mission controller always wins.
#   --no-goal      never publish a goal; just bring the chain up.
#   --check        print the topic list and exit without starting anything.
set +u

HERE="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash
LIO_WS=/home/fanxin/lio_avoid/lio_ws
PLANNER_WS=/home/fanxin/lio_avoid/planner_ws
CALIB=/mnt/d/distill_self/_stage/calib.json
WORLD="$HERE/worlds/forest_all.yaml"
PARAMS="$HERE/params/ego_planner_humble.yaml"

GOAL=""
CHECK=0
RATE=4.0
MAXVEL=1.0
while [ $# -gt 0 ]; do
  case "$1" in
    --goal) GOAL="$2"; shift 2 ;;
    --no-goal) GOAL="none"; shift ;;
    --rate) RATE="$2"; shift 2 ;;
    --max-vel) MAXVEL="$2"; shift 2 ;;
    --check) CHECK=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

cat <<EOF
=== ego-planner chain (ROS 2 Humble, ArduPilot SITL is started separately) ===
  repo          $HERE
  planner       $PLANNER_WS/install/ego_planner/lib/ego_planner/ego_planner_node
  adapter       $HERE/planner/ego_adapter.py
  params        $PARAMS
  calibration   $CALIB
  world         $WORLD
  goal          ${GOAL:-from worlds/forest_all.yaml mission_b.goal}

  publishes:
    /planner/traj                 nav_msgs/Path          frame lio_world   >= 2 Hz, points at 50 Hz of
                                                                          trajectory time
    /move_base_simple/goal        geometry_msgs/PoseStamped  frame lio_world  (the planner's input)
    planning/bspline              traj_utils/Bspline     frame lio_world   (the planner itself)
    grid_map/occupancy[_inflate]  sensor_msgs/PointCloud2
    planning/data_display         traj_utils/DataDisp
    /drone_0_planning/swarm_trajs traj_utils/MultiBsplines
  subscribes:
    /cloud_registered             sensor_msgs/PointCloud2  (L, 10 Hz)  -> the planner's map
    /lio/robo/odom                nav_msgs/Odometry        (L, 200 Hz) -> state + map centre
    /goal_pose                    geometry_msgs/PoseStamped (W)        -> the adapter's goal input
EOF
if [ "$CHECK" = "1" ]; then exit 0; fi

set -e
[ -f "$ROS_SETUP" ] || { echo "no $ROS_SETUP" >&2; exit 1; }
[ -d "$PLANNER_WS/install" ] || {
  echo "the planner is not built: run the build in docs/STATUS.md (planner_ws) first" >&2; exit 1; }
# shellcheck disable=SC1090
source "$ROS_SETUP"
[ -f "$LIO_WS/install/setup.bash" ] && source "$LIO_WS/install/setup.bash"
source "$PLANNER_WS/install/setup.bash"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<default>}   (upstream suggests rmw_cyclonedds_cpp; this"
echo "  script does NOT change it, because the LIO/FSM processes would have to change with it)"

PIDS=""
cleanup() {
  echo
  echo "=== stopping ==="
  for p in $PIDS; do kill -INT "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup INT TERM

ros2 run ego_planner ego_planner_node --ros-args \
  --params-file "$PARAMS" \
  -p "manager/max_vel:=$MAXVEL" \
  -r odom_world:=/lio/robo/odom \
  -r grid_map/odom:=/lio/robo/odom \
  -r grid_map/cloud:=/cloud_registered &
PIDS="$PIDS $!"

AGOAL=()
if [ "$GOAL" != "none" ]; then
  if [ -z "$GOAL" ]; then
    GOAL="$("$HERE/tools/world-goal.py" --world "$WORLD" 2>/dev/null || echo "3.0,4.0,2.0")"
    echo "goal from the world file: $GOAL"
  fi
  IFS=',' read -r gx gy gz <<EOF
$GOAL
EOF
  AGOAL=(--auto-goal "$gx" "$gy" "$gz")
fi

python3 "$HERE/planner/ego_adapter.py" \
  --calib-file "$CALIB" \
  --world "$WORLD" \
  --rate "$RATE" \
  "${AGOAL[@]}" &
PIDS="$PIDS $!"

echo "=== running (ctrl-c to stop) ==="
wait
