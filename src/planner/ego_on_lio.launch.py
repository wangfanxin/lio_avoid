#!/usr/bin/env python3
"""ego-planner-swarm ON THIS STACK: our LIO in, the `/plan/cmd` contract out.

WHY THIS FILE EXISTS.  `advanced_param.launch.py` starts exactly ONE node (`ego_planner_node`) with the
tuning of ego's OWN simulator, and the launch files that DO start `traj_server`
(`single_run_in_sim.launch.py:142-154`, `run_in_sim.launch.py:88-100`) also start ego's own simulator
(`so3_quadrotor_simulator`, `so3_control`, `poscmd_2_odom`, `pcl_render_node`), which would fight ArduPilot
for the same topics.  Neither is usable here, so this file starts the planner and NOTHING ELSE.

WHAT STARTS: one node, `drone_<id>_ego_planner_node`, fed our cloud and our odometry.

WHAT DELIBERATELY DOES NOT START, and each one is a decision rather than an omission:

  * `traj_server` -- the ONLY producer of `quadrotor_msgs/PositionCommand` in that tree
    (`plan_manage/src/traj_server.cpp:211,244`; `grep -rn PositionCommand` matches nothing else).  It is not
    started because the CONTRACT is produced on our side instead: `src/planner/ego_source.py` samples the
    same B-spline itself and publishes `/plan/cmd` with `trajectory_id` per WAYPOINT and a synthesised
    COMPLETED.  Two reasons, both measured: `traj_server` hard-codes `frame_id="world"` (`:210`) and writes
    `trajectory_flag` exactly once, to READY (`:211`) -- it can never say the mission is over -- and its
    `time_forward` parameter defaults to **-1.0** (`:261`), which evaluates the yaw look-ahead one second in
    the PAST.  Route B (`traj_server` + remap `position_cmd:=/plan/cmd`) remains possible: start it
    yourself, remap it, and set `traj_server/time_forward: 1.0`.
  * ego's simulator -- ArduPilot is the flight controller here.
  * `map_generator` / `mockamap` -- the world is `worlds/pillars.yaml`, flown by SITL.
  * `rosmsg_tcp_bridge` -- a multi-machine swarm relay (TCP 8080 ring + UDP 8081 broadcast), launched by no
    launch file in that tree, not built in `planner_ws`, and it never touches `PositionCommand`.  This
    stack has its own bridge, with its own two MAVLink endpoints.

THE WIRING THAT MATTERS, and it is the one item that silently ruins a sortie if it is missed: the FSM reads
`twist.twist.linear` from `odom_world` (`ego_replan_fsm.cpp:443-445`) and uses it as the `> 0.1 m/s` gate to
leave EMERGENCY_STOP (`:1047`), while the map reads ONLY `pose.pose.position` (`grid_map.cpp:799-801`).  Our
`/lio/robo/odom` carries a pose and an identically ZERO twist (Super-LIO fills only the pose), so:

    odom_world      -> /lio/odom        (10 Hz, and the ONLY one of our odometry topics with a real twist)
    grid_map/odom   -> /lio/robo/odom   (200 Hz, pose only -- which is all the map reads)

Those are two independent remap rules in the upstream launch file too (`advanced_param.launch.py:96,108`),
so this is the intended way to wire it, not a trick.

NO TF, ANYWHERE.  `grep -rn tf2 plan_env plan_manage` is empty and `tf2` is in none of their `package.xml`
files: the cloud is compared to the odometry position AS-IS, so both must already be in L.  Ours are.
`grid_map/frame_id` is only a LABEL on outgoing clouds (`grid_map.cpp:957,1007`).

THE MAP BOX IS FIXED AND CENTRED ON THE L ORIGIN (`grid_map.cpp:92`), and a point OUTSIDE it reads as
OCCUPIED: `getInflateOccupancy` returns -1 there and every caller casts it to `bool`
(`grid_map.h:357-366`, `dyn_a_star.h:62`).  The upstream defaults are 42 x 30 x 5 m, which is ~94 MB of
map plus ~56 MB of A* pool for an arena of +-6 m -- so the sizes below are an ARGUMENT, not a constant.

usage:  ros2 launch ego_planner ego_on_lio.launch.py
        (the goals come from `src/planner/ego_source.py`, which also produces /plan/cmd)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------ arguments
    args = [
        DeclareLaunchArgument('drone_id', default_value='0',
                              description='must be >= 0: a negative id switches the swarm topic name'),
        DeclareLaunchArgument('odom_vel_topic', default_value='/lio/odom',
                              description='10 Hz, pose AND the twist the FSM needs for its >0.1 m/s gate'),
        DeclareLaunchArgument('odom_topic', default_value='/lio/robo/odom',
                              description='200 Hz, pose only: all the MAP reads'),
        DeclareLaunchArgument('cloud_topic', default_value='/lio/cloud_world',
                              description='the registered scan, ALREADY in L (no TF is applied)'),
        DeclareLaunchArgument('bspline_topic', default_value='planning/bspline',
                              description='what ego publishes and our adapter samples'),
        DeclareLaunchArgument('map_size_x', default_value='16.0',
                              description='the map box is FIXED, centred on the L origin'),
        DeclareLaunchArgument('map_size_y', default_value='16.0',
                              description='-- and outside it reads as OCCUPIED'),
        DeclareLaunchArgument('map_size_z', default_value='5.0',
                              description='ground_height .. virtual_ceil_height'),
        DeclareLaunchArgument('ground_height', default_value='-0.50',
                              description='THE MAP Z ORIGIN.  It was -0.02 until a MEASURED defect: at -0.02 the '
                                          'map box is [-0.02, 4.98], so a vehicle that touches down -- whose '
                                          'odometry z reads about -0.02, exactly the ground plane -- falls '
                                          'OUTSIDE its own map, isInMap fails, getInflateOccupancy returns -1, '
                                          'and the collision check reads -1 as OCCUPIED '
                                          '(bspline_optimizer.cpp:1329).  DIAG12: 1 058 710 "the drone is in '
                                          'obstacle", GEN_NEW_TRAJ 19 049 against EXEC_TRAJ 207, 370 s of '
                                          'deadlock.  At -0.50 the box is [-0.50, 4.50]: a landed vehicle is '
                                          'inside it, the 4.2 m pillars are still inside it, and the virtual '
                                          'ceiling lands at 2.9 m, above the 1.88 m the missions fly.'),
        DeclareLaunchArgument('virtual_ceil_height', default_value='3.00',
                              description='our ceiling in L: above the 2.50 m mission altitude, below the 6.60 m geofence'),
        DeclareLaunchArgument('max_vel', default_value='2.0',
                              description='SOFT: a quadratic penalty, not a constraint'),
        DeclareLaunchArgument('max_acc', default_value='3.0', description='SOFT, likewise'),
        DeclareLaunchArgument('planning_horizon', default_value='7.5', description='both fsm/ and manager/'),
    ]

    params = {
        # ---- the FSM: one goal at a time on /move_base_simple/goal, and it must WAIT for one
        'fsm/flight_type': 1,              # MANUAL_TARGET: our adapter owns the tour (upstream: 2, preset)
        'fsm/realworld_experiment': True,  # have_trigger_ = !flag_realworld_experiment_ (`ego_replan_fsm.cpp:35`)
        'fsm/thresh_replan_time': 1.0,
        'fsm/thresh_no_replan_meter': 1.0,
        'fsm/planning_horizon': LaunchConfiguration('planning_horizon'),
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/fail_safe': True,
        # kept in the upstream shape, but inert under flight_type 1 (the preset list is not read).  The
        # in-tree patch's own additions (`fsm/waypoint0_dwell_s`, `waypoint_advance_m`, `waypoint_loop`,
        # `waypoint_start_alt`) are DELIBERATELY left at their declared defaults: they belong to the patched
        # waypoint sequencer, which `flight_type 1` does not enter.
        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': 0.0, 'fsm/waypoint0_y': 0.0, 'fsm/waypoint0_z': 0.0,

        # ---- the map: a boolean inflated grid, 0.1 m voxels, and NO distance query anywhere
        'grid_map/resolution': 0.1,
        'grid_map/map_size_x': LaunchConfiguration('map_size_x'),
        'grid_map/map_size_y': LaunchConfiguration('map_size_y'),
        'grid_map/map_size_z': LaunchConfiguration('map_size_z'),
        'grid_map/local_update_range_x': 5.5,
        'grid_map/local_update_range_y': 5.5,
        'grid_map/local_update_range_z': 4.5,
        'grid_map/obstacles_inflation': 0.099,   # ONE VOXEL.  The two-cell route is now CLOSED, and not for the
                                                # old reason: `cloud_relay.py --self-blind` did fix the defect
                                                # that made two cells hang -- `the drone is in obstacle` fell
                                                # from 112 729 to **3 107** and the sortie completed (DIAG16) --
                                                # but the flown result got WORSE, not better: truth minimum
                                                # -0.100 m and **27 999 of 100 127** samples inside the C2
                                                # footprint (15.8%), the worst of any ego crossing.  So the
                                                # 0.20 m footprint is not reachable by raising this parameter,
                                                # with or without the bubble, and the map-side levers are
                                                # exhausted.  See STATUS row 84.
                                                # ONE CELL is flown and healthy (DIAG14: rc=0, `a star error` 1,
                                                # truth min -0.222 m); its best crossing cleared by +0.150 m.
        'grid_map/local_map_margin': 10,
        'grid_map/ground_height': LaunchConfiguration('ground_height'),
        'grid_map/virtual_ceil_height': LaunchConfiguration('virtual_ceil_height'),
        # NO DEPTH CAMERA.  `pose_type` 1 builds a message_filters synchroniser that can never fire, and
        # `use_depth_filter` True arms a watchdog that would then never be fed -- so both are off, and the
        # cloud path (which never raycasts: `occupancy_buffer_` is never written) is the only input.
        'grid_map/use_depth_filter': False,
        'grid_map/pose_type': 0,
        'grid_map/frame_id': 'lio_world',        # a LABEL on outgoing clouds only
        'grid_map/cx': 321.04638671875,          # unused without depth, kept so the set matches upstream
        'grid_map/cy': 243.44969177246094,
        'grid_map/fx': 387.229248046875,
        'grid_map/fy': 387.229248046875,
        # DEAD PARAMETER, named here so nobody waits for output that will never come: `grid_map/show_occ_time`
        # is declared (`grid_map.cpp:42`) and read (`:79`) but its only USE is a COMMENTED-OUT log
        # (`:750-752`).  There is no way to time ego's occupancy update from inside the node -- a round of
        # this project assumed otherwise and the plan_env audit refuted it.  It is set anyway, so that this
        # file's parameter set can still be diffed against upstream's and come out equal.
        'grid_map/show_occ_time': False,
        # The DEPTH-FILTER and RAY-FUSION blocks are INERT here (`use_depth_filter` False, `pose_type` 0 --
        # nothing on the depth path is ever called), and they are set anyway so that this file's parameter
        # set can be diffed against `advanced_param.launch.py` and come out equal except where our world
        # decides otherwise.  MEASURED: the first version of this file omitted `optimization/max_vel`,
        # `optimization/max_acc` and the four `optimization/lambda_*` weights that upstream DOES set, which
        # would have left the optimiser on its C++ defaults with nothing said -- found by comparing the
        # parameter list the running node declares against the names this file sets.
        'grid_map/depth_filter_tolerance': 0.15,
        'grid_map/depth_filter_maxdist': 5.0,
        'grid_map/depth_filter_mindist': 0.2,
        'grid_map/depth_filter_margin': 2,
        'grid_map/k_depth_scaling_factor': 1000.0,
        'grid_map/skip_pixel': 2,
        'grid_map/p_hit': 0.65,
        'grid_map/p_miss': 0.35,
        'grid_map/p_min': 0.12,
        'grid_map/p_max': 0.90,
        'grid_map/p_occ': 0.80,
        'grid_map/min_ray_length': 0.1,
        'grid_map/max_ray_length': 4.5,
        'grid_map/visualization_truncate_height': 1.8,

        # ---- the optimiser and the trajectory
        'manager/max_vel': LaunchConfiguration('max_vel'),
        'manager/max_acc': LaunchConfiguration('max_acc'),
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.4,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': LaunchConfiguration('planning_horizon'),
        'manager/use_distinctive_trajs': True,
        'manager/drone_id': LaunchConfiguration('drone_id'),
        # The weights come from upstream VERBATIM.  They are the ones `advanced_param.launch.py` sets, and
        # a file that leaves them out does not get the upstream behaviour -- it gets whatever the C++
        # defaults happen to be, silently.
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 0.5,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,      # UPSTREAM'S VALUE, and the 4.0 experiment is now CLOSED:
                                                 # DIAG15 (4.0) cleared by +0.068 m with 1231 of 54097 samples
                                                 # inside the C2 footprint; the reproduction DIAG17 (same
                                                 # configuration) PENETRATED at -0.078 m with 6741 inside.  The
                                                 # same pattern as the relay window -- one good run, then it does
                                                 # not hold -- and the reason is now quantified: across ELEVEN
                                                 # ego crossings of this mission the truth's minimum clearance has
                                                 # a spread of about +-0.15 m around -0.10 m, so every lever tried
                                                 # (+0.05 .. +0.15 m) is smaller than the run-to-run VARIANCE.
                                                 # C2's 0.20 m footprint sits about two standard deviations above
                                                 # that mean, which is why no parameter reaches it.  See STATUS row
                                                 # 85 for the distribution and row 86 for what is left.
                                                 # `calcFitnessCost` is the only term that pulls the B-spline onto
                                                 # the A* reference; `lambda_collision`/`dist0` multiply a SWARM
                                                 # separation cost that is empty for one vehicle.
                                                 # DIAG15 (4.0) moved the plan's 1st-percentile clearance from
                                                 # -0.031 to +0.124 m and gave the best C2 numbers at one cell
                                                 # (truth min +0.068 m, 1231 of 54097 inside the 0.20 m footprint,
                                                 # against 2108-12028 elsewhere) with tracking unaffected.  It was
                                                 # NOT promoted on one run; the reproduction is DIAG17 and the
                                                 # decision is STATUS row 85.  Note what 4.0 did NOT move: the
                                                 # plan's MINIMUM stayed at -0.285 m, unchanged from DIAG8/DIAG14,
                                                 # so the worst single excursion has another cause (a trajectory
                                                 # published and then replaced, most likely).
                                                 # `calcFitnessCost` is the only term that pulls the B-spline onto
                                                 # the A* reference; `lambda_collision`/`dist0` below multiply a
                                                 # SWARM separation cost that is empty for one vehicle.
        'optimization/dist0': 0.5,
        'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': LaunchConfiguration('max_vel'),
        'optimization/max_acc': LaunchConfiguration('max_acc'),
        'bspline/limit_vel': LaunchConfiguration('max_vel'),
        'bspline/limit_acc': LaunchConfiguration('max_acc'),
        'bspline/limit_ratio': 1.1,
        'prediction/obj_num': 10,          # inert: nothing in THIS stack publishes dynamic obstacles
        'prediction/lambda': 1.0,
        'prediction/predict_rate': 1.0,
    }

    remaps = [
        # THE WIRING.  Two independent rules, and they must NOT be pointed at the same topic: the FSM needs
        # the twist, the map needs only the pose, and our 200 Hz pose topic has an identically zero twist.
        ('odom_world', LaunchConfiguration('odom_vel_topic')),
        ('grid_map/odom', LaunchConfiguration('odom_topic')),
        ('grid_map/cloud', LaunchConfiguration('cloud_topic')),
        ('planning/bspline', LaunchConfiguration('bspline_topic')),
        ('grid_map/occupancy_inflate', ['drone_', LaunchConfiguration('drone_id'),
                                        '_grid/grid_map/occupancy_inflate']),
        ('goal_point', ['drone_', LaunchConfiguration('drone_id'), '_plan_vis/goal_point']),
        ('global_list', ['drone_', LaunchConfiguration('drone_id'), '_plan_vis/global_list']),
        ('init_list', ['drone_', LaunchConfiguration('drone_id'), '_plan_vis/init_list']),
        ('optimal_list', ['drone_', LaunchConfiguration('drone_id'), '_plan_vis/optimal_list']),
        ('a_star_list', ['drone_', LaunchConfiguration('drone_id'), '_plan_vis/a_star_list']),
    ]

    node = Node(
        package='ego_planner',
        executable='ego_planner_node',
        name=['drone_', LaunchConfiguration('drone_id'), '_ego_planner_node'],
        output='screen',
        parameters=[params],
        remappings=remaps,
    )

    note = LogInfo(msg=[
        'ego_on_lio: ONE node (ego_planner_node).  NOT started, on purpose: traj_server (the contract is '
        'produced by src/planner/ego_source.py instead -- see this file\'s header), ego\'s own simulator, '
        'map_generator, and rosmsg_tcp_bridge.  ',
        'Wiring: odom_world <- ', LaunchConfiguration('odom_vel_topic'),
        ' (the twist the FSM gate needs); grid_map/odom <- ', LaunchConfiguration('odom_topic'),
        ' (pose only); grid_map/cloud <- ', LaunchConfiguration('cloud_topic'),
        ' (must be in L: there is no TF).',
    ])

    return LaunchDescription(args + [note, node])
