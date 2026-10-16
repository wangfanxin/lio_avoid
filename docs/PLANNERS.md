# PLANNERS — the planner contract, and how to swap one in

This stack has ONE place where a planner plugs in: **`/plan/cmd`**, a `quadrotor_msgs/PositionCommand`
stream in the LIO world frame `L`.  Everything downstream — the BRIDGE that puts it on MAVLink, the
SUPERVISOR that watches the mission and can abort it, the FCU that flies it — is written against that one
interface and does not know, and must not know, which planner produced it.  This file is the contract, the
safety split, and what each planner in the tree actually does.

*Status: the contract and the swap mechanism are implemented and tested offline; `rectangle` is the flown
planner; `ego` has an adapter and has NOT been flown on this stack since F1/F2 (see §5).*


## 1. THE CONTRACT (what a planner must do)

    topic      /plan/cmd
    type       quadrotor_msgs/msg/PositionCommand
    frame      header.frame_id = "lio_world"      (the bridge also accepts ego's name, "world")
    qos        RELIABLE + KEEP_LAST(5) + **TRANSIENT_LOCAL** (latched).  THIS IS PART OF THE CONTRACT.
               Both consumers subscribe latched — the BRIDGE (`src/bridge/bridge_node.py:113-115`) and the
               recorder (`src/tools/record-traj.py:123-128`) — so that a consumer starting AFTER the planner,
               which is the start-up order, still finds the newest sample instead of waiting for the next
               leg.  A planner that publishes VOLATILE (the default of `create_publisher(T, topic, 10)`)
               connects to NOTHING: MEASURED 2026-10-15, the first ego adapter did exactly that and the
               console said `requesting incompatible QoS ... Last incompatible policy: DURABILITY` while
               `/plan/cmd` reached no one.  `src/tools/test-planner-swap.py` now checks that every producer
               agrees with the flown one on this, rather than trusting that they do.
    rate       >= 20 Hz; ours publishes at 100 Hz on its own wall timer and the bridge samples the newest
               at 50 Hz (`wire.csv`), so anything >= 20 Hz is indistinguishable on the wire
    content    position = the point being flown to, in L (metres)
               velocity, acceleration = whatever you like; THE WIRE CARRIES POSITION ONLY
               (type_mask 2552 = POS_YAW_MASK: vx,vy,vz,afx,afy,afz,yaw_rate are ignored), so the speed
               profile between targets is the FCU's own position controller's
               yaw = the direction policy you choose; yaw_dot is ignored
               kx, kv = 0 (the message has no gain fields on the wire)

    trajectory_id     an integer that CHANGES WHEN THE MISSION MOVES ON -- a new waypoint, a new leg.
                      The consumer turns each change into a `leg` phase, and the B2/B3 metrics are per leg,
                      so the meaning you give it is the meaning the evidence has.
    trajectory_flag   0 EMPTY, 1 READY, 3 COMPLETED  (`src/planner/cmd_stream.py:64-66`).
                      COMPLETED is a STATEMENT, not a stop: keep publishing after it.

Two rules that are not negotiable, because the supervisor is built on them:

1. **THE END OF THE MISSION IS A FIELD, NOT SILENCE.**  Publish `trajectory_flag = 3` and keep publishing
   the last point (the aircraft is meant to hold there).  A planner that simply stops talking is treated as
   a FAILURE: the supervisor aborts with `THE PLAN BOX HAS STOPPED TALKING: nothing on /plan/cmd for 3.0 s`
   and lands (`src/supervisor/supervisor.py`, the plan-silence gate, `--plan-silence-s 3.0`).
2. **A CHANGE OF `trajectory_id` IS A CLAIM THAT THE MISSION MOVED ON.**  If you bump it while the vehicle
   is still flying one leg, you have told the consumer the tour advanced and the phase log, and every
   metric windowed on it, will say so.

**AND THE RULE IS IMPLEMENTED TWICE, IN TWO PROCESSES — deliberately, and both copies are checked.**  The
BRIDGE has its own liveness rule (`src/bridge/bridge_node.py`, `CmdRelay.stalled_reason`) and the SUPERVISOR
has its own plan-silence gate and its own COMPLETED test (`src/supervisor/supervisor.py:281-288`, `:552`),
reading `/plan/cmd` on its own subscription (`:164-176`).  The supervisor must not depend on the bridge's
health to decide whether the mission is over, so the duplication IS the design — but a rule written twice
can be written differently twice, which is why `src/tools/test-planner-swap.py` drives **both**
implementations with all three planners: each sees the COMPLETED statement, neither aborts a stream that is
still talking, and each fires its OWN named abort on a truncated stream and on silence after COMPLETED.  A
swap that satisfied only one of them would be a planner that half the stack believes.

One rule that is a *courtesy* to the evidence: publish at a steady rate.  A stream that declares 100 Hz and
emits 4 Hz was this project's S3 defect; `src/tools/measure-rates.py` measures the achieved rate on the
recorded stream, and every sortie's `traj.log` prints declared vs achieved.


## 2. WHO IS RESPONSIBLE FOR WHAT (and the planner is not responsible for most of it)

| duty | who | where |
|---|---|---|
| WHERE to go: waypoints, arrival test, dwell, leg timeout | **the planner** (the plan box) | `src/planner/` |
| the frame of the command, and refusing a command in an unknown frame | BRIDGE (`DEFAULT_FRAME_IDS = (FRAME_L, "world")`) | `src/bridge/bridge_node.py:54-58`, `:121-129` |
| position-only mask, 50 Hz wire rate, ONE writer of class 84 | BRIDGE | `src/bridge/bridge_node.py`, `src/bridge/mavlink_impl.py` |
| the estimator feed (class 331) and its rate floor | BRIDGE | same |
| arm, take-off, the go (`/plan/trigger`), mode changes, LAND | SUPERVISOR, on its OWN endpoint, class 76 only | `src/supervisor/supervisor.py` |
| geofence (measured on the TRUTH tap), navigation-rate floor, plan-silence abort, named ABORT | SUPERVISOR | same |
| the truth tap, the world, the sensor simulation, the recordings | the harness | `src/tools/run-mission.sh`, `src/sim/` |
| **anything about the aircraft's stability, limits or safety envelopes** | **the FCU** (ArduPilot) | it is a position-target interface; the planner cannot command a tilt, a thrust or a rate |

A planner therefore CANNOT: fly the aircraft somewhere the FCU's own limits refuse, decide when to arm, take
off or land, or keep the vehicle safe by itself.  What it can do is ask for a sequence of positions, and
state when it is done.  Everything the planner decides that the safety chain depends on is **the two
semantics above** — which is why they are asserted by a test rather than trusted.


## 3. HOW TO SWAP ONE IN

    PLANNER=rectangle      # default: src/planner/rectangle_source.py -- the Mission A rectangle
    PLANNER=ego            # src/planner/ego_source.py        -- ego-planner behind the same contract
    PLANNER=<path.py>      # any file that publishes the contract above

`src/tools/run-mission.sh` prints, in its start-up banner, WHICH planner it started and on which topic, and
the phase log records the executor's own first line, so a sortie can never be read as "some planner" — the
console says which.  **Swapping a planner changes nothing in the bridge or the supervisor**: that is the
property `src/tools/test-planner-swap.py` checks, by driving the SAME consumer code with three different
planners — ours, a minimal third-party-style one, and this tree's ego adapter — and asserting the consumer's
rules hold for all three, with negative controls that make the assertion mean something.

**To see the selection without starting anything**, set `PLANNER_DRY_RUN=1`: the harness prints the planner,
its file, its topic and the exact argv it would be handed, then exits 0 — *before* the sortie lock, before
ROS and before SITL, so it cannot collide with a live flight or leave a lock behind.

**And that dry run exists because the switch was BROKEN, which is worth stating here rather than in a
commit message.** The first version of the selector stood ~40 lines too early and built the planner's argv
while `VERTICAL_ARG` and `TRAJ_CALIB_ARG` did not exist yet; under `set -u` the harness died on the first
unbound variable, so `PLANNER=rectangle` — the default, the planner every accepted sortie flies — could not
start at all. Had `set -u` not been there, the plan box would have been launched without the vertical climb
leg and without the L→W calibration: the class of defect that ruins a sortie quietly. `--calib-file` and
`--vertical-first-m` are therefore two of the things `src/tools/test-planner-knob.py` asserts on the
default's argv, and it asserts them in BOTH directions (the climb leg present at `VERTICAL_FIRST=1` and
ABSENT at `VERTICAL_FIRST=0`), because a check that only ever sees a flag present cannot tell a working knob
from a hard-coded one.

What a new planner needs from us, if it wants to be more than a script: the LIO odometry (`/lio/robo/odom`,
200 Hz, pose only; `/lio/odom`, 10 Hz, with velocity), the registered scan (`/lio/cloud_world`, in L), the
world file (for any ground-truth-free geometry it wants to know in advance), and the calibration
(`--calib-file`, i.e. `_stage/calib.json`) if it works in W.  What it must NEVER do is reach for an FCU
endpoint: there are two MAVLink endpoints, `serial0` belongs to the bridge, `serial1` to the supervisor.


## 4. THE PLANNERS IN THIS TREE

### 4.1 `rectangle` — `src/planner/rectangle_source.py` (ours, FLOWN)

The Mission A tour: five waypoints (corner 1, then the four legs, then corner 1 again), a vertical climb leg
first if `--vertical-first-m` is given, arrival gate 0.060 m on its OWN estimate, settle gate 0.080 m/s,
dwell 5.0 s, leg timeout 20 s.  It publishes ONE target at a time with velocity and acceleration zero and
`trajectory_id` incrementing per waypoint; `trajectory_flag` becomes COMPLETED after the last dwell and the
stream keeps publishing.  Optional `--approach-shaping-m` inserts a waypoint short of each corner, stopped at
but not held (`docs/metrics/b1-approach-shaping-20261016.txt`).

### 4.2 `ego` — `src/planner/ego_source.py` (adapter) and `src/planner/ego_contract_shim.py` (Route B)

**CORRECTED 2026-10-16 (mission B): this heading said "NOT flown since F1/F2", which is no longer true.**
The ego planner has flown repeatedly since: the window-8 arm and the alternatives (`DIAG20`-`DIAG28`), the
distance-cost sorties (`DIAG29`), the reference-fix sorties (`S4` = `DIAG31`, `S6` = `DIAG32`), the Route B
sorties (`DIAG33`, `DIAG34`) and the parked benches -- `STATUS.md` rows 123-139 carry the numbers.  There
are now TWO contracts in this package: the adapter samples ego's B-spline itself, and the shell lets ego's
OWN `traj_server` sample it while the shell owns the tour and rewrites `trajectory_id`/`trajectory_flag`
(`EGO_CONTRACT=adapter|shim`).

ego-planner-swarm (ROS2 port, upstream `23a8d5a` under `/home/fanxin/lio_avoid/src/ego-planner-swarm`) is a
different KIND of planner: it optimises a timed cubic B-spline through a local occupancy map, replanning
continuously.  Every fact below was read in THIS tree and carries its line; the line numbers are the LIVE
WORKING TREE (branch `ros2_version`, HEAD `23a8d5a`, three uncommitted patched files), not HEAD.  The two
full audits are `_stage/ego-audit-launch-and-packaging.md` (launch files, messages, packaging) and
`_stage/ego-traj-representation.md` (the trajectory representation and its evaluator).

WHAT IT GIVES US, and it is close to our contract already:

| fact | evidence |
|---|---|
| it publishes `quadrotor_msgs/PositionCommand` at 100 Hz with real position, velocity, acceleration | `plan_manage/src/traj_server.cpp:214-231`, timer `:248` |
| `kx`, `kv` are zero globals, exactly like ours | `:12-13`, `:253-259` |
| `header.frame_id` is hard-coded `"world"` — a name our bridge already accepts | `:210`, `src/bridge/bridge_node.py:54-58` |
| past the end of the spline it holds the endpoint with zero velocity and zero acceleration | `:191-202` |

WHAT IT CANNOT DO, and these are the four reasons an adapter is mandatory rather than optional:

| blocker | evidence | consequence |
|---|---|---|
| **it can never say COMPLETED** — `trajectory_flag` is written exactly once, to READY | `traj_server.cpp:211`; a whole-tree grep for `TRAJECTORY_STATUS` returns that line and nothing else | the supervisor's "the plan finished" signal does not exist; the adapter must synthesise flag 3 on arrival |
| **it never goes silent either** — the 100 Hz timer keeps running past the end | `:191-202` | so the OTHER discriminator (silence) also cannot be used as a completion signal: silence must stay reserved for failure |
| `trajectory_id` changes **per replan**, not per waypoint (~1 Hz inside one leg) | `planner_manager.cpp:680` (`local_data_.traj_id_ += 1`) | the consumer's `leg` phases would multiply; the adapter re-numbers per WAYPOINT |
| `yaw_pts` / `yaw_dt` are never filled, and the yaw parse in the consumer is commented out | `traj_utils/msg/Bspline.msg:10-11`, `traj_server.cpp:51-56` | ego's stream carries no yaw at all; the yaw policy must be the adapter's (travel direction, as `YAW_MODE=velocity` in the flown stack) |

Three more things a swapper must know, all read from this tree:

* **which launch file you use decides whether a PositionCommand exists at all.**  `advanced_param.launch.py`
  starts `ego_planner_node` ONLY (`:89-92`, added at `:252`) — it produces B-splines and no command.
  `traj_server` is launched by `single_run_in_sim.launch.py:142-154` and `run_in_sim.launch.py:88-100`
  (and indirectly by the swarm files), but those also start ego's own simulator (`so3_quadrotor_simulator`,
  `so3_control`, `poscmd_2_odom`, `pcl_render_node`), which would fight ArduPilot for the same topics.
  **THIS IS NOW ANSWERED, NOT JUST DIAGNOSED**: this repository ships
  **`src/planner/ego_on_lio.launch.py`**, which starts `ego_planner_node` and nothing else — no
  `traj_server`, no simulator, no map generator, no swarm bridge — with our topic names, our map box and
  our ceiling, and it is installed into the ego checkout by
  **`src/tools/install-ego-launch.py`** (copy → `colcon build --symlink-install --packages-select
  ego_planner` → check).  The route it takes is the SECOND one below: the contract is produced by
  `planner/ego_source.py`, which samples the B-spline itself.  The `traj_server` route remains legitimate —
  start it yourself, remap `position_cmd:=/plan/cmd`, and set `traj_server/time_forward: 1.0`.
* **the planner's trajectory is a TIMED spline whose start is NOT the vehicle** — `reboundReplan`'s else
  branch resamples the previous trajectory from `t_cur` and never re-anchors to `start_pt`
  (`planner_manager.cpp:140-208`; `start_pt` is read only for the goal-proximity early-out at `:55` and the
  time-step choice at `:70`).  A consumer that samples the spline on its own clock inherits exactly the
  anchoring the planner chose.
* **the map is a boolean inflated grid, not an ESDF** (`plan_env`, `optimization/dist0` is a soft cost), and
  the hard margin in that map is `grid_map/obstacles_inflation` = 0.099 m
  (`plan_manage/launch/advanced_param.launch.py:148`).  A clearance claim must come from OUR world file and
  OUR C2 row, never from ego's map.

FOUR TRAPS RATHER THAN FACTS, each read in that tree and each able to waste a sortie.  The full audits are
`_stage/ego-audit-launch-and-packaging.md`, `_stage/ego-traj-representation.md` and
`_stage/ego-audit-bspline-opt-and-path-searching.md`:

1. **THE OPTIMISER'S PARAMETERS DO NOT HAVE SAFE DEFAULTS.**  All nine are declared in one `setParam`
   (`bspline_opt/src/bspline_optimizer.cpp:8-34`), called ONCE with no parameter callback, and the four
   weights plus `dist0` default to **−1.0** (`:11-19`).  With no YAML that is not "untuned": `lambda_smooth
   = −1` makes the optimiser **MAXIMISE jerk**, and `dist0 = −1` makes the obstacle-distance cost a **silent
   no-op** (the `dist_err < 0` branch, `:982-988`).  This is why the launch file in §4.2 sets all of them
   explicitly and why `src/tools/install-ego-launch.py` checks every name against the node's declared list —
   a file that stays silent about them does not inherit a sane default, it inherits a broken objective.
2. **THE MAP'S EDGE AND A*'s POOL ARE DIFFERENT WALLS, AND THE SECOND ONE IS NEARER.**  Outside the map box
   a query reads OCCUPIED (`grid_map.h:357-366`), and unknown cells INSIDE it read FREE — but `dyn_a_star`'s
   pool is not the map: it is centred on the MIDPOINT OF EACH QUERY with a fixed ±`pool/2` extent
   (`path_searching/src/dyn_a_star.cpp:17,128`), 100³ cells at the caller's hard-coded 0.1 m step
   (`bspline_optimizer.cpp:581`), i.e. about **±5 m per axis from the midpoint**.  Two query points more
   than ~10 m apart on any axis therefore put an endpoint OUTSIDE the pool, and the escape loops that would
   fix it are unbounded (`dyn_a_star.cpp:101-113`) — which is what the Mission B log line
   `Ran out of pool, index=88 100 25, POOL_SIZE=100 100 100` is: index `y == POOL_SIZE_(1)`, ~5 m off the
   midpoint.  **Our arena is 12 × 12 m, so this is not hypothetical for a long diagonal leg.**
3. **THE HEADER COMMENTS INVERT THE DATA LAYOUT.**  `bspline_optimizer.h:16-17` and `uniform_bspline.h:18-20`
   say the control points are "Nx3, each row is a point"; the code treats them as **3×N column-major** (column
   j is point j).  A port written from the comments gets a transposed trajectory — which evaluates, and is
   wrong.
4. **THE FEASIBILITY LIMITS ARE SOFT, AND THE ONLY HARD CHECK IS ELSEWHERE.**  `max_vel`/`max_acc` enter as
   quadratic penalties (`bspline_optimizer.cpp:1240,1271`); nothing is clamped or projected there.  The
   hard-ish enforcement is `checkFeasibility`/refine with a 5 % tolerance
   (`uniform_bspline.cpp:144,166`).  So "the planner respected our limits" is a claim about the
   FEASIBILITY CHECK, not about the optimiser, and the FCU remains the only thing that bounds the flight.

AND THREE MORE FROM THE `plan_manage`/`plan_env` AUDIT, each of which changes what a swapper may assume
(`_stage/ego-audit-plan-manage-and-plan-env.md`):

* **ARRIVAL NEEDS THE VEHICLE, NOT JUST THE CLOCK — and there is an unset default that hangs the FSM.**  The
  arrival test itself is on the trajectory's clock (`ego_replan_fsm.cpp:946-948`), but PATCH 0007 vetoes it
  against the vehicle: `if (goal_dist > no_replan_thresh_) → REPLAN_TRAJ` (`:963-977`).  With the parameter's
  DECLARED default of **−1.0** (`:19`) that condition is always true, so **arrival is never declared and the
  FSM livelocks** — loudly, but livelocks.  This is the second reason the launch file sets parameters it
  "could have left alone": `fsm/thresh_no_replan_meter: 1.0` is load-bearing, not tuning.
* **OCCUPANCY IS STICKY OUTSIDE A SMALL BOX.**  The cloud path clears `occupancy_buffer_inflate_` only within
  `camera_pos ± grid_map/local_update_range` (`grid_map.cpp:826-827,234`), so a cell marked occupied STAYS
  occupied once the vehicle moves away — the map is an accumulating obstacle memory, not a sliding window.
  Inside that box it is rebuilt from each scan, which is why the relay's decimation is safe (the map cannot
  represent finer than 0.1 m anyway) and why a stale cloud is worse than a coarse one.
* **SOME PARAMETERS IN THAT TREE ARE DEAD, AND `show_occ_time` IS ONE OF THEM.**  `fsm/planning_horizen_time`
  (`:21,30`), `manager/max_jerk` (`planner_manager.cpp:17,26`) and `grid_map/{show_occ_time, min_ray_length,
  virtual_ceil_yp, virtual_ceil_yn, depth_filter_tolerance}` are declared and read but never used
  (`grid_map.cpp:79` → the only use is a COMMENTED-OUT log at `:750-752`).  So a swapper cannot time ego's
  occupancy update by asking for it: **this note said earlier that ego "can print it" and the audit refuted
  that** — the parameter exists and does nothing.  Setting a dead parameter is not harmful; BELIEVING it is
  what costs a round.

WHAT ITS MAP DEMANDS OF US — the wiring, read in this tree, and the one item that will silently ruin a
sortie if it is missed:

| requirement | evidence | our side |
|---|---|---|
| cloud and odometry in ONE frame: `header.frame_id` is never read and there is no TF anywhere in the map code | `plan_env/grid_map.cpp:851` compares cloud xyz straight against the odometry position | ours are both in L already; remap `grid_map/cloud:=/lio/cloud_world`, `grid_map/odom:=/lio/robo/odom` |
| the map is a FIXED box centred on the L origin — x ±21 m, y ±15 m, z −0.01…4.99 m, and there is NO map-origin parameter | `grid_map.cpp:92` (`map_origin_ = -map_size/2`), sizes at `:93` | our L origin is the landing pad, so the box is centred on the pad; **outside the box reads as OCCUPIED**, because `getInflateOccupancy` returns −1 there and every caller casts it to `bool` (`grid_map.h:357-366`, `dyn_a_star.h:62`) |
| the scan must be decimated and local: 0.1 m voxels, 1-voxel inflation, the whole update inline in the cloud callback at O(N×27) writes, and only points within ±5.5 × ±5.5 × ±4.5 m of the vehicle are accepted | `grid_map.cpp:845-887`, `:851-855`, `:832-833` | do NOT forward our registered scan raw: voxel-filter and publish at 5–10 Hz, or the callback blocks the odometry it needs (`grid_map.cpp` has one mutually-exclusive callback group) |
| the subscription is RELIABLE — a best-effort publisher does not connect and says nothing | `grid_map.cpp:168-172` | our `/lio/cloud_world` and odom publishers must be RELIABLE |
| **the FSM's own odometry reads `twist.twist.linear` from `odom_world`** | `ego_replan_fsm.cpp:443-445`; the velocity is the `> 0.1 m/s` gate to LEAVE EmergencyStop (`:1047`) | our `/lio/robo/odom` carries a pose and an IDENTICALLY ZERO twist (Super-LIO fills only the pose), so that gate would be satisfied by nothing at all.  **THE WIRING, and it needs no new node**: the launch file remaps `odom_world` and `grid_map/odom` independently (`advanced_param.launch.py:96,108`), so point **`odom_world` at `/lio/odom`** (10 Hz, carries the twist) and **`grid_map/odom` at `/lio/robo/odom`** (200 Hz, pose only — which is all the map reads, `grid_map.cpp:799-801`).  The 10 Hz is the trade: fuse the two into one 200 Hz topic if the FSM's seeds matter more than the extra node |
| no distance-to-obstacle query exists: occupancy is boolean, unknown reads as FREE, and the hard margin is the inflation radius | `grid_map.h:346-366`, grep for ESDF/SDF/distance in the tree returns nothing; `optimization/dist0` is a soft cost | a clearance claim comes from OUR world file and our C2 row, never from ego's map |
| the "odom or depth lost" watchdog only arms when depth fusion is on, i.e. never in cloud-only mode | `grid_map.cpp:714-726` | ego has no way to notice that its map stopped updating: the supervisor's plan-silence gate is the only detector, which is another reason the adapter must keep publishing |

A MINIMAL LAUNCH FOR EGO ON THIS STACK — **written, and it is `src/planner/ego_on_lio.launch.py`**.  The
parameters below are the ones decided by our world rather than by taste; every other parameter is upstream's
own value, and the launch file says so in place.  Column two is the `advanced_param` default it replaces:

| parameter | advanced_param default | set it to | why |
|---|---|---|---|
| `fsm/flight_type` | 2 (preset waypoints + `/traj_start_trigger`) | **1** (manual, one goal on `/move_base_simple/goal`) | our adapter owns the tour and sends one goal at a time; the trigger topic is hard-coded (`ego_replan_fsm.cpp:162`) |
| `fsm/realworld_experiment` | False | **True** | `have_trigger_ = !flag_realworld_experiment_` (`ego_replan_fsm.cpp:35`): with False the FSM does not wait for a trigger and can start planning from the pad |
| `grid_map/ground_height` | −0.01 | **−0.02** (a few cm below the pad's z **in L**) | the code forces `max_z = max(max_z, ground_height_)` (`grid_map.cpp:496,897`) and the launch's −0.01 assumes the pad sits at z≈0 of the odometry frame |
| `grid_map/virtual_ceil_height` | 2.9 | **3.00** — our ceiling in L | written into the inflated map as an x-y plane (`grid_map.cpp:908-914`); above the 2.50 m mission altitude and below the 6.60 m geofence |
| `grid_map/map_size_x/y/z` | 42 / 30 / 5 m | **16 / 16 / 5 m** — the size of L we use | the whole box is allocated at start-up (~94 MB per map, plus ~56 MB of A* pool for the upstream size), and the arena is ±6 m |
| `grid_map/use_depth_filter`, `grid_map/pose_type` | True / 1 | **False / 0** | there is no camera: `pose_type` 1 builds a `message_filters` synchroniser that can never fire and `use_depth_filter` True arms a watchdog nothing feeds |
| `manager/max_vel`, `manager/max_acc` | 2.0 / 3.0 | our own limits (`--max-vel`/`--max-acc` arguments) | propagated to `optimization/max_*` and `bspline/limit_*`; note they are SOFT penalties in the optimiser, not constraints |
| `traj_server/time_forward` | C++ default **−1.0** (not set by advanced_param) | **1.0** | the yaw look-ahead point is `t_cur + time_forward_` (`traj_server.cpp:80`): at −1.0 the yaw comes from a direction one second in the PAST.  Only matters on the `traj_server` route, which this file does not take — so it is not set at all, and that is a decision |
| `manager/drone_id` | 0 | keep ≥ 0 | `drone_id <= -1` switches the swarm topic name (`ego_replan_fsm.cpp:127-134`) |

**HOW THE FILE IS CHECKED, AND WHY THAT IS NOT A FORMALITY.**  `src/tools/install-ego-launch.py` copies it
into the ego checkout, runs `colcon build --symlink-install --packages-select ego_planner` (launch files
there are INDIVIDUALLY symlinked, so a new one is invisible until the package is rebuilt), then checks every
parameter name it sets against the list the NODE declares.  **A launch file that sets a parameter the node
never declares does nothing at all, and says nothing** — the sortie simply flies a default.  That check found
four real omissions in the first version of this file: upstream `advanced_param.launch.py` also sets
`optimization/max_vel`, `optimization/max_acc` and the four `optimization/lambda_*` weights, and a file that
leaves them out gets the C++ defaults instead.  MEASURED after the fix: **67 parameters set, 0 of them
undeclared by the node (which declares 75)**, with the remaining eight at their declared defaults and each
one named as a decision (`docs/metrics/ego-launch-20261016.txt`).  The node starts from this file — its own
console reaches `[FSM]: state: INIT` and `wait for goal or trigger` — and **nothing beyond that is claimed:
no cloud, no odometry, no goal, no flight.**

And one package to ignore entirely: `rosmsg_tcp_bridge` is a multi-machine swarm relay (TCP 8080 ring + UDP
8081 broadcast), it is not launched by any launch file in that tree, it is not even built in `planner_ws`,
and it never touches `PositionCommand`.  This stack's own bridge keeps its two MAVLink endpoints.

The adapter's job, in one sentence: **feed ego our odometry and our scan, ask it for one goal at a time in L,
sample its B-spline on the trajectory's own clock, and publish the result as OUR contract** — position only,
`trajectory_id` per waypoint, and `trajectory_flag = 3` when the waypoint is reached and its dwell is over.

### 4.3 a planner that is not a program

`PLANNER=<path.py>` accepts any file that publishes the contract.  The offline test ships a minimal one
(`src/tools/test-planner-swap.py`) precisely so that "the planner is swappable" is a checked property and not
a claim about one adapter.


## 5. WHAT IS *NOT* CLAIMED

* No ego sortie has been flown on this stack since F1/F2.  The adapter exists, and its B-spline evaluator is
  held to three independent references on two cases (a hand-known straight line, a Cox–de Boor recursion, a
  central difference of the position) and then ATTACKED by `src/tools/test-bspline-mutations.py`, which
  injects nine single-token defects into it and requires the selftest to catch every one — the first version
  let three survive, which is why the second case and the difference check exist.  Everything the adapter
  does is asserted offline — but **"ego flies Mission A" is NOT demonstrated.**
* Mission B's own record stands as it was written (`docs/metrics/ego-*.txt`, `phaseB-*.txt`): the anchoring
  defect, the clock defect, the map pool exhaustion.  Nothing in this file supersedes those measurements.
* **The launch file HAS now been fed live data, the ADAPTER has produced the contract from real ego output,
  and it still has never flown.**  MEASURED 2026-10-15, on a PARKED stack (SITL, the simulated MID-360, the
  LIO, the relay and `ego_planner_node`; no arming, no supervisor, no bridge):
  (a) the relay carried **100+ scans, 1 945 868 → 847 564 points (43.6 % kept), 0 cropped**, the FSM's
  `no odom.` count fell from **11 to 1** — so both remaps in §4.2 deliver — and a goal in L produced a real
  `planning/bspline` (`order: 3`, `traj_id: 14`, knots starting at −0.6 s), which our evaluator rebuilds
  without modification (`docs/metrics/ego-pregate-20261015.txt`);
  (b) with `planner/ego_source.py` in the loop, a four-waypoint tour produced **`trajectory_id` 1→4, one per
  WAYPOINT**, and a **sustained COMPLETED that keeps talking** on `/plan/cmd`, recorded off the wire by the
  same recorder the flights use (`docs/metrics/ego-adapter-20261015.txt`).  That run also found the two
  defects that stood in the way, and both are the kind only a live loop finds: a `NameError` for an
  unimported `DurabilityPolicy` (every offline gate passed, because none of them starts a ROS node), and
  UNDER it a silent QoS mismatch — the adapter published `VOLATILE` while the bridge subscribes
  `TRANSIENT_LOCAL`, so **the bridge would have received nothing from an ego planner.**  That is now in the
  contract above and checked against the flown plan box by `src/tools/test-planner-swap.py`.
  What is still NOT demonstrated: **the assembly** — `PLANNER=ego` with the bridge and the supervisor
  attached, and therefore the abort path, and of course flight.
* **And the pregate found a defect that had nothing to do with ego.**  `sim/lidar_imu.py` and
  `planner/rectangle_source.py` both defaulted `--world` to a path built by climbing one level out of the
  CODE root — correct before the code moved under `src/`, and `<repo>/src/worlds/…` afterwards, which does
  not exist.  Every FLIGHT was unaffected (`run-mission.sh` passes `--world` explicitly), so it survived a
  whole round and only surfaced when a tool that relies on the default ran the sensor simulator:
  `sitl-lio.sh` and this pregate, both of which died on `FileNotFoundError` while the LIO came up on an empty
  feed.  Fixed, and `src/tools/test-layout.py` now refuses that shape ANYWHERE under `src/`, comments
  included.
* The acceptance rows (A1–A4, B1–B3, C1, C2) are defined against a commanded rectangle and a leg phase log.
  A planner that does not fly a rectangle can be swapped in, but those rows then measure something else —
  which is why a swap must state which rows still apply.
