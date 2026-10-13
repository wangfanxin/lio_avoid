# ARCHITECTURE — components, dataflow, topics, frames, interfaces, safety

One of the project's four living documents (see `PLAN.md` for the documentation budget: **at most 5**).
Everything here is a contract: rates, frames and interfaces are quoted from measurements or from the
firmware source, and each conversion has a test named in `FRAMES.md` §5.

## 1. Components — one folder each

| folder | responsibility | state |
|---|---|---|
| `sim/` | the world file, the ray-cast lidar, the IMU generator, the truth receiver, RViz world editing, the clock | raycast + world + real MID-360 pattern done |
| `lio/` | the LIO process (Super-LIO or FAST-LIO) + config + odometry/cloud adaptation | to build |
| `planner/` | ego-planner: map from the cloud, goal in, trajectory out | to build |
| `fsm/` | mission scheduling, arrival checks, setpoint streaming, safety | to build |
| `bridge/` | the transport switch (`BRIDGE=mavlink\|mavros`), bidirectional, normalised frame | to build |
| `common/` | `frames.py` (the only conversions) + `Calib` (per-flight calibration and self-check) + interface types | to build |
| `tools/` | recorder (all streams, both stamps), metrics, calibration scripts, replay, the docs-budget check | partial |

`bridge/` is deliberately NOT inside `fsm/`: the FSM must never mention MAVLink or MAVROS, and both
implementations must return identical numbers in one frame (§5).

### Supplements the first draft of this design missed

1. **The truth path is a producer**, not a test fixture: the lidar cannot be generated without the true
   pose, so `sim/truth_rx` sits in the loop (fed by the simulator-only tap in SITL).
2. **The clock**: with no Gazebo nobody publishes `/clock`; `sim/` republishes the SITL model's clock so
   every node runs `use_sim_time=true` and the whole pipeline shares one time base.
3. **The odometry sanity gate** (§7.1): the FCU trusts ExternalNav completely -- in the old project a
   LIO glitch *drove* the aircraft into a wall 7 m away.
4. **The trajectory resampler**: ego-planner emits a B-spline horizon; ArduPilot takes position/velocity
   setpoints and gives up after `GUID_TIMEOUT = 3 s`, so the FSM samples and streams continuously.
5. **A fake-truth mode** (`worlds/pillars.yaml` -> `fake_truth`) so lidar -> LIO -> planner can be
   exercised before the SITL build exists.
6. **FCU stream rates**: `LOCAL_POSITION_NED` (31/32) arrives at whatever `SR*_POSITION`/`SR*_EXTRA1`
   say; the defaults give ~1 Hz, which is useless for arrival checks.
7. **Geofence and loss handling**: owned by `fsm/`, not by the planner.

## 2. The chain, end to end

```
        worlds/pillars.yaml (geometry, mission, mount)                 file
                    |
   SITL (Copter-4.7.1, built-in quad model, simulator-only truth tap)  sim/
        |  UDP truth 400 Hz (NED/FRD, declared in the header)
        v
   sim/truth_rx --> sim/lidar_imu --> /livox/lidar 10 Hz (S)
        |                             /livox/imu   200 Hz (S)
        |                                   |
        |                                   v
        |                                 lio/  (Super-LIO or FAST-LIO)
        |                                   |  /lio/odom 10 Hz (L)
        |                                   |  /odom 200 Hz fitted (L)
        |                                   |  /cloud_registered 10 Hz (L)
        |                                   |
        |                                   +--> fsm/bridge --> FCU: VISION_POSITION_ESTIMATE / ODOMETRY
        |                                   v
        |                              planner/ (ego-planner)
        |                                   |  /planner/traj 10-50 Hz (L)
        |                                   v
        |                                 fsm/  sample + convert + stream
        |                                   |  SET_POSITION_TARGET_LOCAL_NED 20-50 Hz
        |                                   v
        +-------------------------> FCU (GUIDED) --LOCAL_POSITION_NED/ATTITUDE back--> fsm/
                                            |
                                   tools/recorder (everything, both stamps)
                                            |
                                   tools/metrics  (acceptance + counts of diverged)
```

## 3. Topics — rate, content, frame

Frames: **W** world ENU (the file, origin = pad) · **L** LIO world (z up, yaw arbitrary, origin = LIO
init) · **F** FCU local (NED on the wire) · **B** body FLU · **S** MID-360 housing.  L and F are the
same frame up to axis labels; W differs from L by a known constant rotation + offset.

| topic | type | rate | content | frame | producer -> consumer |
|---|---|---|---|---|---|
| truth tap (UDP, not ROS) | ASCII line | **400 Hz** | t, p_ned, v_ned, q(w,x,y,z body->NED), gyro FRD, accel FRD | SITL native, declared | SITL -> `sim/truth_rx` |
| `/clock` | `rosgraph_msgs/Clock` | 100 Hz | sim time | -- | `sim/` -> all |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | **10 Hz** | 20 000 points; x,y,z,intensity,time; `time` = per-point offset 0..0.1 s | **S** | `sim/` -> `lio/` |
| `/livox/imu` | `sensor_msgs/Imu` | **200 Hz** | gyro rad/s, accel m/s^2, no orientation | **S** | `sim/` -> `lio/` |
| `/sim/truth/odom` | `nav_msgs/Odometry` | 100 Hz | true body pose/velocity (never used for control) | **W** | `sim/` -> tools |
| `/sim/world/markers` | `visualization_msgs/MarkerArray` | 1 Hz / on change | pillars, arena, clear box | **W** | `sim/` -> RViz |
| `/clicked_point` | `geometry_msgs/PointStamped` | on click | operator adds a pillar | **W** | RViz -> `sim/` |
| `/lio/odom` | `nav_msgs/Odometry` | **10 Hz** | ESKF state at scan rate, child `mid360_link` | **L** | `lio/` -> tools |
| `/odom` | `nav_msgs/Odometry` | **200 Hz** | fitted body pose + velocity, child `base_link` | **L** | `lio/` -> planner, bridge, RViz |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | **10 Hz** | current scan registered into the map | **L** | `lio/` -> planner, RViz |
| `/lio/map` | `sensor_msgs/PointCloud2` | 0.2-1 Hz | accumulated map (optional, debug) | **L** | `lio/` -> RViz |
| `/lio/status` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | scan rate, effective points, degeneracy lambda2/lambda1, map size | -- | `lio/` -> fsm gate |
| `/planner/goal` | `geometry_msgs/PoseStamped` | on demand | next goal, W -> L by the FSM | **L** | fsm -> planner |
| `/planner/traj` | `traj_utils/Bspline` (ego-planner native) | **10-50 Hz** | horizon: control points, start/end time, yaw | **L** | planner -> fsm |
| `/planner/vis/*` | `visualization_msgs/MarkerArray` | 10 Hz | goal, path, ESDF slice | **L** | planner -> RViz |
| `/planner/status` | `std_msgs/String` | 1-10 Hz | replan ok/fail + reason | -- | planner -> fsm |
| `SET_POSITION_TARGET_LOCAL_NED` (84) | MAVLink | **20-50 Hz** | sampled position (+optional velocity/accel/yaw/yaw_rate), `LOCAL_NED` | F (NED on wire) | bridge -> FCU |
| `COMMAND_LONG` (76) | MAVLink | on demand | arm / takeoff / land / mode | -- | bridge -> FCU |
| `VISION_POSITION_ESTIMATE` (102) / `ODOMETRY` (331) | MAVLink | **20-100 Hz** | LIO pose + velocity (+attitude), converted per FRAMES §7 | F (NED on wire) | bridge -> FCU |
| `LOCAL_POSITION_NED` (32) | MAVLink | **20-50 Hz** | FCU local position/velocity (**z DOWN**) | F | FCU -> bridge |
| `ATTITUDE_QUATERNION` (31) | MAVLink | **20-50 Hz** | FCU attitude (FRD body -> NED) | F/B | FCU -> bridge |
| `HEARTBEAT` (0) / `STATUSTEXT` (253) | MAVLink | 1-2 Hz / on event | armed, mode, health / text | -- | FCU -> bridge |

With `BRIDGE=mavros` the last six rows become mavros topics (`/mavros/local_position/pose` 30-50 Hz ENU,
`/mavros/state` 1 Hz, `/mavros/statustext/recv`, setpoints on `/mavros/setpoint_position/local`, vision on
`/mavros/vision_pose/pose_cov`) and **mavros performs the conversions**.

## 4. TF tree

```
world_w  (W: world.yaml geometry, pillar markers)
   |  static, from Calib (yaw + translation, measured per flight, checked against construction)
odom     (L: the LIO world, also the planner's world)
   |  from /odom at 200 Hz
base_link (B)
   |  static mount: 45 deg nose-down + (0.05, 0, 0.10)
mid360_link (S)
```

`map` is deliberately not used: the LIO has no loop closure or jumps, so one continuous frame suffices.
RViz shows pillars (in `world_w`) and the cloud (in `odom`) together because of that static edge.  The
FSM and the planner never touch W except when the mission is set up.

## 5. The transport interface (bidirectional, normalised)

Both implementations return the SAME numbers in the SAME frame: **`F`, ENU-labelled, metres, z up**,
quaternion = body(FLU) -> F.  The FCU's wire convention is reached inside the implementation, and each
implementation crosses exactly once.

```python
class Transport(Protocol):                       # both directions
    # outbound
    def arm(self, force=False) -> bool: ...
    def set_mode(self, mode: str) -> bool: ...
    def takeoff(self, alt_m: float) -> bool: ...
    def send_position(self, p_f, yaw_rad=None) -> None: ...      # ENU labels, z up
    def send_velocity(self, v_f, yaw_rate=0.0) -> None: ...
    def send_external_nav(self, stamp, p_f, q_f, v_f) -> None:   # LIO -> FCU
    # inbound
    def state(self) -> VehicleState: ...          # connected / armed / mode / statustext
    def local_pose(self) -> Pose: ...             # msg 31+32, normalised to ENU / z up
    def local_velocity(self) -> Vec3: ...
```

| direction | `mavlink_impl` (pymavlink) | `mavros_impl` |
|---|---|---|
| setpoints out | `f_to_ned()` then msg 84 (z becomes DOWN) | publish `/mavros/setpoint_position/local` (mavros converts) |
| external nav out | ENU->NED + FLU->FRD + origin, then msg 102/331 | publish `/mavros/vision_pose/pose_cov` (mavros converts) |
| pose in | msg 32 + 31 -> `ned_to_f()` + `frd_to_flu()` | subscribe `/mavros/local_position/pose` (already ENU/FLU) |
| state in | `HEARTBEAT` + `STATUSTEXT` | `/mavros/state` + `/mavros/statustext/recv` |

**The trap this interface exists to kill:** `LOCAL_POSITION_NED.z` is DOWN, `/mavros/...pose.z` is UP.
"Hover at 2.5 m" is `-2.5` on one path and `+2.5` on the other, so an arrival check written against the
raw message is correct under one bridge and silently broken under the other.  The tests assert that BOTH
bridges report `+2.5`.

## 6. Rates, latency, compute (measured where noted)

| stage | rate | cost | note |
|---|---|---|---|
| truth tap | 400 Hz | negligible | one UDP line per model step |
| lidar sim | 10 Hz x 20 000 rays | **9.8 ms/scan** (46 pillars) | 103 Hz capable; CPU is enough, GPU not needed |
| IMU sim | 200 Hz | negligible | generated from the tap |
| LIO | 10 Hz in, 200 Hz out | ~1 core | dominant cost |
| planner | 10-50 Hz replan | ~1-2 cores | ESDF build dominates |
| FSM | 50-100 Hz | negligible | arrival checks + setpoint streaming |
| FCU | 400 Hz loop | SITL built-in model | no Gazebo anywhere |
| end-to-end | -- | target < 100 ms | scan 100 ms + LIO + planner horizon |

## 7. Safety gates (owned by `fsm/`)

1. **Odometry sanity** before feeding the FCU: rate >= 5 Hz, position jump < 0.5 m between scans, speed
   < 3 m/s, `/lio/status` degeneracy above threshold.  On violation: stop ExternalNav, hold; if it
   persists, LAND.
2. **Geofence**: |x|, |y| inside the arena minus the vehicle radius; z between 0.3 m and ceiling - 0.3 m.
   On violation: abort the mission, hold, LAND.
3. **Setpoint continuity**: stream at >= 20 Hz even while holding (`GUID_TIMEOUT` is 3 s).
4. **Planner failure**: if `/planner/status` reports failure for > 1 s while moving, hold; if it
   persists, fly straight to the goal slowly.
5. **No GPS, no compass**: arming depends on a healthy EKF (origin set, vision healthy).  A transiently
   invalid FCU state is tolerated for 3 s -- the old project lost 4 of 8 flights to a single bad sample.
