# lio_avoid

LiDAR-inertial odometry flying an ArduPilot Copter SITL in a simulated pillar forest, with a flight stack
whose three responsibilities are **three separate processes**: a PLAN box that decides the mission, a BRIDGE
that carries the estimator feed and the setpoints to the flight controller, and a SUPERVISOR that owns
arming, take-off, the safety gates and the abort — on **its own MAVLink endpoint**, so it is a watchdog and
not a passenger.

The mission ("Mission A") is a 2.0 × 1.2 m rectangle between 46 pillars at 2.0 m altitude, a 5.0 s hover at
every corner, all of it flown on external navigation: **no GPS, no compass** — the EKF's position, velocity
and yaw all come from the LIO.

```
   /livox/lidar + /livox/imu ──► super_lio_node ──► /lio/robo/odom   (pose, 200 Hz)
   (from the simulated sensor,                    /lio/odom         (with twist, 10 Hz)
    the SAME world file the                        /lio/cloud_world  (registered scan)
    mission is scored in)                                 │
                                                          ▼
   planner/rectangle_source.py  ── /plan/cmd ──►  bridge/bridge_node.py  ── MAVLink 84 @50 Hz, mask pos
   (PLAN: waypoints, arrival gate,                       │                  MAVLink 331 @48 Hz (odometry)
    settle, dwell, leg timeout;                          │                  serial0 = tcp:5760
    one target at a time, velocity = 0)                  ▼
                                                    ArduPilot Copter SITL  ──► the aircraft
   supervisor/supervisor.py ── class 76 only ────────────▲                    (truth tap on UDP, 1200 Hz)
   (arm, take-off, the go, geofence on TRUTH,        serial1 = tcp:5761
    nav floor, plan-silence abort, LAND)
```

## Start here

| file | what it is |
|---|---|
| `docs/STATUS.md` | **the living record**: every claim with its number and its denominator, and what is NOT achieved |
| `docs/ARCHITECTURE.md` | the components, topics, transports and the responsibility split |
| `docs/FRAMES.md` | the coordinate conventions (W / L / F / B / S) and where the ONE conversion lives |
| `docs/PLAN.md` | the goal, the decisions, the acceptance rows and the rules of evidence |
| `docs/metrics/` | the evidence files: raw tool output for each round, each naming its run directory |
| `frozen/rect46-release.txt` | **the frozen version** this repo is published at, and how to roll back to it |

## The frozen version

```bash
git switch --detach mission-a-rect46     # the frozen Mission A release (promoted-EKF configuration)
python3 tools/frozen-check.py            # 53 of 53 frozen artifacts unchanged
git switch main
```

`frozen/mission_a.manifest` pins every mission-A artifact by SHA-256 and is checked before every commit;
`frozen/rect46-release.txt` says what the version is, which sortie is its reference, and what the parts that
do **not** live in this repository are pinned to (the ArduPilot commit, the Super-LIO commit, the LIO→world
calibration file).

## The SITL firmware

`firmware/` carries the built Copter SITL binary (**upstream `dbe7921` + one patch**, the truth tap) and the
README that pins it: the patch, the `waf` recipe, the runtime knob `LIO_TRUTH_TAP=<udp port>`, the tap's
datagram format and the GPLv3 corresponding-source statement. The whole ArduPilot tree (1.5 GB) is not here
on purpose — the reproducible unit is the pin plus the patch, both in this repository.

## What is outside this repository (and therefore named, not shipped)

| piece | where | pinned as |
|---|---|---|
| ArduPilot checkout | `<runtime>/src/ardupilot` | `dbe7921` + `patches/apm-0001-sitl-truth-tap.patch` |
| Super-LIO | `<runtime>/src/Super-LIO` (+ its `lio_ws` build) | `f89f48d`, `liblio.so` md5 `e4b82a73cbfb70a7a538514f74936b09` |
| LIO → world calibration | `_stage/calib.json` | md5 `399390f7b08905bd3cf1be5164505fb4` (yaw +90.049771°, t = [-0.00080 -0.00409 +0.11557] m) |
| flight recordings | `<runtime>/logs/missionA-<tag>/` | one directory per sortie: `tap.csv`, `lio.csv`, `setpoints.csv`, `wire.csv`, `traj.csv.cmd.csv`, `flight.BIN`, the process logs and `metrics.txt` |

## Fly it

```bash
ALT_MODE=lio NAV_SOURCE=lio WORLD="$PWD/worlds/pillars.yaml" \
MISSION_PARM="$PWD/params/phase_a_mission_fast.parm" \
EKF_PARM="$PWD/params/ekf_trust_lio_measured.parm" \
GEOFENCE_M=6.60 TAKEOFF_COMMAND_M=0.2 VERTICAL_FIRST=1 VERTICAL_HOLD_S=3.0 \
    bash tools/run-mission.sh <TAG>
```

One flight at a time: the launcher refuses to start while the stack is alive. `SHAPING_M=0.30` turns on the
approach-shaping experiment (off by default); `EXTRA_PARM=<file.parm>` appends one parameter file last in
`--defaults` for a one-variable experiment without touching a frozen file. Offline checks that need no
simulator: `tools/frozen-check.py`, `tools/docs-budget.py`, `tools/test-cmd-stream.py`,
`tools/test-stall-gate.py`, `tools/test-supervisor-authority.py`.

## The result, honestly

The reference sortie of the frozen version (`RECT46-accept-promoted`) reports **A1 3.37 mm** route accuracy,
**A2 1.86 mm**, the FCU's own estimate **A4 8.04 mm**, **4 of 4 corners hovered ≥ 5.0 s**, **0 of 60336
samples inside the 0.20 m pillar footprint** (minimum pillar-surface distance 0.569 m), landed and disarmed —
and **B1 fails**: the truth's box over the route is E 2.113 m against a 2.10 m bound, because the position
loop rings at every corner arrival. That failure is documented with its mechanism and two costed candidates
in `docs/metrics/b1-approach-shaping-20261016.txt`; no threshold was ever moved to make a row pass.

## 中文导览

这是一个"把 LIO 当唯一导航源"的 ArduPilot Copter SITL 飞行栈：**PLAN（计划）/ BRIDGE（桥）/ SUPERVISOR
（监督）三个独立进程**，监督者自带独立 MAVLink 端点、只管解锁/起飞/放行/安全门/迫降。任务 A 是在 46 根柱子之
间飞 2.0×1.2 m 矩形、每个角悬停 5 s，全程无 GPS、无罗盘。

- 先看 `docs/STATUS.md`（每一条结论都带数字与分母，以及**没做到什么**）
- 冻结版本：`git switch --detach mission-a-rect46`，说明在 `frozen/rect46-release.txt`
- 固件在 `firmware/`（上游 `dbe7921` + 一个真值 tap 补丁，含二进制与复现步骤）
- 图只看 `docs/figures/*.png` 的俯视图；其余诊断图在 `docs/figures/archive/`
