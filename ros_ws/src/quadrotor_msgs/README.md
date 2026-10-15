# `quadrotor_msgs` — vendored, msg-only (PLAN box → BRIDGE command interface)

**What this is.** The ROS 2 message package `quadrotor_msgs`, copied byte-for-byte into our tree. The one
message the refactor needs is `PositionCommand`, published by the PLAN box on **`/plan/cmd`** and
consumed by the BRIDGE (`_stage/refactor-bridge-plan.md` §3.3). This directory is the canonical source;
the colcon workspace the run scripts already source
(`/home/fanxin/lio_avoid/lio_ws`) reaches it through a **symlink** at `lio_ws/src/quadrotor_msgs`.

    $ ls -la /home/fanxin/lio_avoid/lio_ws/src/
    lrwxrwxrwx  basic            -> /home/fanxin/lio_avoid/src/Super-LIO/src/basic
    drwxr-xr-x  livox_ros_driver2
    lrwxrwxrwx  super_lio        -> /home/fanxin/lio_avoid/src/Super-LIO/src/super_lio
    lrwxrwxrwx  quadrotor_msgs   -> /mnt/d/distill_self/lio_avoid/ros_ws/src/quadrotor_msgs

A symlink is what the workspace already does for `basic` and `super_lio`, so this adds no new mechanism
and no copy that can drift: `tools/run-mission.sh` sources `lio_ws/install/setup.bash` (line 86), so the
interface is found by every script that already exists, with **no edit to any of them**.

## Why it exists (why the interface is vendored rather than imported)

Two packages of the same name cannot coexist in one workspace, so the vendored copy and ego's must *be*
the same package — but ego's copy is inside `/home/fanxin/lio_avoid/src/ego-planner-swarm`, whose line is
being dropped, and the previous integration's equivalent lived in a workspace (`planner_ws`) that is going
with it. The interface a *frozen* PLAN→BRIDGE contract is written against must not be owned by the thing
that is being replaced: **the plane's command interface outlives any one planner.** Vendoring it here is
the `PLAN.md` rule-7 move ("pin and print revisions") applied to a message type: the SHA-256 of every
`.msg` is recorded in `PROVENANCE.txt`, so "the same interface" is a checkable fact rather than a hope.

## Why all 13 `.msg` files and not only `PositionCommand.msg`

`PositionCommand` itself needs only `std_msgs/Header`, `geometry_msgs/Point` and
`geometry_msgs/Vector3`. The other 12 are kept because this package is the same *package* as ego's: a
dropped-in ego-planner-swarm builds against `SO3Command`, `PolynomialTrajectory`, `Odometry`, `Gains`,
`LQRTrajectory`, `AuxCommand`, `Corrections`, `OutputData`, `PPROutputData`, `Serial`, `StatusData` and
`TRPYCommand`, and the acceptance test of this interface is "ego drops in without an adapter"
(§3.2–3.3). Shipping a subset would make the vendored package a *different* package that happens to
share one message, i.e. exactly the shim the refactor is deleting.

## Why it is MSG-ONLY

Ego's own `CMakeLists.txt` also builds two C++ libraries, `decode_msgs` and `encode_msgs`, and pulls in
`rclcpp` and Eigen3 to do it. Both libraries encode/decode the `Serial`/`LQRTrajectory`/`PPROutputData`
side of ego's `uav_simulator`, which nothing in this repository publishes or subscribes to. They are not
vendored, so this package compiles interfaces and nothing else: no runtime, no link step, and no C++ that
can go stale against a planner it is not part of. `CMakeLists.txt` lists what was removed and why.

## Provenance and pinning

| item | value |
|---|---|
| source | `/home/fanxin/lio_avoid/src/ego-planner-swarm/src/uav_simulator/Utils/quadrotor_msgs` |
| source repo HEAD | `23a8d5a` ("Update Readme.md"), working tree carries the previous integration's 3 modified files (`ego_replan_fsm.h`, `ego_replan_fsm.cpp`, `planner_manager.cpp`) — **none of them is a `.msg`** |
| `msg/PositionCommand.msg` | sha256 `9e0e570fa796d18ea4170c569491d1ba6a63daa78007d0d12c3b7b208c597944` |
| all 13 `.msg` | sha256 recorded in `PROVENANCE.txt`; `_stage/_vendor-rest.sh` re-checks all 13 |
| line endings | the upstream files are **CRLF**; the copy keeps CRLF byte-for-byte, and `.gitattributes` in this directory disables git's line-ending normalisation for `*.msg`. A Windows-side editor rewrote the first attempt to LF and dropped a trailing space — which would have produced a *different* message type that still compiled. That is why the pin is a hash and not a diff. |

## A typo that is deliberately preserved

`PositionCommand.msg:15` defines `TRAJECTROY_STATUS_ABORT = 4` — **"TRAJECTROY"**, not "TRAJECTORY".
Ego's file says that, and ego's own code and the previous integration's patches spell it the same way. It
is preserved because this package must generate the same constant names ego's code compiles against;
fixing the spelling here would break the drop-in property that is the package's entire reason to exist.

## How to check the interface is what it claims

    $ source /opt/ros/humble/setup.bash
    $ source /home/fanxin/lio_avoid/lio_ws/install/setup.bash
    $ ros2 interface show quadrotor_msgs/msg/PositionCommand

and the build:

    $ colcon build --packages-select quadrotor_msgs --symlink-install \
          --base-paths /home/fanxin/lio_avoid/lio_ws/src

Nothing in this package is specific to the rectangle trajectory source, to ego, or to this mission: it is
the 13 interfaces, and only `PositionCommand` is used today.
