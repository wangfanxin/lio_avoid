cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

# ---------------------------------------------------------------------------------------------------
# A MSG-ONLY VENDORED PACKAGE.  Read README.md in this directory first: it says why the package exists,
# why it is minimal, and what it is pinned to.
#
# WHAT WAS REMOVED FROM EGO'S OWN CMakeLists.txt AND WHY:
#   * `find_package(rclcpp)` and `DEPENDENCIES rclcpp` -- our copy is Python-side only; no C++ client
#     library is compiled against these interfaces, so the dependency would be dead weight.
#   * `find_package(Eigen3)` + `include_directories(${EIGEN3_INCLUDE_DIR})` -- only ego's
#     `decode_msgs`/`encode_msgs` C++ sources need Eigen, and those are not vendored.
#   * `add_library(decode_msgs ...)` / `add_library(encode_msgs ...)`, their `rosidl_get_typesupport_target`
#     links and their `install(TARGETS ...)` -- that C++ is the Serial/LQR/PPR side of ego's uav_simulator
#     (`src/decode_msgs.cpp`, `src/encode_msgs.cpp`, `include/`), which nothing in this repository uses.
#     This is what makes the copy msg-ONLY: no runtime, nothing to link, nothing to go stale.
#   * the BUILD_TESTING / ament_lint block -- ego's copy skips the linters anyway, and a lint failure in
#     a vendored interface is not a fact about our flight code.
# WHAT WAS KEPT, VERBATIM: the generated interface itself.  `rosidl_generate_interfaces` lists the SAME
# 13 .msg files ego lists, in the same order, with the same DEPENDENCIES -- because `std_msgs/Header`,
# `geometry_msgs/Point` and `geometry_msgs/Vector3` have to resolve, and the other 12 .msg files are what
# ego's own package generates.  See README.md for why all 13 are kept rather than only PositionCommand.
# ---------------------------------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs std_msgs)

ament_package()
