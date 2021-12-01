cmake_minimum_required(VERSION 3.5)
project(autoware_planning_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Approval.msg"
  "msg/EnableAvoidance.msg"
  "msg/ExpandStopRange.msg"
  "msg/IsAvoidancePossible.msg"
  "msg/LaneChangeCommand.msg"
  "msg/LaneChangeStatus.msg"
  "msg/LaneSequence.msg"
  "msg/LateralOffset.msg"
  "msg/Path.msg"
  "msg/PathChangeModule.msg"
  "msg/PathChangeModuleArray.msg"
  "msg/PathChangeModuleId.msg"
  "msg/PathPoint.msg"
  "msg/PathWithLaneId.msg"
  "msg/PathPointWithLaneId.msg"
  "msg/Route.msg"
  "msg/RouteSection.msg"
  "msg/Scenario.msg"
  "msg/StopFactor.msg"
  "msg/StopReason.msg"
  "msg/StopReasonArray.msg"
  "msg/StopSpeedExceeded.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  "msg/VelocityLimit.msg"
  "msg/VelocityLimitClearCommand.msg"
  "msg/VelocityLimitConstraints.msg"
  DEPENDENCIES
    geometry_msgs
    nav_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
