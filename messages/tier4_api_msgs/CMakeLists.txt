cmake_minimum_required(VERSION 3.5)
project(tier4_api_msgs)

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
  "msg/AwapiAutowareStatus.msg"
  "msg/AwapiVehicleStatus.msg"
  "msg/CrosswalkStatus.msg"
  "msg/DoorControlCommand.msg"
  "msg/DoorStatus.msg"
  "msg/Euler.msg"
  "msg/IntersectionStatus.msg"
  "msg/LaneChangeStatus.msg"
  "msg/ObstacleAvoidanceStatus.msg"
  "msg/StopCommand.msg"
  "msg/VelocityLimit.msg"
  DEPENDENCIES
    tier4_planning_msgs
    tier4_system_msgs
    diagnostic_msgs
    geographic_msgs
    geometry_msgs
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
