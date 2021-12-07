cmake_minimum_required(VERSION 3.5)
project(planning_error_monitor)

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

ament_auto_add_library(planning_error_monitor_node SHARED
  src/planning_error_monitor_node.cpp
  src/debug_marker.cpp
)
rclcpp_components_register_node(planning_error_monitor_node
  PLUGIN "planning_diagnostics::PlanningErrorMonitorNode"
  EXECUTABLE planning_error_monitor
)

ament_auto_add_library(invalid_trajectory_publisher_node SHARED
  src/invalid_trajectory_publisher.cpp
)
rclcpp_components_register_node(invalid_trajectory_publisher_node
  PLUGIN "planning_diagnostics::InvalidTrajectoryPublisherNode"
  EXECUTABLE invalid_trajectory_publisher
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_planning_error_monitor
    test/src/test_main.cpp
    test/src/test_planning_error_monitor_functions.cpp
    test/src/test_planning_error_monitor_helper.hpp
    test/src/test_planning_error_monitor_pubsub.cpp
  )
  ament_target_dependencies(test_planning_error_monitor
    rclcpp
    autoware_auto_planning_msgs
  )
  target_link_libraries(test_planning_error_monitor
    planning_error_monitor_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
