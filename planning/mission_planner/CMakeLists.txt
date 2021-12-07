cmake_minimum_required(VERSION 3.5)
project(mission_planner)

### Compile options
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

ament_auto_add_library(mission_planner_node SHARED
  lib/mission_planner_base.cpp
  src/mission_planner_lanelet2/mission_planner_lanelet2.cpp
  src/mission_planner_lanelet2/utility_functions.cpp
)

rclcpp_components_register_node(mission_planner_node
  PLUGIN "mission_planner::MissionPlannerLanelet2"
  EXECUTABLE mission_planner
)

ament_auto_add_library(goal_pose_visualizer_node SHARED
  src/goal_pose_visualizer/goal_pose_visualizer.cpp
)

rclcpp_components_register_node(goal_pose_visualizer_node
  PLUGIN "mission_planner::GoalPoseVisualizer"
  EXECUTABLE goal_pose_visualizer
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
