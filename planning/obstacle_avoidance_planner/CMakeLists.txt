cmake_minimum_required(VERSION 3.5)
project(obstacle_avoidance_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_library(obstacle_avoidance_planner SHARED
  src/vehicle_model/vehicle_model_interface.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.cpp
  src/util.cpp
  src/debug.cpp
  src/eb_path_optimizer.cpp
  src/mpt_optimizer.cpp
  src/process_cv.cpp
  src/node.cpp
)

target_include_directories(obstacle_avoidance_planner
  PUBLIC
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(obstacle_avoidance_planner
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(obstacle_avoidance_planner
  PLUGIN "ObstacleAvoidancePlanner"
  EXECUTABLE obstacle_avoidance_planner_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    scripts
)
