cmake_minimum_required(VERSION 3.5)
project(obstacle_velocity_limiter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)

ament_auto_add_library(obstacle_velocity_limiter_node SHARED
  DIRECTORY src
)

ament_target_dependencies(obstacle_velocity_limiter_node PCL)
target_include_directories(obstacle_velocity_limiter_node
  SYSTEM PUBLIC
    "${PCL_INCLUDE_DIRS}"
)

# Disable warnings due to external dependencies
get_target_property(lanelet2_core_INCLUDE_DIR
  lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES
)

rclcpp_components_register_node(obstacle_velocity_limiter_node
  PLUGIN "obstacle_velocity_limiter::ObstacleVelocityLimiterNode"
  EXECUTABLE obstacle_velocity_limiter
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_forward_projection.cpp
    test/test_obstacles.cpp
    test/test_collision_distance.cpp
    test/test_occupancy_grid_utils.cpp
    test/test_${PROJECT_NAME}_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    obstacle_velocity_limiter_node
  )
endif()

add_executable(collision_benchmark
  benchmarks/collision_checker_benchmark.cpp
)
target_link_libraries(collision_benchmark
  obstacle_velocity_limiter_node
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
    script
)
