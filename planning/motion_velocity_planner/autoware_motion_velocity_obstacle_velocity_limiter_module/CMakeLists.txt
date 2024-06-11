cmake_minimum_required(VERSION 3.5)
project(autoware_motion_velocity_obstacle_velocity_limiter_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner_node plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_forward_projection.cpp
    test/test_obstacles.cpp
    test/test_collision_distance.cpp
    test/test_occupancy_grid_utils.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

add_executable(collision_benchmark
  benchmarks/collision_checker_benchmark.cpp
)
target_link_libraries(collision_benchmark
  ${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
)
