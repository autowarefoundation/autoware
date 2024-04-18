cmake_minimum_required(VERSION 3.14)
project(tier4_autoware_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)

ament_auto_add_library(tier4_autoware_utils SHARED
  src/geometry/geometry.cpp
  src/geometry/pose_deviation.cpp
  src/geometry/boost_polygon_utils.cpp
  src/math/sin_table.cpp
  src/math/trigonometry.cpp
  src/ros/msg_operation.cpp
  src/ros/marker_helper.cpp
  src/ros/logger_level_configure.cpp
  src/system/backtrace.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  file(GLOB_RECURSE test_files test/**/*.cpp)

  ament_add_ros_isolated_gtest(test_tier4_autoware_utils ${test_files})

  target_link_libraries(test_tier4_autoware_utils
    tier4_autoware_utils
  )
endif()

ament_auto_package()
