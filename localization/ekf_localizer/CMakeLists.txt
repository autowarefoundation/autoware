cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Eigen3 REQUIRED)

ament_auto_add_executable(ekf_localizer
  src/ekf_localizer_node.cpp
  src/ekf_localizer.cpp
)
ament_target_dependencies(ekf_localizer kalman_filter)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(ekf_localizer-test test/test_ekf_localizer.test
#     test/src/test_ekf_localizer.cpp
#     src/ekf_localizer.cpp
#     src/kalman_filter/kalman_filter.cpp
#     src/kalman_filter/time_delay_kalman_filter.cpp
#   )
#   target_include_directories(ekf_localizer-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(ekf_localizer-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
