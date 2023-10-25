cmake_minimum_required(VERSION 3.8)
project(universe_dbw_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(dbw_ford_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(Lincoln_MKZ_Bridge src/Lincoln_MKZ_Bridge.cpp)
ament_target_dependencies(Lincoln_MKZ_Bridge rclcpp autoware_auto_vehicle_msgs autoware_auto_control_msgs dbw_ford_msgs sensor_msgs)

install(TARGETS
  Lincoln_MKZ_Bridge
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
