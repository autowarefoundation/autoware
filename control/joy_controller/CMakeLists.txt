cmake_minimum_required(VERSION 3.5)
project(joy_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(joy_controller_node SHARED
  src/joy_controller/joy_controller_node.cpp
)

rclcpp_components_register_node(joy_controller_node
  PLUGIN "joy_controller::AutowareJoyControllerNode"
  EXECUTABLE joy_controller
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
