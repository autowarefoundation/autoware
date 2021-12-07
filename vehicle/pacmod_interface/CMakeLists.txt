cmake_minimum_required(VERSION 3.5)
project(pacmod_interface)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

ament_auto_add_executable(pacmod_additional_debug_publisher
  src/pacmod_additional_debug_publisher/pacmod_additional_debug_publisher_node.cpp
  src/pacmod_additional_debug_publisher/pacmod_additional_debug_publisher_main.cpp
)

ament_auto_add_executable(pacmod_dynamic_parameter_changer
  src/pacmod_dynamic_parameter_changer/pacmod_dynamic_parameter_changer_node.cpp
  src/pacmod_dynamic_parameter_changer/pacmod_dynamic_parameter_changer_main.cpp
)

ament_auto_add_executable(pacmod_interface
  src/pacmod_interface/pacmod_interface.cpp
  src/pacmod_interface/pacmod_interface_node.cpp
)

ament_auto_add_executable(pacmod_diag_publisher
  src/pacmod_interface/pacmod_diag_publisher.cpp
  src/pacmod_interface/pacmod_diag_publisher_node.cpp
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
