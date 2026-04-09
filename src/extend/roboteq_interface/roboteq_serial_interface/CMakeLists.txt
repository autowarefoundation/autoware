cmake_minimum_required(VERSION 3.5)
project(roboteq_serial_interface)

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

find_package(Boost REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(roboteq_serial_interface_node SHARED
  src/roboteq_serial_interface.cpp
)

target_link_libraries(roboteq_serial_interface_node
  ${Boost_LIBRARIES}
)

rclcpp_components_register_node(roboteq_serial_interface_node
  PLUGIN "RoboteqSerialInterface"
  EXECUTABLE roboteq_serial_interface
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
