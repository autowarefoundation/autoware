cmake_minimum_required(VERSION 3.5)
project(control_performance_analysis)

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

find_package(Boost REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(control_performance_analysis_msgs
  msg/Error.msg
  msg/ErrorStamped.msg
  DEPENDENCIES
    std_msgs
)

ament_auto_add_library(control_performance_analysis_core SHARED
  src/control_performance_analysis_utils.cpp
  src/control_performance_analysis_core.cpp
)

ament_auto_add_library(control_performance_analysis_node SHARED
  src/control_performance_analysis_node.cpp
)

rosidl_target_interfaces(control_performance_analysis_node
  control_performance_analysis_msgs "rosidl_typesupport_cpp")

target_link_libraries(control_performance_analysis_node
  control_performance_analysis_core
)

rclcpp_components_register_node(control_performance_analysis_node
  PLUGIN "control_performance_analysis::ControlPerformanceAnalysisNode"
  EXECUTABLE control_performance_analysis
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
