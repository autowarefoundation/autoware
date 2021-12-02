cmake_minimum_required(VERSION 3.5)
project(topic_state_monitor)

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

ament_auto_add_library(topic_state_monitor SHARED
  src/topic_state_monitor/topic_state_monitor.cpp
  src/topic_state_monitor_core.cpp
)

rclcpp_components_register_node(topic_state_monitor
  PLUGIN "topic_state_monitor::TopicStateMonitorNode"
  EXECUTABLE topic_state_monitor_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
## directories
ament_auto_package(INSTALL_TO_SHARE
  launch
)
