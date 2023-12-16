cmake_minimum_required(VERSION 3.5)
project(planning_debug_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  planning_debug_tools
  "msg/TrajectoryDebugInfo.msg"
  DEPENDENCIES builtin_interfaces
)

ament_auto_add_library(trajectory_analyzer_node SHARED
  src/trajectory_analyzer.cpp
)

ament_auto_add_library(stop_reason_visualizer_node SHARED
  src/stop_reason_visualizer.cpp
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(trajectory_analyzer_node
    planning_debug_tools "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target planning_debug_tools "rosidl_typesupport_cpp")
    target_link_libraries(trajectory_analyzer_node "${cpp_typesupport_target}")
endif()


rclcpp_components_register_node(trajectory_analyzer_node
  PLUGIN "planning_debug_tools::TrajectoryAnalyzerNode"
  EXECUTABLE trajectory_analyzer_exe
)

rclcpp_components_register_node(stop_reason_visualizer_node
  PLUGIN "planning_debug_tools::StopReasonVisualizerNode"
  EXECUTABLE stop_reason_visualizer_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

install(PROGRAMS
  scripts/processing_time_checker.py
  scripts/trajectory_visualizer.py
  scripts/closest_velocity_checker.py
  scripts/perception_replayer/perception_reproducer.py
  scripts/perception_replayer/perception_replayer.py
  scripts/update_logger_level.sh
  DESTINATION lib/${PROJECT_NAME}
)
