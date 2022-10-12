cmake_minimum_required(VERSION 3.5)
project(vehicle_cmd_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_cmd_gate_node SHARED
  src/vehicle_cmd_gate.cpp
  src/vehicle_cmd_filter.cpp
  src/pause_interface.cpp
)

rclcpp_components_register_node(vehicle_cmd_gate_node
  PLUGIN "vehicle_cmd_gate::VehicleCmdGate"
  EXECUTABLE vehicle_cmd_gate
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_vehicle_cmd_gate
    test/src/test_main.cpp
    test/src/test_vehicle_cmd_filter.cpp
  )
  ament_target_dependencies(test_vehicle_cmd_gate
    rclcpp
    tier4_control_msgs
  )
  target_link_libraries(test_vehicle_cmd_gate
    vehicle_cmd_gate_node
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
