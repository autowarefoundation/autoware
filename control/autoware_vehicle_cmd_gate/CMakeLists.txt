cmake_minimum_required(VERSION 3.5)
project(autoware_vehicle_cmd_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(vehicle_cmd_gate_node SHARED
  src/vehicle_cmd_gate.cpp
  src/vehicle_cmd_filter.cpp
  src/adapi_pause_interface.cpp
  src/moderate_stop_interface.cpp
)

rclcpp_components_register_node(vehicle_cmd_gate_node
  PLUGIN "autoware::vehicle_cmd_gate::VehicleCmdGate"
  EXECUTABLE vehicle_cmd_gate_exe
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/IsFilterActivated.msg"
  DEPENDENCIES builtin_interfaces
)

# to use same package defined message
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(vehicle_cmd_gate_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(vehicle_cmd_gate_node "${cpp_typesupport_target}")
endif()


if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_vehicle_cmd_gate
    test/src/test_main.cpp
    test/src/test_vehicle_cmd_filter.cpp
    test/src/test_filter_in_vehicle_cmd_gate_node.cpp
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
