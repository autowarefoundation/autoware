cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/diagnostics.cpp
  src/fail_safe.cpp
  src/heartbeat.cpp
  src/interface.cpp
  src/localization.cpp
  src/motion.cpp
  src/operation_mode.cpp
  src/perception.cpp
  src/planning.cpp
  src/routing.cpp
  src/vehicle.cpp
  src/vehicle_info.cpp
  src/vehicle_door.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
  src/compatibility/autoware_state.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::default_adapi::AutowareStateNode"
  "autoware::default_adapi::DiagnosticsNode"
  "autoware::default_adapi::FailSafeNode"
  "autoware::default_adapi::HeartbeatNode"
  "autoware::default_adapi::InterfaceNode"
  "autoware::default_adapi::LocalizationNode"
  "autoware::default_adapi::MotionNode"
  "autoware::default_adapi::OperationModeNode"
  "autoware::default_adapi::PerceptionNode"
  "autoware::default_adapi::PlanningNode"
  "autoware::default_adapi::RoutingNode"
  "autoware::default_adapi::VehicleNode"
  "autoware::default_adapi::VehicleInfoNode"
  "autoware::default_adapi::VehicleDoorNode"
)

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py script/guide.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch test)
