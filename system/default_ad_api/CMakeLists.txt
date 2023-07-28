cmake_minimum_required(VERSION 3.14)
project(default_ad_api)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fail_safe.cpp
  src/interface.cpp
  src/localization.cpp
  src/motion.cpp
  src/operation_mode.cpp
  src/perception.cpp
  src/planning.cpp
  src/routing.cpp
  src/vehicle.cpp
  src/vehicle_info.cpp
  src/utils/route_conversion.cpp
  src/compatibility/autoware_state.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "default_ad_api::AutowareStateNode"
  "default_ad_api::FailSafeNode"
  "default_ad_api::InterfaceNode"
  "default_ad_api::LocalizationNode"
  "default_ad_api::MotionNode"
  "default_ad_api::OperationModeNode"
  "default_ad_api::PerceptionNode"
  "default_ad_api::PlanningNode"
  "default_ad_api::RoutingNode"
  "default_ad_api::VehicleNode"
  "default_ad_api::VehicleInfoNode"
)

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

install(
  PROGRAMS script/web_server.py script/guide.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch test)
