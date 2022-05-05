cmake_minimum_required(VERSION 3.14)
project(ad_service_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(ad_service_state_monitor
  src/ad_service_state_monitor_node/main.cpp
  src/ad_service_state_monitor_node/ad_service_state_monitor_node.cpp
  src/ad_service_state_monitor_node/state_machine.cpp
  src/ad_service_state_monitor_node/diagnostics.cpp
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
