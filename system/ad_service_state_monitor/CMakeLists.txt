cmake_minimum_required(VERSION 3.5)
project(ad_service_state_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(ad_service_state_monitor
  src/ad_service_state_monitor_node/main.cpp
  src/ad_service_state_monitor_node/ad_service_state_monitor_node.cpp
  src/ad_service_state_monitor_node/state_machine.cpp
  src/ad_service_state_monitor_node/diagnostics.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(ad_service_state_monitor PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
