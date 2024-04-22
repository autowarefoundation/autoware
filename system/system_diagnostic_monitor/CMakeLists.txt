cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

install(PROGRAMS
  script/component_state_diagnostics.py
  RENAME component_state_diagnostics
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch)
