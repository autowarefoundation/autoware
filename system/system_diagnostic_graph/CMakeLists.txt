cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_graph)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(aggregator
  src/core/config.cpp
  src/core/debug.cpp
  src/core/graph.cpp
  src/core/nodes.cpp
  src/core/exprs.cpp
  src/core/modes.cpp
  src/main.cpp
)

ament_auto_add_executable(converter
  src/tool.cpp
)

install(
  PROGRAMS util/debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config example launch)
