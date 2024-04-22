cmake_minimum_required(VERSION 3.14)
project(diagnostic_graph_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lib/graph.cpp
  src/lib/subscription.cpp
)

ament_auto_add_executable(dump
  src/node/dump.cpp
)

ament_auto_add_executable(converter
  src/node/converter.cpp
)

ament_auto_package()
