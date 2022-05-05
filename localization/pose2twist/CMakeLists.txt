cmake_minimum_required(VERSION 3.5)
project(pose2twist)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(pose2twist
  src/pose2twist_node.cpp
  src/pose2twist_core.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pose2twist PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
