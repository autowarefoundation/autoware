cmake_minimum_required(VERSION 3.5)
project(route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(route_handler SHARED
  src/route_handler.cpp
)

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(route_handler PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

ament_auto_package()
