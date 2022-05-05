cmake_minimum_required(VERSION 3.5)
project(pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

ament_auto_add_executable(pose_initializer
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)
target_link_libraries(pose_initializer ${PCL_LIBRARIES})

# workaround to allow deprecated header to build on both galactic and rolling
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pose_initializer PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
