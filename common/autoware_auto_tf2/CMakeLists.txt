cmake_minimum_required(VERSION 3.5)
project(autoware_auto_tf2)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  ament_add_gtest(test_tf2_autoware_auto_msgs test/test_tf2_autoware_auto_msgs.cpp)
  # workaround to allow deprecated header to build on both galactic and rolling
  if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
    target_compile_definitions(test_tf2_autoware_auto_msgs PRIVATE
      USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
    )
  endif()
  target_include_directories(test_tf2_autoware_auto_msgs PRIVATE "include")
  ament_target_dependencies(test_tf2_autoware_auto_msgs
    "autoware_auto_common"
    "autoware_auto_perception_msgs"
    "autoware_auto_system_msgs"
    "autoware_auto_geometry_msgs"
    "geometry_msgs"
    "tf2"
    "tf2_geometry_msgs"
    "tf2_ros"
  )
endif()

ament_auto_package()
