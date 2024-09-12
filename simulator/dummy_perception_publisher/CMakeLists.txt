cmake_minimum_required(VERSION 3.14)
project(dummy_perception_publisher)

find_package(autoware_cmake REQUIRED)
autoware_package()

# See ndt_omp package for documentation on why PCL is special
find_package(PCL REQUIRED COMPONENTS common filters)

set(${PROJECT_NAME}_DEPENDENCIES
  autoware_perception_msgs
  tier4_perception_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_auto_add_library(signed_distance_function SHARED
  src/signed_distance_function.cpp
)

ament_auto_add_executable(dummy_perception_publisher_node
  src/main.cpp
  src/node.cpp
  src/pointcloud_creator.cpp
)

target_link_libraries(dummy_perception_publisher_node
  signed_distance_function
)

ament_target_dependencies(dummy_perception_publisher_node ${${PROJECT_NAME}_DEPENDENCIES})

target_include_directories(dummy_perception_publisher_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# PCL dependencies - `ament_target_dependencies` doesn't respect the
# components/modules selected above and only links in `common` ,so we need
# to do this manually.
target_compile_definitions(dummy_perception_publisher_node PRIVATE ${PCL_DEFINITIONS})
target_include_directories(dummy_perception_publisher_node PRIVATE ${PCL_INCLUDE_DIRS})
# Unfortunately, this one can't be PRIVATE because only the plain or only the
# keyword (PRIVATE) signature of target_link_libraries can be used for one
# target, not both. The plain signature is already used inside
# `ament_target_dependencies` and possibly rosidl_target_interfaces.
target_link_libraries(dummy_perception_publisher_node ${PCL_LIBRARIES})
target_link_directories(dummy_perception_publisher_node PRIVATE ${PCL_LIBRARY_DIRS})


ament_auto_add_executable(empty_objects_publisher
  src/empty_objects_publisher.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(signed_distance_function-test
    test/src/test_signed_distance_function.cpp
  )
  target_link_libraries(signed_distance_function-test
    signed_distance_function
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
