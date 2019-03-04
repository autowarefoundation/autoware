cmake_minimum_required(VERSION 3.5)
project(lidar_fake_perception)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(tf REQUIRED)
find_package(tf_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(lidar_fake_perception_node
  nodes/lidar_fake_perception.cpp
  nodes/lidar_fake_perception_node.cpp
)
ament_target_dependencies(lidar_fake_perception_node
  roscpp
  tf
  tf_conversions
  pcl_ros
  geometry_msgs
  autoware_msgs
  )

install(TARGETS
  lidar_fake_perception_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(roscpp)
ament_export_dependencies(tf)
ament_export_dependencies(tf_conversions)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(autoware_msgs)

ament_export_include_directories(include)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()