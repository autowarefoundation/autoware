cmake_minimum_required(VERSION 3.14)
project(autoware_lidar_marker_localizer)

# find dependencies
find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/lidar_marker_localizer.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::lidar_marker_localizer::LidarMarkerLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
