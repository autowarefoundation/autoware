cmake_minimum_required(VERSION 3.17)
project(lidar_apollo_instance_segmentation)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tier4_autoware_utils REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(tier4_perception_msgs REQUIRED)

find_package(tensorrt_common)
if(NOT ${tensorrt_common_FOUND})
  message(WARNING "The tensorrt_common package is not found. Please check its dependencies.")
  # to avoid launch file missing without a gpu
  ament_package()
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  return()
endif()
find_package(cuda_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/detector.cpp
  src/log_table.cpp
  src/feature_generator.cpp
  src/feature_map.cpp
  src/cluster2d.cpp
  src/debugger.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${cuda_utils_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${tier4_autoware_utils_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  pcl_common
  rclcpp::rclcpp
  rclcpp_components::component
  tensorrt_common::tensorrt_common
  tf2_eigen::tf2_eigen
  ${tier4_debug_msgs_TARGETS}
  ${tier4_perception_msgs_TARGETS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "lidar_apollo_instance_segmentation::LidarInstanceSegmentationNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_package()
install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)
