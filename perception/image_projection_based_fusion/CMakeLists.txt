cmake_minimum_required(VERSION 3.14)
project(image_projection_based_fusion)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/fusion_node.cpp
  src/debugger.cpp
  src/utils/geometry.cpp
  src/utils/utils.cpp
  src/roi_cluster_fusion/node.cpp
  src/roi_detected_object_fusion/node.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "image_projection_based_fusion::RoiClusterFusionNode"
  EXECUTABLE roi_cluster_fusion_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "image_projection_based_fusion::RoiDetectedObjectFusionNode"
  EXECUTABLE roi_detected_object_fusion_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
)
