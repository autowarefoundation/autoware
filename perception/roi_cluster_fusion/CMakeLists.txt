cmake_minimum_required(VERSION 3.5)
project(roi_cluster_fusion)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(roi_cluster_fusion_nodelet SHARED src/roi_cluster_fusion_nodelet.cpp)
target_link_libraries(roi_cluster_fusion_nodelet
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

rclcpp_components_register_node(roi_cluster_fusion_nodelet
  PLUGIN "roi_cluster_fusion::RoiClusterFusionNodelet"
  EXECUTABLE roi_cluster_fusion_node
)

ament_auto_package(INSTALL_TO_SHARE
    launch
)

if(BUILD_TESTING)
   find_package(ament_lint_auto REQUIRED)
   ament_lint_auto_find_test_dependencies()
 endif()
