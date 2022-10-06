cmake_minimum_required(VERSION 3.14)
project(ground_segmentation)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(ground_segmentation SHARED
  src/ray_ground_filter_nodelet.cpp
  src/ransac_ground_filter_nodelet.cpp
  src/scan_ground_filter_nodelet.cpp
)

target_link_libraries(ground_segmentation
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

if(OPENMP_FOUND)
  set_target_properties(ground_segmentation PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

# ========== Ground Filter ==========
# -- Ray Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::RayGroundFilterComponent"
  EXECUTABLE ray_ground_filter_node)

# -- RANSAC Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::RANSACGroundFilterComponent"
  EXECUTABLE ransac_ground_filter_node)

# -- Scan Ground Filter --
rclcpp_components_register_node(ground_segmentation
  PLUGIN "ground_segmentation::ScanGroundFilterComponent"
  EXECUTABLE scan_ground_filter_node)


ament_auto_package(INSTALL_TO_SHARE
  launch
)
