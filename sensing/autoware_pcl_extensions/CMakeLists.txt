cmake_minimum_required(VERSION 3.14)
project(autoware_pcl_extensions)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_library(autoware_pcl_extensions SHARED
  src/voxel_grid_nearest_centroid.cpp
)

target_link_libraries(autoware_pcl_extensions ${PCL_LIBRARIES})

ament_auto_package()
