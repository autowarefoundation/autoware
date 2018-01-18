cmake_minimum_required(VERSION 2.8.3)
project(filters)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

IF(PCL_VERSION VERSION_LESS "1.7.2")
  message("fast_pcl requires PCL 1.7.2 or higher versions")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
  set(SUBSYS_NAME filters)
  set(SUBSYS_DESC "Point cloud filters library")
  set(LIB_NAME "fast_pcl_filters")


  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${LIB_NAME}
      DEPENDS PCL
  )

  set(srcs
    src/filter.cpp
    src/voxel_grid.cpp
    src/voxel_grid_covariance.cpp
  )

  set(incs
    "include/fast_pcl/filters/boost.h"
    "include/fast_pcl/filters/filter.h"
    "include/fast_pcl/filters/voxel_grid.h"
    "include/fast_pcl/filters/voxel_grid_covariance.h"
  )

  set(impl_incs
    "include/fast_pcl/filters/impl/filter.hpp"
    "include/fast_pcl/filters/impl/voxel_grid.hpp"
    "include/fast_pcl/filters/impl/voxel_grid_covariance.hpp"
  )

  include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" ${catkin_INCLUDE_DIRS})

  add_library("${LIB_NAME}" ${srcs} ${incs} ${impl_incs})

  target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES} ${catkin_LIBRARIES})

  install(TARGETS "${LIB_NAME}"
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

ENDIF(PCL_VERSION VERSION_LESS "1.7.2")
