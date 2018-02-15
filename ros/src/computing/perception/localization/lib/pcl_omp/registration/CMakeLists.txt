cmake_minimum_required(VERSION 2.8.3)
project(pcl_omp_registration)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

IF(PCL_VERSION VERSION_LESS "1.7.2")
message("pcl_omp requires PCL 1.7.2 or higher versions")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
set(SUBSYS_NAME pcl_omp_registration)
set(SUBSYS_DESC "Point cloud registration library")
set(SUBSYS_DEPS pcl_common octree kdtree search sample_consensus features)

set(LIB_NAME "pcl_omp_registration")

catkin_package(
  #DEPENDS ${SUBSYS_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${LIB_NAME}
  )

set(srcs
  src/ndt.cpp
)

set(incs
  "include/pcl_omp/registration/ndt.h"
  "include/pcl_omp/registration/registration.h"
)

set(impl_incs
  "include/pcl_omp/registration/impl/ndt.hpp"
  "include/pcl_omp/registration/impl/registration.hpp"
)

include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library("${LIB_NAME}" ${srcs} ${incs} ${impl_incs})

target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES})
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")
