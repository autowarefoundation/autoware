cmake_minimum_required(VERSION 2.8.3)
project(registration)

find_package(catkin REQUIRED COMPONENTS)
find_package(PCL REQUIRED)

IF(PCL_VERSION VERSION_LESS "1.7.2")
message("fast_pcl requires PCL 1.7.2 or higher versions")
ELSE(PCL_VERSION VERSION_LESS "1.7.2")
set(SUBSYS_NAME registration)
set(SUBSYS_DESC "Point cloud registration library")
set(SUBSYS_DEPS common octree kdtree search sample_consensus features filters)

set(LIB_NAME "fast_pcl_registration")

catkin_package(
  DEPENDS ${SUBSYS_DEPS}
  INCLUDE_DIRS include
  LIBRARIES ${LIB_NAME}
  )

set(srcs
  src/registration.cpp
  src/correspondence_estimation.cpp
  src/correspondence_types.cpp
  src/icp.cpp
  src/icp_nl.cpp
  src/ndt.cpp
  src/transformation_estimation_svd.cpp
  src/transformation_estimation_lm.cpp
  src/transformation_estimation_point_to_plane_lls.cpp
)

set(incs 
  "include/fast_pcl/registration/eigen.h"
  "include/fast_pcl/registration/boost.h"
  "include/fast_pcl/registration/convergence_criteria.h"
  "include/fast_pcl/registration/default_convergence_criteria.h"
  "include/fast_pcl/registration/correspondence_estimation.h"
  "include/fast_pcl/registration/correspondence_rejection.h"
  "include/fast_pcl/registration/correspondence_sorting.h"
  "include/fast_pcl/registration/correspondence_types.h"
  "include/fast_pcl/registration/icp.h"
  "include/fast_pcl/registration/icp_nl.h"
  "include/fast_pcl/registration/ndt.h"
  "include/fast_pcl/registration/registration.h"
  "include/fast_pcl/registration/transformation_estimation.h"
  "include/fast_pcl/registration/transformation_estimation_svd.h"
  "include/fast_pcl/registration/transformation_estimation_lm.h"
  "include/fast_pcl/registration/transformation_estimation_point_to_plane_lls.h"
  "include/fast_pcl/registration/warp_point_rigid.h"
  "include/fast_pcl/registration/warp_point_rigid_6d.h"
  "include/fast_pcl/registration/distances.h"
)

set(impl_incs 
  "include/fast_pcl/registration/impl/default_convergence_criteria.hpp"
  "include/fast_pcl/registration/impl/correspondence_estimation.hpp"
  "include/fast_pcl/registration/impl/correspondence_rejection.hpp"
  "include/fast_pcl/registration/impl/correspondence_types.hpp"
  "include/fast_pcl/registration/impl/icp.hpp"
  "include/fast_pcl/registration/impl/icp_nl.hpp"
  "include/fast_pcl/registration/impl/ndt.hpp"
  "include/fast_pcl/registration/impl/registration.hpp"
  "include/fast_pcl/registration/impl/transformation_estimation_svd.hpp"
  "include/fast_pcl/registration/impl/transformation_estimation_lm.hpp"
  "include/fast_pcl/registration/impl/transformation_estimation_point_to_plane_lls.hpp"
)

include_directories(${PCL_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_SOURCE_DIR}/../filters/include")

add_library("${LIB_NAME}" ${srcs} ${incs} ${impl_incs})

target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES})
ENDIF(PCL_VERSION VERSION_LESS "1.7.2")
