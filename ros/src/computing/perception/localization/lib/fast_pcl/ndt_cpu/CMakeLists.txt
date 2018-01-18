cmake_minimum_required(VERSION 2.8.3)
project(ndt_cpu)

find_package(PCL REQUIRED)

set(SUBSYS_NAME ndt_cpu)
set(SUBSYS_DESC "Point cloud ndt cpu library for testing")
set(LIB_NAME "fast_pcl_ndt_cpu")

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${LIB_NAME}
    DEPENDS PCL
)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set(srcs
	src/NormalDistributionsTransform.cpp
	src/Registration.cpp
	src/VoxelGrid.cpp
)
	
set(incs
  include/fast_pcl/ndt_cpu/debug.h
  include/fast_pcl/ndt_cpu/NormalDistributionsTransform.h
  include/fast_pcl/ndt_cpu/Registration.h
  include/fast_pcl/ndt_cpu/SymmetricEigenSolver.h
  include/fast_pcl/ndt_cpu/VoxelGrid.h
)

add_library("${LIB_NAME}" ${incs} ${srcs})

target_link_libraries("${LIB_NAME}" ${PCL_LIBRRIES} ${catkin_LIBRARIES})

install(TARGETS "${LIB_NAME}"
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
