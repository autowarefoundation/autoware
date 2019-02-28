cmake_minimum_required(VERSION 2.8.3)
project(autoware_lidar_localizer)

SET(LIB_NAME "autoware_lidar_localizer")

execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(PCL_LIBRARY_PACKAGES pcl_registration pcl_omp_registration ndt_cpu ndt_gpu)
ENDIF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package( OpenMP )
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DOMP_FOUND)
endif()

find_package(CUDA)
if(CUDA_FOUND)
   add_definitions(-DCUDA_FOUND)
endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  ${PCL_LIBRARY_PACKAGES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${LIB_NAME}
   CATKIN_DEPENDS ${PCL_LIBRARY_PACKAGES}
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_library(${LIB_NAME}
  src/lidar_localizer.cpp
  src/ndt/ndt_slam_base.cpp
  src/ndt/ndt_slam_dummy.cpp
  src/ndt/ndt_slam_pcl.cpp
  src/ndt/ndt_slam_pcl_omp.cpp
  src/ndt/ndt_slam_pcl_anh.cpp
  src/ndt/ndt_slam_pcl_anh_gpu.cpp
  src/map_manager/map_manager.cpp
  src/pose_linear_interpolator/pose_linear_interpolator.cpp
  src/util/convert_ros_msgs.cpp
  src/util/util_functions.cpp
)

if(CUDA_FOUND)
target_include_directories(${LIB_NAME} PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})
