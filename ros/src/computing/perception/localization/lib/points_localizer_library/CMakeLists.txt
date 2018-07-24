cmake_minimum_required(VERSION 2.8.3)
project(points_localizer_library)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

IF(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
SET(FAST_PCL_PACKAGES pcl_registration pcl_omp_registration ndt_cpu ndt_gpu)
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
  ${FAST_PCL_PACKAGES}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES points_localizer_library
   CATKIN_DEPENDS ${FAST_PCL_PACKAGES}
#  DEPENDS ndt_tku
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_library(points_localizer_library
  src/points_localizer.cpp
  include/points_localizer/reliability/slam_reliability.cpp
  include/points_localizer/util/libconvert_ros_msgs.cpp
)

if(CUDA_FOUND)
target_include_directories(points_localizer_library PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

target_link_libraries(points_localizer_library ${catkin_LIBRARIES})
