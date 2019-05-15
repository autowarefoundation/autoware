cmake_minimum_required(VERSION 2.8.3)
project(autoware_lidar_localizer)

SET(LIB_NAME "autoware_lidar_localizer")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(PCL REQUIRED)

set(PCL_LIBRARY_PACKAGES pcl_registration)

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
   CATKIN_DEPENDS pcl_ros ${PCL_LIBRARY_PACKAGES}
#  DEPENDS
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
add_library(${LIB_NAME}
  src/lidar_localizer.cpp
  src/ndt/ndt_slam_base.cpp
  src/ndt/ndt_slam_pcl.cpp
  src/map_manager/map_manager.cpp
  src/matching_score/matching_score.cpp
  src/matching_score/matching_score_histogram.cpp
  src/pose_linear_interpolator/pose_linear_interpolator.cpp
  src/util/convert_ros_msgs.cpp
  src/util/util_functions.cpp
)

if(CUDA_FOUND)
target_include_directories(${LIB_NAME} PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )


install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
