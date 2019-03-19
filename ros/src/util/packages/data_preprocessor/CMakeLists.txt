cmake_minimum_required(VERSION 2.8.3)
project(data_preprocessor)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  #libpcl-all-dev
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  std_msgs
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES data_preprocessor
#  CATKIN_DEPENDS cv_bridge libpcl-all-dev pcd_tutorial pcl_conversions pcl_msgs pcl_ros roscpp std_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_Image nodes/get_Image/get_Image.cpp)
add_executable(get_PCD nodes/get_PCD/get_PCD.cpp)
add_executable(get_Depth nodes/get_Depth/get_Depth.cpp)
target_link_libraries(get_Depth ${catkin_LIBRARIES})
target_link_libraries(get_PCD ${catkin_LIBRARIES})
target_link_libraries(get_Image ${catkin_LIBRARIES})

install(TARGETS get_Image get_PCD get_Depth
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})