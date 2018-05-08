cmake_minimum_required(VERSION 2.8.3)
project(icp_localizer)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        autoware_msgs
        pcl_conversions
        velodyne_pointcloud
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS std_msgs autoware_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(icp_matching nodes/icp_matching/icp_matching.cpp)

target_link_libraries(icp_matching ${catkin_LIBRARIES})

add_dependencies(icp_matching ${catkin_EXPORTED_TARGETS})

install(TARGETS icp_matching
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)