cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter_localizer)

find_package(catkin REQUIRED
        autoware_build_flags
        roscpp
        std_msgs
        tf2
        tf2_ros
        geometry_msgs
        autoware_msgs
        sensor_msgs
        rostest
        rosunit
        amathutils_lib
)

find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
        autoware_build_flags
        roscpp
        std_msgs
        tf2
        tf2_ros
        geometry_msgs
        autoware_msgs
        sensor_msgs
        amathutils_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(kalman_filter_localizer src/kalman_filter_localizer.cpp)
add_dependencies(kalman_filter_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kalman_filter_localizer ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS kalman_filter_localizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/kalman_filter/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############
