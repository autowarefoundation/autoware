cmake_minimum_required(VERSION 2.8.3)
project(ekf_localizer)

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

add_executable(ekf_localizer src/ekf_localizer_node.cpp src/ekf_localizer.cpp)
add_dependencies(ekf_localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_localizer ${catkin_LIBRARIES})

#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS ekf_localizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/ekf_localizer/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

        
if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)
        add_rostest_gtest(ekf_localizer-test test/test_ekf_localizer.test 
                        test/src/test_ekf_localizer.cpp 
                        src/ekf_localizer.cpp)
        target_link_libraries(ekf_localizer-test ${catkin_LIBRARIES})
endif()
