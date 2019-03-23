cmake_minimum_required(VERSION 2.8.3)
project(lidar_shape_estimation)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        autoware_msgs
        pcl_ros
        roscpp
        tf2
        tf2_geometry_msgs
        rostest
        rosunit
        )

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
            include
        CATKIN_DEPENDS
            autoware_build_flags
            autoware_msgs
            pcl_ros
            roscpp
            tf2
            tf2_geometry_msgs
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        )

add_executable(lidar_shape_estimation
        src/main.cpp
        src/node.cpp
        src/shape_estimator.cpp
        src/model/bounding_box.cpp
        src/model/convex_hull.cpp
        src/model/cylinder.cpp
        )

add_dependencies(lidar_shape_estimation
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(lidar_shape_estimation
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        )

install(TARGETS lidar_shape_estimation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
        )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_shape_estimation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(test-lidar_shape_estimation 
    test/test_lidar_shape_estimation.test 
    test/src/test_lidar_shape_estimation.cpp
    src/shape_estimator.cpp
    src/model/bounding_box.cpp
    src/model/convex_hull.cpp
    src/model/cylinder.cpp)
    target_link_libraries(test-lidar_shape_estimation 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES})
endif ()
## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
