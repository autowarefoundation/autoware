cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)

#cpp 11
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        cv_bridge
        message_filters
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        tf
        velodyne_pointcloud
)

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

# Ring Ground Filter
add_definitions(${PCL_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        autoware_msgs
        message_filters
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        tf
        velodyne_pointcloud

    LIBRARIES ray_ground_filter_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Space Filter
add_executable(space_filter
    nodes/space_filter/space_filter.cpp
)
target_link_libraries(space_filter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(space_filter ${catkin_EXPORTED_TARGETS})

# ring ground filter
add_executable(ring_ground_filter
    nodes/ring_ground_filter/ring_ground_filter.cpp
)
target_link_libraries(ring_ground_filter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)
add_dependencies(ring_ground_filter ${catkin_EXPORTED_TARGETS})

# Ray Ground Filter lib
add_library(ray_ground_filter_lib
  nodes/ray_ground_filter/ray_ground_filter.cpp)

target_link_libraries(ray_ground_filter_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)
add_dependencies(ray_ground_filter_lib ${catkin_EXPORTED_TARGETS})


if (OPENMP_FOUND)
    set_target_properties(ray_ground_filter_lib PROPERTIES
        COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
        LINK_FLAGS ${OpenMP_CXX_FLAGS}
    )
endif()

# ray ground filter
add_executable(ray_ground_filter
    nodes/ray_ground_filter/ray_ground_filter_main.cpp
)
target_link_libraries(ray_ground_filter
  ray_ground_filter_lib
  ${catkin_LIBRARIES}
)
add_dependencies(ray_ground_filter ${catkin_EXPORTED_TARGETS})

# Points Concat filter
add_executable(points_concat_filter
	nodes/points_concat_filter/points_concat_filter.cpp
)
target_link_libraries(points_concat_filter
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(points_concat_filter ${catkin_EXPORTED_TARGETS})

#Cloud Transformer
add_executable(cloud_transformer
        nodes/cloud_transformer/cloud_transformer_node.cpp
)
target_link_libraries(cloud_transformer
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
)
add_dependencies(cloud_transformer ${catkin_EXPORTED_TARGETS})


# install targets
install(TARGETS space_filter ray_ground_filter_lib ring_ground_filter ray_ground_filter points_concat_filter cloud_transformer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


### Unit Tests ###
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_points_preprocessor test/src/test_points_preprocessor.cpp)
  target_include_directories(test_points_preprocessor PRIVATE
    nodes/ray_ground_filter/include
    test/include)
  target_link_libraries(test_points_preprocessor
    ray_ground_filter_lib
    ${catkin_LIBRARIES})
endif()

#link_directories(${PCL_LIBRARY_DIRS})
#target_include_directories(ring_ground_filter PRIVATE
#    ${PCL_INCLUDE_DIRS}
#)

#target_include_directories(ray_ground_filter_lib PRIVATE
#    ${OpenCV_INCLUDE_DIRS}
#    ${PCL_INCLUDE_DIRS}
#    nodes/ray_ground_filter/include
#)

#target_include_directories(ray_ground_filter PRIVATE
#  nodes/ray_ground_filter/include)

#target_include_directories(cloud_transformer PRIVATE
#        ${PCL_INCLUDE_DIRS}
#)
