cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    pcl_ros
    pcl_conversions
    cv_bridge
    velodyne_pointcloud
)

catkin_package(CATKIN_DEPENDS
    sensor_msgs
    velodyne_pointcloud
)

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
    include
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# Space Filter
add_executable(space_filter
    nodes/space_filter/space_filter.cpp
)
target_link_libraries(space_filter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Ring Ground Filter
add_definitions(${PCL_DEFINITIONS})

add_executable(ring_ground_filter
    nodes/ring_ground_filter/ring_ground_filter.cpp
)

target_include_directories(ring_ground_filter PRIVATE
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(ring_ground_filter
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)


# Ray Ground Filter
add_library(ray_ground_filter_lib SHARED
  nodes/ray_ground_filter/ray_ground_filter.cpp)

if (OPENMP_FOUND)
    set_target_properties(ray_ground_filter_lib PROPERTIES
        COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
        LINK_FLAGS ${OpenMP_CXX_FLAGS}
    )
endif()

target_include_directories(ray_ground_filter_lib PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    nodes/ray_ground_filter/include
)

target_link_libraries(ray_ground_filter_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)

add_executable(ray_ground_filter
    nodes/ray_ground_filter/ray_ground_filter_main.cpp
)

target_include_directories(ray_ground_filter PRIVATE
  nodes/ray_ground_filter/include)

target_link_libraries(ray_ground_filter
  ray_ground_filter_lib)



# Points Concat filter
add_executable(points_concat_filter
	nodes/points_concat_filter/points_concat_filter.cpp
)

target_link_libraries(points_concat_filter
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

#Cloud Transformer
add_executable(cloud_transformer
        nodes/cloud_transformer/cloud_transformer_node.cpp
)

target_include_directories(cloud_transformer PRIVATE
        ${PCL_INCLUDE_DIRS}
)

target_link_libraries(cloud_transformer
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
)

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

