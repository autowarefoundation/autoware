cmake_minimum_required(VERSION 2.8.3)
project(points_preprocessor)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_config_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        tf
        autoware_health_checker
        )

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        autoware_config_msgs
        tf
        autoware_health_checker
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
        include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

# Space Filter
add_executable(space_filter
        nodes/space_filter/space_filter.cpp
        )
target_link_libraries(space_filter
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_dependencies(space_filter ${catkin_EXPORTED_TARGETS})

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

add_dependencies(ring_ground_filter ${catkin_EXPORTED_TARGETS})

# Ray Ground Filter
add_library(ray_ground_filter_lib SHARED
        nodes/ray_ground_filter/ray_ground_filter.cpp)

if (OPENMP_FOUND)
    set_target_properties(ray_ground_filter_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

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

add_dependencies(ray_ground_filter ${catkin_EXPORTED_TARGETS})


# Points Concat filter
add_executable(points_concat_filter
        nodes/points_concat_filter/points_concat_filter.cpp
        )

target_include_directories(points_concat_filter PRIVATE
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(points_concat_filter
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        )

add_dependencies(points_concat_filter ${catkin_EXPORTED_TARGETS})

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
add_dependencies(cloud_transformer ${catkin_EXPORTED_TARGETS})

#Compare Map Filter
add_executable(compare_map_filter
        nodes/compare_map_filter/compare_map_filter.cpp
        )

target_include_directories(compare_map_filter PRIVATE
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(compare_map_filter
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )
add_dependencies(compare_map_filter ${catkin_EXPORTED_TARGETS})

### Unit Tests ###
#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#    find_package(roslaunch REQUIRED)

#    add_rostest_gtest(test_points_preprocessor
#			test/test_points_preprocessor.test
#			test/src/test_points_preprocessor.cpp)
#   target_include_directories(test_points_preprocessor PRIVATE
#          nodes/ray_ground_filter/include
#            test/include)
#    target_link_libraries(test_points_preprocessor
#            ray_ground_filter_lib
#            ${catkin_LIBRARIES})
#endif ()


install(TARGETS
            cloud_transformer
            points_concat_filter
            ray_ground_filter_lib
            ray_ground_filter
            ring_ground_filter
            space_filter
            compare_map_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
