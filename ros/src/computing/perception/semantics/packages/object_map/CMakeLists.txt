cmake_minimum_required(VERSION 2.8.12)

project(object_map)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        pcl_ros
        pcl_conversions
        tf
        sensor_msgs
        nav_msgs
        grid_map_ros
        grid_map_cv
        grid_map_msgs
        autoware_msgs
        vector_map
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

catkin_package(
        CATKIN_DEPENDS roscpp pcl_ros pcl_conversions tf sensor_msgs nav_msgs autoware_msgs vector_map
)

set(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS}
        include
)
### object_map_utils ###
add_library(object_map_utils_lib
        include/object_map_utils.cpp
        include/object_map_utils.hpp
        )
target_link_libraries(object_map_utils_lib
        ${catkin_LIBRARIES}
        )

### laserscan2costmap ###
add_executable(laserscan2costmap
        nodes/laserscan2costmap/laserscan2costmap.cpp
        )

target_link_libraries(laserscan2costmap
        ${catkin_LIBRARIES}
        )

### points2costmap ###
add_executable(points2costmap
        nodes/points2costmap/points2costmap.cpp
        )
target_link_libraries(points2costmap
        ${catkin_LIBRARIES}
        )

### potential_field ###
add_executable(potential_field
        nodes/potential_field/potential_field.cpp
        )
add_dependencies(potential_field
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(potential_field
        ${catkin_LIBRARIES}
        )

### grid_map_filter ###
add_library(grid_map_filter_lib
        nodes/grid_map_filter/grid_map_filter.h
        include/object_map_utils.hpp
        nodes/grid_map_filter/grid_map_filter.cpp
        )
target_link_libraries(grid_map_filter_lib
        ${catkin_LIBRARIES}
        object_map_utils_lib
        )
add_executable(grid_map_filter
        nodes/grid_map_filter/grid_map_filter_node.cpp
        nodes/grid_map_filter/grid_map_filter.h
        )
target_link_libraries(grid_map_filter
        ${catkin_LIBRARIES}
        object_map_utils_lib
        grid_map_filter_lib
        )

### wayarea2grid ###
add_library(wayarea2grid_lib
        nodes/wayarea2grid/wayarea2grid.h
        include/object_map_utils.hpp
        nodes/wayarea2grid/wayarea2grid.cpp
        )
target_link_libraries(wayarea2grid_lib
        ${catkin_LIBRARIES}
        object_map_utils_lib
        )

add_executable(wayarea2grid
        nodes/wayarea2grid/wayarea2grid.h
        nodes/wayarea2grid/wayarea2grid_node.cpp
        )

target_link_libraries(wayarea2grid
        ${catkin_LIBRARIES}
        wayarea2grid_lib
        )

install(TARGETS wayarea2grid grid_map_filter potential_field points2costmap laserscan2costmap
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )
