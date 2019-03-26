cmake_minimum_required(VERSION 2.8.12)
project(road_occupancy_processor)


find_package(autoware_build_flags REQUIRED)

find_package(vector_map REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    roscpp
    tf
    sensor_msgs
    nav_msgs
    grid_map_ros
    grid_map_cv
    grid_map_msgs
)

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

catkin_package(CATKIN_DEPENDS
    roscpp
    tf
    sensor_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    vector_map
)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

#Road Occupancy Processor Library
add_library(road_occupancy_processor_lib SHARED
        include/road_occupancy_processor/road_occupancy_processor.h
    src/road_occupancy_processor.cpp
)

if (OPENMP_FOUND)
    set_target_properties(road_occupancy_processor_lib PROPERTIES
        COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
        LINK_FLAGS ${OpenMP_CXX_FLAGS}
    )
endif()

target_include_directories(road_occupancy_processor_lib PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${vector_map_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include
)

target_link_libraries(road_occupancy_processor_lib
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Qt5Core_LIBRARIES}
)

add_dependencies(road_occupancy_processor_lib
    ${catkin_EXPORTED_TARGETS}
)

#Occupancy Processor Main
add_executable(road_occupancy_processor
    src/road_occupancy_processor_node.cpp
)
target_include_directories(road_occupancy_processor PRIVATE
    ${OpenCV_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${vector_map_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    include)

target_link_libraries(road_occupancy_processor
    road_occupancy_processor_lib)

install(TARGETS road_occupancy_processor road_occupancy_processor_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
