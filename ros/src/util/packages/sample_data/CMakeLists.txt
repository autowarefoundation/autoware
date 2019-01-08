cmake_minimum_required(VERSION 2.8.3)
project(sample_data)

include(FindPkgConfig)
find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        gnss
        std_msgs
        visualization_msgs
        )
pkg_check_modules(PCL_IO REQUIRED pcl_io-1.7)

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        DEPENDS roscpp
        gnss
        std_msgs
        visualization_msgs
)

###########
## Build ##
###########

add_executable(sample_vector_map nodes/sample_vector_map/sample_vector_map.cpp)
target_link_libraries(sample_vector_map ${catkin_LIBRARIES})

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_IO_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

add_executable(sample_points_map nodes/sample_points_map/sample_points_map.cpp)
target_link_libraries(sample_points_map ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(sample_points_map_update nodes/sample_points_map_update/sample_points_map_update.cpp)
target_link_libraries(sample_points_map_update ${catkin_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable(sample_trajectory nodes/sample_trajectory/sample_trajectory.cpp)
target_link_libraries(sample_trajectory ${catkin_LIBRARIES})

add_executable(sample_mobility nodes/sample_mobility/sample_mobility.cpp)
target_link_libraries(sample_mobility ${catkin_LIBRARIES})
