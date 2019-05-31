cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

find_package(PCL REQUIRED)

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(CUDA)
find_package(Eigen3 QUIET)
find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(autoware_config_msgs REQUIRED)

AW_CHECK_CUDA()

if (USE_CUDA)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

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

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        tf
        pcl_ros
        sensor_msgs
        pcl_conversions
        velodyne_pointcloud
        ndt_tku
        ndt_cpu
        autoware_health_checker
        jsk_rviz_plugins
        ${PCL_OPENMP_PACKAGES}
        autoware_msgs
        autoware_config_msgs
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        std_msgs
        velodyne_pointcloud
        autoware_msgs
        autoware_config_msgs
        ndt_tku
        ndt_cpu
        jsk_rviz_plugins
        ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
 ${autoware_config_msgs_INCLUDE_DIRS}
 ${autoware_msgs_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_matching nodes/ndt_matching/ndt_matching.cpp)
target_link_libraries(ndt_matching ${catkin_LIBRARIES})
add_dependencies(ndt_matching ${catkin_EXPORTED_TARGETS})

add_executable(ndt_mapping nodes/ndt_mapping/ndt_mapping.cpp)
target_link_libraries(ndt_mapping ${catkin_LIBRARIES})
add_dependencies(ndt_mapping ${catkin_EXPORTED_TARGETS})

if (USE_CUDA)
    target_include_directories(ndt_matching PRIVATE ${CUDA_INCLUDE_DIRS})
    target_include_directories(ndt_mapping PRIVATE ${CUDA_INCLUDE_DIRS})
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(ndt_matching PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
    set_target_properties(ndt_mapping PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))


add_executable(approximate_ndt_mapping nodes/approximate_ndt_mapping/approximate_ndt_mapping.cpp)
target_link_libraries(approximate_ndt_mapping ${catkin_LIBRARIES})
add_dependencies(approximate_ndt_mapping ${catkin_EXPORTED_TARGETS})

add_executable(queue_counter nodes/queue_counter/queue_counter.cpp)
target_link_libraries(queue_counter ${catkin_LIBRARIES})
add_dependencies(queue_counter ${catkin_EXPORTED_TARGETS})


add_executable(ndt_matching_tku nodes/ndt_matching_tku/ndt_matching_tku.cpp)
target_link_libraries(ndt_matching_tku ${catkin_LIBRARIES})
add_dependencies(ndt_matching_tku ${catkin_EXPORTED_TARGETS})

add_executable(ndt_mapping_tku nodes/ndt_mapping_tku/ndt_mapping_tku.cpp)
target_link_libraries(ndt_mapping_tku ${catkin_LIBRARIES})
add_dependencies(ndt_mapping_tku ${catkin_EXPORTED_TARGETS})

add_executable(mapping nodes/ndt_mapping_tku/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES})
add_dependencies(mapping ${catkin_EXPORTED_TARGETS})

add_library(ndt_matching_monitor_lib SHARED
        nodes/ndt_matching_monitor/ndt_matching_monitor.h
        nodes/ndt_matching_monitor/ndt_matching_monitor.cpp
        )
target_include_directories(ndt_matching_monitor_lib PRIVATE
        ${catkin_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )
add_dependencies(ndt_matching_monitor_lib
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(ndt_matching_monitor_lib ${catkin_LIBRARIES})

add_executable(ndt_matching_monitor nodes/ndt_matching_monitor/ndt_matching_monitor_node.cpp)
target_link_libraries(ndt_matching_monitor ndt_matching_monitor_lib ${catkin_LIBRARIES})

add_executable(icp_matching nodes/icp_matching/icp_matching.cpp)
target_link_libraries(icp_matching ${catkin_LIBRARIES})
add_dependencies(icp_matching ${catkin_EXPORTED_TARGETS})

install(TARGETS
            ndt_matching
            ndt_mapping
            approximate_ndt_mapping
            ndt_mapping_tku
            mapping
            ndt_matching_monitor_lib
            ndt_matching_monitor
            icp_matching
            queue_counter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
