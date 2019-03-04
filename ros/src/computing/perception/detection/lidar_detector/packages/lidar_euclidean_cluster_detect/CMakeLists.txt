cmake_minimum_required(VERSION 3.5)
project(lidar_euclidean_cluster_detect)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf REQUIRED)
find_package(vector_map_server REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(OpenMP)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

include_directories(
        include
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#Euclidean Cluster
add_executable(lidar_euclidean_cluster_detect
        nodes/lidar_euclidean_cluster_detect/lidar_euclidean_cluster_detect.cpp
        nodes/lidar_euclidean_cluster_detect/cluster.cpp)

find_package(CUDA)
if (${CUDA_FOUND})
    INCLUDE(FindCUDA)
    message("-- USING ACCELERATED CLUSTERING --")
    message("Version: " ${CUDA_VERSION})
    message("Library: " ${CUDA_CUDA_LIBRARY})
    message("Runtime: " ${CUDA_CUDART_LIBRARY})
    set(ROS_VERSION $ENV{ROS_DISTRO})
    target_compile_definitions(lidar_euclidean_cluster_detect PRIVATE
            GPU_CLUSTERING=1
            )

    cuda_add_library(gpu_euclidean_clustering
            include/gpu_euclidean_clustering.h
            nodes/lidar_euclidean_cluster_detect/gpu_euclidean_clustering.cu
            )

    target_link_libraries(lidar_euclidean_cluster_detect
            ${OpenCV_LIBRARIES}
            ${PCL_LIBRARIES}
            ${YAML_CPP_LIBRARIES}
            gpu_euclidean_clustering)

else ()
    target_link_libraries(lidar_euclidean_cluster_detect
            ${OpenCV_LIBRARIES}
            ${PCL_LIBRARIES}
            ${YAML_CPP_LIBRARIES})

endif ()

ament_target_dependencies(lidar_euclidean_cluster_detect
        pcl_ros
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        autoware_msgs
        tf
        vector_map_server
        grid_map_ros
        grid_map_cv
        grid_map_msgs
        )

if (OPENMP_FOUND)
    set_target_properties(lidar_euclidean_cluster_detect PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

install(TARGETS
        lidar_euclidean_cluster_detect
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(roscpp)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(std_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(tf)
ament_export_dependencies(vector_map_server)
ament_export_dependencies(grid_map_ros)
ament_export_dependencies(grid_map_cv)
ament_export_dependencies(grid_map_msgs)

ament_export_include_directories(include)

ament_export_libraries(lidar_euclidean_cluster_detect)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()