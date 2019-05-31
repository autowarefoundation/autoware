cmake_minimum_required(VERSION 2.8.3)
project(lidar_euclidean_cluster_detect)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        geometry_msgs
        std_msgs
        sensor_msgs
        tf
        vector_map_server
        grid_map_ros
        grid_map_cv
        grid_map_msgs
        autoware_msgs
        jsk_rviz_plugins
        )

find_package(OpenMP)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS
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
        jsk_rviz_plugins
        INCLUDE_DIRS include
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${autoware_msgs_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})

#Euclidean Cluster
add_executable(lidar_euclidean_cluster_detect
        nodes/lidar_euclidean_cluster_detect/lidar_euclidean_cluster_detect.cpp
        nodes/lidar_euclidean_cluster_detect/cluster.cpp)

find_package(CUDA)
find_package(Eigen3 QUIET)

AW_CHECK_CUDA()

if (USE_CUDA)
    message("-- USING ACCELERATED CLUSTERING --")
    message("Version: " ${CUDA_VERSION})
    message("Library: " ${CUDA_CUDA_LIBRARY})
    message("Runtime: " ${CUDA_CUDART_LIBRARY})
    target_compile_definitions(lidar_euclidean_cluster_detect PRIVATE
            GPU_CLUSTERING=1
            )

    cuda_add_library(gpu_euclidean_clustering
            include/gpu_euclidean_clustering.h
            nodes/lidar_euclidean_cluster_detect/gpu_euclidean_clustering.cu
            )

    target_link_libraries(lidar_euclidean_cluster_detect
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${YAML_CPP_LIBRARIES}
            gpu_euclidean_clustering)

    install(TARGETS
            gpu_euclidean_clustering
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            )

else ()
    target_link_libraries(lidar_euclidean_cluster_detect
            ${OpenCV_LIBRARIES}
            ${catkin_LIBRARIES}
            ${YAML_CPP_LIBRARIES})

endif ()

add_dependencies(lidar_euclidean_cluster_detect
        ${catkin_EXPORTED_TARGETS}
        )

if (OPENMP_FOUND)
    set_target_properties(lidar_euclidean_cluster_detect PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS
        lidar_euclidean_cluster_detect
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
