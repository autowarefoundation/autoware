cmake_minimum_required(VERSION 2.8.12)
project(pixel_cloud_fusion)


find_package(autoware_build_flags REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP)

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

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

#fusion Library
add_library(pixel_cloud_fusion_lib SHARED
        src/pixel_cloud_fusion.cpp
        include/pixel_cloud_fusion/pixel_cloud_fusion.h
        )

if (OPENMP_FOUND)
    set_target_properties(pixel_cloud_fusion_lib PROPERTIES
            COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
            LINK_FLAGS ${OpenMP_CXX_FLAGS}
            )
endif ()

target_include_directories(pixel_cloud_fusion_lib PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(pixel_cloud_fusion_lib
        ${OpenCV_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_dependencies(pixel_cloud_fusion_lib
        ${catkin_EXPORTED_TARGETS}
        )

#Fusion node
add_executable(pixel_cloud_fusion
        src/pixel_cloud_fusion_node.cpp
        )
target_include_directories(pixel_cloud_fusion PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(pixel_cloud_fusion
        pixel_cloud_fusion_lib)

install(TARGETS pixel_cloud_fusion pixel_cloud_fusion_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
