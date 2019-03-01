cmake_minimum_required(VERSION 3.5)
project(pixel_cloud_fusion)


find_package(autoware_build_flags REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(roscpp REQUIRED)
find_package(tf REQUIRED)
find_package(ament_cmake REQUIRED)

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
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(pixel_cloud_fusion_lib
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

ament_target_dependencies(pixel_cloud_fusion_lib
        cv_bridge
        image_transport
        pcl_conversions
        pcl_ros
        roscpp
        tf
        )

#Fusion node
add_executable(pixel_cloud_fusion
        src/pixel_cloud_fusion_node.cpp
        )

target_include_directories(pixel_cloud_fusion PRIVATE
        ${OpenCV_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
        )

target_link_libraries(pixel_cloud_fusion
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        pixel_cloud_fusion_lib
        )

ament_target_dependencies(pixel_cloud_fusion
        cv_bridge
        roscpp
        )

install(TARGETS pixel_cloud_fusion
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        PATTERN ".svn" EXCLUDE)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(image_transport)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(roscpp)
ament_export_dependencies(tf)

ament_export_include_directories(include)

ament_export_libraries(pixel_cloud_fusion_lib)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()