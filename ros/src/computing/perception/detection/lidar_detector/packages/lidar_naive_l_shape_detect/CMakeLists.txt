cmake_minimum_required(VERSION 3.5)
project(lidar_naive_l_shape_detect)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)
find_package(roscpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf REQUIRED)
find_package(ament_cmake REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")


link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(
  include
  ${autoware_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCE_FILES nodes/lidar_naive_l_shape_detect/lidar_naive_l_shape_detect_node.cpp nodes/lidar_naive_l_shape_detect/lidar_naive_l_shape_detect.cpp)

add_executable(lidar_naive_l_shape_detect ${SOURCE_FILES})

ament_target_dependencies(lidar_naive_l_shape_detect
        roscpp
        pcl_ros
        autoware_msgs
        tf
        )

target_link_libraries(lidar_naive_l_shape_detect ${OpenCV_LIBRARIES})

install(TARGETS
        lidar_naive_l_shape_detect
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(roscpp)
ament_export_dependencies(pcl_ros)
ament_export_dependencies(autoware_msgs)
ament_export_dependencies(tf)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()