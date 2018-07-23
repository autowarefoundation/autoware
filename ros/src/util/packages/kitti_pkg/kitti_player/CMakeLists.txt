cmake_minimum_required(VERSION 2.8.3)
project(kitti_player)

SET(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        tf
        tf2
        std_msgs
        pcl_ros
        geometry_msgs
        stereo_msgs
        cv_bridge
        image_transport
        )

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5Core REQUIRED)

generate_dynamic_reconfigure_options(cfg/kitti_player.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_LIBS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kitti_player src/kitti_player.cpp)

target_link_libraries(kitti_player
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS})

add_dependencies(kitti_player ${catkin_EXPORTED_TARGETS})
