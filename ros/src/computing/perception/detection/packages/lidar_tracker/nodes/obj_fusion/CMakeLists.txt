cmake_minimum_required(VERSION 2.8.3)
project(obj_fusion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        autoware_msgs
        tf
        jsk_recognition_msgs
        jsk_rviz_plugins
        rosinterface
        cv_bridge
        vector_map
        vector_map_server
        op_ros_helpers
        vector_map
        grid_map_ros
        grid_map_cv
        grid_map_msgs
        )

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenMP)

execute_process(
        COMMAND rosversion -d
        OUTPUT_VARIABLE ROS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(OpenCV REQUIRED)

execute_process(
        COMMAND pkg-config --variable=host_bins Qt5Core
        OUTPUT_VARIABLE Qt5BIN
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
catkin_package(
        CATKIN_DEPENDS std_msgs geometry_msgs pcl_ros vector_map_server autoware_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})


#Object Fusion
add_executable(obj_fusion nodes/obj_fusion/obj_fusion.cpp)
target_link_libraries(obj_fusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} m)
add_dependencies(obj_fusion ${catkin_EXPORTED_TARGETS})
