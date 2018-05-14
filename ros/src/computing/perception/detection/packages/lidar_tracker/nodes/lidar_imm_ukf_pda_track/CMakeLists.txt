cmake_minimum_required(VERSION 2.8.3)
project(lidar_imm_ukf_pda_track)

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
        )

set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

execute_process(
        COMMAND rosversion -d
        OUTPUT_VARIABLE ROS_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS std_msgs geometry_msgs pcl_ros autoware_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

#imm_ukf_pda
add_executable(imm_ukf_pda
                nodes/imm_ukf_pda/src/imm_ukf_pda_main.cpp
                nodes/imm_ukf_pda/src/imm_ukf_pda.cpp
                nodes/imm_ukf_pda/src/ukf.cpp)
target_include_directories(imm_ukf_pda PRIVATE nodes/imm_ukf_pda/include)
target_link_libraries(imm_ukf_pda ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(imm_ukf_pda ${catkin_EXPORTED_TARGETS})

#visualize_detected_objects
add_executable(visualize_detected_objects
                nodes/visualize_detected_objects/src/visualize_detected_objects_main.cpp
                nodes/visualize_detected_objects/src/visualize_detected_objects.cpp)
target_include_directories(visualize_detected_objects PRIVATE nodes/visualize_detected_objects/include)
target_link_libraries(visualize_detected_objects ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(visualize_detected_objects ${catkin_EXPORTED_TARGETS})
