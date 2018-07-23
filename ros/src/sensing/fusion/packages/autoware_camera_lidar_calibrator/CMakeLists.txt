cmake_minimum_required(VERSION 2.8.12)
project(autoware_camera_lidar_calibrator)

include(FindPkgConfig)

find_package(catkin REQUIRED COMPONENTS
    autoware_build_flags
    cv_bridge
    image_transport
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    autoware_msgs
    geometry_msgs
    tf
)
find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL)

catkin_package(CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    pcl_conversions
    pcl_ros
    autoware_msgs
    geometry_msgs
    tf
)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#camera lidar node
add_executable(autoware_camera_lidar_calibration_node
        src/autoware_camera_lidar_calibration/camera_lidar_calibration_node.cpp
)
target_link_libraries(autoware_camera_lidar_calibration_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
add_dependencies(autoware_camera_lidar_calibration_node ${catkin_EXPORTED_TARGETS})

#camera intrinsics node
catkin_python_setup()

catkin_install_python(PROGRAMS
    nodes/cameracalibrator.py
    nodes/cameracheck.py
    scripts/tarfile_calibration.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
