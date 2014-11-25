cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(/usr/include/libxml2)

add_executable(camera_lidar2d_offline_calib nodes/camera_lidar2d_offline_calib/camera_lidar2d_offline_calib.cpp)

set(PARAM_YAML "${CMAKE_CURRENT_SOURCE_DIR}/param.yaml")
set(CAMERA_YAML "$ENV{HOME}/.ros/camera_info/camera.yaml")

set_target_properties(camera_lidar2d_offline_calib
  PROPERTIES COMPILE_FLAGS
  "-DPARAM_YAML=${PARAM_YAML} -DCAMERA_YAML=${CAMERA_YAML}")

target_link_libraries(camera_lidar2d_offline_calib
   ${catkin_LIBRARIES} ${OpenCV_LIBS} xml2)
