cmake_minimum_required(VERSION 2.8.3)
project(camera_lidar_2d)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
  image_transport
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(/usr/include/libxml2)

add_executable(calibration_of_camera_and_lrf src/calibration_of_camera_and_lrf.cpp)
target_link_libraries(calibration_of_camera_and_lrf 
   ${catkin_LIBRARIES} ${OpenCV_LIBS} xml2)
