cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ImageLaneObjects.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_detector
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  nodes/lane_detector
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detector
  nodes/lane_detector/lane_detector.cpp
)

target_link_libraries(lane_detector
  ${catkin_LIBRARIES}
  opencv_highgui opencv_core opencv_imgproc m
)
add_dependencies(lane_detector lane_detector_generate_messages_cpp)
