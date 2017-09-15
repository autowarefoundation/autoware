cmake_minimum_required(VERSION 2.8.3)
project(lane_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  autoware_msgs
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_detector
   CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
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
add_dependencies(lane_detector 
  ${catkin_EXPORTED_TARGETS}
)
