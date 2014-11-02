cmake_minimum_required(VERSION 2.8.3)
project(lane)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane
   CATKIN_DEPENDS deal_image
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ../../include
)

## Declare a cpp executable
add_executable(lane_ocv
  src/operate_ShareMem.cpp
  src/line_detection.cpp
)

target_link_libraries(lane_ocv
  ${catkin_LIBRARIES}
  opencv_highgui opencv_core opencv_imgproc m deal_image pthread
)
