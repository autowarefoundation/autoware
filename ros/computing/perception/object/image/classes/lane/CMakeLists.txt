cmake_minimum_required(VERSION 2.8.3)
project(lane)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)

IF(USE_POSIX_SHARED_MEMORY)
ELSE()
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)
ENDIF()

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

SET(CMAKE_C_FLAGS "-O2 -Wall ${CMAKE_C_FLAGS}")

IF(USE_POSIX_SHARED_MEMORY)
add_executable(lane_ocv
  src/operate_ShareMem.cpp
  src/line_detection.cpp
)
set_target_properties(lane_ocv PROPERTIES COMPILE_FLAGS "-DUSE_POSIX_SHARED_MEMORY")
target_link_libraries(lane_ocv
  ${catkin_LIBRARIES}
  opencv_highgui opencv_core opencv_imgproc m deal_image pthread
)
ELSE()
add_executable(lane_ocv
  src/line_detection.cpp
)

target_link_libraries(lane_ocv
  ${catkin_LIBRARIES}
  opencv_highgui opencv_core opencv_imgproc m
)
ENDIF()
