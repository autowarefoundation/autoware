cmake_minimum_required(VERSION 2.8.3)
project(kf)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  autoware_msgs
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES kf
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS autoware_msgs
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(${catkin_INCLUDE_DIRS}
 include)

## Declare a cpp library
add_library(kf
   src/kf.cpp
)

target_link_libraries(kf
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(kf ${catkin_EXPORTED_TARGETS})
endif()
