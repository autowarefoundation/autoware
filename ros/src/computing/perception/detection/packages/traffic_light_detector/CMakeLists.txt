cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_generation
  sensor_msgs
  tf
  libvectormap
  )
find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

## Generate added messages and services with any dependencies listed here
add_message_files(FILES
  ExtractedPosition.msg
  Signals.msg
  )

generate_messages(DEPENDENCIES
  sensor_msgs
  std_msgs
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime libvectormap
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

###########
## Build ##
###########

### traffic_light_lkf ###
add_executable(traffic_light_lkf
  nodes/traffic_light_lkf/TrafficLight.cpp
  nodes/traffic_light_lkf/TrafficLightDetector.cpp
  nodes/traffic_light_lkf/Context.cpp
  )

target_link_libraries(traffic_light_lkf
  ${catkin_LIBRARIES}
)

### traffic_light_extract ###
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${libvectormap_INCLUDE_DIRS}
)

add_executable(traffic_light_extract
  nodes/traffic_light_extract/signals.cpp
)

target_link_libraries(traffic_light_extract
  ${catkin_LIBRARIES}
  ${libvectormap_LIBRARIES}
)