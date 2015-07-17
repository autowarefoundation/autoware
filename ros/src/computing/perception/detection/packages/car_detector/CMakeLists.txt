cmake_minimum_required(VERSION 2.8.3)
project(car_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  dpm
  fusion
)
find_package(OpenCV REQUIRED)

EXECUTE_PROCESS(
  COMMAND uname -m
  OUTPUT_VARIABLE ARCHITECTURE
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

## Generate added messages and services with any dependencies listed here
add_message_files(
  FILES
  FusedObjects.msg
)

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
  CATKIN_DEPENDS dpm scan2image fusion
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)

add_executable(car_dpm
  nodes/car_dpm/car_dpm.cpp
)

set_target_properties(car_dpm PROPERTIES
  COMPILE_FLAGS "-DMODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/data/"
)

target_link_libraries(car_dpm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dpm
)

## Declare a cpp executable
add_executable(car_fusion
  nodes/car_fusion/car_fusion.cpp
)

target_link_libraries(car_fusion
  ${catkin_LIBRARIES} ${OpenCV_LIBS} fusion
)
add_dependencies(car_fusion car_detector_generate_messages_cpp)

add_executable(car_kf
  nodes/car_kf/car_kf.cpp
)
target_link_libraries(car_kf
  ${catkin_LIBRARIES}
  kf
)
add_dependencies(car_kf dpm_generate_messages_cpp kf_generate_messages_cpp)

add_executable(car_locate
  nodes/car_locate/car_locate.cpp
)
target_link_libraries(car_locate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(car_locate
  PROPERTIES COMPILE_FLAGS
  "-DCAMERA_YAML=${CAMERA_YAML}")
