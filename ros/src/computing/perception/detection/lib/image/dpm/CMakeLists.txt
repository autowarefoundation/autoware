cmake_minimum_required(VERSION 2.8.3)
project(dpm)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dpm
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-std=c++0x -O2 -g -Wall -Wno-unused-result -DROS ${CMAKE_CXX_FLAGS}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(/usr/include/opencv2/legacy)
include_directories(src)

file(GLOB DPM_OCV_CXX_SOURCES "src/*.cpp")

add_library(dpm
  ${DPM_OCV_CXX_SOURCES}
)

set_target_properties(dpm PROPERTIES
  COMPILE_FLAGS "-DOPENCV_MODEL_DIR=${CMAKE_CURRENT_SOURCE_DIR}/models_VOC2007/"
)

target_link_libraries(dpm
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_dependencies(dpm dpm_generate_messages_cpp)
