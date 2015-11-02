cmake_minimum_required(VERSION 2.8.3)
project(pos_db)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  visualization_msgs
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp vehicle_socket
   DEPENDS ssh2
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(pos_db
  lib/pos_db/SendData.cpp
  lib/pos_db/util.cpp
)

add_executable(pos_downloader nodes/pos_downloader/pos_downloader.cpp)
target_link_libraries(pos_downloader ${catkin_LIBRARIES} gnss pos_db ssh2)

add_executable(pos_uploader nodes/pos_uploader/pos_uploader.cpp)
target_link_libraries(pos_uploader ${catkin_LIBRARIES} pos_db ssh2)
add_dependencies(pos_uploader cv_tracker_generate_messages_cpp)

set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(pos_uploader
  PROPERTIES COMPILE_FLAGS
  "-DCAMERA_YAML=${CAMERA_YAML}")

