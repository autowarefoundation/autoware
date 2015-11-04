cmake_minimum_required(VERSION 2.8.3)
project(obj_db)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  visualization_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp vehicle_socket
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(obj_db
  lib/obj_db/SendData.cpp
  lib/obj_db/util.cpp
)

add_executable(obj_downloader nodes/obj_downloader/obj_downloader.cpp)
target_link_libraries(obj_downloader ${catkin_LIBRARIES} gnss obj_db)

add_executable(obj_uploader nodes/obj_uploader/obj_uploader.cpp)
target_link_libraries(obj_uploader ${catkin_LIBRARIES} obj_db)

set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")

set_target_properties(obj_uploader
  PROPERTIES COMPILE_FLAGS
  "-DCAMERA_YAML=${CAMERA_YAML}")

add_executable(can_uploader nodes/can_uploader/can_uploader.cpp)
target_link_libraries(can_uploader ${catkin_LIBRARIES} obj_db)
add_dependencies(can_uploader vehicle_socket_generate_messages_cpp)
