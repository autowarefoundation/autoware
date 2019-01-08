cmake_minimum_required(VERSION 2.8.3)
project(fake_drivers)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  cv_bridge
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(fake_camera nodes/fake_camera/fake_camera.cpp)
target_link_libraries(fake_camera
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
