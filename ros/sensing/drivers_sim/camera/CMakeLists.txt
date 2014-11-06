cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  cv_bridge
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES camera
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

add_executable(image_viewer src/image_viewer.cpp)
target_link_libraries(image_viewer
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)

add_executable(camera_sim src/camera_sim.cpp)
target_link_libraries(camera_sim
   ${catkin_LIBRARIES}
   opencv_highgui opencv_core opencv_imgproc
)
