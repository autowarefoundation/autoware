cmake_minimum_required(VERSION 2.8.3)
project(scan2image)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  calibration_camera_lidar
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ScanImage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs calibration_camera_lidar
  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan2image nodes/scan2image/scan2image.cpp)

set(CAMERA_YAML "$ENV{HOME}/.ros/autoware/camera_lidar_2d.yaml")
set(MANUAL_YAML "${CMAKE_CURRENT_SOURCE_DIR}/manual.yaml")

set_target_properties(scan2image
  PROPERTIES COMPILE_FLAGS
  "-DCAMERA_YAML=${CAMERA_YAML} -DMANUAL_YAML=${MANUAL_YAML}")

target_link_libraries(scan2image ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(scan2image
  scan2image_generate_messages_cpp
  calibration_camera_lidar_generate_messages_cpp
)
