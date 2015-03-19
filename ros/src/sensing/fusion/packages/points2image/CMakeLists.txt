cmake_minimum_required(VERSION 2.8.3)
project(points2image)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosinterface
  std_msgs
  sensor_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

 add_message_files(
   FILES
   PointsImage.msg
   CameraExtrinsic.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(points2image nodes/points2image/points2image.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(points2image ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_dependencies(points2image points2image_generate_messages_cpp)
