# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(jsk_gui_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs geometry_msgs)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
Action.msg
MagneticField.msg
Touch.msg
AndroidSensor.msg
Gravity.msg
MultiTouch.msg
TouchEvent.msg
DeviceSensor.msg
Tablet.msg
VoiceMessage.msg
)

add_service_files(FILES
Query.srv
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES
)
