cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)

if(EXISTS /usr/include/mosquitto.h)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    autoware_msgs
  )

  set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

  catkin_package(
    CATKIN_DEPENDS roscpp roslib std_msgs autoware_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
  target_link_libraries(mqtt_sender -lmosquitto -lyaml-cpp ${catkin_LIBRARIES})
  add_dependencies(mqtt_sender
    autoware_msgs_generate_messages_cpp
  )

  add_executable(mqtt_receiver nodes/mqtt_receiver/mqtt_receiver.cpp)
  target_link_libraries(mqtt_receiver -lmosquitto -lyaml-cpp ${catkin_LIBRARIES})
  add_dependencies(mqtt_receiver
    autoware_msgs_generate_messages_cpp
  )
else()
  message("'libmosquitto-dev' is not installed. 'mqtt_sender' and 'mqtt_receiver' will not be built.")
endif()
