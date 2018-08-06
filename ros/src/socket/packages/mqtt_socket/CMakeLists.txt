cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        roslib
        std_msgs
        autoware_msgs
        tablet_socket_msgs
        )

catkin_package(
        CATKIN_DEPENDS roscpp roslib std_msgs autoware_msgs tablet_socket_msgs
)

if (EXISTS /usr/include/mosquitto.h)
    set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
    )

    add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
    target_link_libraries(mqtt_sender -lmosquitto -lyaml-cpp ${catkin_LIBRARIES})
    add_dependencies(mqtt_sender
            ${catkin_EXPORTED_TARGETS}
            )

    add_executable(mqtt_receiver nodes/mqtt_receiver/mqtt_receiver.cpp)
    target_link_libraries(mqtt_receiver -lmosquitto -lyaml-cpp ${catkin_LIBRARIES})
    add_dependencies(mqtt_receiver ${catkin_EXPORTED_TARGETS}
            )
else ()
    message("'libmosquitto-dev' is not installed. 'mqtt_sender' and 'mqtt_receiver' will not be built.")
endif ()
