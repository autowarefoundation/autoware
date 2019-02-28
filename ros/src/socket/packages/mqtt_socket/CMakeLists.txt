cmake_minimum_required(VERSION 2.8.3)
project(mqtt_socket)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        std_msgs
        autoware_can_msgs
        autoware_msgs
        )

catkin_package(CATKIN_DEPENDS
        roscpp
        roslib
        std_msgs
        autoware_msgs
        autoware_can_msgs
        tablet_socket_msgs
        )

if (EXISTS /usr/include/mosquitto.h)
    set(CMAKE_CXX_FLAGS "-O2 -Wall -fpermissive ${CMAKE_CXX_FLAGS}")

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${tablet_socket_msgs_INCLUDE_DIRS}
            ${autoware_msgs_INCLUDE_DIRS}
    )

    add_executable(mqtt_sender nodes/mqtt_sender/mqtt_sender.cpp)
    target_link_libraries(mqtt_sender -lmosquitto -lyaml-cpp ${catkin_LIBRARIES})
    add_dependencies(mqtt_sender
            ${catkin_EXPORTED_TARGETS}
            )

    add_executable(mqtt_receiver nodes/mqtt_receiver/mqtt_receiver.cpp)
    target_link_libraries(mqtt_receiver -lmosquitto -lyaml-cpp ${catkin_LIBRARIES}) #TODO: FIX THIS
    add_dependencies(mqtt_receiver ${catkin_EXPORTED_TARGETS})

    install(TARGETS mqtt_receiver mqtt_sender
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
else ()
    message("'libmosquitto-dev' is not installed. 'mqtt_sender' and 'mqtt_receiver' will not be built.")
endif ()
