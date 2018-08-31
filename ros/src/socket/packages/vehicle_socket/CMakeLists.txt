cmake_minimum_required(VERSION 2.8.3)
project(vehicle_socket)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        autoware_msgs
        tablet_socket_msgs
        )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
        CATKIN_DEPENDS roscpp std_msgs autoware_msgs tablet_socket_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(vehicle_receiver nodes/vehicle_receiver/vehicle_receiver.cpp)
target_link_libraries(vehicle_receiver ${catkin_LIBRARIES})
add_dependencies(vehicle_receiver
        ${catkin_EXPORTED_TARGETS}
        )

add_executable(vehicle_sender nodes/vehicle_sender/vehicle_sender.cpp)
target_link_libraries(vehicle_sender ${catkin_LIBRARIES})
add_dependencies(vehicle_sender
        ${catkin_EXPORTED_TARGETS}
        )
