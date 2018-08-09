cmake_minimum_required(VERSION 3.0.2)
project(vision_beyond_track)

SET(CMAKE_BUILD_TYPE "Release" CACHE STRING "Release type")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set (CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -march=native")
set (CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -g")
set (CMAKE_CXX_FLAHS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )

catkin_package(CATKIN_DEPENDS
        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        std_msgs
        autoware_msgs
        )

find_package(Boost COMPONENTS program_options system filesystem thread REQUIRED)

include_directories(
  include
  lib
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(
  third_lib STATIC
  lib/hungarian.cpp
  lib/clipper.cpp
)

add_executable(exec
  src/vision_beyond_track_node.cpp
  #src/main_script_test.cpp
  src/vision_beyond_track.cpp
  #src/vision_beyond_track_node.cpp
  )

target_link_libraries(exec
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  third_lib
)

add_dependencies(exec
            ${catkin_EXPORTED_TARGETS}
            )
