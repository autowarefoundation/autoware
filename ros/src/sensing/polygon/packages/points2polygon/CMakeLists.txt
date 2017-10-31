cmake_minimum_required(VERSION 2.8.3)
project(points2polygon)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if ("${ROS_VERSION}" MATCHES "(indigo|jade)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  autoware_msgs
)

find_package(PCL 1.2 REQUIRED)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS autoware_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(points2polygon nodes/points2polygon/points2polygon.cpp)
target_link_libraries(points2polygon ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Qt5Core_LIBRARIES})
add_dependencies(points2polygon ${catkin_EXPORTED_TARGETS})
endif()
