cmake_minimum_required(VERSION 2.8.3)
project(points2polygon)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  runtime_manager
)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS runtime_manager
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(points2polygon nodes/points2polygon/points2polygon.cpp)
target_link_libraries(points2polygon ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(points2polygon runtime_manager_generate_messages_cpp)
