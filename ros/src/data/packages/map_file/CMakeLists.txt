cmake_minimum_required(VERSION 2.8.3)
project(map_file)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  std_msgs
  visualization_msgs
  geometry_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  AreaData.msg
  BoxData.msg
  CrossRoad.msg
  CrossWalk.msg
  Curb.msg
  DTLaneData.msg
  GuardRail.msg
  LaneData.msg
  LineData.msg
  NodeData.msg
  Object.msg
  PointData.msg
  Pole.msg
  PoleData.msg
  PowerPole.msg
  Primitive.msg
  Road.msg
  RoadEdge.msg
  RoadMark.msg
  RoadSign.msg
  SideWalk.msg
  StopLine.msg
  StreetGutter.msg
  StreetLight.msg
  TrafficLight.msg
  VectorData.msg
  VectorMap.msg
  WhiteLine.msg
  ZebraZone.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fake_drivers
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
   DEPENDS gnss
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.7
  /usr/include/eigen3)

add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} pcl_io)

add_executable(vector_map_loader nodes/vector_map_loader/vector_map_loader.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES})
add_dependencies(vector_map_loader map_file_generate_messages_cpp)
