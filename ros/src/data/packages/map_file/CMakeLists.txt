cmake_minimum_required(VERSION 2.8.3)
project(map_file)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gnss
  std_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  tf
)

set(CMAKE_CXX_FLAGS "-std=c++0x -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  AreaClass.msg
  AreaClassArray.msg
  BoxClass.msg
  BoxClassArray.msg
  CrossRoad.msg
  CrossRoadArray.msg
  CrossWalk.msg
  CrossWalkArray.msg
  Curb.msg
  CurbArray.msg
  DTLane.msg
  DTLaneArray.msg
  GuardRail.msg
  GuardRailArray.msg
  Gutter.msg
  GutterArray.msg
  Lane.msg
  LaneArray.msg
  LineClass.msg
  LineClassArray.msg
  Node.msg
  NodeArray.msg
  PointClass.msg
  PointClassArray.msg
  Pole.msg
  PoleArray.msg
  PoleClass.msg
  PoleClassArray.msg
  RoadEdge.msg
  RoadEdgeArray.msg
  RoadMark.msg
  RoadMarkArray.msg
  RoadSign.msg
  RoadSignArray.msg
  SideWalk.msg
  SideWalkArray.msg
  Signal.msg
  SignalArray.msg
  StopLine.msg
  StopLineArray.msg
  StreetLight.msg
  StreetLightArray.msg
  UtilityPole.msg
  UtilityPoleArray.msg
  VectorClass.msg
  VectorClassArray.msg
  WhiteLine.msg
  WhiteLineArray.msg
  ZebraZone.msg
  ZebraZoneArray.msg
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
   DEPENDS gnss curl
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/pcl-1.7
  /usr/include/eigen3)

add_library(map_file
  lib/map_db/GetFile.cpp
)
add_executable(points_map_loader nodes/points_map_loader/points_map_loader.cpp)
target_link_libraries(points_map_loader ${catkin_LIBRARIES} pcl_io map_file curl)

add_executable(vector_map_loader
  nodes/vector_map_loader/vector_map_loader.cpp
  nodes/vector_map_loader/pack_into.cpp)
target_link_libraries(vector_map_loader ${catkin_LIBRARIES} map_file curl)
add_dependencies(vector_map_loader map_file_generate_messages_cpp)
