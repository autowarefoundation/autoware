cmake_minimum_required(VERSION 2.8.3)
project(vector_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_message_files(
  FILES
  Point.msg
  PointArray.msg
  Vector.msg
  VectorArray.msg
  Line.msg
  LineArray.msg
  Area.msg
  AreaArray.msg
  Pole.msg
  PoleArray.msg
  Box.msg
  BoxArray.msg
  DTLane.msg
  DTLaneArray.msg
  Node.msg
  NodeArray.msg
  Lane.msg
  LaneArray.msg
  WayArea.msg
  WayAreaArray.msg
  RoadEdge.msg
  RoadEdgeArray.msg
  Gutter.msg
  GutterArray.msg
  Curb.msg
  CurbArray.msg
  WhiteLine.msg
  WhiteLineArray.msg
  StopLine.msg
  StopLineArray.msg
  ZebraZone.msg
  ZebraZoneArray.msg
  CrossWalk.msg
  CrossWalkArray.msg
  RoadMark.msg
  RoadMarkArray.msg
  RoadPole.msg
  RoadPoleArray.msg
  RoadSign.msg
  RoadSignArray.msg
  Signal.msg
  SignalArray.msg
  StreetLight.msg
  StreetLightArray.msg
  UtilityPole.msg
  UtilityPoleArray.msg
  GuardRail.msg
  GuardRailArray.msg
  SideWalk.msg
  SideWalkArray.msg
  DriveOnPortion.msg
  DriveOnPortionArray.msg
  CrossRoad.msg
  CrossRoadArray.msg
  SideStrip.msg
  SideStripArray.msg
  CurveMirror.msg
  CurveMirrorArray.msg
  Wall.msg
  WallArray.msg
  Fence.msg
  FenceArray.msg
  RailCrossing.msg
  RailCrossingArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
