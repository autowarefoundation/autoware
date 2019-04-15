cmake_minimum_required(VERSION 2.8.3)
project(autoware_map_msgs)

find_package(catkin REQUIRED COMPONENTS
  autoware_build_flags
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Point.msg
  Waypoint.msg
  Lane.msg
  SignalLight.msg
  Signal.msg
  LaneSignalLightRelation.msg
  WaypointSignalRelation.msg
  Area.msg
  WaypointRelation.msg
  LaneRelation.msg
  WaypointLaneRelation.msg
  LaneAttributeRelation.msg
  LaneChangeRelation.msg
  OppositeLaneRelation.msg
  Wayarea.msg
  Route.msg

  PointArray.msg
  WaypointArray.msg
  LaneArray.msg
  SignalLightArray.msg
  SignalArray.msg
  LaneSignalLightRelationArray.msg
  WaypointSignalRelationArray.msg
  AreaArray.msg
  WaypointRelationArray.msg
  LaneRelationArray.msg
  WaypointLaneRelationArray.msg
  LaneAttributeRelationArray.msg
  LaneChangeRelationArray.msg
  OppositeLaneRelationArray.msg
  WayareaArray.msg
  RouteArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
