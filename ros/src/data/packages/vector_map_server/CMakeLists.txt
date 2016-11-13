cmake_minimum_required(VERSION 2.8.3)
project(vector_map_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  waypoint_follower
  visualization_msgs
  vector_map_msgs
  vector_map
  message_generation
)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall ${CMAKE_CXX_FLAGS}")

add_service_files(
  FILES
  GetDTLane.srv
  GetNode.srv
  GetLane.srv
  GetWayArea.srv
  GetRoadEdge.srv
  GetGutter.srv
  GetCurb.srv
  GetWhiteLine.srv
  GetStopLine.srv
  GetZebraZone.srv
  GetCrossWalk.srv
  GetRoadMark.srv
  GetRoadPole.srv
  GetRoadSign.srv
  GetSignal.srv
  GetStreetLight.srv
  GetUtilityPole.srv
  GetGuardRail.srv
  GetSideWalk.srv
  GetDriveOnPortion.srv
  GetCrossRoad.srv
  GetSideStrip.srv
  GetCurveMirror.srv
  GetWall.srv
  GetFence.srv
  GetRailCrossing.srv
  PositionState.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  waypoint_follower
  vector_map_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp geometry_msgs waypoint_follower visualization_msgs vector_map_msgs vector_map
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(vector_map_server nodes/vector_map_server/vector_map_server.cpp)
target_link_libraries(vector_map_server ${catkin_LIBRARIES} vector_map)
add_dependencies(vector_map_server
  geometry_msgs_generate_messages_cpp
  waypoint_follower_generate_messages_cpp
  visualization_msgs_generate_messages_cpp
  vector_map_server_generate_messages_cpp
)

add_executable(vector_map_client nodes/vector_map_client/vector_map_client.cpp)
target_link_libraries(vector_map_client ${catkin_LIBRARIES} vector_map)
add_dependencies(vector_map_client
  geometry_msgs_generate_messages_cpp
  waypoint_follower_generate_messages_cpp
  visualization_msgs_generate_messages_cpp
  vector_map_server_generate_messages_cpp
)
