cmake_minimum_required(VERSION 2.8.3)
project(runtime_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ConfigRcnn.msg
  ConfigCarDpm.msg
  ConfigPedestrianDpm.msg
  ConfigNdt.msg
  ConfigNdtMapping.msg
  ConfigNdtMappingOutput.msg
  ConfigWaypointFollower.msg
  ConfigTwistFilter.msg
  ConfigVelocitySet.msg
  ConfigCarKf.msg
  ConfigPedestrianKf.msg
  ConfigLaneRule.msg
  ConfigLaneSelect.msg
  ConfigLaneStop.msg
  ConfigPoints2Polygon.msg
  accel_cmd.msg
  steer_cmd.msg
  brake_cmd.msg
  lamp_cmd.msg
  indicator_cmd.msg
  traffic_light.msg
  ConfigCarFusion.msg
  ConfigPedestrianFusion.msg
  adjust_xy.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES runtime_manager
  CATKIN_DEPENDS rospy std_msgs message_runtime
#  DEPENDS system_lib
)

#add_executable(runtime_manager scripts/runtime_manager.py)
#add_dependencies(runtime_manager runtime_manager_generate_messages_cpp)
