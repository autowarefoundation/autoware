cmake_minimum_required(VERSION 2.8.3)
project(autoware_config_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES

  ## Runtime Manager ##
  ConfigApproximateNdtMapping.msg
  ConfigCarDpm.msg
  ConfigCarFusion.msg
  ConfigCarKf.msg
  ConfigCompareMapFilter.msg
  ConfigDecisionMaker.msg
  ConfigDistanceFilter.msg
  ConfigICP.msg
  ConfigLaneRule.msg
  ConfigLaneSelect.msg
  ConfigLaneStop.msg
  ConfigLatticeVelocitySet.msg
  ConfigNdtMapping.msg
  ConfigNdtMappingOutput.msg
  ConfigNdt.msg
  ConfigPedestrianDpm.msg
  ConfigPedestrianFusion.msg
  ConfigPedestrianKf.msg
  ConfigPlannerSelector.msg
  ConfigPoints2Polygon.msg
  ConfigRandomFilter.msg
  ConfigRayGroundFilter.msg
  ConfigRcnn.msg
  ConfigRingFilter.msg
  ConfigRingGroundFilter.msg
  ConfigSsd.msg
  ConfigTwistFilter.msg
  ConfigVelocitySet.msg
  ConfigVoxelGridFilter.msg
  ConfigWaypointFollower.msg
  ConfigWaypointLoader.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
)
