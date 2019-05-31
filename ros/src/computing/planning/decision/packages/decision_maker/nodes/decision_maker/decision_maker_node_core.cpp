#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>

// lib
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/Lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker
{
void DecisionMakerNode::tryNextState(cstring_t& key)
{
  ctx_vehicle->nextState(key);
  ctx_mission->nextState(key);
  ctx_behavior->nextState(key);
  ctx_motion->nextState(key);
}

void DecisionMakerNode::update(void)
{
  update_msgs();
  if (ctx_vehicle)
    ctx_vehicle->onUpdate();
  if (ctx_mission)
    ctx_mission->onUpdate();
  if (ctx_behavior)
    ctx_behavior->onUpdate();
  if (ctx_motion)
    ctx_motion->onUpdate();
}

void DecisionMakerNode::run(void)
{
  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    update();

    loop_rate.sleep();
  }
}
}
