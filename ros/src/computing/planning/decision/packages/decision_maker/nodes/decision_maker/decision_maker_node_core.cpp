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

#include <autoware_msgs/lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker
{
void DecisionMakerNode::update(void)
{
  update_msgs();
  if (ctx)
    ctx->update();
}

void DecisionMakerNode::run(void)
{
  ros::Rate loop_rate(1);

  // for subscribe callback function
  ros::AsyncSpinner spinner(3);
  spinner.start();
  while (ros::ok())
  {
    ros::Time begin = ros::Time::now();
    update();
    if (enableDisplayMarker)
      displayMarker();

#ifdef DEBUG_PRINT
    // debug status
    ros::Duration exec_time = ros::Time::now() - begin;
    std_msgs::Float64 exec_time_sec;
    exec_time_sec.data = exec_time.toSec();
    Pubs["exectime"].publish(exec_time_sec);
#endif

    loop_rate.sleep();
  }
}
}
