#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>

// lib
#include <state.hpp>
#include <state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker
{
bool DecisionMakerNode::isCrossRoadByVectorMapServer(const autoware_msgs::lane &lane_msg,
                                                     const geometry_msgs::PoseStamped &pose_msg)
{
#ifdef USE_VMAP_SERVER  // this is not successfully run
  cross_road_srv.request.pose = pose_msg;
  cross_road_srv.request.waypoints.waypoints.clear();

  for (int i = 0; i < 50; i++)
  {
    cross_road_srv.request.waypoints.waypoints.push_back(lane_msg.waypoints[i]);
  }
  for (const auto &wayp : lane_msg.waypoints)
    cross_road_srv.request.waypoints.waypoints.push_back(wayp);

  cross_road_cli.call(cross_road_srv);

  for (const auto &cross_road_d : cross_road_srv.response.objects.data)
  {
    // for DEBUG
    //   std::cout << "DEBUG" << cross_road_d.linkid << std::endl;
  }
#endif
  return false;
}

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
  ros::AsyncSpinner spinner(1);
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
