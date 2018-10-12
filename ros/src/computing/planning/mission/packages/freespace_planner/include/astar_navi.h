/*
*/

#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/LaneArray.h>

#include "astar_search.h"

class AstarNavi
{
public:
  AstarNavi();
  ~AstarNavi();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher lane_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  tf::TransformListener tf_listener_;

  // params
  double waypoints_velocity_;  // [kmph]
  double update_rate_;        // [Hz]

  // classes
  AstarSearch astar;

  // variables
  nav_msgs::OccupancyGrid costmap_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_;
  bool current_pose_initialized_;
  bool goal_pose_initialized_;

  // functions, callback
  void costmapCallback(const nav_msgs::OccupancyGrid& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void goalPoseCallback(const geometry_msgs::PoseStamped& msg);

  // fucntions
  tf::Transform getTransform(const std::string& from, const std::string& to);
  void publishWaypoints(const nav_msgs::Path& path, const double& velocity);
  void publishStopWaypoints();
};

#endif
