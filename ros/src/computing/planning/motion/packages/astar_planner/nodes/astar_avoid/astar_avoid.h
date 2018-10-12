/*
*/

#ifndef ASTAR_AVOID_H
#define ASTAR_AVOID_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/LaneArray.h>

#include "waypoint_follower/libwaypoint_follower.h"
#include "astar_search.h"

class AstarAvoid
{
public:
  AstarAvoid();
  ~AstarAvoid();
  void run();

private:
  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher safety_waypoints_pub_;
  ros::Subscriber costmap_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber base_waypoints_sub_;
  ros::Subscriber closest_waypoint_sub_;
  ros::Subscriber obstacle_waypoint_sub_;
  tf::TransformListener tf_listener_;

  // params
  int safety_waypoints_size_;
  double update_rate_;

  bool avoidance_;
  int search_waypoints_size_;
  int search_waypoints_delta_;
  double avoid_waypoints_velocity_;

  // classes
  AstarSearch astar;

  // variables
  int closest_waypoint_index_;
  int obstacle_waypoint_index_;
  int goal_waypoint_index_;
  nav_msgs::OccupancyGrid costmap_;
  autoware_msgs::lane base_waypoints_;
  autoware_msgs::lane safety_waypoints_;
  geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
  tf::Transform local2costmap_;  // local frame (e.g. velodyne) -> costmap origin

  bool costmap_initialized_;
  bool current_pose_initialized_;
  bool base_waypoints_initialized_;
  bool closest_waypoint_initialized_;
  bool obstacle_waypoint_initialized_;

  // functions, callback
  void costmapCallback(const nav_msgs::OccupancyGrid& msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void baseWaypointsCallback(const autoware_msgs::lane& msg);
  void closestWaypointCallback(const std_msgs::Int32& msg);
  void obstacleWaypointCallback(const std_msgs::Int32& msg);

  // functions
  tf::Transform getTransform(const std::string& from, const std::string& to);
  void publishWaypoints(const autoware_msgs::lane& base_waypoints);
  void createAvoidWaypoints(const nav_msgs::Path& path, autoware_msgs::lane& avoid_waypoints, int& end_of_avoid_index);
};

#endif
