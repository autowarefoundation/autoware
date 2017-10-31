#ifndef SEARCH_INFO_ROS_H
#define SEARCH_INFO_ROS_H

#include "astar_util.h"

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>

class SearchInfo
{
 public:
  SearchInfo();
  ~SearchInfo();

  // ROS Callback
  void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

  // get method
  bool getMapSet() {return map_set_;}
  bool getStartSet() {return start_set_;}
  bool getGoalSet() {return goal_set_;}
  nav_msgs::OccupancyGrid getMap() {return map_;}
  geometry_msgs::PoseStamped getStartPose() {return start_pose_local_;}
  geometry_msgs::PoseStamped getGoalPose() {return goal_pose_local_;}

  // Reset flags
  void reset();

 private:
  nav_msgs::OccupancyGrid map_;
  geometry_msgs::PoseStamped start_pose_global_;
  geometry_msgs::PoseStamped goal_pose_global_;
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;
  // Transform which converts global frame (/map in Autoware) to OccupancyGrid frame
  tf::Transform ogm2map_;
  tf::TransformListener tf_listener_;
  geometry_msgs::Pose current_pose_;

  // set data flag
  bool map_set_;
  bool start_set_;
  bool goal_set_;
};

#endif // SEARCH_INFO_ROS_H
