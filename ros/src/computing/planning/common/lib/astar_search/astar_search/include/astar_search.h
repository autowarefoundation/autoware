/*
*/

#ifndef ASTER_PLANNER_H
#define ASTER_PLANNER_H

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include "astar_util.h"

class AstarSearch
{
public:
  AstarSearch();
  ~AstarSearch();
  void initialize(const nav_msgs::OccupancyGrid& costmap);
  bool makePlan(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);
  void reset();

  nav_msgs::Path getPath() const
  {
    return path_;
  }

private:
  void createStateUpdateTable();
  bool search();
  void poseToIndex(const geometry_msgs::Pose& pose, int* index_x, int* index_y, int* index_theta);
  void pointToIndex(const geometry_msgs::Point& point, int* index_x, int* index_y);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode& goal);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool isObs(int index_x, int index_y);
  bool detectCollision(const SimpleNode& sn);
  bool calcWaveFrontHeuristic(const SimpleNode& sn);
  bool detectCollisionWaveFront(const WaveFrontNode& sn);

  // ros param
  ros::NodeHandle n_;

  // base configs
  bool use_back_;  // use backward driving
  bool use_potential_heuristic_;
  bool use_wavefront_heuristic_;
  double time_limit_;  // msec

  // robot configs
  double robot_length_;
  double robot_width_;
  double robot_base2back_;
  double minimum_turning_radius_;  // varying by vehicles

  // search configs
  int theta_size_;  // descritized angle size
  double goal_angle_range_;
  double curve_weight_;
  double reverse_weight_;
  double lateral_goal_range_;
  double longitudinal_goal_range_;

  // costmap configs
  int obstacle_threshold_;  // more than this value is regarded as obstacles
  double potential_weight_;
  double distance_heuristic_weight_;

  // hybrid astar variables
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // costmap as occupancy grid
  nav_msgs::OccupancyGrid costmap_;

  // pose in costmap frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;
  double goal_yaw_;

  // result path
  nav_msgs::Path path_;
};

#endif
