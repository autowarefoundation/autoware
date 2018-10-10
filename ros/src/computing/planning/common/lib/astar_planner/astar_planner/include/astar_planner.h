/*
*/

#ifndef ASTER_PLANNER_H
#define ASTER_PLANNER_H

#include "astar_util.h"
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <chrono>

class AstarPlanner
{
public:
  AstarPlanner();
  ~AstarPlanner();

  void initialize(const nav_msgs::OccupancyGrid &costmap);
  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose);
  void reset();
  void broadcastPathTF();
  bool getNodeInitialized() const
  {
    return node_initialized_;
  }
  nav_msgs::Path getPath() const
  {
    return path_;
  }
  void publishPoseArray(const ros::Publisher &pub, const std::string &frame);

private:
  void createStateUpdateTable();
  bool search();
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  void pointToIndex(const geometry_msgs::Point &point, int *index_x, int *index_y);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool isObs(int index_x, int index_y);
  bool detectCollision(const SimpleNode &sn);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);

  // for debug
  ros::NodeHandle n_;
  geometry_msgs::PoseArray debug_pose_array_;
  ros::Publisher debug_pose_pub_ = n_.advertise<geometry_msgs::PoseArray>("astar_debug_poses", 1, true);

  // ROS param
  std::string map_frame_;          // publishing path frame
  int theta_size_;                 // descritized angle size
  double minimum_turning_radius_;  // varying by vehicles
  int obstacle_threshold_;         // more than this value is regarded as obstacles
  bool use_back_;                  // use backward driving
  double robot_length_;
  double robot_width_;
  double base2back_;
  double curve_weight_;
  double reverse_weight_;
  double distance_heuristic_weight_;
  double potential_weight_;
  bool use_wavefront_heuristic_;
  bool use_potential_heuristic_;
  bool use_2dnav_goal_;
  double time_limit_;  // msec
  double lateral_goal_range_;
  double longitudinal_goal_range_;
  double goal_angle_range_;
  bool publish_marker_;

  nav_msgs::OccupancyGrid costmap_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  bool node_initialized_;

  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  double goal_yaw_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Searched path
  nav_msgs::Path path_;
};

#endif
