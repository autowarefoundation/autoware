/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ASTAR_NAVI_NODE_H
#define ASTAR_NAVI_NODE_H

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

namespace astar_planner
{
class AstarSearch
{
public:
  AstarSearch();
  ~AstarSearch();

  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose,
                const nav_msgs::OccupancyGrid &map, const double upper_bound_distance = -1);
  bool makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &transit_pose,
                const geometry_msgs::Pose &goal_pose, const nav_msgs::OccupancyGrid &map,
                const double upper_bound_distance = -1);
  void reset();
  // void initializeNode(int width, int height, int angle_size);
  void initializeNode(const nav_msgs::OccupancyGrid &map);
  void broadcastPathTF();
  bool getNodeInitialized() const
  {
    return node_initialized_;
  }
  nav_msgs::Path getPath() const
  {
    return path_;
  }

private:
  bool search();
  // void createStateUpdateTable(int angle_size);
  void createStateUpdateTableLocal(int angle_size);  //
  void poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta);
  bool isOutOfRange(int index_x, int index_y);
  void setPath(const SimpleNode &goal);
  void setMap(const nav_msgs::OccupancyGrid &map);
  bool setStartNode();
  bool setGoalNode();
  bool isGoal(double x, double y, double theta);
  bool isObs(int index_x, int index_y);
  bool detectCollision(const SimpleNode &sn);
  bool calcWaveFrontHeuristic(const SimpleNode &sn);
  bool detectCollisionWaveFront(const WaveFrontNode &sn);

  // for debug
  ros::NodeHandle n_;
  geometry_msgs::PoseArray debug_poses_;
  ros::Publisher debug_pose_pub_ = n_.advertise<geometry_msgs::PoseArray>("astar_debug_poses", 1, true);
  ros::Publisher footprint_pub_ = n_.advertise<visualization_msgs::MarkerArray>("astar_footprint", 1, true);
  void displayFootprint(const nav_msgs::Path &path);

  // ROS param
  std::string map_frame_;          // publishing path frame
  int angle_size_;                 // descritized angle size
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

  bool node_initialized_;
  std::vector<std::vector<NodeUpdate>> state_update_table_;
  nav_msgs::MapMetaData map_info_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // Pose in global(/map) frame
  geometry_msgs::PoseStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  double goal_yaw_;

  // Pose in OccupancyGrid frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;

  // Transform which converts OccupancyGrid frame to global frame
  tf::Transform map2ogm_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // for prunning
  double upper_bound_distance_;

  // Searched path
  nav_msgs::Path path_;
};

}  // namespace astar_planner

#endif  // ASTAR_NAVI_NODE_H
