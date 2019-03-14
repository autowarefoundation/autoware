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

#include "astar_search/astar_util.h"

class AstarSearch
{
  friend class TestClass;

public:
  AstarSearch();
  ~AstarSearch();
  void initialize(const nav_msgs::OccupancyGrid& costmap);
  bool makePlan(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);
  void reset();

  const nav_msgs::Path& getPath() const
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
  bool setStartNode(const geometry_msgs::Pose& start_pose);
  bool setGoalNode(const geometry_msgs::Pose& goal_pose);
  bool isGoal(double x, double y, double theta);
  bool isObs(int index_x, int index_y);
  bool detectCollision(const SimpleNode& sn);
  bool calcWaveFrontHeuristic(const SimpleNode& sn);
  bool detectCollisionWaveFront(const WaveFrontNode& sn);

  // ros param
  ros::NodeHandle n_;

  // base configs
  bool use_back_;                 // backward search
  bool use_potential_heuristic_;  // potential cost function
  bool use_wavefront_heuristic_;  // wavefront cost function
  double time_limit_;             // planning time limit [msec]

  // robot configs (TODO: obtain from vehicle_info)
  double robot_length_;           // X [m]
  double robot_width_;            // Y [m]
  double robot_base2back_;        // base_link to rear [m]
  double minimum_turning_radius_; // [m]]

  // search configs
  int theta_size_;                  // descritized angle table size [-]
  double curve_weight_;             // curve moving cost [-]
  double reverse_weight_;           // backward moving cost [-]
  double lateral_goal_range_;       // reaching threshold, lateral error [m]
  double longitudinal_goal_range_;  // reaching threshold, longitudinal error [m]
  double angle_goal_range_;         // reaching threshold, angle error [deg]

  // costmap configs
  int obstacle_threshold_;            // obstacle threshold on grid [-]
  double potential_weight_;           // weight of potential cost [-]
  double distance_heuristic_weight_;  // obstacle threshold on grid [0,255]

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
