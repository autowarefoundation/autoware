/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
