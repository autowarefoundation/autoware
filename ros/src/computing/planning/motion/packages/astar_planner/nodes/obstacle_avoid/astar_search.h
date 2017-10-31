/*
 *  Copyright (c) 2015, Nagoya University
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
