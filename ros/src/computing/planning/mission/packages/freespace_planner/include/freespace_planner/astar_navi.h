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

#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <autoware_msgs/LaneArray.h>

#include "astar_search/astar_search.h"

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
  double waypoints_velocity_;   // constant velocity on planned waypoints [km/h]
  double update_rate_;          // replanning and publishing rate [Hz]

  // classes
  AstarSearch astar_;

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
