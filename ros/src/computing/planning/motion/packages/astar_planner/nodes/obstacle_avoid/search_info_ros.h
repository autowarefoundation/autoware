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

#ifndef SEARCH_INFO_ROS_H
#define SEARCH_INFO_ROS_H

#include "astar_util.h"
#include "autoware_msgs/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

namespace astar_planner
{
class SearchInfo
{
public:
  SearchInfo();
  ~SearchInfo();

  // ROS Callback
  void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);
  // void startCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
  void waypointsCallback(const autoware_msgs::laneConstPtr &msg);
  void closestWaypointCallback(const std_msgs::Int32ConstPtr &msg);
  void obstacleWaypointCallback(const std_msgs::Int32ConstPtr &msg);
  void stateCallback(const std_msgs::StringConstPtr &msg);

  // get method
  bool getMapSet() const
  {
    return map_set_;
  }
  bool getStartSet() const
  {
    return start_set_;
  }
  bool getGoalSet() const
  {
    return goal_set_;
  }
  bool getPathSet() const
  {
    return path_set_;
  }
  nav_msgs::OccupancyGrid getMap() const
  {
    return map_;
  }
  geometry_msgs::PoseStamped getStartPose() const
  {
    return start_pose_local_;
  }
  geometry_msgs::PoseStamped getGoalPose() const
  {
    return goal_pose_local_;
  }
  geometry_msgs::PoseStamped getTransitPose() const
  {
    return transit_pose_local_;
  }
  geometry_msgs::PoseStamped getCurrentPose() const
  {
    return current_pose_;
  }
  double getCurrentVelocity() const
  {
    return current_velocity_mps_;
  }
  autoware_msgs::lane getSubscribedWaypoints() const
  {
    return subscribed_waypoints_;
  }
  autoware_msgs::lane getCurrentWaypoints() const
  {
    return current_waypoints_;
  }
  int getObstacleWaypointIndex() const
  {
    return obstacle_waypoint_index_;
  }
  int getClosestWaypointIndex() const
  {
    return closest_waypoint_index_;
  }
  int getStartWaypointIndex() const
  {
    return start_waypoint_index_;
  }
  int getGoalWaypointIndex() const
  {
    return goal_waypoint_index_;
  }
  double getUpperBoundDistance() const
  {
    return upper_bound_distance_;
  }
  double getAvoidVelocityLimitMPS() const
  {
    return avoid_velocity_limit_mps_;
  }
  bool getAvoidance() const
  {
    return avoidance_;
  }
  bool getChangePath() const
  {
    return change_path_;
  }

  // set method
  void setCurrentWaypoints(const autoware_msgs::lane &waypoints)
  {
    current_waypoints_ = waypoints;
  }

  // Reset flag
  void reset();

private:
  double calcPathLength(const autoware_msgs::lane &lane, const int start_waypoint_index,
                        const int goal_waypoint_index) const;

  nav_msgs::OccupancyGrid map_;
  geometry_msgs::PoseStamped start_pose_global_;
  geometry_msgs::PoseStamped goal_pose_global_;
  geometry_msgs::PoseStamped transit_pose_global_;
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;
  geometry_msgs::PoseStamped transit_pose_local_;
  // Transform which converts global frame (/map in Autoware) to OccupancyGrid frame
  tf::Transform ogm2map_;
  tf::TransformListener tf_listener_;

  // set data flag
  bool map_set_;
  bool start_set_;
  bool goal_set_;
  bool path_set_;

  // ROS param
  std::string map_frame_;
  int obstacle_detect_count_;        // 1 increment means 100ms
  int avoid_distance_;               // the number of waypoint
  double avoid_velocity_limit_mps_;  // m/s
  double upper_bound_ratio_;
  bool avoidance_;
  bool change_path_;

  // subscribed imformation
  int closest_waypoint_index_;
  int obstacle_waypoint_index_;
  int start_waypoint_index_;
  int goal_waypoint_index_;
  autoware_msgs::lane subscribed_waypoints_;
  autoware_msgs::lane current_waypoints_;
  geometry_msgs::PoseStamped current_pose_;
  double current_velocity_mps_;
  std::string state_;

  // for prunning
  double upper_bound_distance_;
};

}  // namespace astar_planner

#endif  // SEARCH_INFO_ROS_H
