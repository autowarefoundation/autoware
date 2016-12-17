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

#ifndef LANE_SELECT_CORE_H
#define LANE_SELECT_CORE_H

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>

// C++ includes
#include <iostream>
#include <numeric>
#include <tuple>

// User defined includes
#include "waypoint_follower/LaneArray.h"
#include "waypoint_follower/libwaypoint_follower.h"

namespace lane_planner
{
enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

class LaneSelectNode
{
public:
  LaneSelectNode();
  ~LaneSelectNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_;
  ros::Publisher vis_pub1_;

  // subscriber
  ros::Subscriber sub1_, sub2_, sub3_;

  // variables
  int32_t current_lane_idx_;  // the index of the lane we are driving
  int32_t right_lane_idx_;
  int32_t left_lane_idx_;
  std::vector<std::tuple<waypoint_follower::lane, int32_t, ChangeFlag>> tuple_vec_;  // lane, closest_waypoint, change_flag
  bool is_lane_array_subscribed_, is_current_pose_subscribed_, is_current_velocity_subscribed_;
  ros::Time last_change_time_;

  // rosparam
  double distance_threshold_;
  int32_t lane_change_interval_;

  // topics
  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::TwistStamped current_velocity_;
  std::string current_state_;

  // callbacks
  void callbackFromLaneArray(const waypoint_follower::LaneArrayConstPtr &msg);
  void callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackFromTwistStamped(const geometry_msgs::TwistStampedConstPtr &msg);

  // initializer
  void initForROS();

  // visualizer
  void publishVisualizer();
  void createCurrentLaneMarker(visualization_msgs::Marker *marker);
  void createRightLaneMarker(visualization_msgs::Marker *marker);
  void createLeftLaneMarker(visualization_msgs::Marker *marker);
  void createClosestWaypointsMarker(visualization_msgs::Marker *marker);

  // functions
  void processing();
  void publish();
  bool getClosestWaypointNumberForEachLanes();
  int32_t findMostClosestLane(const std::vector<uint32_t> idx_vec, const geometry_msgs::Point p);
  void findCurrentLane();
  void findNeighborLanes();
  void changeLane();
};

int32_t getClosestWaypointNumber(const waypoint_follower::lane &current_lane, const geometry_msgs::Pose &current_pose,
                                 const geometry_msgs::Twist &current_velocity, const int32_t previous_number);

double getTwoDimensionalDistance(const geometry_msgs::Point &target1, const geometry_msgs::Point &target2);

void convertPointIntoRelativeCoordinate(const geometry_msgs::Point &input_point, const geometry_msgs::Pose &pose,
                                        geometry_msgs::Point *output_point);
double getRelativeAngle(const geometry_msgs::Pose &waypoint_pose, const geometry_msgs::Pose &current_pose);
}
#endif  // LANE_SELECT_CORE_H
