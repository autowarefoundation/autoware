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

// C++ includes
#include <iostream>

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

typedef std::underlying_type<ChangeFlag>::type ChangeFlagInteger;

class LaneSelectNode
{
public:
  LaneSelectNode();
  ~LaneSelectNode();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub_;

  // subscriber
  ros::Subscriber sub1_, sub2_;

  // variables
  int32_t num_of_lane_;  // the number of lane we are driving
  int32_t num_of_closest_;
  int32_t size_of_waypoints_;
  int32_t lane_change_interval_;
  ChangeFlag change_flag_;
  waypoint_follower::LaneArray lane_array_;
  geometry_msgs::PoseStamped current_pose_;
  bool is_lane_array_subscribed_, is_current_pose_subscribed_;
  ros::Time last_time_;

  // callbacks
  void callbackFromLaneArray(const waypoint_follower::LaneArrayConstPtr &msg);
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publishLocalLane();
  void createLocalLane(waypoint_follower::lane *lane);
};

int32_t getNumOfClosest(const waypoint_follower::lane &current_path, const geometry_msgs::Pose &current_pose);
}
#endif  // LANE_SELECT_CORE_H
