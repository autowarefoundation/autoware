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

#include "lane_select_core.h"

namespace lane_planner
{
// Constructor
LaneSelectNode::LaneSelectNode()
  : private_nh_("~")
  , num_of_lane_(-1)
  , num_of_closest_(-1)
  , change_flag_(ChangeFlag::unknown)
  , is_lane_array_subscribed_(false)
  , is_current_pose_subscribed_(false)
  , last_time_(ros::Time::now())
{
  initForROS();
}

// Destructor
LaneSelectNode::~LaneSelectNode()
{
}

void LaneSelectNode::initForROS()
{
  // setup subscriber
  sub1_ = nh_.subscribe("traffic_waypoints_array", 100, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 100, &LaneSelectNode::callbackFromCurrentPose, this);

  // setup publisher
  pub_ = nh_.advertise<waypoint_follower::lane>("base_waypoints", 10, true);

  // get from rosparam
  private_nh_.param<int32_t>("size_of_waypoints", size_of_waypoints_, int32_t(30));
  private_nh_.param<int32_t>("lane_change_interval", lane_change_interval_, int32_t(2));
}

void LaneSelectNode::publishLocalLane()
{
  if (!is_current_pose_subscribed_ || !is_lane_array_subscribed_)
  {
    ROS_ERROR("Necessary topics are not subscribed yet.");
    return;
  }

  if (num_of_lane_ == -1)
    num_of_lane_++;

  ros::Time current_time = ros::Time::now();
  double dt = (current_time - last_time_).toSec();
  ROS_INFO("dt: %lf", dt);
  if (dt > lane_change_interval_ && (change_flag_ == ChangeFlag::right && num_of_lane_ < static_cast<int32_t>(lane_array_.lanes.size())))
  {
    num_of_lane_++;
    last_time_ = current_time;
  }

  if (dt > lane_change_interval_ && (change_flag_ == ChangeFlag::left && num_of_lane_ > 0))
  {
    num_of_lane_--;
    last_time_ = current_time;
  }

  waypoint_follower::lane local_lane;
  createLocalLane(&local_lane);
  pub_.publish(local_lane);

  is_current_pose_subscribed_ = false;
}

void LaneSelectNode::createLocalLane(waypoint_follower::lane *lane)
{
  num_of_closest_ = getClosestWaypoint(lane_array_.lanes.at(num_of_lane_), current_pose_.pose);

  if (num_of_closest_ == -1)
  {
    ROS_ERROR("cannot get closest waypoint");
    return;
  }

  // setup
  lane->header.stamp = ros::Time::now();
  lane->header.frame_id = "map";

  // push some waypoints
  for (auto i = 0; i < size_of_waypoints_; i++)
  {
    if (num_of_closest_ + i > static_cast<int32_t>(lane_array_.lanes.at(num_of_lane_).waypoints.size() - 1))
      break;

    lane->waypoints.push_back(lane_array_.lanes.at(num_of_lane_).waypoints.at(num_of_closest_ + i));
  }

  // push current_pose as first waypoint
  waypoint_follower::waypoint first_waypoint;
  first_waypoint = lane->waypoints.at(0);
  first_waypoint.pose.pose.position.x = current_pose_.pose.position.x;
  first_waypoint.pose.pose.position.y = current_pose_.pose.position.y;
  auto it = lane->waypoints.begin();
  lane->waypoints.insert(it, first_waypoint);
  change_flag_ = static_cast<ChangeFlag>(lane_array_.lanes.at(num_of_lane_).waypoints.at(num_of_closest_).change_flag);
  ROS_INFO("change_flag: %d", static_cast<ChangeFlagInteger>(change_flag_));
}

void LaneSelectNode::callbackFromLaneArray(const waypoint_follower::LaneArrayConstPtr &msg)
{
  lane_array_ = *msg;
  is_lane_array_subscribed_ = true;
}

void LaneSelectNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_ = *msg;
  is_current_pose_subscribed_ = true;
  publishLocalLane();
}

// get closest waypoint from current pose
/*int32_t getClosestWaypoint(const waypoint_follower::lane &current_path, const geometry_msgs::Pose &current_pose)
{
  WayPoints wp;
  wp.setPath(current_path);

  if (wp.isEmpty())
    return -1;

  // search closest candidate within a certain meter
  double search_distance = 5.0;
  std::vector<int> waypoint_candidates;
  for (int i = 1; i < wp.getSize(); i++)
  {
    if (getPlaneDistance(wp.getWaypointPosition(i), current_pose.position) > search_distance)
      continue;

    if (!wp.isFront(i, current_pose))
      continue;

    double angle_threshold = 90;
    if (getRelativeAngle(wp.getWaypointPose(i), current_pose) > angle_threshold)
      continue;

    waypoint_candidates.push_back(i);
  }

  // get closest waypoint from candidates
  if (!waypoint_candidates.empty())
  {
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (auto el : waypoint_candidates)
    {
      // ROS_INFO("closest_candidates : %d",el);
      double d = getPlaneDistance(wp.getWaypointPosition(el), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = el;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
  else
  {
    ROS_INFO("no candidate. search closest waypoint from all waypoints...");
    // if there is no candidate...
    int waypoint_min = -1;
    double distance_min = DBL_MAX;
    for (int i = 1; i < wp.getSize(); i++)
    {
      if (!wp.isFront(i, current_pose))
        continue;

      // if (!wp.isValid(i, current_pose))
      //  continue;

      double d = getPlaneDistance(wp.getWaypointPosition(i), current_pose.position);
      if (d < distance_min)
      {
        waypoint_min = i;
        distance_min = d;
      }
    }
    return waypoint_min;
  }
}*/

}  // lane_planner
