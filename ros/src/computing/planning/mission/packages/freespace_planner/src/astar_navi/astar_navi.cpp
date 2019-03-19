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

#include "freespace_planner/astar_navi.h"

AstarNavi::AstarNavi() : nh_(), private_nh_("~")
{
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 1.0);

  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarNavi::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarNavi::currentPoseCallback, this);
  goal_pose_sub_ = nh_.subscribe("move_base_simple/goal", 1, &AstarNavi::goalPoseCallback, this);

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  goal_pose_initialized_ = false;
}

AstarNavi::~AstarNavi()
{
}

void AstarNavi::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);

  costmap_initialized_ = true;
}

void AstarNavi::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  current_pose_global_ = msg;
  current_pose_local_.pose = transformPose(
      current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
  current_pose_local_.header.frame_id = costmap_.header.frame_id;
  current_pose_local_.header.stamp = current_pose_global_.header.stamp;

  current_pose_initialized_ = true;
}

void AstarNavi::goalPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if (!costmap_initialized_)
  {
    return;
  }

  goal_pose_global_ = msg;
  goal_pose_local_.pose =
      transformPose(goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
  goal_pose_local_.header.frame_id = costmap_.header.frame_id;
  goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  goal_pose_initialized_ = true;

  ROS_INFO_STREAM("Subscribed goal pose and transform from " << msg.header.frame_id << " to "
                                                             << goal_pose_local_.header.frame_id << "\n"
                                                             << goal_pose_local_.pose);
}

tf::Transform AstarNavi::getTransform(const std::string& from, const std::string& to)
{
  tf::StampedTransform stf;
  try
  {
    tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  return stf;
}

void AstarNavi::run()
{
  ros::Rate rate(update_rate_);

  nav_msgs::Path empty_path;
  empty_path.header.stamp = ros::Time::now();
  empty_path.header.frame_id = costmap_.header.frame_id;

  while (ros::ok())
  {
    ros::spinOnce();

    if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_)
    {
      rate.sleep();
      continue;
    }

    // initialize vector for A* search, this runs only once
    astar_.initialize(costmap_);

    // update local goal pose
    goalPoseCallback(goal_pose_global_);

    // execute astar search
    ros::WallTime start = ros::WallTime::now();
    bool result = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    ros::WallTime end = ros::WallTime::now();

    ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

    if (result)
    {
      ROS_INFO("Found GOAL!");
      publishWaypoints(astar_.getPath(), waypoints_velocity_);
    }
    else
    {
      ROS_INFO("Can't find goal...");
      publishStopWaypoints();
    }

    astar_.reset();
    rate.sleep();
  }
}

void AstarNavi::publishWaypoints(const nav_msgs::Path& path, const double& velocity)
{
  autoware_msgs::Lane lane;
  lane.header.frame_id = "map";
  lane.header.stamp = path.header.stamp;
  lane.increment = 0;

  for (const auto& pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = lane.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(lane.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    wp.twist.twist.linear.x = velocity / 3.6;  // velocity = const
    lane.waypoints.push_back(wp);
  }

  autoware_msgs::LaneArray lane_array;
  lane_array.lanes.push_back(lane);
  lane_pub_.publish(lane_array);
}

void AstarNavi::publishStopWaypoints()
{
  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;  // stop path
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = current_pose_global_.header.frame_id;
  pose.pose = current_pose_global_.pose;
  path.poses.push_back(pose);
  publishWaypoints(path, 0.0);
}
