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

#include "search_info_ros.h"

namespace astar_planner
{
SearchInfo::SearchInfo()
  : map_set_(false)
  , start_set_(false)
  , goal_set_(false)
  , path_set_(false)
  , closest_waypoint_index_(-1)
  , obstacle_waypoint_index_(-1)
  , start_waypoint_index_(-1)
  , goal_waypoint_index_(-1)
  , state_("")
  , upper_bound_distance_(-1)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<int>("obstacle_detect_count", obstacle_detect_count_, 10);
  private_nh_.param<int>("avoid_distance", avoid_distance_, 13);
  private_nh_.param<double>("avoid_velocity_limit_mps", avoid_velocity_limit_mps_, 4.166);
  private_nh_.param<double>("upper_bound_ratio", upper_bound_ratio_, 1.04);
  private_nh_.param<bool>("avoidance", avoidance_, false);
  private_nh_.param<bool>("change_path", change_path_, true);
}

SearchInfo::~SearchInfo()
{
}

double SearchInfo::calcPathLength(const autoware_msgs::Lane &lane, const int start_waypoint_index,
                                  const int goal_waypoint_index) const
{
  if (lane.waypoints.size() <= 1)
    return 0;

  // calulate the length of the path
  double dist_sum = 0;
  for (int i = start_waypoint_index; i < goal_waypoint_index; i++)
  {
    geometry_msgs::Pose p1 = lane.waypoints[i].pose.pose;
    geometry_msgs::Pose p2 = lane.waypoints[i + 1].pose.pose;

    dist_sum += astar_planner::calcDistance(p1.position.x, p1.position.y, p2.position.x, p2.position.y);
  }

  // return the path lengh
  return dist_sum;
}

void SearchInfo::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  map_ = *msg;

  std::string map_frame = map_frame_;
  std::string ogm_frame = msg->header.frame_id;

  // Set transform between map frame and OccupancyGrid frame
  tf::StampedTransform map2ogm_frame;
  try
  {
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set transform between map frame and the origin of OccupancyGrid
  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = astar_planner::transformPose(map_.info.origin, map2ogm_frame);
  tf::poseMsgToTF(ogm_in_map, map2ogm);
  ogm2map_ = map2ogm.inverse();

  map_set_ = true;
}

void SearchInfo::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_ = *msg;

  return;
}

void SearchInfo::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_mps_ = msg->twist.linear.x;
}

void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed goal pose!");

  std::string map_frame = map_frame_;
  std::string goal_frame = msg->header.frame_id;

  // Get transform of map to the frame of goal pose
  tf::StampedTransform map2world;
  try
  {
    tf_listener_.lookupTransform(map_frame, goal_frame, ros::Time(0), map2world);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set goal pose
  geometry_msgs::Pose pose_msg = msg->pose;
  goal_pose_global_.pose = astar_planner::transformPose(pose_msg, map2world);
  goal_pose_global_.header = msg->header;
  goal_pose_local_.pose = astar_planner::transformPose(goal_pose_global_.pose, ogm2map_);
  goal_pose_local_.header = goal_pose_global_.header;

  goal_set_ = true;

  // Get transform of map to the frame of start pose
  std::string start_frame = current_pose_.header.frame_id;
  tf::StampedTransform map2start_frame;
  try
  {
    tf_listener_.lookupTransform(map_frame_, start_frame, ros::Time(0), map2start_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set start pose
  start_pose_global_.pose = astar_planner::transformPose(current_pose_.pose, map2start_frame);
  start_pose_global_.header = current_pose_.header;
  start_pose_local_.pose = astar_planner::transformPose(start_pose_global_.pose, ogm2map_);
  start_pose_local_.header = start_pose_global_.header;

  start_set_ = true;
}

// get waypoints
void SearchInfo::waypointsCallback(const autoware_msgs::LaneConstPtr &msg)
{
  subscribed_waypoints_ = *msg;

  if (!path_set_)
  {
    current_waypoints_ = *msg;
    path_set_ = true;
  }
}

void SearchInfo::closestWaypointCallback(const std_msgs::Int32ConstPtr &msg)
{
  closest_waypoint_index_ = msg->data;
}

void SearchInfo::obstacleWaypointCallback(const std_msgs::Int32ConstPtr &msg)
{
  // not always avoid AND current state is not avoidance
  if (!avoidance_ && state_ != "OBSTACLE_AVOIDANCE")
  {
    ROS_WARN("current state is not OBSTACLE_AVOIDANCE");
    return;
  }

  // there are no obstacles
  if (msg->data < 0 || closest_waypoint_index_ < 0 || current_waypoints_.waypoints.empty())
  {
    return;
  }

  // msg->data : local index
  // closest   : global index
  // Conver local index to global index
  obstacle_waypoint_index_ = msg->data + closest_waypoint_index_;

  // Handle when detecting sensor noise as an obstacle
  static int prev_obstacle_waypoint_index = -1;
  static int obstacle_count = 0;
  int same_obstacle_threshold = 2;
  if (obstacle_waypoint_index_ >= prev_obstacle_waypoint_index - same_obstacle_threshold &&
      obstacle_waypoint_index_ <= prev_obstacle_waypoint_index + same_obstacle_threshold)
  {
    obstacle_count++;
  }
  else
  {
    obstacle_count = 1;
  }

  prev_obstacle_waypoint_index = obstacle_waypoint_index_;

  if (obstacle_count < obstacle_detect_count_)
    return;

  // not debug mode
  if (change_path_)
    obstacle_count = 0;

  // Decide start and goal waypoints for planning
  start_waypoint_index_ = obstacle_waypoint_index_ - avoid_distance_;
  goal_waypoint_index_ = obstacle_waypoint_index_ + avoid_distance_;

  // Handle out of range
  if (start_waypoint_index_ < 0)
    start_waypoint_index_ = 0;

  // Handle out of range
  if (goal_waypoint_index_ >= static_cast<int>(getCurrentWaypoints().waypoints.size()))
    goal_waypoint_index_ = getCurrentWaypoints().waypoints.size() - 1;

  double original_path_length = calcPathLength(current_waypoints_, start_waypoint_index_, goal_waypoint_index_);
  upper_bound_distance_ = original_path_length * upper_bound_ratio_;

  // Do not avoid if (the obstacle is too close || current velocity is too fast)
  if (closest_waypoint_index_ + 1 > start_waypoint_index_)
  {
    ROS_WARN("The obstacle is too close!");
    return;
  }

  // apply velocity limit for avoiding
  if (current_velocity_mps_ > avoid_velocity_limit_mps_)
  {
    ROS_WARN("Velocity of the vehicle exceeds the avoid velocity limit");
    return;
  }

  // Set start pose
  start_pose_global_ = current_waypoints_.waypoints[start_waypoint_index_].pose;
  start_pose_local_.pose = astar_planner::transformPose(start_pose_global_.pose, ogm2map_);
  start_set_ = true;

  // Set transit pose
  // TODO:
  double actual_car_width = 2.5;  // [m]
  geometry_msgs::Pose relative_transit_pose;
  // TODO: always right avoidance ???
  relative_transit_pose.position.y -= actual_car_width;
  relative_transit_pose.orientation = current_waypoints_.waypoints[obstacle_waypoint_index_].pose.pose.orientation;
  tf::Pose obstacle_pose_tf;
  tf::poseMsgToTF(current_waypoints_.waypoints[obstacle_waypoint_index_].pose.pose, obstacle_pose_tf);

  transit_pose_global_.pose = astar_planner::transformPose(relative_transit_pose, obstacle_pose_tf);
  transit_pose_local_.pose = astar_planner::transformPose(transit_pose_global_.pose, ogm2map_);

  // Set goal pose
  goal_pose_global_ = current_waypoints_.waypoints[goal_waypoint_index_].pose;
  goal_pose_local_.pose = astar_planner::transformPose(goal_pose_global_.pose, ogm2map_);

  goal_set_ = true;
}

void SearchInfo::stateCallback(const std_msgs::StringConstPtr &msg)
{
  state_ = msg->data;
}

void SearchInfo::reset()
{
  map_set_ = false;
  start_set_ = false;
  goal_set_ = false;
  obstacle_waypoint_index_ = -1;
}
}
