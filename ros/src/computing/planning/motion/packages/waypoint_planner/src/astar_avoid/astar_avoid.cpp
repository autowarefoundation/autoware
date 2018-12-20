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

#include "waypoint_planner/astar_avoid/astar_avoid.h"

AstarAvoid::AstarAvoid()
  : nh_()
  , private_nh_("~")
  , closest_waypoint_index_(-1)
  , obstacle_waypoint_index_(-1)
  , goal_waypoint_index_(-1)
  , costmap_initialized_(false)
  , current_pose_initialized_(false)
  , current_velocity_initialized_(false)
  , base_waypoints_initialized_(false)
  , closest_waypoint_initialized_(false)
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 5.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);
}

AstarAvoid::~AstarAvoid()
{
}

void AstarAvoid::costmapCallback(const nav_msgs::OccupancyGrid& msg)
{
  costmap_ = msg;
  tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
  costmap_initialized_ = true;
}

void AstarAvoid::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  current_pose_global_ = msg;

  if (!enable_avoidance_)
  {
    current_pose_initialized_ = true;
  }
  else
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
    current_pose_initialized_ = true;
  }
}

void AstarAvoid::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  current_velocity_ = msg;
  current_velocity_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::Lane& msg)
{
  base_waypoints_ = msg;
  base_waypoints_initialized_ = true;
}

void AstarAvoid::closestWaypointCallback(const std_msgs::Int32& msg)
{
  closest_waypoint_index_ = msg.data;
  closest_waypoint_initialized_ = true;
}

void AstarAvoid::obstacleWaypointCallback(const std_msgs::Int32& msg)
{
  obstacle_waypoint_index_ = msg.data;
}

void AstarAvoid::run()
{
  // check topics
  state_ = AstarAvoid::STATE::INITIALIZING;

  while (ros::ok())
  {
    ros::spinOnce();
    if (checkInitialized())
    {
      obstacle_waypoint_index_ = -1;  // reset once
      state_ = AstarAvoid::STATE::RELAYING;
      break;
    }
    ROS_WARN("Waiting for subscribing topics...");
    ros::Duration(1.0).sleep();
  }

  // main loop
  int end_of_avoid_index = -1;
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();
  autoware_msgs::Lane current_waypoints, avoid_waypoints;
  ros::Rate rate(update_rate_);

  while (ros::ok())
  {
    ros::spinOnce();

    // relay mode
    if (!enable_avoidance_)
    {
      publishWaypoints(base_waypoints_);
      rate.sleep();
      continue;
    }

    // avoidance mode
    bool found_obstacle = (obstacle_waypoint_index_ >= 0);
    bool avoid_velocity = (current_velocity_.twist.linear.x < avoid_start_velocity_ / 3.6);

    // update state
    if (state_ == AstarAvoid::STATE::RELAYING)
    {
      if (found_obstacle)
      {
        ROS_INFO("RELAYING -> STOPPING, Decelerate for stopping");
        state_ = AstarAvoid::STATE::STOPPING;
      }
    }
    else if (state_ == AstarAvoid::STATE::STOPPING)
    {
      if (!found_obstacle)
      {
        ROS_INFO("STOPPING -> RELAYING, Obstacle disappers");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (avoid_velocity)
      {
        bool replan = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);
        if (replan)
        {
          ROS_INFO("STOPPING -> PLANNING, Start A* planning");
          state_ = AstarAvoid::STATE::PLANNING;
          start_plan_time = ros::WallTime::now();
          startPlanThread(current_waypoints, avoid_waypoints, end_of_avoid_index, start_avoid_time);
        }
      }
    }
    else if (state_ == AstarAvoid::STATE::PLANNING)
    {
      // stop planning if obstacle disappers
      if (!found_obstacle)
      {
        ROS_INFO("PLANNNG -> RELAYING, Obstacle disappers");
        state_ = AstarAvoid::STATE::RELAYING;
      }
    }
    else if (state_ == AstarAvoid::STATE::AVOIDING)
    {
      bool reached = (getClosestWaypoint(current_waypoints, current_pose_global_.pose) > end_of_avoid_index);
      if (reached)
      {
        ROS_INFO("AVOIDING -> RELAYING, Reached goal");
        state_ = AstarAvoid::STATE::RELAYING;
      }
      else if (found_obstacle && avoid_velocity)
      {
        bool replan = ((ros::WallTime::now() - start_avoid_time).toSec() > replan_interval_);
        if (replan)
        {
          ROS_INFO("AVOIDING -> STOPPING, Abort avoiding");
          state_ = AstarAvoid::STATE::STOPPING;
        }
      }
    }

    // select waypoints
    switch (state_)
    {
      case AstarAvoid::STATE::RELAYING:
        current_waypoints = base_waypoints_;
        break;
      case AstarAvoid::STATE::STOPPING:
        // do nothing, keep current waypoints
        break;
      case AstarAvoid::STATE::PLANNING:
        // do nothing, keep current waypoints
        break;
      case AstarAvoid::STATE::AVOIDING:
        current_waypoints = avoid_waypoints;
        break;
      default:
        current_waypoints = base_waypoints_;
        break;
    }

    // publish waypoints
    publishWaypoints(current_waypoints);
    rate.sleep();
  }
}

bool AstarAvoid::checkInitialized()
{
  bool initialized = false;

  // check for relay mode
  initialized = (current_pose_initialized_ && closest_waypoint_initialized_ && base_waypoints_initialized_ &&
                 (closest_waypoint_index_ >= 0));

  // check for avoidance mode, additionally
  if (enable_avoidance_)
  {
    initialized = (initialized && (current_velocity_initialized_ && costmap_initialized_));
  }

  return initialized;
}

void AstarAvoid::startPlanThread(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints,
                                 int& end_of_avoid_index, ros::WallTime& start_avoid_time)
{
  // start A* planning thread
  astar_thread_ = std::thread(&AstarAvoid::planWorker, this, std::ref(current_waypoints), std::ref(avoid_waypoints),
                              std::ref(end_of_avoid_index), std::ref(state_), std::ref(start_avoid_time));
  astar_thread_.detach();
}

void AstarAvoid::planWorker(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints,
                            int& end_of_avoid_index, State& state, ros::WallTime& start_avoid_time)
{
  if (planAvoidWaypoints(current_waypoints, avoid_waypoints, end_of_avoid_index))
  {
    if (state_ == AstarAvoid::STATE::PLANNING)
    {
      ROS_INFO("PLANNING -> AVOIDING, Found path");
      start_avoid_time = ros::WallTime::now();
      state_ = AstarAvoid::STATE::AVOIDING;
    }
  }
  else
  {
    ROS_INFO("PLANNING -> STOPPING, Cannot find path");
    state_ = AstarAvoid::STATE::STOPPING;
  }
}

bool AstarAvoid::planAvoidWaypoints(const autoware_msgs::Lane& current_waypoints, autoware_msgs::Lane& avoid_waypoints,
                                    int& end_of_avoid_index)
{
  bool found_path = false;

  // update goal pose incrementally and execute A* search
  for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    goal_waypoint_index_ = closest_waypoint_index_ + obstacle_waypoint_index_ + i;
    if (goal_waypoint_index_ >= static_cast<int>(base_waypoints_.waypoints.size()))
    {
      break;
    }

    // update goal pose
    goal_pose_global_ = current_waypoints.waypoints[goal_waypoint_index_].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

    // initialize costmap for A* search
    astar_.initialize(costmap_);

    // execute astar search
    // ros::WallTime start = ros::WallTime::now();
    found_path = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    // ros::WallTime end = ros::WallTime::now();

    // ROS_INFO("Astar planning: %f [s], at index = %d", (end - start).toSec(), goal_waypoint_index_);

    if (found_path)
    {
      mergeAvoidWaypoints(astar_.getPath(), current_waypoints, avoid_waypoints, end_of_avoid_index);
      if (avoid_waypoints.waypoints.size() > 0)
      {
        ROS_INFO("Found GOAL at index = %d", goal_waypoint_index_);
        astar_.reset();
        return true;
      }
      else
      {
        found_path = false;
      }
    }
    astar_.reset();
  }

  ROS_ERROR("Can't find goal...");
  return false;
}

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const autoware_msgs::Lane& current_waypoints,
                                     autoware_msgs::Lane& avoid_waypoints, int& end_of_avoid_index)
{
  // reset
  avoid_waypoints.waypoints.clear();
  avoid_waypoints.header = current_waypoints.header;
  avoid_waypoints.increment = current_waypoints.increment;

  // add waypoints before start index
  for (int i = 0; i < closest_waypoint_index_; ++i)
  {
    avoid_waypoints.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // set waypoints for avoiding
  for (const auto& pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = avoid_waypoints.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(avoid_waypoints.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
    avoid_waypoints.waypoints.push_back(wp);
  }

  // end of avoiding waypoints
  end_of_avoid_index = getClosestWaypoint(avoid_waypoints, current_pose_global_.pose) + path.poses.size();

  // add waypoints after goal index
  for (int i = 0; goal_waypoint_index_ + i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    int index = goal_waypoint_index_ + i;
    avoid_waypoints.waypoints.push_back(current_waypoints.waypoints.at(index));
  }
}

void AstarAvoid::publishWaypoints(const autoware_msgs::Lane& global_waypoints)
{
  autoware_msgs::Lane local_waypoints;
  local_waypoints.header = global_waypoints.header;
  local_waypoints.increment = global_waypoints.increment;

  // push waypoints from closest index
  for (int i = 0; i < safety_waypoints_size_; ++i)
  {
    int index = getClosestWaypoint(global_waypoints, current_pose_global_.pose) + i;
    if (index < 0 || static_cast<int>(global_waypoints.waypoints.size()) <= index)
    {
      break;
    }
    const autoware_msgs::Waypoint& wp = global_waypoints.waypoints[index];
    local_waypoints.waypoints.push_back(wp);
  }

  if (local_waypoints.waypoints.size() > 0)
  {
    safety_waypoints_pub_.publish(local_waypoints);
  }
}

tf::Transform AstarAvoid::getTransform(const std::string& from, const std::string& to)
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
