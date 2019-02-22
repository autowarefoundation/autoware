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

#include "waypoint_planner/astar_avoid/astar_avoid.h"

AstarAvoid::AstarAvoid()
  : nh_()
  , private_nh_("~")
  , closest_waypoint_index_(-1)
  , obstacle_waypoint_index_(-1)
  , costmap_initialized_(false)
  , current_pose_initialized_(false)
  , current_velocity_initialized_(false)
  , base_waypoints_initialized_(false)
  , closest_waypoint_initialized_(false)
  , terminate_thread_(false)
  , stop_check_avoidance_(false)
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("enable_avoidance", enable_avoidance_, false);
  private_nh_.param<bool>("use_decision_state", use_decision_state_, false);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);
  private_nh_.param<double>("avoid_start_velocity", avoid_start_velocity_, 5.0);
  private_nh_.param<double>("replan_interval", replan_interval_, 2.0);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 50);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 10, true);
  state_cmd_pub_ = nh_.advertise<std_msgs::String>("state_cmd", 10, true);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("current_velocity", 1, &AstarAvoid::currentVelocityCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);
  state_sub_ = nh_.subscribe("decision_maker/state", 1, &AstarAvoid::stateCallback, this);

  rate_ = new ros::Rate(update_rate_);
}

AstarAvoid::~AstarAvoid()
{
  publish_thread_.join();
  delete rate_;
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

void AstarAvoid::stateCallback(const std_msgs::StringConstPtr& msg)
{
  if (!use_decision_state_)
  {
    return; // do nothing
  }

  if (msg->data.find("Go") != std::string::npos)
  {
    state_ = AstarAvoid::STATE::RELAYING; // not execute astar search
  }
  else if (msg->data.find("TryAvoidance") != std::string::npos)
  {
    state_ = AstarAvoid::STATE::PLANNING; // execute astar search
  }
  else if (msg->data.find("CheckAvoidance") != std::string::npos)
  {
    // TODO: decision making for avoidance execution
    stop_check_avoidance_ = true; // stop on path
    state_ = AstarAvoid::STATE::AVOIDING;
  }
  else if (msg->data.find("Avoidance") != std::string::npos)
  {
    stop_check_avoidance_ = false;  // go on avoiding path
  }
  else if (msg->data.find("ReturnToLane") != std::string::npos)
  {
    // NOTE: A* search can produce both avoid and return paths once time.
    state_ = AstarAvoid::STATE::RELAYING; // not execute astar search
    publishStateCmd("completed_return");
  }
}

void AstarAvoid::publishStateCmd(const std::string& str)
{
  std_msgs::String msg;
  msg.data = str;
  state_cmd_pub_.publish(msg);
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
      break;
    }
    ROS_WARN("Waiting for subscribing topics...");
    ros::Duration(1.0).sleep();
  }

  // main loop
  int end_of_avoid_index = -1;
  ros::WallTime start_plan_time = ros::WallTime::now();
  ros::WallTime start_avoid_time = ros::WallTime::now();

  // reset obstacle index
  obstacle_waypoint_index_ = -1;

  // relaying mode at startup
  state_ = AstarAvoid::STATE::RELAYING;

  // start publish thread
  publish_thread_ = std::thread(&AstarAvoid::publishWaypoints, this);

  while (ros::ok())
  {
    ros::spinOnce();

    // relay mode
    if (!enable_avoidance_)
    {
      rate_->sleep();
      continue;
    }

    // avoidance mode
    bool found_obstacle = (obstacle_waypoint_index_ >= 0);
    bool avoid_velocity = (current_velocity_.twist.linear.x < avoid_start_velocity_ / 3.6);

    // update state
    if (!use_decision_state_)
    {
      if (state_ == AstarAvoid::STATE::RELAYING)
      {
        avoid_waypoints_ = base_waypoints_;

        if (found_obstacle)
        {
          ROS_INFO("RELAYING -> STOPPING, Decelerate for stopping");
          state_ = AstarAvoid::STATE::STOPPING;
        }
      }
      else if (state_ == AstarAvoid::STATE::STOPPING)
      {
        bool replan = ((ros::WallTime::now() - start_plan_time).toSec() > replan_interval_);

        if (!found_obstacle)
        {
          ROS_INFO("STOPPING -> RELAYING, Obstacle disappers");
          state_ = AstarAvoid::STATE::RELAYING;
        }
        else if (replan && avoid_velocity)
        {
          ROS_INFO("STOPPING -> PLANNING, Start A* planning");
          state_ = AstarAvoid::STATE::PLANNING;
        }
      }
      else if (state_ == AstarAvoid::STATE::PLANNING)
      {
        start_plan_time = ros::WallTime::now();

        if (planAvoidWaypoints(end_of_avoid_index))
        {
          ROS_INFO("PLANNING -> AVOIDING, Found path");
          state_ = AstarAvoid::STATE::AVOIDING;
          start_avoid_time = ros::WallTime::now();
        }
        else
        {
          ROS_INFO("PLANNING -> STOPPING, Cannot find path");
          state_ = AstarAvoid::STATE::STOPPING;
        }
      }
      else if (state_ == AstarAvoid::STATE::AVOIDING)
      {
        bool reached = (getClosestWaypoint(avoid_waypoints_, current_pose_global_.pose) > end_of_avoid_index);
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
    }
    else
    {
      if (state_ == AstarAvoid::STATE::RELAYING)
      {
        avoid_waypoints_ = base_waypoints_;
      }
      else if (state_ == AstarAvoid::STATE::PLANNING)
      {
        if (planAvoidWaypoints(end_of_avoid_index))
        {
          publishStateCmd("found_path");
        }
      }
      else if (state_ == AstarAvoid::STATE::AVOIDING)
      {
        bool reached = (getClosestWaypoint(avoid_waypoints_, current_pose_global_.pose) > end_of_avoid_index);
        if (reached)
        {
          publishStateCmd("completed_avoidance");
        }
      }
    }

    rate_->sleep();
  }

  terminate_thread_ = true;
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

bool AstarAvoid::planAvoidWaypoints(int& end_of_avoid_index)
{
  bool found_path = false;
  int closest_waypoint_index = getClosestWaypoint(avoid_waypoints_, current_pose_global_.pose);

  // update goal pose incrementally and execute A* search
  for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_); i += search_waypoints_delta_)
  {
    // update goal index
    int goal_waypoint_index = closest_waypoint_index + obstacle_waypoint_index_ + i;
    if (goal_waypoint_index >= static_cast<int>(avoid_waypoints_.waypoints.size()))
    {
      break;
    }

    // update goal pose
    goal_pose_global_ = avoid_waypoints_.waypoints[goal_waypoint_index].pose;
    goal_pose_local_.header = costmap_.header;
    goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                          getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

    // initialize costmap for A* search
    astar_.initialize(costmap_);

    // execute astar search
    // ros::WallTime start = ros::WallTime::now();
    found_path = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
    // ros::WallTime end = ros::WallTime::now();
    // ROS_INFO("Astar planning: %f [s], at index = %d", (end - start).toSec(), goal_waypoint_index);

    if (found_path)
    {
      const nav_msgs::Path& path = astar_.getPath();
      end_of_avoid_index = closest_waypoint_index + path.poses.size();
      mergeAvoidWaypoints(path, goal_waypoint_index);
      if (avoid_waypoints_.waypoints.size() > 0)
      {
        ROS_INFO("Found GOAL at index = %d", goal_waypoint_index);
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

void AstarAvoid::mergeAvoidWaypoints(const nav_msgs::Path& path, const int& goal_waypoint_index)
{
  autoware_msgs::Lane current_waypoints = avoid_waypoints_;

  // reset
  std::lock_guard<std::mutex> lock(mutex_);
  avoid_waypoints_.waypoints.clear();

  // add waypoints before start index
  int closest_waypoint_index = getClosestWaypoint(current_waypoints, current_pose_global_.pose);
  for (int i = 0; i < closest_waypoint_index; ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }

  // set waypoints for avoiding
  for (const auto& pose : path.poses)
  {
    autoware_msgs::Waypoint wp;
    wp.pose.header = avoid_waypoints_.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(avoid_waypoints_.header.frame_id, pose.header.frame_id));
    wp.pose.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;       // velocity = const
    avoid_waypoints_.waypoints.push_back(wp);
  }

  // add waypoints after goal index
  for (int i = goal_waypoint_index; i < static_cast<int>(current_waypoints.waypoints.size()); ++i)
  {
    avoid_waypoints_.waypoints.push_back(current_waypoints.waypoints.at(i));
  }
}

void AstarAvoid::publishWaypoints()
{
  autoware_msgs::Lane current_waypoints;

  while (!terminate_thread_)
  {
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
        current_waypoints = avoid_waypoints_;
        break;
      default:
        current_waypoints = base_waypoints_;
        break;
    }

    autoware_msgs::Lane safety_waypoints;
    safety_waypoints.header = current_waypoints.header;
    safety_waypoints.increment = current_waypoints.increment;

    // push waypoints from closest index
    for (int i = 0; i < safety_waypoints_size_; ++i)
    {
      int index = getClosestWaypoint(current_waypoints, current_pose_global_.pose) + i;
      if (index < 0 || static_cast<int>(current_waypoints.waypoints.size()) <= index)
      {
        break;
      }
      autoware_msgs::Waypoint wp = current_waypoints.waypoints[index];
      wp.twist.twist.linear.x = stop_check_avoidance_ ? 0.0 : wp.twist.twist.linear.x;
      safety_waypoints.waypoints.push_back(wp);
    }

    if (safety_waypoints.waypoints.size() > 0)
    {
      safety_waypoints_pub_.publish(safety_waypoints);
    }

    rate_->sleep();
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
