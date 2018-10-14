/*
*/

#include "astar_avoid.h"

AstarAvoid::AstarAvoid() : nh_(), private_nh_("~")
{
  private_nh_.param<int>("safety_waypoints_size", safety_waypoints_size_, 100);
  private_nh_.param<double>("update_rate", update_rate_, 10.0);

  private_nh_.param<bool>("use_avoidance_state", use_avoidance_state_, true);
  private_nh_.param<bool>("avoidance", avoidance_, false);
  private_nh_.param<int>("search_waypoints_size", search_waypoints_size_, 30);
  private_nh_.param<int>("search_waypoints_delta", search_waypoints_delta_, 2);
  private_nh_.param<double>("avoid_waypoints_velocity", avoid_waypoints_velocity_, 10.0);

  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::lane>("safety_waypoints", 1, true);
  state_cmd_pub_ = nh_.advertise<std_msgs::String>("state_cmd", 1);
  costmap_sub_ = nh_.subscribe("costmap", 1, &AstarAvoid::costmapCallback, this);
  current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarAvoid::currentPoseCallback, this);
  base_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &AstarAvoid::baseWaypointsCallback, this);
  closest_waypoint_sub_ = nh_.subscribe("closest_waypoint", 1, &AstarAvoid::closestWaypointCallback, this);
  obstacle_waypoint_sub_ = nh_.subscribe("obstacle_waypoint", 1, &AstarAvoid::obstacleWaypointCallback, this);
  state_sub_ = nh_.subscribe("decision_maker/state", 1, &AstarAvoid::stateCallback, this);

  closest_waypoint_index_ = -1;
  obstacle_waypoint_index_ = -1;
  goal_waypoint_index_ = -1;

  costmap_initialized_ = false;
  current_pose_initialized_ = false;
  base_waypoints_initialized_ = false;
  closest_waypoint_initialized_ = false;
  obstacle_waypoint_initialized_ = false;

  avoiding_ = false;
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

  if (costmap_initialized_)
  {
    current_pose_local_.pose = transformPose(
        current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
    current_pose_local_.header.frame_id = costmap_.header.frame_id;
    current_pose_local_.header.stamp = current_pose_global_.header.stamp;
  }

  current_pose_initialized_ = true;
}

void AstarAvoid::baseWaypointsCallback(const autoware_msgs::lane& msg)
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
  obstacle_waypoint_initialized_ = true;
}

void AstarAvoid::stateCallback(const std_msgs::String& msg)
{
  if (!use_avoidance_state_)
  {
    return; // do nothing
  }

  if (msg.data.find("Go") != std::string::npos)
  {
    avoidance_ = false; // not execute astar search
    stop_by_state_ = false;  // go on path
  }
  else if (msg.data.find("TryAvoidance") != std::string::npos)
  {
    avoidance_ = true; // execute aster search
    stop_by_state_ = true;  // stop on path

    if (found_avoid_path_)
    {
      // state transition occur by this event
      state_cmd_pub_.publish(createStringMsg("found_path"));
    }
  }
  else if (msg.data.find("CheckAvoidance") != std::string::npos)
  {
    // TODO: decision making for avoidance execution
    stop_by_state_ = true;  // stop on path
  }
  else if (msg.data.find("Avoidance") != std::string::npos)
  {
    stop_by_state_ = false;  // go on avoiding path

    if (!avoiding_)
    {
      state_cmd_pub_.publish(createStringMsg("completed_avoidance"));
    }
  }
  else if (msg.data.find("ReturnToLane") != std::string::npos)
  {
    // NOTE: A* search can produce both avoid and return paths once time.
    state_cmd_pub_.publish(createStringMsg("completed_return"));
  }
}

void AstarAvoid::run()
{
  ros::Rate rate(update_rate_);

  int end_of_avoid_index = -1;
  autoware_msgs::lane avoid_waypoints;

  while (ros::ok())
  {
    ros::spinOnce();

    if (!current_pose_initialized_ || !closest_waypoint_initialized_ || !base_waypoints_initialized_ ||
        closest_waypoint_index_ < 0)
    {
      rate.sleep();
      continue;
    }

    if (!avoidance_)  // relay mode
    {
      // publish base waypoints
      publishWaypoints(base_waypoints_);
      avoiding_ = false;
    }
    else  // avoidance mode
    {
      if (!avoiding_ && costmap_initialized_ && !(obstacle_waypoint_index_ < 0))
      {
        // update goal pose incrementally and execute A* search
        for (int i = search_waypoints_delta_; i < static_cast<int>(search_waypoints_size_);
             i += search_waypoints_delta_)
        {
          // update goal index
          goal_waypoint_index_ = closest_waypoint_index_ + obstacle_waypoint_index_ + i;
          if (goal_waypoint_index_ >= static_cast<int>(base_waypoints_.waypoints.size()))
          {
            break;
          }

          // update goal pose
          goal_pose_global_ = base_waypoints_.waypoints[goal_waypoint_index_].pose;
          goal_pose_local_.header = costmap_.header;
          goal_pose_local_.pose = transformPose(
              goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));

          // initialize costmap for A* search
          astar.initialize(costmap_);

          // execute astar search
          ros::WallTime start = ros::WallTime::now();
          found_avoid_path_ = astar.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
          ros::WallTime end = ros::WallTime::now();

          ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

          if (found_avoid_path_)
          {
            ROS_INFO("Found GOAL at index = %d", goal_waypoint_index_);
            createAvoidWaypoints(astar.getPath(), avoid_waypoints, end_of_avoid_index);
            avoiding_ = true;
            break;
          }
          else
          {
            ROS_INFO("Can't find goal...");
            avoiding_ = false;
          }
          astar.reset();
        }
      }

      // update avoiding
      if (avoiding_ && avoid_waypoints.waypoints.size() > 0)
      {
        // check reaching goal
        avoiding_ = (getClosestWaypoint(avoid_waypoints, current_pose_global_.pose) < end_of_avoid_index);
      }

      // publish "safety" waypoints
      autoware_msgs::lane safety_waypoints = avoiding_ ? avoid_waypoints : base_waypoints_;
      publishWaypoints(safety_waypoints);
    }

    rate.sleep();
  }
}

std_msgs::String AstarAvoid::createStringMsg(const std::string& str)
{
  std_msgs::String msg;
  msg.data = str;
  return msg;
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

void AstarAvoid::publishWaypoints(const autoware_msgs::lane& waypoints)
{
  autoware_msgs::lane safety_waypoints;
  safety_waypoints.header = waypoints.header;
  safety_waypoints.increment = waypoints.increment;

  // push waypoints from closest index
  for (int i = 0; i < safety_waypoints_size_; ++i)
  {
    int index = getClosestWaypoint(waypoints, current_pose_global_.pose) + i;
    if (index >= static_cast<int>(waypoints.waypoints.size()))
    {
      break;
    }
    autoware_msgs::waypoint wp = waypoints.waypoints[index];
    // if state is not "Avoidance", vehicle stops until changing state
    wp.twist.twist.linear.x = (use_avoidance_state_ && stop_by_state_) ? 0.0 : wp.twist.twist.linear.x;
    safety_waypoints.waypoints.push_back(wp);
  }

  if (safety_waypoints.waypoints.size())
  {
    safety_waypoints_pub_.publish(safety_waypoints);
  }
}

void AstarAvoid::createAvoidWaypoints(const nav_msgs::Path& path, autoware_msgs::lane& avoid_waypoints,
                                      int& end_of_avoid_index)
{
  // reset
  avoid_waypoints.waypoints.clear();
  avoid_waypoints.header = base_waypoints_.header;
  avoid_waypoints.increment = base_waypoints_.increment;

  // add waypoints before start index
  for (int i = 0; i < closest_waypoint_index_; ++i)
  {
    avoid_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
  }

  // set waypoints for avoiding
  for (const auto& pose : path.poses)
  {
    autoware_msgs::waypoint wp;
    wp.pose.header = avoid_waypoints.header;
    wp.pose.pose = transformPose(pose.pose, getTransform(avoid_waypoints.header.frame_id, pose.header.frame_id));
    wp.twist.twist.linear.x = avoid_waypoints_velocity_ / 3.6;
    avoid_waypoints.waypoints.push_back(wp);
  }

  // end of avoiding waypoints
  end_of_avoid_index = getClosestWaypoint(avoid_waypoints, current_pose_global_.pose) + path.poses.size();

  // add waypoints after goal index
  for (int i = 0; goal_waypoint_index_ + i < static_cast<int>(base_waypoints_.waypoints.size()); ++i)
  {
    int index = goal_waypoint_index_ + i;
    avoid_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(index));
  }
}
