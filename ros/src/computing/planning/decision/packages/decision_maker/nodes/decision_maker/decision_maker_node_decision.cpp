#include <stdio.h>
#include <numeric>
#include <numeric>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <autoware_msgs/Lane.h>
#include <autoware_msgs/TrafficLight.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

namespace decision_maker
{
/* do not use this within callback */
bool DecisionMakerNode::waitForEvent(cstring_t& key, const bool& flag)
{
  const uint32_t monitoring_rate = 20;  // Hz

  ros::Rate loop_rate(monitoring_rate);
  while (ros::ok())
  {
    if (isEventFlagTrue(key) == flag)
    {
      break;
    }
    loop_rate.sleep();
  }
  return true;
}

bool DecisionMakerNode::waitForEvent(cstring_t& key, const bool& flag, const double& timeout_sec)
{
  const uint32_t monitoring_rate = 20;  // Hz
  ros::Rate loop_rate(monitoring_rate);

  ros::Time entry_time = ros::Time::now();

  while (ros::ok())
  {
    if (isEventFlagTrue(key) == flag)
    {
      return true;
    }
    if ((ros::Time::now() - entry_time).toSec() >= timeout_sec)
    {
      break;
    }
    loop_rate.sleep();
  }
  return false;
}
double DecisionMakerNode::calcIntersectWayAngle(const autoware_msgs::Lane& laneinArea)
{
  double diff = 0.0;
  if (laneinArea.waypoints.empty())
  {
    ROS_INFO("Not inside CrossRoad");
  }
  else
  {
    const geometry_msgs::Pose InPose = laneinArea.waypoints.front().pose.pose;
    const geometry_msgs::Pose OutPose = laneinArea.waypoints.back().pose.pose;

    diff = amathutils::calcPosesAngleDiffDeg(InPose, OutPose);
  }

  return diff;
}

double DecisionMakerNode::getDistToWaypointIdx(const int wpidx) const
{
  double distance = 0.0;
  geometry_msgs::Pose prev_pose = current_status_.pose;

  for (unsigned int idx = 0; idx < current_status_.finalwaypoints.waypoints.size() - 1; idx++)
  {
    distance += amathutils::find_distance(prev_pose, current_status_.finalwaypoints.waypoints.at(idx).pose.pose);

    if (current_status_.finalwaypoints.waypoints.at(idx).gid == wpidx)
    {
      break;
    }

    prev_pose = current_status_.finalwaypoints.waypoints.at(idx).pose.pose;
  }

  return distance;
}

double DecisionMakerNode::calcRequiredDistForStop(void) const
{
  static const double mu = 0.7;  // dry ground/ asphalt/ normal tire
  static const double g = 9.80665;
  static const double margin = 5;
  static const double reaction_time = 0.3 + margin;  // system delay(sec)
  const double velocity = amathutils::kmph2mps(current_status_.velocity);

  const double free_running_distance = reaction_time * velocity;
  const double braking_distance = velocity * velocity / (2 * g * mu);
  const double distance_to_target = (free_running_distance + braking_distance) * 2 /* safety margin*/;

  return distance_to_target;
}

bool DecisionMakerNode::isLocalizationConvergence(const geometry_msgs::Point& _current_point) const
{
  static std::vector<double> distances;
  static uint32_t distances_count = 0;
  static geometry_msgs::Point prev_point;
  static const int param_convergence_count = 10;

  bool ret = false;

  // if current point is not set, localization is failure
  if (_current_point.x == 0 && _current_point.y == 0 && _current_point.z == 0 && prev_point.x == 0 &&
      prev_point.y == 0 && prev_point.z == 0)
  {
    return ret;
  }

  distances.push_back(amathutils::find_distance(prev_point, _current_point));
  if (++distances_count > param_convergence_count) /* num of count to judge convergence*/
  {
    distances.erase(distances.begin());
    distances_count--;
    double avg_distances = std::accumulate(distances.begin(), distances.end(), 0.0) / (double)distances.size();
    if (avg_distances <= 2) /*meter*/
    {
      ret = true;
    }
  }

  prev_point = _current_point;
  return ret;
}
bool DecisionMakerNode::isArrivedGoal() const
{
  const auto goal_point = current_status_.finalwaypoints.waypoints.back().pose.pose.position;

  if (amathutils::find_distance(goal_point, current_status_.pose.position) < goal_threshold_dist_
      && fabs(current_status_.velocity) <= goal_threshold_vel_)
  {
    return true;
  }

  return false;
}
}
