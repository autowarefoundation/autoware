#include <numeric>
#include <stdio.h>
#include <numeric>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>

#include <cross_road_area.hpp>
#include <decision_maker_node.hpp>
#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

namespace decision_maker
{
double DecisionMakerNode::getPoseAngle(const geometry_msgs::Pose &pose)
{
  double r, p, y;

  tf::Quaternion quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);

  // convert to [-pi : pi]
  return y;
}

double DecisionMakerNode::calcPosesAngleDiffN(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &p_to)
{
  // convert to [-pi : pi]
  return getPoseAngle(p_from) - getPoseAngle(p_to);
}

double DecisionMakerNode::calcPosesAngleDiff(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &p_to)
{
  // convert to [-pi : pi]
  double diff = std::fmod(calcPosesAngleDiffN(p_from, p_to), 2 * M_PI);
  diff = diff > M_PI ? diff - 2 * M_PI : diff < -M_PI ? 2 * M_PI + diff : diff;
  diff = diff * 180 / M_PI;
  return diff;
}

double DecisionMakerNode::calcIntersectWayAngle(const autoware_msgs::lane &laneinArea)
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

    diff = calcPosesAngleDiff(InPose, OutPose);
  }

  return diff;
}

bool DecisionMakerNode::isLocalizationConvergence(double _x, double _y, double _z, double _roll, double _pitch,
                                                  double _yaw)
{
  static amathutils::point a;
  static amathutils::point b;

  static std::vector<double> distances;
  static uint32_t distances_count = 0;

  bool ret = false;

  a.x = b.x;
  a.y = b.y;
  a.z = b.z;

  b.x = _x;
  b.y = _y;
  b.z = _z;

  distances.push_back(amathutils::find_distance(&a, &b));
  if (++distances_count > param_convergence_count_)
  {
    distances.erase(distances.begin());
    distances_count--;
    double avg_distances = std::accumulate(distances.begin(), distances.end(), 0) / distances.size();
    if (avg_distances <= param_convergence_threshold_)
    {
      ret =  ctx->setCurrentState(state_machine::DRIVE_STATE);
    }
  }
  
  return ret;
}
}
