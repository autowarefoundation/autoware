// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "planning_error_monitor/planning_error_monitor_node.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <utility>

namespace planning_diagnostics
{
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_autoware_utils::calcCurvature;
using tier4_autoware_utils::calcDistance2d;

PlanningErrorMonitorNode::PlanningErrorMonitorNode(const rclcpp::NodeOptions & node_options)
: Node("planning_error_monitor", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  debug_marker_.initialize(this);

  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&PlanningErrorMonitorNode::onCurrentTrajectory, this, _1));

  updater_.setHardwareID("planning_error_monitor");

  updater_.add(
    "trajectory_point_validation", this, &PlanningErrorMonitorNode::onTrajectoryPointValueChecker);
  updater_.add(
    "trajectory_interval_validation", this, &PlanningErrorMonitorNode::onTrajectoryIntervalChecker);
  updater_.add(
    "trajectory_curvature_validation", this,
    &PlanningErrorMonitorNode::onTrajectoryCurvatureChecker);
  updater_.add(
    "trajectory_relative_angle_validation", this,
    &PlanningErrorMonitorNode::onTrajectoryRelativeAngleChecker);

  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&PlanningErrorMonitorNode::onTimer, this));

  // Parameter
  error_interval_ = declare_parameter("error_interval", 100.0);
  error_curvature_ = declare_parameter("error_curvature", 1.0);
  error_sharp_angle_ = declare_parameter("error_sharp_angle", M_PI_4);
  ignore_too_close_points_ = declare_parameter("ignore_too_close_points", 0.05);
}

void PlanningErrorMonitorNode::onTimer()
{
  updater_.force_update();
  debug_marker_.publish();
}

void PlanningErrorMonitorNode::onCurrentTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_trajectory_ = msg;
}

void PlanningErrorMonitorNode::onTrajectoryPointValueChecker(DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) {
    stat.summary(DiagnosticStatus::OK, "No trajectory message was set.");
    return;
  }

  std::string error_msg;
  const auto diag_level = checkTrajectoryPointValue(*current_trajectory_, error_msg)
                            ? DiagnosticStatus::OK
                            : DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

void PlanningErrorMonitorNode::onTrajectoryIntervalChecker(DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) {
    stat.summary(DiagnosticStatus::OK, "No trajectory message was set.");
    return;
  }

  std::string error_msg;
  const auto diag_level =
    checkTrajectoryInterval(*current_trajectory_, error_interval_, error_msg, debug_marker_)
      ? DiagnosticStatus::OK
      : DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

void PlanningErrorMonitorNode::onTrajectoryCurvatureChecker(DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) {
    stat.summary(DiagnosticStatus::OK, "No trajectory message was set.");
    return;
  }

  std::string error_msg;
  const auto diag_level =
    checkTrajectoryCurvature(*current_trajectory_, error_curvature_, error_msg, debug_marker_)
      ? DiagnosticStatus::OK
      : DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

void PlanningErrorMonitorNode::onTrajectoryRelativeAngleChecker(DiagnosticStatusWrapper & stat)
{
  if (!current_trajectory_) {
    stat.summary(DiagnosticStatus::OK, "No trajectory message was set.");
    return;
  }

  std::string error_msg;
  const auto diag_level =
    checkTrajectoryRelativeAngle(
      *current_trajectory_, error_sharp_angle_, ignore_too_close_points_, error_msg, debug_marker_)
      ? DiagnosticStatus::OK
      : DiagnosticStatus::ERROR;
  stat.summary(diag_level, error_msg);
}

bool PlanningErrorMonitorNode::checkTrajectoryPointValue(
  const Trajectory & traj, std::string & error_msg)
{
  error_msg = "This Trajectory doesn't have any invalid values";
  for (const auto & p : traj.points) {
    if (!checkFinite(p)) {
      error_msg = "This trajectory has an infinite value";
      return false;
    }
  }
  return true;
}

bool PlanningErrorMonitorNode::checkFinite(const TrajectoryPoint & point)
{
  const auto & o = point.pose.orientation;
  const auto & p = point.pose.position;
  const auto & v = point.longitudinal_velocity_mps;
  const auto & w = point.heading_rate_rps;
  const auto & a = point.acceleration_mps2;

  const bool quat_result =
    std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) && std::isfinite(o.w);
  const bool p_result = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  const bool v_result = std::isfinite(v);
  const bool w_result = std::isfinite(w);
  const bool a_result = std::isfinite(a);

  return quat_result && p_result && v_result && w_result && a_result;
}

bool PlanningErrorMonitorNode::checkTrajectoryInterval(
  const Trajectory & traj, const double & interval_threshold, std::string & error_msg,
  PlanningErrorMonitorDebugNode & debug_marker)
{
  error_msg = "Trajectory Interval Length is within the expected range";
  debug_marker.clearPoseMarker("trajectory_interval");
  for (size_t i = 1; i < traj.points.size(); ++i) {
    double ds = calcDistance2d(traj.points.at(i), traj.points.at(i - 1));

    if (ds > interval_threshold) {
      error_msg = "Trajectory Interval Length is longer than the expected range";
      debug_marker.pushPoseMarker(traj.points.at(i - 1).pose, "trajectory_interval");
      debug_marker.pushPoseMarker(traj.points.at(i).pose, "trajectory_interval");
      return false;
    }
  }
  return true;
}

bool PlanningErrorMonitorNode::checkTrajectoryRelativeAngle(
  const Trajectory & traj, const double angle_threshold, const double min_dist_threshold,
  std::string & error_msg, PlanningErrorMonitorDebugNode & debug_marker)
{
  error_msg = "This trajectory's relative angle is within the expected range";
  debug_marker.clearPoseMarker("trajectory_relative_angle");

  // We need at least three points to compute relative angle
  const size_t relative_angle_points_num = 3;
  if (traj.points.size() < relative_angle_points_num) {
    return true;
  }

  for (size_t p1_id = 0; p1_id <= traj.points.size() - relative_angle_points_num; ++p1_id) {
    // Get Point1
    const auto & p1 = traj.points.at(p1_id).pose.position;

    // Get Point2
    const auto & p2 = traj.points.at(p1_id + 1).pose.position;

    // Get Point3
    const auto & p3 = traj.points.at(p1_id + 2).pose.position;

    // ignore invert driving direction
    if (
      traj.points.at(p1_id).longitudinal_velocity_mps < 0 ||
      traj.points.at(p1_id + 1).longitudinal_velocity_mps < 0 ||
      traj.points.at(p1_id + 2).longitudinal_velocity_mps < 0) {
      continue;
    }

    // convert to p1 coordinate
    const double x3 = p3.x - p1.x;
    const double x2 = p2.x - p1.x;
    const double y3 = p3.y - p1.y;
    const double y2 = p2.y - p1.y;

    // skip too close points case
    if (std::hypot(x3, y3) < min_dist_threshold || std::hypot(x2, y2) < min_dist_threshold) {
      continue;
    }

    // calculate relative angle of vector p3 based on p1p2 vector
    const double th = std::atan2(y2, x2);
    const double th2 =
      std::atan2(-x3 * std::sin(th) + y3 * std::cos(th), x3 * std::cos(th) + y3 * std::sin(th));
    if (std::abs(th2) > angle_threshold) {
      error_msg = "This Trajectory's relative angle has larger value than the expected value";
      // std::cout << error_msg << std::endl;
      debug_marker.pushPoseMarker(traj.points.at(p1_id).pose, "trajectory_relative_angle", 0);
      debug_marker.pushPoseMarker(traj.points.at(p1_id + 1).pose, "trajectory_relative_angle", 1);
      debug_marker.pushPoseMarker(traj.points.at(p1_id + 2).pose, "trajectory_relative_angle", 2);
      return false;
    }
  }
  return true;
}
bool PlanningErrorMonitorNode::checkTrajectoryCurvature(
  const Trajectory & traj, const double & curvature_threshold, std::string & error_msg,
  PlanningErrorMonitorDebugNode & debug_marker)
{
  error_msg = "This trajectory's curvature is within the expected range";
  debug_marker.clearPoseMarker("trajectory_curvature");

  // We need at least three points to compute curvature
  if (traj.points.size() < 3) {
    return true;
  }

  constexpr double points_distance = 1.0;
  const auto isValidDistance = [points_distance](const auto & p1, const auto & p2) {
    return calcDistance2d(p1, p2) >= points_distance;
  };

  for (size_t p1_id = 0; p1_id < traj.points.size() - 2; ++p1_id) {
    // Get Point1
    const auto p1 = traj.points.at(p1_id).pose.position;

    // Get Point2
    const auto p2_id = getIndexAfterDistance(traj, p1_id, points_distance);
    const auto p2 = traj.points.at(p2_id).pose.position;

    // Get Point3
    const auto p3_id = getIndexAfterDistance(traj, p2_id, points_distance);
    const auto p3 = traj.points.at(p3_id).pose.position;

    // no need to check for pi, since there is no point with "points_distance" from p1.
    if (p1_id == p2_id || p1_id == p3_id || p2_id == p3_id) {
      break;
    }
    if (!isValidDistance(p1, p2) || !isValidDistance(p1, p3) || !isValidDistance(p2, p3)) {
      break;
    }

    const double curvature = calcCurvature(p1, p2, p3);

    if (std::fabs(curvature) > curvature_threshold) {
      error_msg = "This Trajectory's curvature has larger value than the expected value";
      debug_marker.pushPoseMarker(traj.points.at(p1_id).pose, "trajectory_curvature");
      debug_marker.pushPoseMarker(traj.points.at(p2_id).pose, "trajectory_curvature");
      debug_marker.pushPoseMarker(traj.points.at(p3_id).pose, "trajectory_curvature");
      return false;
    }
  }
  return true;
}

size_t PlanningErrorMonitorNode::getIndexAfterDistance(
  const Trajectory & traj, const size_t curr_id, const double distance)
{
  // Get Current Trajectory Point
  const TrajectoryPoint & curr_p = traj.points.at(curr_id);

  size_t target_id = curr_id;
  double current_distance = 0.0;
  for (size_t traj_id = curr_id + 1; traj_id < traj.points.size(); ++traj_id) {
    current_distance = calcDistance2d(traj.points.at(traj_id), curr_p);
    if (current_distance >= distance) {
      target_id = traj_id;
      break;
    }
  }

  return target_id;
}
}  // namespace planning_diagnostics

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(planning_diagnostics::PlanningErrorMonitorNode)
