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

#ifndef PLANNING_ERROR_MONITOR__PLANNING_ERROR_MONITOR_NODE_HPP_
#define PLANNING_ERROR_MONITOR__PLANNING_ERROR_MONITOR_NODE_HPP_

#include "planning_error_monitor/debug_marker.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <string>

namespace planning_diagnostics
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

class PlanningErrorMonitorNode : public rclcpp::Node
{
public:
  explicit PlanningErrorMonitorNode(const rclcpp::NodeOptions & node_options);

  void onCurrentTrajectory(const Trajectory::ConstSharedPtr msg);
  void onTimer();

  void onTrajectoryPointValueChecker(DiagnosticStatusWrapper & stat);
  void onTrajectoryIntervalChecker(DiagnosticStatusWrapper & stat);
  void onTrajectoryCurvatureChecker(DiagnosticStatusWrapper & stat);
  void onTrajectoryRelativeAngleChecker(DiagnosticStatusWrapper & stat);
  static bool checkTrajectoryRelativeAngle(
    const Trajectory & traj, const double relative_angle_threshold, const double min_dist_threshold,
    std::string & error_msg, PlanningErrorMonitorDebugNode & debug_marker);

  static bool checkTrajectoryPointValue(const Trajectory & traj, std::string & error_msg);
  static bool checkTrajectoryInterval(
    const Trajectory & traj, const double & interval_threshold, std::string & error_msg,
    PlanningErrorMonitorDebugNode & debug_marker);
  static bool checkTrajectoryCurvature(
    const Trajectory & traj, const double & curvature_threshold, std::string & error_msg,
    PlanningErrorMonitorDebugNode & debug_marker);

private:
  static bool checkFinite(const TrajectoryPoint & p);
  static size_t getIndexAfterDistance(
    const Trajectory & traj, const size_t curr_id, const double distance);

  // ROS
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Updater updater_{this};

  Trajectory::ConstSharedPtr current_trajectory_;

  // Parameter
  double error_interval_;
  double error_curvature_;
  double error_sharp_angle_;
  double ignore_too_close_points_;

  PlanningErrorMonitorDebugNode debug_marker_;
};
}  // namespace planning_diagnostics

#endif  // PLANNING_ERROR_MONITOR__PLANNING_ERROR_MONITOR_NODE_HPP_
