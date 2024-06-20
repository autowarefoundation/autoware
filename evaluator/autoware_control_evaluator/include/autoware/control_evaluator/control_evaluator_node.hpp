// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
#define AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_

#include "autoware/control_evaluator/metrics/deviation_metrics.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_diagnostics
{

using autoware_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

/**
 * @brief Node for control evaluation
 */
class controlEvaluatorNode : public rclcpp::Node
{
public:
  explicit controlEvaluatorNode(const rclcpp::NodeOptions & node_options);
  void removeOldDiagnostics(const rclcpp::Time & stamp);
  void removeDiagnosticsByName(const std::string & name);
  void addDiagnostic(const DiagnosticStatus & diag, const rclcpp::Time & stamp);
  void updateDiagnosticQueue(
    const DiagnosticArray & input_diagnostics, const std::string & function,
    const rclcpp::Time & stamp);

  DiagnosticStatus generateLateralDeviationDiagnosticStatus(
    const Trajectory & traj, const Point & ego_point);
  DiagnosticStatus generateYawDeviationDiagnosticStatus(
    const Trajectory & traj, const Pose & ego_pose);
  std::optional<DiagnosticStatus> generateStopDiagnosticStatus(
    const DiagnosticArray & diag, const std::string & function_name);

  DiagnosticStatus generateAEBDiagnosticStatus(const DiagnosticStatus & diag);

  void onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg);
  void onTimer();

private:
  // The diagnostics cycle is faster than timer, and each node publishes diagnostic separately.
  // takeData() in onTimer() with a polling subscriber will miss a topic, so save all topics with
  // onDiagnostics().
  rclcpp::Subscription<DiagnosticArray>::SharedPtr control_diag_sub_;

  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> odometry_sub_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> traj_sub_{
    this, "~/input/trajectory"};

  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;

  // Calculator
  // Metrics
  std::deque<rclcpp::Time> stamps_;

  // queue for diagnostics and time stamp
  std::deque<std::pair<DiagnosticStatus, rclcpp::Time>> diag_queue_;
  const std::vector<std::string> target_functions_ = {"autonomous_emergency_braking"};

  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace control_diagnostics

#endif  // AUTOWARE__CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
