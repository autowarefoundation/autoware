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

#ifndef CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
#define CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace control_diagnostics
{

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @brief Node for control evaluation
 */
class controlEvaluatorNode : public rclcpp::Node
{
public:
  explicit controlEvaluatorNode(const rclcpp::NodeOptions & node_options);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(const bool is_emergency_brake) const;
  void onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg);
  void onTimer();

private:
  rclcpp::Subscription<DiagnosticArray>::SharedPtr control_diag_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;

  // Calculator
  // Metrics
  std::deque<rclcpp::Time> stamps_;
  DiagnosticArray metrics_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace control_diagnostics

#endif  // CONTROL_EVALUATOR__CONTROL_EVALUATOR_NODE_HPP_
