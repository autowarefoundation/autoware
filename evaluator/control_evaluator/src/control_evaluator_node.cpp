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

#include "control_evaluator/control_evaluator_node.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_diagnostics
{
controlEvaluatorNode::controlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options)
{
  using std::placeholders::_1;

  control_diag_sub_ = create_subscription<DiagnosticArray>(
    "~/input/diagnostics", 1, std::bind(&controlEvaluatorNode::onDiagnostics, this, _1));

  // Publisher
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&controlEvaluatorNode::onTimer, this));
}

DiagnosticStatus controlEvaluatorNode::generateDiagnosticStatus(const bool is_emergency_brake) const
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "autonomous_emergency_braking";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "decision";
  key_value.value = (is_emergency_brake) ? "stop" : "none";
  status.values.push_back(key_value);
  return status;
}

void controlEvaluatorNode::onTimer()
{
  if (!metrics_msg_.status.empty()) {
    metrics_pub_->publish(metrics_msg_);
    metrics_msg_.status.clear();
  }
}

void controlEvaluatorNode::onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg)
{
  const auto start = now();
  const auto aeb_status =
    std::find_if(diag_msg->status.begin(), diag_msg->status.end(), [](const auto & status) {
      const bool aeb_found = status.name.find("autonomous_emergency_braking") != std::string::npos;
      return aeb_found;
    });

  if (aeb_status == diag_msg->status.end()) return;

  const bool is_emergency_brake = (aeb_status->level == DiagnosticStatus::ERROR);
  metrics_msg_.header.stamp = now();
  metrics_msg_.status.emplace_back(generateDiagnosticStatus(is_emergency_brake));

  const auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(get_logger(), "control evaluation calculation time: %2.2f ms", runtime * 1e3);
}

}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::controlEvaluatorNode)
