// Copyright 2023 Autoware Foundation
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

#ifndef POSE_ESTIMATOR_ARBITER__SWITCH_RULE__BASE_SWITCH_RULE_HPP_
#define POSE_ESTIMATOR_ARBITER__SWITCH_RULE__BASE_SWITCH_RULE_HPP_

#include "pose_estimator_arbiter/pose_estimator_type.hpp"

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace pose_estimator_arbiter::switch_rule
{
class BaseSwitchRule
{
protected:
  using MarkerArray = visualization_msgs::msg::MarkerArray;

public:
  explicit BaseSwitchRule(rclcpp::Node & node)
  : logger_ptr_(std::make_shared<rclcpp::Logger>(node.get_logger()))
  {
  }

  virtual ~BaseSwitchRule() = default;
  virtual std::unordered_map<PoseEstimatorType, bool> update() = 0;
  virtual std::string debug_string() { return std::string{}; }
  virtual MarkerArray debug_marker_array() { return MarkerArray{}; }

protected:
  rclcpp::Logger get_logger() const { return *logger_ptr_; }
  std::shared_ptr<rclcpp::Logger> logger_ptr_{nullptr};
};

}  // namespace pose_estimator_arbiter::switch_rule

#endif  // POSE_ESTIMATOR_ARBITER__SWITCH_RULE__BASE_SWITCH_RULE_HPP_
