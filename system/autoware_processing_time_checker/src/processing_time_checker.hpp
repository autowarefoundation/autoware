// Copyright 2024 TIER IV, Inc.
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

#ifndef PROCESSING_TIME_CHECKER_HPP_
#define PROCESSING_TIME_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "tier4_debug_msgs/msg/float64_stamped.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::processing_time_checker
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_debug_msgs::msg::Float64Stamped;

class ProcessingTimeChecker : public rclcpp::Node
{
public:
  explicit ProcessingTimeChecker(const rclcpp::NodeOptions & node_options);

private:
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<DiagnosticArray>::SharedPtr diag_pub_;
  std::vector<rclcpp::Subscription<Float64Stamped>::SharedPtr> processing_time_subscribers_;

  // topic name - module name
  std::unordered_map<std::string, std::string> module_name_map_{};
  // module name - processing time
  std::unordered_map<std::string, double> processing_time_map_{};
};
}  // namespace autoware::processing_time_checker

#endif  // PROCESSING_TIME_CHECKER_HPP_
