// Copyright 2023 The Autoware Contributors
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

#ifndef CORE__MODES_HPP_
#define CORE__MODES_HPP_

#include "types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/operation_mode_availability.hpp>

#include <vector>

namespace system_diagnostic_graph
{

class OperationModes
{
public:
  explicit OperationModes(rclcpp::Node & node, const std::vector<BaseNode *> & graph);
  void update(const rclcpp::Time & stamp) const;

private:
  using Availability = tier4_system_msgs::msg::OperationModeAvailability;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Availability>::SharedPtr pub_;

  BaseNode * stop_mode_;
  BaseNode * autonomous_mode_;
  BaseNode * local_mode_;
  BaseNode * remote_mode_;
  BaseNode * emergency_stop_mrm_;
  BaseNode * comfortable_stop_mrm_;
  BaseNode * pull_over_mrm_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__MODES_HPP_
