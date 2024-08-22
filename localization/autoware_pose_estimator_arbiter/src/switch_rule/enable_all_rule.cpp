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

#include "switch_rule/enable_all_rule.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::pose_estimator_arbiter::switch_rule
{
EnableAllRule::EnableAllRule(
  rclcpp::Node & node, std::unordered_set<PoseEstimatorType> running_estimator_list,
  const std::shared_ptr<const SharedData> &)
: BaseSwitchRule(node), running_estimator_list_(std::move(running_estimator_list))
{
}

std::unordered_map<PoseEstimatorType, bool> EnableAllRule::update()
{
  return {
    {PoseEstimatorType::ndt, true},
    {PoseEstimatorType::yabloc, true},
    {PoseEstimatorType::eagleye, true},
    {PoseEstimatorType::artag, true},
  };
}

}  // namespace autoware::pose_estimator_arbiter::switch_rule
