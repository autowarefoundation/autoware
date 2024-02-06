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

#ifndef POSE_ESTIMATOR_ARBITER__SWITCH_RULE__PCD_MAP_BASED_RULE_HPP_
#define POSE_ESTIMATOR_ARBITER__SWITCH_RULE__PCD_MAP_BASED_RULE_HPP_

#include "pose_estimator_arbiter/pose_estimator_type.hpp"
#include "pose_estimator_arbiter/rule_helper/pcd_occupancy.hpp"
#include "pose_estimator_arbiter/shared_data.hpp"
#include "pose_estimator_arbiter/switch_rule/base_switch_rule.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace pose_estimator_arbiter::switch_rule
{
class PcdMapBasedRule : public BaseSwitchRule
{
public:
  PcdMapBasedRule(
    rclcpp::Node & node, const std::unordered_set<PoseEstimatorType> & running_estimator_list,
    const std::shared_ptr<const SharedData> shared_data);

  std::unordered_map<PoseEstimatorType, bool> update() override;

  std::string debug_string() override { return debug_string_; }

  MarkerArray debug_marker_array() override;

protected:
  const std::unordered_set<PoseEstimatorType> running_estimator_list_;
  std::shared_ptr<const SharedData> shared_data_{nullptr};

  std::unique_ptr<rule_helper::PcdOccupancy> pcd_occupancy_{nullptr};

  // Store the reason why which pose estimator is enabled
  mutable std::string debug_string_;
};
}  // namespace pose_estimator_arbiter::switch_rule

#endif  // POSE_ESTIMATOR_ARBITER__SWITCH_RULE__PCD_MAP_BASED_RULE_HPP_
