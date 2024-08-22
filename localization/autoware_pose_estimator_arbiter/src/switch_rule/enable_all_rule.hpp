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

#ifndef SWITCH_RULE__ENABLE_ALL_RULE_HPP_
#define SWITCH_RULE__ENABLE_ALL_RULE_HPP_

#include "shared_data.hpp"
#include "switch_rule/base_switch_rule.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace autoware::pose_estimator_arbiter::switch_rule
{
class EnableAllRule : public BaseSwitchRule
{
public:
  EnableAllRule(
    rclcpp::Node & node, std::unordered_set<PoseEstimatorType> running_estimator_list,
    const std::shared_ptr<const SharedData> & shared_data);

  std::unordered_map<PoseEstimatorType, bool> update() override;

protected:
  const std::unordered_set<PoseEstimatorType> running_estimator_list_;
};

}  // namespace autoware::pose_estimator_arbiter::switch_rule

#endif  // SWITCH_RULE__ENABLE_ALL_RULE_HPP_
