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

#include "switch_rule/vector_map_based_rule.hpp"

#include <magic_enum.hpp>

#include <utility>

namespace autoware::pose_estimator_arbiter::switch_rule
{
VectorMapBasedRule::VectorMapBasedRule(
  rclcpp::Node & node, std::unordered_set<PoseEstimatorType> running_estimator_list,
  std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node),
  running_estimator_list_(std::move(running_estimator_list)),
  shared_data_(std::move(shared_data))
{
  pose_estimator_area_ = std::make_unique<rule_helper::PoseEstimatorArea>(node.get_logger());

  // Register callback
  shared_data_->vector_map.register_callback(
    [this](autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg) -> void {
      pose_estimator_area_->init(msg);
    });

  RCLCPP_INFO_STREAM(get_logger(), "VectorMapBasedRule is initialized successfully");
}

VectorMapBasedRule::MarkerArray VectorMapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  if (pose_estimator_area_) {
    const auto & additional = pose_estimator_area_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  return array_msg;
}

std::unordered_map<PoseEstimatorType, bool> VectorMapBasedRule::update()
{
  const auto ego_position = shared_data_->localization_pose_cov()->pose.pose.position;

  std::unordered_map<PoseEstimatorType, bool> enable_list;
  bool at_least_one_is_enabled = false;
  for (const auto & estimator_type : running_estimator_list_) {
    const std::string estimator_name{magic_enum::enum_name(estimator_type)};
    const bool result = pose_estimator_area_->within(ego_position, estimator_name);
    enable_list.emplace(estimator_type, result);

    at_least_one_is_enabled |= result;
  }
  if (at_least_one_is_enabled) {
    debug_string_ =
      "Enable at least one pose_estimators: self vehicle is within the area of at least one "
      "pose_estimator_area";
  } else {
    debug_string_ =
      "Enable no pose_estimator: self vehicle is out of the area of all pose_estimator_area";
  }
  RCLCPP_DEBUG(get_logger(), "%s", debug_string_.c_str());

  return enable_list;
}

}  // namespace autoware::pose_estimator_arbiter::switch_rule
