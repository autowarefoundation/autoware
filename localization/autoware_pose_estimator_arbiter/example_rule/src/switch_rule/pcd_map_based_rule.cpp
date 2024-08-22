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

#include "switch_rule/pcd_map_based_rule.hpp"

#include <autoware/universe_utils/ros/parameter.hpp>

#include <utility>

namespace autoware::pose_estimator_arbiter::switch_rule
{
PcdMapBasedRule::PcdMapBasedRule(
  rclcpp::Node & node, std::unordered_set<PoseEstimatorType> running_estimator_list,
  std::shared_ptr<const SharedData> shared_data)
: BaseSwitchRule(node),
  running_estimator_list_(std::move(running_estimator_list)),
  shared_data_(std::move(shared_data))
{
  const int pcd_density_upper_threshold =
    autoware::universe_utils::getOrDeclareParameter<int>(node, "pcd_density_upper_threshold");
  const int pcd_density_lower_threshold =
    autoware::universe_utils::getOrDeclareParameter<int>(node, "pcd_density_lower_threshold");

  pcd_occupancy_ = std::make_unique<rule_helper::PcdOccupancy>(
    pcd_density_upper_threshold, pcd_density_lower_threshold);

  // Register callback
  shared_data_->point_cloud_map.register_callback(
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      pcd_occupancy_->init(msg);
    });
}

PcdMapBasedRule::MarkerArray PcdMapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;

  if (pcd_occupancy_) {
    const auto & additional = pcd_occupancy_->debug_marker_array().markers;
    array_msg.markers.insert(array_msg.markers.end(), additional.begin(), additional.end());
  }

  return array_msg;
}

std::unordered_map<PoseEstimatorType, bool> PcdMapBasedRule::update()
{
  const auto position = shared_data_->localization_pose_cov()->pose.pose.position;
  const bool ndt_can_operate = pcd_occupancy_->ndt_can_operate(position);

  if (ndt_can_operate) {
    debug_string_ = "Enable ndt: ";
    return {
      {PoseEstimatorType::ndt, true},
      {PoseEstimatorType::yabloc, false},
      {PoseEstimatorType::eagleye, false},
      {PoseEstimatorType::artag, false},
    };
  }

  debug_string_ = "Enable yabloc: ";
  return {
    {PoseEstimatorType::ndt, false},
    {PoseEstimatorType::yabloc, true},
    {PoseEstimatorType::eagleye, false},
    {PoseEstimatorType::artag, false}};
}

}  // namespace autoware::pose_estimator_arbiter::switch_rule
