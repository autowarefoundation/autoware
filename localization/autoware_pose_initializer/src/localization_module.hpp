// Copyright 2024 The Autoware Contributors
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

#ifndef LOCALIZATION_MODULE_HPP_
#define LOCALIZATION_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>

#include <string>
#include <tuple>

namespace autoware::pose_initializer
{
class LocalizationModule
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using RequestPoseAlignment = tier4_localization_msgs::srv::PoseWithCovarianceStamped;

public:
  LocalizationModule(rclcpp::Node * node, const std::string & service_name);
  std::tuple<PoseWithCovarianceStamped, bool> align_pose(const PoseWithCovarianceStamped & pose);

private:
  rclcpp::Logger logger_;
  rclcpp::Client<RequestPoseAlignment>::SharedPtr cli_align_;
};
}  // namespace autoware::pose_initializer

#endif  // LOCALIZATION_MODULE_HPP_
