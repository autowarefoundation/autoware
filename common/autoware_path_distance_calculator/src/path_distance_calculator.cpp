// Copyright 2021 Tier IV, Inc.
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

#include "path_distance_calculator.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::path_distance_calculator
{
PathDistanceCalculator::PathDistanceCalculator(const rclcpp::NodeOptions & options)
: Node("path_distance_calculator", options), self_pose_listener_(this)
{
  pub_dist_ =
    create_publisher<tier4_debug_msgs::msg::Float64Stamped>("~/output/distance", rclcpp::QoS(1));

  using std::chrono_literals::operator""s;
  timer_ = rclcpp::create_timer(this, get_clock(), 1s, [this]() {
    const auto path = sub_path_.takeData();
    const auto pose = self_pose_listener_.getCurrentPose();
    if (!pose) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no pose");
      return;
    }
    if (!path) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no path");
      return;
    }
    if (path->points.size() <= 1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "too short or empty path");
      return;
    }

    const double distance = autoware::motion_utils::calcSignedArcLength(
      path->points, pose->pose.position, path->points.size() - 1);

    tier4_debug_msgs::msg::Float64Stamped msg;
    msg.stamp = pose->header.stamp;
    msg.data = distance;
    pub_dist_->publish(msg);
  });
}
}  // namespace autoware::path_distance_calculator
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::path_distance_calculator::PathDistanceCalculator)
