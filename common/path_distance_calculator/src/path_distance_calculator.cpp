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

#include <autoware_utils/autoware_utils.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

PathDistanceCalculator::PathDistanceCalculator(const rclcpp::NodeOptions & options)
: Node("path_distance_calculator", options), self_pose_listener_(this)
{
  sub_path_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS(1),
    [this](const autoware_auto_planning_msgs::msg::Path::SharedPtr msg) { path_ = msg; });
  pub_dist_ =
    create_publisher<autoware_debug_msgs::msg::Float64Stamped>("~/output/distance", rclcpp::QoS(1));

  auto timer_callback = [this]() {
    const auto path = path_;
    const auto pose = self_pose_listener_.getCurrentPose();
    if (!pose) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no pose");
      return;
    }
    if (!path) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no path");
      return;
    }
    if (path->points.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "path empty");
    }

    const double distance = autoware_utils::calcSignedArcLength(
      path->points, pose->pose.position, path->points.size() - 1);

    autoware_debug_msgs::msg::Float64Stamped msg;
    msg.stamp = pose->header.stamp;
    msg.data = distance;
    pub_dist_->publish(msg);
  };

  using namespace std::literals::chrono_literals;
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), 1s, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PathDistanceCalculator)
