// Copyright 2020 Tier IV, Inc.
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

#ifndef TIER4_AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/transform_listener.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace tier4_autoware_utils
{
class SelfPoseListener
{
public:
  explicit SelfPoseListener(rclcpp::Node * node) : transform_listener_(node) {}

  void waitForFirstPose()
  {
    while (rclcpp::ok()) {
      if (getCurrentPose()) {
        return;
      }
      RCLCPP_INFO(transform_listener_.getLogger(), "waiting for self pose...");
      rclcpp::Rate(0.2).sleep();
    }
  }

  geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose()
  {
    const auto tf = transform_listener_.getLatestTransform("map", "base_link");
    if (!tf) {
      return {};
    }

    return std::make_shared<const geometry_msgs::msg::PoseStamped>(transform2pose(*tf));
  }

private:
  TransformListener transform_listener_;
};
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__ROS__SELF_POSE_LISTENER_HPP_
