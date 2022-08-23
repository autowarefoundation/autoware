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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/normalization.hpp"

#include <tf2/utils.h>

namespace tier4_autoware_utils
{
struct PoseDeviation
{
  double lateral{0.0};
  double longitudinal{0.0};
  double yaw{0.0};
};

inline double calcLateralDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  const Eigen::Vector3d cross_vec = base_unit_vec.cross(diff_vec);

  return cross_vec.z();
}

inline double calcLongitudinalDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = base_pose.position;

  const auto yaw = tf2::getYaw(base_pose.orientation);
  const Eigen::Vector3d base_unit_vec{std::cos(yaw), std::sin(yaw), 0};

  const auto dx = target_point.x - base_point.x;
  const auto dy = target_point.y - base_point.y;
  const Eigen::Vector3d diff_vec{dx, dy, 0};

  return base_unit_vec.dot(diff_vec);
}

inline double calcYawDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  const auto base_yaw = tf2::getYaw(base_pose.orientation);
  const auto target_yaw = tf2::getYaw(target_pose.orientation);
  return normalizeRadian(target_yaw - base_yaw);
}

inline PoseDeviation calcPoseDeviation(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & target_pose)
{
  PoseDeviation deviation{};

  deviation.lateral = calcLateralDeviation(base_pose, target_pose.position);
  deviation.longitudinal = calcLongitudinalDeviation(base_pose, target_pose.position);
  deviation.yaw = calcYawDeviation(base_pose, target_pose);

  return deviation;
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__POSE_DEVIATION_HPP_
