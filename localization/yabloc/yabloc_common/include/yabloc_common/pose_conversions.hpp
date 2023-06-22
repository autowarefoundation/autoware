// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_COMMON__POSE_CONVERSIONS_HPP_
#define YABLOC_COMMON__POSE_CONVERSIONS_HPP_

#include <eigen3/Eigen/Geometry>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace yabloc::common
{
Eigen::Affine3f pose_to_affine(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose affine_to_pose(const Eigen::Affine3f & affine);

Sophus::SE3f pose_to_se3(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Pose se3_to_pose(const Sophus::SE3f & se3f);
}  // namespace yabloc::common

#endif  // YABLOC_COMMON__POSE_CONVERSIONS_HPP_
