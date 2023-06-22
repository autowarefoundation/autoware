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

#include "yabloc_common/pose_conversions.hpp"

#include <eigen3/Eigen/Geometry>

namespace yabloc::common
{
geometry_msgs::msg::Pose affine_to_pose(const Eigen::Affine3f & affine)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3f pos = affine.translation();
  Eigen::Quaternionf ori(affine.rotation());
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.w = ori.w();
  pose.orientation.x = ori.x();
  pose.orientation.y = ori.y();
  pose.orientation.z = ori.z();
  return pose;
}

Eigen::Affine3f pose_to_affine(const geometry_msgs::msg::Pose & pose)
{
  const auto pos = pose.position;
  const auto ori = pose.orientation;
  Eigen::Translation3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return t * q;
}

Sophus::SE3f pose_to_se3(const geometry_msgs::msg::Pose & pose)
{
  auto ori = pose.orientation;
  auto pos = pose.position;
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  Eigen::Vector3f t(pos.x, pos.y, pos.z);
  return {q, t};
}

geometry_msgs::msg::Pose se3_to_pose(const Sophus::SE3f & se3f)
{
  geometry_msgs::msg::Pose pose;
  Eigen::Vector3f pos = se3f.translation();
  Eigen::Quaternion ori = se3f.so3().unit_quaternion();
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.w = ori.w();
  pose.orientation.x = ori.x();
  pose.orientation.y = ori.y();
  pose.orientation.z = ori.z();
  return pose;
}
}  // namespace yabloc::common
