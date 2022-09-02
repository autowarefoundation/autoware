// Copyright 2022 Tier IV, Inc.
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

#ifndef INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
#define INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_

#include "interpolation/interpolation_utils.hpp"

#include <geometry_msgs/msg/quaternion.hpp>

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>

namespace interpolation
{
geometry_msgs::msg::Quaternion slerp(
  const geometry_msgs::msg::Quaternion & src_quat, const geometry_msgs::msg::Quaternion & dst_quat,
  const double ratio);

std::vector<geometry_msgs::msg::Quaternion> slerp(
  const std::vector<double> & base_keys,
  const std::vector<geometry_msgs::msg::Quaternion> & base_values,
  const std::vector<double> & query_keys);
}  // namespace interpolation

#endif  // INTERPOLATION__SPHERICAL_LINEAR_INTERPOLATION_HPP_
