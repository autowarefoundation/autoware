// Copyright 2022 TIER IV, Inc.
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

#include "autoware/universe_utils/ros/msg_operation.hpp"

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

// NOTE: Do not use autoware_universe_utils namespace
namespace geometry_msgs
{
namespace msg
{
Quaternion operator+(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a + quat_b);
}

Quaternion operator-(Quaternion a) noexcept
{
  tf2::Quaternion quat_a;
  tf2::fromMsg(a, quat_a);
  return tf2::toMsg(quat_a * -1.0);
}

Quaternion operator-(Quaternion a, Quaternion b) noexcept
{
  tf2::Quaternion quat_a;
  tf2::Quaternion quat_b;
  tf2::fromMsg(a, quat_a);
  tf2::fromMsg(b, quat_b);
  return tf2::toMsg(quat_a * quat_b.inverse());
}
}  // namespace msg
}  // namespace geometry_msgs
