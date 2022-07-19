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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_

#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

namespace tier4_autoware_utils
{
template <>
inline geometry_msgs::msg::Point getPoint(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose.position;
}

template <>
inline geometry_msgs::msg::Pose getPose(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.pose;
}

template <>
inline double getLongitudinalVelocity(
  const autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  return p.point.longitudinal_velocity_mps;
}

template <>
inline void setPose(
  const geometry_msgs::msg::Pose & pose, autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  p.point.pose = pose;
}

template <>
inline void setLongitudinalVelocity(
  const double velocity, autoware_auto_planning_msgs::msg::PathPointWithLaneId & p)
{
  p.point.longitudinal_velocity_mps = velocity;
}
}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__PATH_WITH_LANE_ID_GEOMETRY_HPP_
