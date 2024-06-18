// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>

#include <autoware_adapi_v1_msgs/msg/velocity_factor.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
struct SlowdownInterval
{
  SlowdownInterval(
    const geometry_msgs::msg::Point & from_, const geometry_msgs::msg::Point & to_,
    const double vel)
  : from{from_}, to{to_}, velocity{vel}
  {
  }
  geometry_msgs::msg::Point from{};
  geometry_msgs::msg::Point to{};
  double velocity{};
};
struct VelocityPlanningResult
{
  std::vector<geometry_msgs::msg::Point> stop_points{};
  std::vector<SlowdownInterval> slowdown_intervals{};
  std::optional<autoware_adapi_v1_msgs::msg::VelocityFactor> velocity_factor{};
};
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__VELOCITY_PLANNING_RESULT_HPP_
