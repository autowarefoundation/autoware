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

#ifndef PATH_UTILS_HPP_
#define PATH_UTILS_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <tier4_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace run_out_utils
{

geometry_msgs::msg::Point findLongitudinalNearestPoint(
  const std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point,
  const std::vector<geometry_msgs::msg::Point> & target_points);

}  // namespace run_out_utils
}  // namespace autoware::behavior_velocity_planner
#endif  // PATH_UTILS_HPP_
