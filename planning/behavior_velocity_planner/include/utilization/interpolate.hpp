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

#ifndef UTILIZATION__INTERPOLATE_HPP_
#define UTILIZATION__INTERPOLATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <cmath>
#include <vector>

namespace behavior_velocity_planner
{
namespace interpolation
{
bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger);

class LinearInterpolate
{
public:
  LinearInterpolate() {}
  static bool interpolate(
    const std::vector<double> & base_index, const std::vector<double> & base_value,
    const std::vector<double> & return_index, std::vector<double> & return_value);
};

/*
 * helper functions
 */
bool isIncrease(const std::vector<double> & x);
bool isValidInput(
  const std::vector<double> & base_index, const std::vector<double> & base_value,
  const std::vector<double> & return_index);
std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y);

}  // namespace interpolation
}  // namespace behavior_velocity_planner

#endif  // UTILIZATION__INTERPOLATE_HPP_
