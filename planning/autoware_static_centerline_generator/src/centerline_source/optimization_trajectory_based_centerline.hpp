// Copyright 2024 TIER IV, Inc.
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

#ifndef CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT
#define CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT

#include "rclcpp/rclcpp.hpp"
#include "type_alias.hpp"

#include <vector>

namespace autoware::static_centerline_generator
{
class OptimizationTrajectoryBasedCenterline
{
public:
  OptimizationTrajectoryBasedCenterline() = default;
  explicit OptimizationTrajectoryBasedCenterline(rclcpp::Node & node);
  std::vector<TrajectoryPoint> generate_centerline_with_optimization(
    rclcpp::Node & node, const RouteHandler & route_handler,
    const std::vector<lanelet::Id> & route_lane_ids);

private:
  std::vector<TrajectoryPoint> optimize_trajectory(const Path & raw_path) const;

  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_raw_path_with_lane_id_{nullptr};
  rclcpp::Publisher<Path>::SharedPtr pub_raw_path_{nullptr};
};
}  // namespace autoware::static_centerline_generator
// clang-format off
#endif  // CENTERLINE_SOURCE__OPTIMIZATION_TRAJECTORY_BASED_CENTERLINE_HPP_  // NOLINT
// clang-format on
