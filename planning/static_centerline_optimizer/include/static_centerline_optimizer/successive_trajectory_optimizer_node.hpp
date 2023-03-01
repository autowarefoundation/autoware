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

// NOTE: This file was copied from a part of implementation in
// https://github.com/autowarefoundation/autoware.universe/blob/main/planning/obstacle_avoidance_planner/include/obstacle_avoidance_planner/node.hpp

#ifndef STATIC_CENTERLINE_OPTIMIZER__SUCCESSIVE_TRAJECTORY_OPTIMIZER_NODE_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__SUCCESSIVE_TRAJECTORY_OPTIMIZER_NODE_HPP_

#include "obstacle_avoidance_planner/node.hpp"
#include "static_centerline_optimizer/type_alias.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
class SuccessiveTrajectoryOptimizer : public obstacle_avoidance_planner::ObstacleAvoidancePlanner
{
private:
  // subscriber
  rclcpp::Subscription<Path>::SharedPtr centerline_sub_;

public:
  explicit SuccessiveTrajectoryOptimizer(const rclcpp::NodeOptions & node_options);

  // subscriber callback functions
  Trajectory on_centerline(const Path & path);
};
}  // namespace static_centerline_optimizer

#endif  // STATIC_CENTERLINE_OPTIMIZER__SUCCESSIVE_TRAJECTORY_OPTIMIZER_NODE_HPP_
