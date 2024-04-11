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

#ifndef STATIC_CENTERLINE_OPTIMIZER__CENTERLINE_SOURCE__BAG_EGO_TRAJECTORY_BASED_CENTERLINE_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__CENTERLINE_SOURCE__BAG_EGO_TRAJECTORY_BASED_CENTERLINE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "static_centerline_optimizer/type_alias.hpp"

#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
std::vector<TrajectoryPoint> generate_centerline_with_bag(rclcpp::Node & node);
}  // namespace static_centerline_optimizer
#endif  // STATIC_CENTERLINE_OPTIMIZER__CENTERLINE_SOURCE__BAG_EGO_TRAJECTORY_BASED_CENTERLINE_HPP_
