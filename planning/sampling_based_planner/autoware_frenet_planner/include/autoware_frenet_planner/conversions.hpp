// Copyright 2023 Tier IV, Inc.
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

#ifndef AUTOWARE_FRENET_PLANNER__CONVERSIONS_HPP_
#define AUTOWARE_FRENET_PLANNER__CONVERSIONS_HPP_

#include "autoware_frenet_planner/polynomials.hpp"

#include <autoware_sampler_common/structures.hpp>

#include <vector>

namespace autoware::frenet_planner
{

/// @brief calculate the lengths and yaws vectors of the given trajectory
/// @param [inout] trajectory trajectory
inline void calculateLengthsAndYaws(Trajectory & trajectory)
{
  if (trajectory.points.empty()) return;
  trajectory.lengths.clear();
  trajectory.yaws.clear();
  trajectory.yaws.reserve(trajectory.points.size());
  trajectory.lengths.reserve(trajectory.points.size());

  trajectory.lengths.push_back(0.0);
  for (auto it = trajectory.points.begin(); it != std::prev(trajectory.points.end()); ++it) {
    const auto dx = std::next(it)->x() - it->x();
    const auto dy = std::next(it)->y() - it->y();
    trajectory.lengths.push_back(trajectory.lengths.back() + std::hypot(dx, dy));
    trajectory.yaws.push_back(std::atan2(dy, dx));
  }
  const auto last_yaw = trajectory.yaws.empty() ? 0.0 : trajectory.yaws.back();
  trajectory.yaws.push_back(last_yaw);
}
}  // namespace autoware::frenet_planner

#endif  // AUTOWARE_FRENET_PLANNER__CONVERSIONS_HPP_
