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

#ifndef DISTANCE_HPP_
#define DISTANCE_HPP_

#include "obstacles.hpp"
#include "parameters.hpp"
#include "types.hpp"

#include <geometry_msgs/msg/vector3.hpp>

#include <optional>
#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{
/// @brief calculate the closest distance to a collision
/// @param [in] projection forward projection line
/// @param [in] footprint footprint of the projection
/// @param [in] collision_checker object to retrieve collision points
/// @param [in] params projection parameters
/// @return distance to the closest collision if any
std::optional<double> distanceToClosestCollision(
  const linestring_t & projection, const polygon_t & footprint,
  const CollisionChecker & collision_checker, const ProjectionParameters & params);

/// @brief calculate the closest distance along a circle to a given target point
/// @param [in] origin starting point
/// @param [in] heading heading used to calculate the tangent to the circle at the origin
/// @param [in] target target point
/// @return distance from origin to target along a circle
double arcDistance(const point_t & origin, const double heading, const point_t & target);

}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter

#endif  // DISTANCE_HPP_
