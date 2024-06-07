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

#include "autoware_sampler_common/constraints/hard_constraint.hpp"

#include "autoware_sampler_common/constraints/footprint.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/within.hpp>

#include <vector>

namespace autoware::sampler_common::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

bool has_collision(
  const MultiPoint2d & footprint, const MultiPolygon2d & obstacles, const double min_distance)
{
  if (footprint.empty()) return false;
  for (const auto & o : obstacles)
    if (boost::geometry::distance(o, footprint) <= min_distance) return true;
  return false;
}

MultiPoint2d checkHardConstraints(Path & path, const Constraints & constraints)
{
  const auto footprint = buildFootprintPoints(path, constraints);
  if (!footprint.empty()) {
    if (constraints.hard.limit_footprint_inside_drivable_area)
      path.constraint_results.inside_drivable_area =
        boost::geometry::within(footprint, constraints.drivable_polygons);
    path.constraint_results.collision_free = !has_collision(
      footprint, constraints.obstacle_polygons, constraints.hard.min_dist_from_obstacles);
  }
  if (!satisfyMinMax(
        path.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
    path.constraint_results.valid_curvature = false;
  }
  return footprint;
}
}  // namespace autoware::sampler_common::constraints
