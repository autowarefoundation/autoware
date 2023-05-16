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

#include "sampler_common/constraints/hard_constraint.hpp"

#include "sampler_common/constraints/footprint.hpp"

#include <boost/geometry/algorithms/within.hpp>

#include <vector>

namespace sampler_common::constraints
{
bool satisfyMinMax(const std::vector<double> & values, const double min, const double max)
{
  for (const auto value : values) {
    if (value < min || value > max) return false;
  }
  return true;
}

bool has_collision(const MultiPoint2d & footprint, const MultiPolygon2d & obstacles)
{
  for (const auto & o : obstacles)
    for (const auto & p : footprint)
      if (boost::geometry::within(p, o)) return true;
  return false;
}

MultiPoint2d checkHardConstraints(Path & path, const Constraints & constraints)
{
  const auto footprint = buildFootprintPoints(path, constraints);
  if (!footprint.empty()) {
    if (!boost::geometry::within(footprint, constraints.drivable_polygons)) {
      path.constraint_results.drivable_area = false;
    }
  }
  path.constraint_results.collision = !has_collision(footprint, constraints.obstacle_polygons);
  if (!satisfyMinMax(
        path.curvatures, constraints.hard.min_curvature, constraints.hard.max_curvature)) {
    path.constraint_results.curvature = false;
  }
  return footprint;
}
}  // namespace sampler_common::constraints
