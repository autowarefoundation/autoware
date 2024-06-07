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

#include "autoware_sampler_common/constraints/soft_constraint.hpp"

#include "autoware_sampler_common/structures.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"

#include <numeric>

namespace autoware::sampler_common::constraints
{
void calculateCurvatureCost(Path & path, const Constraints & constraints)
{
  double curvature_sum = 0.0;
  for (const auto curvature : path.curvatures) {
    curvature_sum += std::abs(curvature);
  }
  path.cost +=
    constraints.soft.curvature_weight * curvature_sum / static_cast<double>(path.curvatures.size());
}
void calculateLengthCost(Path & path, const Constraints & constraints)
{
  if (!path.lengths.empty()) path.cost -= constraints.soft.length_weight * path.lengths.back();
}

void calculateLateralDeviationCost(
  Path & path, const Constraints & constraints, const transform::Spline2D & reference)
{
  const auto fp = reference.frenet(path.points.back());
  const double lateral_deviation = std::abs(fp.d);
  path.cost += constraints.soft.lateral_deviation_weight * lateral_deviation;
}

void calculateCost(
  Path & path, const Constraints & constraints, const transform::Spline2D & reference)
{
  if (path.points.empty()) return;
  calculateCurvatureCost(path, constraints);
  calculateLengthCost(path, constraints);
  calculateLateralDeviationCost(path, constraints, reference);
}
}  // namespace autoware::sampler_common::constraints
