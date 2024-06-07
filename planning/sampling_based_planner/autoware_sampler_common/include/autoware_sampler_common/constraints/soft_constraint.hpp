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

#ifndef AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP_
#define AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP_

#include "autoware_sampler_common/structures.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"

namespace autoware::sampler_common::constraints
{
/// @brief calculate the curvature cost of the given path
void calculateCurvatureCost(Path & path, const Constraints & constraints);
/// @brief calculate the length cost of the given path
void calculateLengthCost(Path & path, const Constraints & constraints);
/// @brief calculate the lateral deviation cost at the end of the given path
void calculateLateralDeviationCost(
  Path & path, const Constraints & constraints, const transform::Spline2D & reference);
/// @brief calculate the overall cost of the given path
void calculateCost(
  Path & path, const Constraints & constraints, const transform::Spline2D & reference);
}  // namespace autoware::sampler_common::constraints

#endif  // AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__SOFT_CONSTRAINT_HPP_
