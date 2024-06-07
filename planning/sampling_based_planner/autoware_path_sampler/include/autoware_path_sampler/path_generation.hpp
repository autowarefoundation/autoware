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

#ifndef AUTOWARE_PATH_SAMPLER__PATH_GENERATION_HPP_
#define AUTOWARE_PATH_SAMPLER__PATH_GENERATION_HPP_

#include "autoware_bezier_sampler/bezier_sampling.hpp"
#include "autoware_frenet_planner/structures.hpp"
#include "autoware_path_sampler/parameters.hpp"
#include "autoware_sampler_common/constraints/hard_constraint.hpp"
#include "autoware_sampler_common/structures.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"

#include <autoware_planning_msgs/msg/path.hpp>

#include <vector>

namespace autoware::path_sampler
{
/**
 * @brief generate candidate paths for the given problem inputs
 * @param [in] initial_state initial ego state
 * @param [in] path_spline spline of the reference path
 * @param [in] base_length base length of the reuse path (= 0.0 if not reusing a previous path)
 * @param [in] params parameters
 * @return candidate paths
 */
std::vector<autoware::sampler_common::Path> generateCandidatePaths(
  const autoware::sampler_common::State & initial_state,
  const autoware::sampler_common::transform::Spline2D & path_spline, const double base_length,
  const Parameters & params);

std::vector<autoware::sampler_common::Path> generateBezierPaths(
  const autoware::sampler_common::State & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline, const Parameters & params);
std::vector<autoware::frenet_planner::Path> generateFrenetPaths(
  const autoware::sampler_common::State & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline, const Parameters & params);
}  // namespace autoware::path_sampler

#endif  // AUTOWARE_PATH_SAMPLER__PATH_GENERATION_HPP_
