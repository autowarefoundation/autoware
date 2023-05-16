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

#ifndef PATH_SAMPLER__PATH_GENERATION_HPP_
#define PATH_SAMPLER__PATH_GENERATION_HPP_

#include "bezier_sampler/bezier_sampling.hpp"
#include "frenet_planner/structures.hpp"
#include "path_sampler/parameters.hpp"
#include "sampler_common/constraints/hard_constraint.hpp"
#include "sampler_common/structures.hpp"
#include "sampler_common/transform/spline_transform.hpp"

#include <autoware_auto_planning_msgs/msg/path.hpp>

#include <vector>

namespace path_sampler
{
/**
 * @brief generate candidate paths for the given problem inputs
 * @param [in] initial_state initial ego state
 * @param [in] path_spline spline of the reference path
 * @param [in] base_length base length of the reuse path (= 0.0 if not reusing a previous path)
 * @param [in] params parameters
 * @return candidate paths
 */
std::vector<sampler_common::Path> generateCandidatePaths(
  const sampler_common::State & initial_state,
  const sampler_common::transform::Spline2D & path_spline, const double base_length,
  const Parameters & params);

std::vector<sampler_common::Path> generateBezierPaths(
  const sampler_common::State & initial_state, const double base_length,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params);
std::vector<frenet_planner::Path> generateFrenetPaths(
  const sampler_common::State & initial_state, const double base_length,
  const sampler_common::transform::Spline2D & path_spline, const Parameters & params);
}  // namespace path_sampler

#endif  // PATH_SAMPLER__PATH_GENERATION_HPP_
