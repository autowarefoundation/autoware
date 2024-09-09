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

#ifndef AUTOWARE_FRENET_PLANNER__FRENET_PLANNER_HPP_
#define AUTOWARE_FRENET_PLANNER__FRENET_PLANNER_HPP_

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "autoware_frenet_planner/structures.hpp"
#include "autoware_sampler_common/structures.hpp"
#include "autoware_sampler_common/transform/spline_transform.hpp"

#include <optional>
#include <vector>

namespace autoware::frenet_planner
{
/// @brief generate trajectories relative to the reference for the given initial state and sampling
/// parameters
std::vector<Trajectory> generateTrajectories(
  const autoware::sampler_common::transform::Spline2D & reference_spline,
  const FrenetState & initial_state, const SamplingParameters & sampling_parameters);
/// @brief generate paths relative to the reference for the given initial state and sampling
/// parameters
std::vector<Path> generatePaths(
  const autoware::sampler_common::transform::Spline2D & reference_spline,
  const FrenetState & initial_state, const SamplingParameters & sampling_parameters);
/// @brief generate a candidate path
/// @details one polynomial for lateral motion (d) is calculated over the longitudinal displacement
/// (s): d(s).
Path generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double s_resolution);
/// @brief generate a candidate trajectory
/// @details the polynomials for lateral motion (d) and longitudinal motion (s) are calculated over
/// time: d(t) and s(t).
Trajectory generateCandidate(
  const FrenetState & initial_state, const FrenetState & target_state, const double duration,
  const double time_resolution);
/// @brief calculate the cartesian frame of the given path
void calculateCartesian(
  const autoware::sampler_common::transform::Spline2D & reference, Path & path);
/// @brief calculate the cartesian frame of the given trajectory
void calculateCartesian(
  const autoware::sampler_common::transform::Spline2D & reference, Trajectory & trajectory);

}  // namespace autoware::frenet_planner

#endif  // AUTOWARE_FRENET_PLANNER__FRENET_PLANNER_HPP_
