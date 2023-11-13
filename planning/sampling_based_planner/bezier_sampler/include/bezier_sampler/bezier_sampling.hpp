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

#ifndef BEZIER_SAMPLER__BEZIER_SAMPLING_HPP_
#define BEZIER_SAMPLER__BEZIER_SAMPLING_HPP_

#include <bezier_sampler/bezier.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sampler_common/structures.hpp>

#include <vector>

namespace bezier_sampler
{
struct SamplingParameters
{
  int nb_t;       // Number of samples of normalized tangent vector magnitude
  double mt_min;  // Minimum normalized tangent vector magnitude
  double mt_max;  // Maximum normalized tangent vector magnitude
  int nb_k;       // Number of samples of normalized curvature vector magnitude
  double mk_min;  // Minimum normalized curvature vector magnitude
  double mk_max;  // Minimum normalized curvature vector magnitude
};
/// @brief sample Bezier curves given an initial and final state and sampling parameters
// cspell: ignore Artuñedoet
/// @details from Section IV of A. Artuñedoet al.: Real-Time Motion Planning Approach for Automated
/// Driving in Urban Environments
std::vector<Bezier> sample(
  const sampler_common::State & initial, const sampler_common::State & final,
  const SamplingParameters & params);
/// @brief generate a Bezier curve for the given states, velocities, and accelerations
Bezier generate(
  const Eigen::Vector2d & initial_pose, const Eigen::Vector2d & final_pose,
  const Eigen::Vector2d & initial_velocity, const Eigen::Vector2d & initial_acceleration,
  const Eigen::Vector2d & final_velocity, const Eigen::Vector2d & final_acceleration);
}  // namespace bezier_sampler

#endif  // BEZIER_SAMPLER__BEZIER_SAMPLING_HPP_
