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

#ifndef PATH_SAMPLER__PARAMETERS_HPP_
#define PATH_SAMPLER__PARAMETERS_HPP_

#include "bezier_sampler/bezier_sampling.hpp"
#include "sampler_common/structures.hpp"

#include <vector>

struct Parameters
{
  sampler_common::Constraints constraints;
  struct
  {
    bool enable_frenet{};
    bool enable_bezier{};
    double resolution{};
    int previous_path_reuse_points_nb{};
    std::vector<double> target_lengths{};
    std::vector<double> target_lateral_positions{};
    int nb_target_lateral_positions{};
    struct
    {
      std::vector<double> target_lateral_velocities{};
      std::vector<double> target_lateral_accelerations{};
    } frenet;
    bezier_sampler::SamplingParameters bezier{};
  } sampling;

  struct
  {
    bool force_zero_deviation{};
    bool force_zero_heading{};
    bool smooth_reference{};
  } preprocessing{};
};

#endif  // PATH_SAMPLER__PARAMETERS_HPP_
