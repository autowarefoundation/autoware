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

#ifndef AUTOWARE_PATH_SAMPLER__PARAMETERS_HPP_
#define AUTOWARE_PATH_SAMPLER__PARAMETERS_HPP_

#include "autoware_bezier_sampler/bezier_sampling.hpp"
#include "autoware_sampler_common/structures.hpp"

#include <vector>

struct Parameters
{
  autoware::sampler_common::Constraints constraints;
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
    autoware::bezier_sampler::SamplingParameters bezier{};
  } sampling;

  struct
  {
    bool force_zero_deviation{};
    bool force_zero_heading{};
    bool smooth_reference{};
  } preprocessing{};

  struct
  {
    double max_lat_dev{};
    double direct_reuse_dist{};
  } path_reuse{};
};

#endif  // AUTOWARE_PATH_SAMPLER__PARAMETERS_HPP_
