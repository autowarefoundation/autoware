// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_PARAMETERS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_PARAMETERS_HPP_

#include "autoware_bezier_sampler/bezier_sampling.hpp"
#include "autoware_sampler_common/structures.hpp"

#include <vector>
namespace autoware::behavior_path_planner
{
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::MultiPoint2d;
using autoware::universe_utils::MultiPolygon2d;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;

struct SamplingPlannerParameters
{  // constraints.hard
  double max_curvature;
  double min_curvature;
  // constraints.soft
  double lateral_deviation_weight;
  double length_weight;
  double curvature_weight;
  std::vector<double> weights;

  // sampling
  bool enable_frenet;
  bool enable_bezier;
  double resolution;
  int previous_path_reuse_points_nb;
  int nb_target_lateral_positions;

  std::vector<double> target_lengths;
  std::vector<double> target_lateral_positions;
  // frenet
  std::vector<double> target_lateral_velocities;
  std::vector<double> target_lateral_accelerations;

  bool force_zero_deviation;
  bool force_zero_heading;
  bool smooth_reference;
};

struct Preprocessing
{
  bool force_zero_deviation{};
  bool force_zero_heading{};
  bool smooth_reference{};
};

struct Frenet
{
  std::vector<double> target_lateral_velocities{};
  std::vector<double> target_lateral_accelerations{};
};

struct Sampling
{
  bool enable_frenet{};
  bool enable_bezier{};
  double resolution{};
  int previous_path_reuse_points_nb{};
  std::vector<double> target_lengths{};
  std::vector<double> target_lateral_positions{};
  int nb_target_lateral_positions{};
  Frenet frenet;
  autoware::bezier_sampler::SamplingParameters bezier{};
};

struct SamplingPlannerInternalParameters
{
  autoware::sampler_common::Constraints constraints;
  Sampling sampling;
  Preprocessing preprocessing{};
};
}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_SAMPLING_PLANNER_MODULE__SAMPLING_PLANNER_PARAMETERS_HPP_
