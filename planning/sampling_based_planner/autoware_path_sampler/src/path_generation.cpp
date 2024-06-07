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

#include "autoware_path_sampler/path_generation.hpp"

#include "autoware_sampler_common/structures.hpp"

#include <autoware_bezier_sampler/bezier_sampling.hpp>
#include <autoware_frenet_planner/frenet_planner.hpp>
#include <autoware_path_sampler/prepare_inputs.hpp>

#include <autoware_planning_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

namespace autoware::path_sampler
{
std::vector<autoware::sampler_common::Path> generateCandidatePaths(
  const autoware::sampler_common::State & initial_state,
  const autoware::sampler_common::transform::Spline2D & path_spline, const double base_length,
  const Parameters & params)
{
  std::vector<autoware::sampler_common::Path> paths;
  const auto move_to_paths = [&](auto & paths_to_move) {
    paths.insert(
      paths.end(), std::make_move_iterator(paths_to_move.begin()),
      std::make_move_iterator(paths_to_move.end()));
  };

  if (params.sampling.enable_frenet) {
    auto frenet_paths = generateFrenetPaths(initial_state, base_length, path_spline, params);
    move_to_paths(frenet_paths);
  }
  if (params.sampling.enable_bezier) {
    const auto bezier_paths = generateBezierPaths(initial_state, base_length, path_spline, params);
    move_to_paths(bezier_paths);
  }
  return paths;
}
std::vector<autoware::sampler_common::Path> generateBezierPaths(
  const autoware::sampler_common::State & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  const auto initial_s = path_spline.frenet(initial_state.pose).s;
  const auto max_s = path_spline.lastS();
  std::vector<autoware::sampler_common::Path> bezier_paths;
  for (const auto target_length : params.sampling.target_lengths) {
    if (target_length <= base_length) continue;
    const auto target_s = std::min(max_s, initial_s + target_length - base_length);
    if (target_s >= max_s) break;
    autoware::sampler_common::State target_state{};
    target_state.pose = path_spline.cartesian({target_s, 0});
    target_state.curvature = path_spline.curvature(target_s);
    target_state.heading = path_spline.yaw(target_s);
    const auto bezier_samples =
      autoware::bezier_sampler::sample(initial_state, target_state, params.sampling.bezier);

    const auto step = std::min(0.1, params.sampling.resolution / target_length);
    for (const auto & bezier : bezier_samples) {
      autoware::sampler_common::Path path;
      path.lengths.push_back(0.0);
      for (double t = 0.0; t <= 1.0; t += step) {
        const auto x_y = bezier.valueM(t);
        path.points.emplace_back(x_y[0], x_y[1]);
        path.yaws.emplace_back(bezier.heading(t));
        path.curvatures.push_back(bezier.curvature(t));
      }
      for (size_t i = 0; i + 1 < path.points.size(); ++i) {
        path.lengths.push_back(
          path.lengths.back() + std::hypot(
                                  path.points[i + 1].x() - path.points[i].x(),
                                  path.points[i + 1].y() - path.points[i].y()));
      }
      bezier_paths.push_back(path);
    }
  }
  return bezier_paths;
}

std::vector<autoware::frenet_planner::Path> generateFrenetPaths(
  const autoware::sampler_common::State & initial_state, const double base_length,
  const autoware::sampler_common::transform::Spline2D & path_spline, const Parameters & params)
{
  const auto sampling_parameters =
    prepareSamplingParameters(initial_state, base_length, path_spline, params);

  autoware::frenet_planner::FrenetState initial_frenet_state;
  initial_frenet_state.position = path_spline.frenet(initial_state.pose);
  const auto s = initial_frenet_state.position.s;
  const auto d = initial_frenet_state.position.d;
  // Calculate Velocity and acceleration parametrized over arc length
  // From appendix I of Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
  const auto frenet_yaw = initial_state.heading - path_spline.yaw(s);
  const auto path_curvature = path_spline.curvature(s);
  const auto delta_s = 0.001;
  initial_frenet_state.lateral_velocity = (1 - path_curvature * d) * std::tan(frenet_yaw);
  const auto path_curvature_deriv = (path_spline.curvature(s + delta_s) - path_curvature) / delta_s;
  const auto cos_yaw = std::cos(frenet_yaw);
  if (cos_yaw == 0.0) {
    initial_frenet_state.lateral_acceleration = 0.0;
  } else {
    initial_frenet_state.lateral_acceleration =
      -(path_curvature_deriv * d + path_curvature * initial_frenet_state.lateral_velocity) *
        std::tan(frenet_yaw) +
      ((1 - path_curvature * d) / (cos_yaw * cos_yaw)) *
        (initial_state.curvature * ((1 - path_curvature * d) / cos_yaw) - path_curvature);
  }
  return autoware::frenet_planner::generatePaths(
    path_spline, initial_frenet_state, sampling_parameters);
}
}  // namespace autoware::path_sampler
