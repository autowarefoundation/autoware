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

#include <bezier_sampler/bezier_sampling.hpp>

namespace bezier_sampler
{
std::vector<Bezier> sample(
  const sampler_common::State & initial, const sampler_common::State & final,
  const SamplingParameters & params)
{
  std::vector<Bezier> curves;
  curves.reserve(params.nb_t * params.nb_t * params.nb_k);

  double distance_initial_to_final = std::sqrt(
    (initial.pose.x() - final.pose.x()) * (initial.pose.x() - final.pose.x()) +
    (initial.pose.y() - final.pose.y()) * (initial.pose.y() - final.pose.y()));
  // Tangent vectors
  const Eigen::Vector2d initial_tangent_unit(std::cos(initial.heading), std::sin(initial.heading));
  const Eigen::Vector2d final_tangent_unit(std::cos(final.heading), std::sin(final.heading));
  // Unit vectors
  const Eigen::Vector2d initial_normal_unit = initial_tangent_unit.unitOrthogonal();
  const Eigen::Vector2d final_normal_unit = final_tangent_unit.unitOrthogonal();

  double step_t = (params.mt_max - params.mt_min) / (params.nb_t - 1);
  double step_k = (params.mk_max - params.mk_min) / (params.nb_k - 1);
  for (double m_initial = params.mt_min; m_initial <= params.mt_max; m_initial += step_t) {
    double initial_tangent_length = m_initial * distance_initial_to_final;
    for (double m_final = params.mt_min; m_final <= params.mt_max; m_final += step_t) {
      double final_tangent_length = m_final * distance_initial_to_final;
      for (double k = params.mk_min; k <= params.mk_max; k += step_k) {
        double acceleration_length = k * distance_initial_to_final;
        Eigen::Vector2d initial_acceleration =
          acceleration_length * initial_tangent_unit +
          initial.curvature * initial_tangent_length * initial_tangent_length * initial_normal_unit;
        Eigen::Vector2d final_acceleration =
          acceleration_length * final_tangent_unit +
          final.curvature * final_tangent_length * final_tangent_length * final_normal_unit;
        curves.push_back(generate(
          {initial.pose.x(), initial.pose.y()}, {final.pose.x(), final.pose.y()},
          initial_tangent_unit * initial_tangent_length, initial_acceleration,
          final_tangent_unit * final_tangent_length, final_acceleration));
      }
    }
  }
  return curves;
}
Bezier generate(
  const Eigen::Vector2d & initial_pose, const Eigen::Vector2d & final_pose,
  const Eigen::Vector2d & initial_velocity, const Eigen::Vector2d & initial_acceleration,
  const Eigen::Vector2d & final_velocity, const Eigen::Vector2d & final_acceleration)
{
  Eigen::Matrix<double, 6, 2> control_points;
  // P0 and P5 correspond to the initial and final configurations
  control_points.row(0) = initial_pose;
  control_points.row(5) = final_pose;
  // P1 and P4 depend on P0, P5, and the initial/final velocities
  control_points.row(1) = control_points.row(0) + (1.0 / 5.0) * initial_velocity.transpose();
  control_points.row(4) = control_points.row(5) - (1.0 / 5.0) * final_velocity.transpose();
  // P2 and P3 depend on P1, P4, and the initial/final accelerations
  control_points.row(2) = 2 * control_points.row(1) - control_points.row(0) +
                          (1.0 / 20.0) * initial_acceleration.transpose();
  control_points.row(3) = 2 * control_points.row(4) - control_points.row(5) -
                          (1.0 / 20.0) * final_acceleration.transpose();
  return Bezier(control_points);
}
}  // namespace bezier_sampler
