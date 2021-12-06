// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <cmath>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
KinematicsBicycleModel::KinematicsBicycleModel(
  const float64_t wheelbase, const float64_t steer_lim, const float64_t steer_tau)
: VehicleModelInterface(/* dim_x */ 3, /* dim_u */ 1, /* dim_y */ 2, wheelbase)
{
  m_steer_lim = steer_lim;
  m_steer_tau = steer_tau;
}

void KinematicsBicycleModel::calculateDiscreteMatrix(
  Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
  const float64_t dt)
{
  auto sign = [](float64_t x) {return (x > 0.0) - (x < 0.0);};

  /* Linearize delta around delta_r (reference delta) */
  float64_t delta_r = atan(m_wheelbase * m_curvature);
  if (std::abs(delta_r) >= m_steer_lim) {
    delta_r = m_steer_lim * static_cast<float64_t>(sign(delta_r));
  }
  float64_t cos_delta_r_squared_inv = 1 / (cos(delta_r) * cos(delta_r));
  float64_t velocity = m_velocity;
  if (std::abs(m_velocity) < 1e-04) {velocity = 1e-04 * (m_velocity >= 0 ? 1 : -1);}

  a_d << 0.0, velocity, 0.0, 0.0, 0.0, velocity / m_wheelbase * cos_delta_r_squared_inv, 0.0, 0.0,
    -1.0 / m_steer_tau;
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  a_d = (I - dt * 0.5 * a_d).inverse() * (I + dt * 0.5 * a_d);  // bilinear discretization

  b_d << 0.0, 0.0, 1.0 / m_steer_tau;
  b_d *= dt;

  c_d << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;

  w_d << 0.0,
    -velocity * m_curvature +
    velocity / m_wheelbase * (tan(delta_r) - delta_r * cos_delta_r_squared_inv),
    0.0;
  w_d *= dt;
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd & u_ref)
{
  u_ref(0, 0) = std::atan(m_wheelbase * m_curvature);
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
