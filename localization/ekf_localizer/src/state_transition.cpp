// Copyright 2022 Autoware Foundation
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

#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/state_index.hpp"

#include <cmath>

double normalizeYaw(const double & yaw)
{
  // FIXME(IshitaTakeshi) I think the computation here can be simplified
  // FIXME(IshitaTakeshi) Rename the function. This is not normalization
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

/*  == Nonlinear model ==
 *
 * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
 * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
 * yaw_{k+1} = yaw_k + (wz_k) * dt
 * b_{k+1}   = b_k
 * vx_{k+1}  = vz_k
 * wz_{k+1}  = wz_k
 *
 * (b_k : yaw_bias_k)
 */

Vector6d predictNextState(const Vector6d & X_curr, const double dt)
{
  const double x = X_curr(IDX::X);
  const double y = X_curr(IDX::Y);
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);

  Vector6d X_next;
  X_next(IDX::X) = x + vx * std::cos(yaw + yaw_bias) * dt;  // dx = v * cos(yaw)
  X_next(IDX::Y) = y + vx * std::sin(yaw + yaw_bias) * dt;  // dy = v * sin(yaw)
  X_next(IDX::YAW) = normalizeYaw(yaw + wz * dt);           // dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;
  return X_next;
}

/*  == Linearized model ==
 *
 * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
 *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
 *     [ 0, 0,                 1,                 0,             0, dt]
 *     [ 0, 0,                 0,                 1,             0,  0]
 *     [ 0, 0,                 0,                 0,             1,  0]
 *     [ 0, 0,                 0,                 0,             0,  1]
 */
Matrix6d createStateTransitionMatrix(const Vector6d & X_curr, const double dt)
{
  const double yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double vx = X_curr(IDX::VX);

  Matrix6d A = Matrix6d::Identity();
  A(IDX::X, IDX::YAW) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw + yaw_bias) * dt;
  A(IDX::X, IDX::VX) = cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw + yaw_bias) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw + yaw_bias) * dt;
  A(IDX::YAW, IDX::WZ) = dt;
  return A;
}

Matrix6d processNoiseCovariance(
  const double proc_cov_yaw_d, const double proc_cov_vx_d, const double proc_cov_wz_d)
{
  Matrix6d Q = Matrix6d::Zero();
  Q(IDX::X, IDX::X) = 0.0;
  Q(IDX::Y, IDX::Y) = 0.0;
  Q(IDX::YAW, IDX::YAW) = proc_cov_yaw_d;  // for yaw
  Q(IDX::YAWB, IDX::YAWB) = 0.0;
  Q(IDX::VX, IDX::VX) = proc_cov_vx_d;  // for vx
  Q(IDX::WZ, IDX::WZ) = proc_cov_wz_d;  // for wz
  return Q;
}
