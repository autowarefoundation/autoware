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

#include "ekf_localizer/measurement.hpp"

#include "ekf_localizer/state_index.hpp"

Eigen::Matrix<double, 3, 6> poseMeasurementMatrix()
{
  Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
  C(0, IDX::X) = 1.0;    // for pos x
  C(1, IDX::Y) = 1.0;    // for pos y
  C(2, IDX::YAW) = 1.0;  // for yaw
  return C;
}

Eigen::Matrix<double, 2, 6> twistMeasurementMatrix()
{
  Eigen::Matrix<double, 2, 6> C = Eigen::Matrix<double, 2, 6>::Zero();
  C(0, IDX::VX) = 1.0;  // for vx
  C(1, IDX::WZ) = 1.0;  // for wz
  return C;
}

Eigen::Matrix3d poseMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix3d R;
  R << covariance.at(6 * 0 + 0), covariance.at(6 * 0 + 1), covariance.at(6 * 0 + 5),
    covariance.at(6 * 1 + 0), covariance.at(6 * 1 + 1), covariance.at(6 * 1 + 5),
    covariance.at(6 * 5 + 0), covariance.at(6 * 5 + 1), covariance.at(6 * 5 + 5);
  return R * static_cast<double>(smoothing_step);
}

Eigen::Matrix2d twistMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix2d R;
  R << covariance.at(6 * 0 + 0), covariance.at(6 * 0 + 5), covariance.at(6 * 5 + 0),
    covariance.at(6 * 5 + 5);
  return R * static_cast<double>(smoothing_step);
}
