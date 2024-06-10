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
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

Eigen::Matrix<double, 3, 6> pose_measurement_matrix()
{
  Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
  c(0, IDX::X) = 1.0;    // for pos x
  c(1, IDX::Y) = 1.0;    // for pos y
  c(2, IDX::YAW) = 1.0;  // for yaw
  return c;
}

Eigen::Matrix<double, 2, 6> twist_measurement_matrix()
{
  Eigen::Matrix<double, 2, 6> c = Eigen::Matrix<double, 2, 6>::Zero();
  c(0, IDX::VX) = 1.0;  // for vx
  c(1, IDX::WZ) = 1.0;  // for wz
  return c;
}

Eigen::Matrix3d pose_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix3d r;
  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  r << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_Y), covariance.at(COV_IDX::X_YAW),
    covariance.at(COV_IDX::Y_X), covariance.at(COV_IDX::Y_Y), covariance.at(COV_IDX::Y_YAW),
    covariance.at(COV_IDX::YAW_X), covariance.at(COV_IDX::YAW_Y), covariance.at(COV_IDX::YAW_YAW);
  return r * static_cast<double>(smoothing_step);
}

Eigen::Matrix2d twist_measurement_covariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix2d r;
  using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  r << covariance.at(COV_IDX::X_X), covariance.at(COV_IDX::X_YAW), covariance.at(COV_IDX::YAW_X),
    covariance.at(COV_IDX::YAW_YAW);
  return r * static_cast<double>(smoothing_step);
}
