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

#include "ekf_localizer/covariance.hpp"

#include "ekf_localizer/state_index.hpp"
#include "tier4_autoware_utils/ros/msg_covariance.hpp"

using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;

std::array<double, 36> ekfCovarianceToPoseMessageCovariance(const Matrix6d & P)
{
  std::array<double, 36> covariance;
  covariance.fill(0.);

  covariance[COV_IDX::X_X] = P(IDX::X, IDX::X);
  covariance[COV_IDX::X_Y] = P(IDX::X, IDX::Y);
  covariance[COV_IDX::X_YAW] = P(IDX::X, IDX::YAW);
  covariance[COV_IDX::Y_X] = P(IDX::Y, IDX::X);
  covariance[COV_IDX::Y_Y] = P(IDX::Y, IDX::Y);
  covariance[COV_IDX::Y_YAW] = P(IDX::Y, IDX::YAW);
  covariance[COV_IDX::YAW_X] = P(IDX::YAW, IDX::X);
  covariance[COV_IDX::YAW_Y] = P(IDX::YAW, IDX::Y);
  covariance[COV_IDX::YAW_YAW] = P(IDX::YAW, IDX::YAW);

  return covariance;
}

std::array<double, 36> ekfCovarianceToTwistMessageCovariance(const Matrix6d & P)
{
  std::array<double, 36> covariance;
  covariance.fill(0.);

  covariance[COV_IDX::X_X] = P(IDX::VX, IDX::VX);
  covariance[COV_IDX::X_YAW] = P(IDX::VX, IDX::WZ);
  covariance[COV_IDX::YAW_X] = P(IDX::WZ, IDX::VX);
  covariance[COV_IDX::YAW_YAW] = P(IDX::WZ, IDX::WZ);

  return covariance;
}
