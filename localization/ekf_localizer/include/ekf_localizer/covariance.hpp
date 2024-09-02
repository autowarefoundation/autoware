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

#ifndef EKF_LOCALIZER__COVARIANCE_HPP_
#define EKF_LOCALIZER__COVARIANCE_HPP_

#include "ekf_localizer/matrix_types.hpp"

namespace autoware::ekf_localizer
{

std::array<double, 36> ekf_covariance_to_pose_message_covariance(const Matrix6d & P);
std::array<double, 36> ekf_covariance_to_twist_message_covariance(const Matrix6d & P);

}  // namespace autoware::ekf_localizer

#endif  // EKF_LOCALIZER__COVARIANCE_HPP_
