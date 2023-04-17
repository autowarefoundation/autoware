// Copyright 2020-2022 Arm Ltd., TierIV
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

#ifndef LIDAR_APOLLO_SEGMENTATION_TVM__LOG_TABLE_HPP_
#define LIDAR_APOLLO_SEGMENTATION_TVM__LOG_TABLE_HPP_

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{

/// \brief Use a lookup table to compute the natural logarithm of 1+num.
/// \param[in] num
/// \return ln(1+num)
float calcApproximateLog(float num);
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
#endif  // LIDAR_APOLLO_SEGMENTATION_TVM__LOG_TABLE_HPP_
