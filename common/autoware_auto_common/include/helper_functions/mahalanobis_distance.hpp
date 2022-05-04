// Copyright 2021 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_
#define HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_

#include <Eigen/Cholesky>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// \brief Calculate square of mahalanobis distance
/// \tparam T Type of elements in the matrix
/// \tparam kNumOfStates Number of states
/// \param sample Single column matrix containing sample whose distance needs to be computed
/// \param mean Single column matrix containing mean of samples received so far
/// \param covariance_factor Covariance matrix
/// \return Square of mahalanobis distance
template <typename T, std::int32_t kNumOfStates>
types::float32_t calculate_squared_mahalanobis_distance(
  const Eigen::Matrix<T, kNumOfStates, 1> & sample, const Eigen::Matrix<T, kNumOfStates, 1> & mean,
  const Eigen::Matrix<T, kNumOfStates, kNumOfStates> & covariance_factor)
{
  using Vector = Eigen::Matrix<T, kNumOfStates, 1>;
  // This is equivalent to the squared Mahalanobis distance of the form: diff.T * C.inv() * diff
  // Instead of the covariance matrix C we have its lower-triangular factor L, such that C = L * L.T
  // squared_mahalanobis_distance = diff.T * C.inv() * diff
  // = diff.T * (L * L.T).inv() * diff
  // = diff.T * L.T.inv() * L.inv() * diff
  // = (L.inv() * diff).T * (L.inv() * diff)
  // this allows us to efficiently find the squared Mahalanobis distance using (L.inv() * diff),
  // which can be found as a solution to: L * x = diff.
  const Vector diff = sample - mean;
  const Vector x = covariance_factor.ldlt().solve(diff);
  return x.transpose() * x;
}

/// \brief Calculate mahalanobis distance
/// \tparam T Type of elements in the matrix
/// \tparam kNumOfStates Number of states
/// \param sample Single column matrix containing sample whose distance needs to be computed
/// \param mean Single column matrix containing mean of samples received so far
/// \param covariance_factor Covariance matrix
/// \return Mahalanobis distance
template <typename T, std::int32_t kNumOfStates>
types::float32_t calculate_mahalanobis_distance(
  const Eigen::Matrix<T, kNumOfStates, 1> & sample, const Eigen::Matrix<T, kNumOfStates, 1> & mean,
  const Eigen::Matrix<T, kNumOfStates, kNumOfStates> & covariance_factor)
{
  return sqrtf(calculate_squared_mahalanobis_distance(sample, mean, covariance_factor));
}
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__MAHALANOBIS_DISTANCE_HPP_
