// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/detail/nearest_neighbor_common_impl.hpp"

namespace autoware::motion_utils::trajectory_container::interpolator
{

/**
 * @brief Template class for nearest neighbor interpolation.
 *
 * This class provides methods to perform nearest neighbor interpolation on a set of data points.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighbor;

/**
 * @brief Template class for nearest neighbor interpolation.
 *
 * This class provides the interface for nearest neighbor interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class NearestNeighbor : public detail::NearestNeighborCommonImpl<T>
{
  template <typename InterpolatorType>
  friend class InterpolatorCreator;

private:
  NearestNeighbor() = default;
};

/**
 * @brief Specialization of NearestNeighbor for double values.
 *
 * This class provides methods to perform nearest neighbor interpolation on double values.
 */
template <>
class NearestNeighbor<double> : public detail::NearestNeighborCommonImpl<double>
{
  template <typename InterpolatorType>
  friend class InterpolatorCreator;

private:
  NearestNeighbor() = default;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] double compute_first_derivative_impl(const double &) const override { return 0.0; }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] double compute_second_derivative_impl(const double &) const override { return 0.0; }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__NEAREST_NEIGHBOR_HPP_
