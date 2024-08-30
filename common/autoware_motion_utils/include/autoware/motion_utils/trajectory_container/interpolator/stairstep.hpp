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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__STAIRSTEP_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__STAIRSTEP_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/detail/stairstep_common_impl.hpp"

#include <memory>

namespace autoware::motion_utils::trajectory_container::interpolator
{

/**
 * @brief Template class for stairstep interpolation.
 *
 * This class provides methods to perform stairstep interpolation on a set of data points.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class Stairstep;

/**
 * @brief Template class for stairstep interpolation.
 *
 * This class provides the interface for stairstep interpolation.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class Stairstep : public detail::StairstepCommonImpl<T>
{
public:
  Stairstep() = default;

  /**
   * @brief Clone the interpolator.
   *
   * @return A shared pointer to a new instance of the interpolator.
   */
  [[nodiscard]] std::shared_ptr<Interpolator<T>> clone() const override
  {
    return std::make_shared<Stairstep<T>>(*this);
  }
};

/**
 * @brief Specialization of Stairstep for double values.
 *
 * This class provides methods to perform stairstep interpolation on double values.
 */
template <>
class Stairstep<double> : public detail::StairstepCommonImpl<double>
{
private:
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

public:
  Stairstep() = default;

  /**
   * @brief Clone the interpolator.
   *
   * @return A shared pointer to a new instance of the interpolator.
   */
  [[nodiscard]] std::shared_ptr<Interpolator<double>> clone() const override
  {
    return std::make_shared<Stairstep<double>>(*this);
  }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__STAIRSTEP_HPP_
