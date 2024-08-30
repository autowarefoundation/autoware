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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/detail/interpolator_common_impl.hpp"

#include <memory>

namespace autoware::motion_utils::trajectory_container::interpolator
{
/**
 * @brief Template class for interpolation.
 *
 * This class serves as the base class for specific interpolation types.
 *
 * @tparam T The type of the values being interpolated. (e.g. double, int, etc.)
 */
template <typename T>
class Interpolator : public detail::InterpolatorCommonImpl<T>
{
public:
  [[nodiscard]] virtual std::shared_ptr<Interpolator<T>> clone() const = 0;
};

/**
 * @brief Specialization of Interpolator for double values.
 *
 * This class adds methods for computing first and second derivatives.
 */
template <>
class Interpolator<double> : public detail::InterpolatorCommonImpl<double>
{
protected:
  /**
   * @brief Compute the first derivative at the given point.
   *
   * This method should be overridden by subclasses to provide the specific logic.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] virtual double compute_first_derivative_impl(const double & s) const = 0;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * This method should be overridden by subclasses to provide the specific logic.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] virtual double compute_second_derivative_impl(const double & s) const = 0;

public:
  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] double compute_first_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_first_derivative_impl(s);
  }

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] double compute_second_derivative(const double & s) const
  {
    this->validate_compute_input(s);
    return compute_second_derivative_impl(s);
  }

  [[nodiscard]] virtual std::shared_ptr<Interpolator<double>> clone() const = 0;
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_HPP_
