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

// clang-format off
#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__INTERPOLATOR_COMMON_IMPL_HPP_  // NOLINT
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__INTERPOLATOR_COMMON_IMPL_HPP_  // NOLINT
// clang-format on

#include <Eigen/Dense>
#include <rclcpp/logging.hpp>

#include <fmt/format.h>

#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator::detail
{
/**
 * @brief Base class for interpolation implementations.
 *
 * This class provides the basic interface and common functionality for different types
 * of interpolation. It is intended to be subclassed by specific interpolation algorithms.
 *
 * @tparam T The type of the values being interpolated.
 */
template <typename T>
class InterpolatorCommonImpl
{
protected:
  Eigen::VectorXd axis_;  ///< Axis values for the interpolation.

  /**
   * @brief Get the start of the interpolation range.
   */
  [[nodiscard]] double start() const { return axis_(0); }

  /**
   * @brief Get the end of the interpolation range.
   */
  [[nodiscard]] double end() const { return axis_(axis_.size() - 1); }

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * This method should be overridden by subclasses to provide the specific interpolation logic.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] virtual T compute_impl(const double & s) const = 0;

  /**
   * @brief Build the interpolator with the given values.
   *
   * This method should be overridden by subclasses to provide the specific build logic.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  virtual void build_impl(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values) = 0;

  /**
   * @brief Validate the input to the compute method.
   *
   * Checks that the interpolator has been built and that the input value is within range.
   *
   * @param s The input value.
   * @throw std::runtime_error if the interpolator has not been built.
   */
  void validate_compute_input(const double & s) const
  {
    if (s < start() || s > end()) {
      RCLCPP_WARN(
        rclcpp::get_logger("Interpolator"),
        "Input value %f is outside the range of the interpolator [%f, %f].", s, start(), end());
    }
  }

  [[nodiscard]] int32_t get_index(const double & s, bool end_inclusive = true) const
  {
    if (end_inclusive && s == end()) {
      return static_cast<int32_t>(axis_.size()) - 2;
    }
    auto comp = [](const double & a, const double & b) { return a <= b; };
    return std::distance(axis_.begin(), std::lower_bound(axis_.begin(), axis_.end(), s, comp)) - 1;
  }

public:
  /**
   * @brief Build the interpolator with the given axis and values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   * @return True if the interpolator was built successfully, false otherwise.
   */
  bool build(const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<T> & values)
  {
    if (axis.size() != static_cast<Eigen::Index>(values.size())) {
      return false;
    }
    if (axis.size() < static_cast<Eigen::Index>(minimum_required_points())) {
      return false;
    }
    build_impl(axis, values);
    return true;
  }

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * This method should be overridden by subclasses to return the specific requirement.
   *
   * @return The minimum number of required points.
   */
  [[nodiscard]] virtual size_t minimum_required_points() const = 0;

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] T compute(const double & s) const
  {
    validate_compute_input(s);
    return compute_impl(s);
  }
};
}  // namespace autoware::motion_utils::trajectory_container::interpolator::detail

// clang-format off
#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__DETAIL__INTERPOLATOR_COMMON_IMPL_HPP_  // NOLINT
// clang-format on
