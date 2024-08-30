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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__AKIMA_SPLINE_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__AKIMA_SPLINE_HPP_

#include "autoware/motion_utils/trajectory_container/interpolator/interpolator.hpp"

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

/**
 * @brief Class for Akima spline interpolation.
 *
 * This class provides methods to perform Akima spline interpolation on a set of data points.
 */
class AkimaSpline : public Interpolator<double>
{
private:
  Eigen::VectorXd a_, b_, c_, d_;  ///< Coefficients for the Akima spline.

  /**
   * @brief Compute the spline parameters.
   *
   * This method computes the coefficients for the Akima spline.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  void compute_parameters(
    const Eigen::Ref<const Eigen::VectorXd> & axis,
    const Eigen::Ref<const Eigen::VectorXd> & values);

  /**
   * @brief Build the interpolator with the given values.
   *
   * @param axis The axis values.
   * @param values The values to interpolate.
   */
  void build_impl(
    const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values) override;

  /**
   * @brief Compute the interpolated value at the given point.
   *
   * @param s The point at which to compute the interpolated value.
   * @return The interpolated value.
   */
  [[nodiscard]] double compute_impl(const double & s) const override;

  /**
   * @brief Compute the first derivative at the given point.
   *
   * @param s The point at which to compute the first derivative.
   * @return The first derivative.
   */
  [[nodiscard]] double compute_first_derivative_impl(const double & s) const override;

  /**
   * @brief Compute the second derivative at the given point.
   *
   * @param s The point at which to compute the second derivative.
   * @return The second derivative.
   */
  [[nodiscard]] double compute_second_derivative_impl(const double & s) const override;

public:
  AkimaSpline() = default;

  /**
   * @brief Get the minimum number of required points for the interpolator.
   *
   * @return The minimum number of required points.
   */
  [[nodiscard]] size_t minimum_required_points() const override { return 5; }

  /**
   * @brief Clone the interpolator.
   *
   * @return A shared pointer to a new instance of the interpolator.
   */
  [[nodiscard]] std::shared_ptr<Interpolator<double>> clone() const override;
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__AKIMA_SPLINE_HPP_
