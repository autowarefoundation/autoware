// Copyright 2023 Tier IV, Inc.
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

#ifndef BEZIER_SAMPLER__BEZIER_HPP_
#define BEZIER_SAMPLER__BEZIER_HPP_

#include <eigen3/Eigen/Core>

#include <vector>

namespace bezier_sampler
{

// Coefficients for matrix calculation of the quintic Bézier curve.
const Eigen::Matrix<double, 6, 6> quintic_bezier_coefficients(
  (Eigen::Matrix<double, 6, 6>() << 1, 0, 0, 0, 0, 0, -5, 5, 0, 0, 0, 0, 10, -20, 10, 0, 0, 0, -10,
   30, -30, 10, 0, 0, 5, -20, 30, -20, 5, 0, -1, 5, -10, 10, -5, 1)
    .finished());
const Eigen::Matrix<double, 5, 6> quintic_bezier_velocity_coefficients(
  (Eigen::Matrix<double, 5, 6>() << quintic_bezier_coefficients.row(1) * 1,
   quintic_bezier_coefficients.row(2) * 2, quintic_bezier_coefficients.row(3) * 3,
   quintic_bezier_coefficients.row(4) * 4, quintic_bezier_coefficients.row(5) * 5)
    .finished());
const Eigen::Matrix<double, 4, 6> quintic_bezier_acceleration_coefficients(
  (Eigen::Matrix<double, 4, 6>() << quintic_bezier_velocity_coefficients.row(1) * 1,
   quintic_bezier_velocity_coefficients.row(2) * 2, quintic_bezier_velocity_coefficients.row(3) * 3,
   quintic_bezier_velocity_coefficients.row(4) * 4)
    .finished());

/// @brief Quintic Bezier curve
class Bezier
{
  Eigen::Matrix<double, 6, 2> control_points_;

public:
  /// @brief constructor from a matrix
  explicit Bezier(Eigen::Matrix<double, 6, 2> control_points);
  /// @brief constructor from a set of control points
  explicit Bezier(const std::vector<Eigen::Vector2d> & control_points);
  /// @brief return the control points
  [[nodiscard]] const Eigen::Matrix<double, 6, 2> & getControlPoints() const;
  /// @brief return the curve in cartesian frame with the desired resolution
  [[nodiscard]] std::vector<Eigen::Vector2d> cartesian(const double resolution) const;
  /// @brief return the curve in cartesian frame with the desired number of points
  [[nodiscard]] std::vector<Eigen::Vector2d> cartesian(const int nb_points) const;
  /// @brief return the curve in cartesian frame (including angle) with the desired number of
  /// points
  [[nodiscard]] std::vector<Eigen::Vector3d> cartesianWithHeading(const int nb_points) const;
  /// @brief calculate the curve value for the given parameter t
  [[nodiscard]] Eigen::Vector2d value(const double t) const;
  /// @brief calculate the curve value for the given parameter t (using matrix formulation)
  [[nodiscard]] Eigen::Vector2d valueM(const double t) const;
  /// @brief calculate the velocity (1st derivative) for the given parameter t
  [[nodiscard]] Eigen::Vector2d velocity(const double t) const;
  /// @brief calculate the acceleration (2nd derivative) for the given parameter t
  [[nodiscard]] Eigen::Vector2d acceleration(const double t) const;
  /// @brief return the heading (in radians) of the tangent for the given parameter t
  [[nodiscard]] double heading(const double t) const;
  /// @brief calculate the curvature for the given parameter t
  [[nodiscard]] double curvature(const double t) const;
};
}  // namespace bezier_sampler

#endif  // BEZIER_SAMPLER__BEZIER_HPP_
