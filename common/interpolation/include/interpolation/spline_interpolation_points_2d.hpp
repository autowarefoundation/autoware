// Copyright 2021 Tier IV, Inc.
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

#ifndef INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
#define INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_

#include "interpolation/spline_interpolation.hpp"

#include <vector>

namespace interpolation
{
std::array<std::vector<double>, 3> slerp2dFromXY(
  const std::vector<double> & base_keys, const std::vector<double> & base_x_values,
  const std::vector<double> & base_y_values, const std::vector<double> & query_keys);

template <typename T>
std::vector<double> splineYawFromPoints(const std::vector<T> & points);
}  // namespace interpolation

// non-static points spline interpolation
// NOTE: We can calculate yaw from the x and y by interpolation derivatives.
//
// Usage:
// ```
// SplineInterpolationPoints2d spline;
// // memorize pre-interpolation result internally
// spline.calcSplineCoefficients(base_keys, base_values);
// const auto interpolation_result1 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys1);
// const auto interpolation_result2 = spline.getSplineInterpolatedPoint(
//   base_keys, query_keys2);
// const auto yaw_interpolation_result = spline.getSplineInterpolatedYaw(
//   base_keys, query_keys1);
// ```
class SplineInterpolationPoints2d
{
public:
  SplineInterpolationPoints2d() = default;
  template <typename T>
  explicit SplineInterpolationPoints2d(const std::vector<T> & points)
  {
    std::vector<geometry_msgs::msg::Point> points_inner;
    for (const auto & p : points) {
      points_inner.push_back(tier4_autoware_utils::getPoint(p));
    }
    calcSplineCoefficientsInner(points_inner);
  }

  // TODO(murooka) implement these functions
  // std::vector<geometry_msgs::msg::Point> getSplineInterpolatedPoints(const double width);
  // std::vector<geometry_msgs::msg::Pose> getSplineInterpolatedPoses(const double width);

  // pose (= getSplineInterpolatedPoint + getSplineInterpolatedYaw)
  geometry_msgs::msg::Pose getSplineInterpolatedPose(const size_t idx, const double s) const;

  // point
  geometry_msgs::msg::Point getSplineInterpolatedPoint(const size_t idx, const double s) const;

  // yaw
  double getSplineInterpolatedYaw(const size_t idx, const double s) const;
  std::vector<double> getSplineInterpolatedYaws() const;

  // curvature
  double getSplineInterpolatedCurvature(const size_t idx, const double s) const;
  std::vector<double> getSplineInterpolatedCurvatures() const;

  size_t getSize() const { return base_s_vec_.size(); }
  size_t getOffsetIndex(const size_t idx, const double offset) const;
  double getAccumulatedLength(const size_t idx) const;

private:
  void calcSplineCoefficientsInner(const std::vector<geometry_msgs::msg::Point> & points);
  SplineInterpolation spline_x_;
  SplineInterpolation spline_y_;

  std::vector<double> base_s_vec_;
};

#endif  // INTERPOLATION__SPLINE_INTERPOLATION_POINTS_2D_HPP_
