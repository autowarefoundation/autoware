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

#include "interpolation/spline_interpolation_points_2d.hpp"

#include <vector>

namespace
{
std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    return std::vector<double>{};
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (size_t i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}

std::array<std::vector<double>, 3> getBaseValues(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  // calculate x, y
  std::vector<double> base_x;
  std::vector<double> base_y;
  for (size_t i = 0; i < points.size(); i++) {
    const auto & current_pos = points.at(i);
    if (i > 0) {
      const auto & prev_pos = points.at(i - 1);
      if (
        std::fabs(current_pos.x - prev_pos.x) < 1e-6 &&
        std::fabs(current_pos.y - prev_pos.y) < 1e-6) {
        continue;
      }
    }
    base_x.push_back(current_pos.x);
    base_y.push_back(current_pos.y);
  }

  // calculate base_keys, base_values
  if (base_x.size() < 2 || base_y.size() < 2) {
    throw std::logic_error("The numbef of unique points is not enough.");
  }

  const std::vector<double> base_s = calcEuclidDist(base_x, base_y);

  return {base_s, base_x, base_y};
}
}  // namespace

namespace interpolation
{

std::array<std::vector<double>, 3> slerp2dFromXY(
  const std::vector<double> & base_keys, const std::vector<double> & base_x_values,
  const std::vector<double> & base_y_values, const std::vector<double> & query_keys)
{
  SplineInterpolation interpolator_x, interpolator_y;
  std::vector<double> yaw_vec;

  // calculate spline coefficients
  interpolator_x.calcSplineCoefficients(base_keys, base_x_values);
  interpolator_y.calcSplineCoefficients(base_keys, base_y_values);
  const auto diff_x = interpolator_x.getSplineInterpolatedDiffValues(query_keys);
  const auto diff_y = interpolator_y.getSplineInterpolatedDiffValues(query_keys);
  for (size_t i = 0; i < diff_x.size(); i++) {
    double yaw = std::atan2(diff_y[i], diff_x[i]);
    yaw_vec.push_back(yaw);
  }
  // interpolate base_keys at query_keys
  return {
    interpolator_x.getSplineInterpolatedValues(query_keys),
    interpolator_y.getSplineInterpolatedValues(query_keys), yaw_vec};
}

template <typename T>
std::vector<double> splineYawFromPoints(const std::vector<T> & points)
{
  SplineInterpolationPoints2d interpolator;

  // calculate spline coefficients
  interpolator.calcSplineCoefficients(points);

  // interpolate base_keys at query_keys
  std::vector<double> yaw_vec;
  for (size_t i = 0; i < points.size(); ++i) {
    const double yaw = interpolator.getSplineInterpolatedYaw(i, 0.0);
    yaw_vec.push_back(yaw);
  }
  return yaw_vec;
}
template std::vector<double> splineYawFromPoints(
  const std::vector<geometry_msgs::msg::Point> & points);

}  // namespace interpolation

geometry_msgs::msg::Point SplineInterpolationPoints2d::getSplineInterpolatedPoint(
  const size_t idx, const double s) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }

  double whole_s = base_s_vec_.at(idx) + s;
  if (whole_s < base_s_vec_.front()) {
    whole_s = base_s_vec_.front();
  }
  if (whole_s > base_s_vec_.back()) {
    whole_s = base_s_vec_.back();
  }

  const double x = spline_x_.getSplineInterpolatedValues({whole_s}).at(0);
  const double y = spline_y_.getSplineInterpolatedValues({whole_s}).at(0);

  geometry_msgs::msg::Point geom_point;
  geom_point.x = x;
  geom_point.y = y;
  return geom_point;
}

double SplineInterpolationPoints2d::getSplineInterpolatedYaw(const size_t idx, const double s) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }

  double whole_s = base_s_vec_.at(idx) + s;
  if (whole_s < base_s_vec_.front()) {
    whole_s = base_s_vec_.front();
  }
  if (whole_s > base_s_vec_.back()) {
    whole_s = base_s_vec_.back();
  }

  const double diff_x = spline_x_.getSplineInterpolatedDiffValues({whole_s}).at(0);
  const double diff_y = spline_y_.getSplineInterpolatedDiffValues({whole_s}).at(0);

  return std::atan2(diff_y, diff_x);
}

double SplineInterpolationPoints2d::getAccumulatedLength(const size_t idx) const
{
  if (base_s_vec_.size() <= idx) {
    throw std::out_of_range("idx is out of range.");
  }
  return base_s_vec_.at(idx);
}

void SplineInterpolationPoints2d::calcSplineCoefficientsInner(
  const std::vector<geometry_msgs::msg::Point> & points)
{
  const auto base = getBaseValues(points);

  base_s_vec_ = base.at(0);
  const auto & base_x_vec = base.at(1);
  const auto & base_y_vec = base.at(2);

  // calculate spline coefficients
  spline_x_.calcSplineCoefficients(base_s_vec_, base_x_vec);
  spline_y_.calcSplineCoefficients(base_s_vec_, base_y_vec);
}
