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

#include "autoware/motion_utils/trajectory_container/interpolator/cubic_spline.hpp"

#include <Eigen/Dense>

namespace autoware::motion_utils::trajectory_container::interpolator
{

void CubicSpline::compute_parameters(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const Eigen::Ref<const Eigen::VectorXd> & values)
{
  const int32_t n = static_cast<int32_t>(axis.size()) - 1;

  h_ = axis.tail(n) - axis.head(n);
  a_ = values.transpose();

  for (int32_t i = 0; i < n; ++i) {
    h_(i) = axis(i + 1) - axis(i);
  }

  Eigen::VectorXd alpha(n - 1);
  for (int32_t i = 1; i < n; ++i) {
    alpha(i - 1) = (3.0 / h_(i)) * (a_(i + 1) - a_(i)) - (3.0 / h_(i - 1)) * (a_(i) - a_(i - 1));
  }

  Eigen::VectorXd l(n + 1);
  Eigen::VectorXd mu(n + 1);
  Eigen::VectorXd z(n + 1);
  l(0) = 1.0;
  mu(0) = z(0) = 0.0;

  for (int32_t i = 1; i < n; ++i) {
    l(i) = 2.0 * (axis(i + 1) - axis(i - 1)) - h_(i - 1) * mu(i - 1);
    mu(i) = h_(i) / l(i);
    z(i) = (alpha(i - 1) - h_(i - 1) * z(i - 1)) / l(i);
  }
  b_.resize(n);
  d_.resize(n);
  c_.resize(n + 1);

  l(n) = 1.0;
  z(n) = c_(n) = 0.0;

  for (int32_t j = n - 1; j >= 0; --j) {
    c_(j) = z(j) - mu(j) * c_(j + 1);
    b_(j) = (a_(j + 1) - a_(j)) / h_(j) - h_(j) * (c_(j + 1) + 2.0 * c_(j)) / 3.0;
    d_(j) = (c_(j + 1) - c_(j)) / (3.0 * h_(j));
  }
}

void CubicSpline::build_impl(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis_ = axis;
  compute_parameters(
    this->axis_,
    Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size())));
}

double CubicSpline::compute_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_(i);
  return a_(i) + b_(i) * dx + c_(i) * dx * dx + d_(i) * dx * dx * dx;
}

double CubicSpline::compute_first_derivative_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_(i);
  return b_(i) + 2 * c_(i) * dx + 3 * d_(i) * dx * dx;
}

double CubicSpline::compute_second_derivative_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_(i);
  return 2 * c_(i) + 6 * d_(i) * dx;
}

std::shared_ptr<Interpolator<double>> CubicSpline::clone() const
{
  return std::make_shared<CubicSpline>(*this);
}

}  // namespace autoware::motion_utils::trajectory_container::interpolator
