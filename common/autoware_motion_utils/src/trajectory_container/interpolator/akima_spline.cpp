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

#include "autoware/motion_utils/trajectory_container/interpolator/akima_spline.hpp"

#include <Eigen/Dense>

#include <cmath>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

void AkimaSpline::compute_parameters(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const Eigen::Ref<const Eigen::VectorXd> & values)
{
  const auto n = static_cast<int32_t>(axis.size());

  Eigen::VectorXd h = axis.tail(n - 1) - axis.head(n - 1);

  Eigen::VectorXd m(n - 1);
  for (int32_t i = 0; i < n - 1; ++i) {
    m[i] = (values[i + 1] - values[i]) / h[i];
  }

  Eigen::VectorXd s(n);
  s[0] = m[0];
  s[1] = (m[0] + m[1]) / 2;
  for (int32_t i = 2; i < n - 2; ++i) {
    if ((std::abs(m[i + 1] - m[i]) + std::abs(m[i - 1] - m[i - 2])) == 0) {
      s[i] = (m[i] + m[i - 1]) / 2;
    } else {
      s[i] = (std::abs(m[i + 1] - m[i]) * m[i - 1] + std::abs(m[i - 1] - m[i - 2]) * m[i]) /
             (std::abs(m[i + 1] - m[i]) + std::abs(m[i - 1] - m[i - 2]));
    }
  }
  s[n - 2] = (m[n - 2] + m[n - 3]) / 2;
  s[n - 1] = m[n - 2];

  a_.resize(n - 1);
  b_.resize(n - 1);
  c_.resize(n - 1);
  d_.resize(n - 1);
  for (int32_t i = 0; i < n - 1; ++i) {
    a_[i] = values[i];
    b_[i] = s[i];
    c_[i] = (3 * m[i] - 2 * s[i] - s[i + 1]) / h[i];
    d_[i] = (s[i] + s[i + 1] - 2 * m[i]) / (h[i] * h[i]);
  }
}

void AkimaSpline::build_impl(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis_ = axis;
  compute_parameters(
    this->axis_,
    Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size())));
}

double AkimaSpline::compute_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_[i];
  return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
}

double AkimaSpline::compute_first_derivative_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_[i];
  return b_[i] + 2 * c_[i] * dx + 3 * d_[i] * dx * dx;
}

double AkimaSpline::compute_second_derivative_impl(const double & s) const
{
  const int32_t i = this->get_index(s);
  const double dx = s - this->axis_[i];
  return 2 * c_[i] + 6 * d_[i] * dx;
}

}  // namespace autoware::motion_utils::trajectory_container::interpolator
