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

#include "autoware/motion_utils/trajectory_container/interpolator/linear.hpp"

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

void Linear::build_impl(
  const Eigen::Ref<const Eigen::VectorXd> & axis, const std::vector<double> & values)
{
  this->axis_ = axis;
  this->values_ =
    Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
}

double Linear::compute_impl(const double & s) const
{
  const int32_t idx = this->get_index(s);
  const double x0 = this->axis_(idx);
  const double x1 = this->axis_(idx + 1);
  const double y0 = this->values_(idx);
  const double y1 = this->values_(idx + 1);
  return y0 + (y1 - y0) * (s - x0) / (x1 - x0);
}

double Linear::compute_first_derivative_impl(const double & s) const
{
  const int32_t idx = this->get_index(s);
  const double x0 = this->axis_(idx);
  const double x1 = this->axis_(idx + 1);
  const double y0 = this->values_(idx);
  const double y1 = this->values_(idx + 1);
  return (y1 - y0) / (x1 - x0);
}

double Linear::compute_second_derivative_impl(const double &) const
{
  return 0.0;
}

size_t Linear::minimum_required_points() const
{
  return 2;
}

std::shared_ptr<Interpolator<double>> Linear::clone() const
{
  return std::make_shared<Linear>(*this);
}

}  // namespace autoware::motion_utils::trajectory_container::interpolator
