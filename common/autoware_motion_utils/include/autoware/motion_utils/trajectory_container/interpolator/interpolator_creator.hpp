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

#ifndef AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_CREATOR_HPP_
#define AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_CREATOR_HPP_

#include <Eigen/Dense>

#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion_utils::trajectory_container::interpolator
{

// Forward declaration
template <typename T>
class Interpolator;

template <typename InterpolatorType>
class InterpolatorCreator
{
private:
  Eigen::VectorXd axis_;
  std::vector<double> values_;

public:
  [[nodiscard]] InterpolatorCreator & set_axis(const Eigen::Ref<const Eigen::VectorXd> & axis)
  {
    axis_ = axis;
    return *this;
  }

  [[nodiscard]] InterpolatorCreator & set_axis(const std::vector<double> & axis)
  {
    axis_ = Eigen::Map<const Eigen::VectorXd>(axis.data(), static_cast<Eigen::Index>(axis.size()));
    return *this;
  }

  [[nodiscard]] InterpolatorCreator & set_values(const std::vector<double> & values)
  {
    values_ = values;
    return *this;
  }

  [[nodiscard]] InterpolatorCreator & set_values(const Eigen::Ref<const Eigen::VectorXd> & values)
  {
    values_ = std::vector<double>(values.begin(), values.end());
    return *this;
  }

  template <typename... Args>
  [[nodiscard]] std::optional<InterpolatorType> create(Args &&... args)
  {
    auto interpolator = InterpolatorType(std::forward<Args>(args)...);
    bool success = interpolator.build(axis_, values_);
    if (!success) {
      return std::nullopt;
    }
    return interpolator;
  }
};

}  // namespace autoware::motion_utils::trajectory_container::interpolator

#endif  // AUTOWARE__MOTION_UTILS__TRAJECTORY_CONTAINER__INTERPOLATOR__INTERPOLATOR_CREATOR_HPP_
